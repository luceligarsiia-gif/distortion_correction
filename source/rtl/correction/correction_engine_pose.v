// Four coordinate slots, two source-cache banks and two output banks.
// Mapping, DDR loading, interpolation and DDR storing run independently.
// Complete per-pixel bounds and frame-locked pose/calibration are preserved.
// Addresses count 32-bit words; one 256-bit beat contains 16 RGB565 pixels.
module correction_engine_pose #(parameter STREAM_CAPTURE=0)(input clk,rst_n,start,
 input [10:0] source_rows,input source_bad,
 input [27:0] source_base,dest_base,input [287:0] pose_matrix,input affine_crop,
 output busy,output reg done,fault,
 output [27:0] araddr,output [3:0] arlen,output arvalid,input arready,
 input [255:0] rdata,input rvalid,rlast,
 output [27:0] awaddr,output [3:0] awlen,output awvalid,input awready,
 output [255:0] wdata,input wready,wlast,
 output reg [31:0] frame_cycles,clipped_tiles);
localparam MWAIT=0,MSCAN=1,MBOUNDS=2,MFIN=3;
localparam LWAIT=0,LRADDR=1,LRDATA=2,LFIN=3;
localparam CWAIT=0,CRENDER=1,CDRAIN=2,CFIN=3;
localparam WWAIT=0,WPREP=1,WADDR=2,WDATA=3,WFIN=4;
reg running;
reg [1:0] map_state,load_state,compute_state;
reg [2:0] write_state;
wire [8:0] state={map_state,load_state,compute_state,write_state}; // simulation diagnostic
reg [27:0] src,dst;
reg [287:0] matrix;
reg frame_affine_crop;
reg [1:0] map_slot,load_slot,compute_slot,write_slot;
wire load_bank=load_slot[0],compute_bank=compute_slot[0],write_bank=write_slot[0];
reg [3:0] coord_reserved,coord_ready;
reg [1:0] cache_reserved,cache_ready,out_reserved,out_ready;
reg [10:0] tx,ty;
reg [7:0] tile_cols;
wire [4:0] tile_rows=(ty==1072)?5'd8:5'd16;
wire [10:0] pairs=(tile_cols==128)?{tile_rows,6'd0}:
                         ((tile_cols==64)?{1'b0,tile_rows,5'd0}:{2'b0,tile_rows,4'd0});
wire [10:0] remaining=11'd1920-tx-tile_cols;
reg [10:0] issued,received,minx,miny,maxx,maxy;
reg [10:0] desc_x[0:3],desc_y[0:3],desc_bx[0:3],desc_by[0:3],desc_pairs[0:3];
reg [7:0] desc_cols[0:3];
reg [4:0] desc_height[0:3],desc_beats[0:3];
reg [6:0] desc_rows[0:3];
reg [3:0] desc_blank,desc_last;
reg [10:0] load_x,load_y;
reg [4:0] load_beats;
reg [6:0] load_rows,load_row;
reg [3:0] load_beat;
reg load_last;
reg [10:0] compute_x,compute_y,sent,results,compute_pairs;
reg compute_blank,compute_last;
reg [10:0] out_x[0:1],out_y[0:1];
reg [7:0] out_cols[0:1];
reg [4:0] out_height[0:1];
reg [1:0] out_last;
reg [10:0] write_x,write_y;
reg [7:0] write_cols;
reg [4:0] write_rows;
reg [3:0] write_row;
reg [2:0] write_beat;
reg write_last;
wire [3:0] write_beats=write_cols>>4;
wire begin_frame=start && !running;
assign busy=running;
assign araddr=src+(load_y+load_row)*28'd960+({17'd0,load_x}>>1);
assign arlen=load_beats-1'b1;
assign arvalid=running && load_state==LRADDR;
assign awaddr=dst+(write_y+write_row)*28'd960+({17'd0,write_x}>>1);
assign awlen=write_beats-1'b1;
assign awvalid=running && write_state==WADDR;
wire ra=running && rvalid && (load_state==LRDATA || (load_state==LRADDR && arready));
wire wa=running && wready && (write_state==WDATA || (write_state==WADDR && awready));
wire map_start=running && map_state==MWAIT && !coord_reserved[map_slot];
wire [11:0] load_row_limit={1'b0,desc_by[load_slot]}+desc_rows[load_slot];
wire load_safe=!STREAM_CAPTURE || source_bad || desc_blank[load_slot] || {1'b0,source_rows}>=load_row_limit;
wire load_start=load_safe && running && load_state==LWAIT && coord_ready[load_slot] && !cache_reserved[load_bank];
wire load_finished=(load_start && desc_blank[load_slot]) ||
                   (ra && load_beat==load_beats-1'b1 && load_row==load_rows-1'b1);
wire compute_start=running && compute_state==CWAIT && cache_ready[compute_bank] && !out_reserved[compute_bank];
wire write_start=running && write_state==WWAIT && out_ready[write_bank];
wire write_finished=wa && write_beat==write_beats-1'b1 && write_row==write_rows-1'b1;
wire [1:0] mapvalid,cachevalid,resvalid,cachefault;
wire signed [31:0] uq[0:1],vq[0:1];
wire [12:0] reciprocal_address[0:1];wire [17:0] reciprocal_data[0:1];
reciprocal_q18_dual reciprocal_table(.clk(clk),
 .address0(reciprocal_address[0]),.address1(reciprocal_address[1]),
 .data0(reciprocal_data[0]),.data1(reciprocal_data[1]));
wire compute_finished=running && compute_state==CDRAIN && (&resvalid) && results==compute_pairs-1'b1;
// Four coordinate slots, now storing only two signed Q10 values (44 bits).
// The codec canonicalizes wholly invisible rays; observable samples are exact.
localparam [43:0] BLACK_COORD={-22'sd2048,-22'sd2048};
reg [43:0] coord0[0:4095],coord1[0:4095];
reg [43:0] cq0,cq1;
(* syn_preserve=1 *) reg [81:0] cq0_pipe,cq1_pipe;
reg cv_pipe,cv;
wire [81:0] decoded0,decoded1;
wire [43:0] packed0,packed1;
coordinate_pack_q10 pack0(.u(uq[0]),.v(vq[0]),.packed_word(packed0));
coordinate_pack_q10 pack1(.u(uq[1]),.v(vq[1]),.packed_word(packed1));
coordinate_unpack_q10 unpack0(.packed_word(cq0),.cache_word(decoded0));
coordinate_unpack_q10 unpack1(.packed_word(cq1),.cache_word(decoded1));
// Reuse the existing timing barrier: no extra video or coordinate cycle.
always @(posedge clk)begin cq0_pipe<=decoded0;cq1_pipe<=decoded1;end
always @(posedge clk or negedge rst_n)if(!rst_n)cv_pipe<=0;else cv_pipe<=cv;
wire render_fire=running && compute_state==CRENDER && sent<compute_pairs;
wire [15:0] result[0:1];
genvar lane;
generate for(lane=0;lane<2;lane=lane+1) begin: pixels
 wire [15:0] ix=(tile_cols==128)?{9'd0,issued[5:0],1'b0}:
                       ((tile_cols==64)?{10'd0,issued[4:0],1'b0}:{11'd0,issued[3:0],1'b0});
 wire [15:0] iy=(tile_cols==128)?{12'd0,issued[9:6]}:
                       ((tile_cols==64)?{12'd0,issued[8:5]}:{12'd0,issued[7:4]});
 wire [15:0] ox={5'd0,tx}+ix+lane;
 wire [15:0] oy={5'd0,ty}+iy;
 pose_distortion_map #(.EXTERNAL_RECIPROCAL(1)) mapper(
 .clk(clk),.rst_n(rst_n),.in_valid(running && map_state==MSCAN && issued<pairs),
 .in_u(ox),.in_v(oy),.matrix(matrix),.affine_crop(frame_affine_crop),
 .out_valid(mapvalid[lane]),.out_u(uq[lane]),.out_v(vq[lane]),
 .reciprocal_address(reciprocal_address[lane]),.reciprocal_data(reciprocal_data[lane]));
 wire [81:0] cq=(lane==0)?cq0_pipe:cq1_pipe;
 wire [15:0] p00,p10,p01,p11;
 wire [9:0] fx,fy;
 tile_cache_pose #(.USE_ADDRESS_HINTS(1)) cache(.clk(clk),.rst_n(rst_n),.load_bank(load_bank),.read_bank(compute_bank),
  .load_en(ra),.load_y(load_row[5:0]),.load_beat(load_beat),.load_data(rdata),
  .in_valid(cv_pipe),.uq($signed(cq[63:32])),.vq($signed(cq[31:0])),.base_x(compute_x),.base_y(compute_y),
  .address_hint(cq[81:64]),.out_valid(cachevalid[lane]),.p00(p00),.p10(p10),.p01(p01),.p11(p11),.fx(fx),.fy(fy),.fault(cachefault[lane]));
 bilinear565 interp(clk,rst_n,1'b1,cachevalid[lane],p00,p10,p01,p11,fx,fy,resvalid[lane],result[lane]);
end endgenerate
always @(posedge clk) begin
 if(&mapvalid) begin coord0[{map_slot,received[9:0]}]<=packed0;coord1[{map_slot,received[9:0]}]<=packed1;end
 if(render_fire) begin
  cq0<=compute_blank?BLACK_COORD:coord0[{compute_slot,sent[9:0]}];
  cq1<=compute_blank?BLACK_COORD:coord1[{compute_slot,sent[9:0]}];
 end
end
// Parallel per-ray clamp and pipelined reduction retain complete bounds.
reg [10:0] lane_minx[0:1],lane_miny[0:1],lane_maxx[0:1],lane_maxy[0:1];
reg [10:0] pair_minx,pair_miny,pair_maxx,pair_maxy;
reg bounds_v0,bounds_v1,bounds_last0,bounds_last1;
genvar bounds_lane;
generate for(bounds_lane=0;bounds_lane<2;bounds_lane=bounds_lane+1) begin: ray_bounds
 wire signed [21:0] ix=uq[bounds_lane][31:10],iy=vq[bounds_lane][31:10];
 wire neighbor_valid=ix>=-1 && ix<1920 && iy>=-1 && iy<1080;
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin lane_minx[bounds_lane]<=1920;lane_miny[bounds_lane]<=1080;lane_maxx[bounds_lane]<=0;lane_maxy[bounds_lane]<=0;end
  else begin
   lane_minx[bounds_lane]<=neighbor_valid?((ix<0)?11'd0:ix[10:0]):11'd1920;
   lane_miny[bounds_lane]<=neighbor_valid?((iy<0)?11'd0:iy[10:0]):11'd1080;
   lane_maxx[bounds_lane]<=neighbor_valid?((ix>=1919)?11'd1919:(ix+1'b1)):11'd0;
   lane_maxy[bounds_lane]<=neighbor_valid?((iy>=1079)?11'd1079:(iy+1'b1)):11'd0;
  end
end endgenerate
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  pair_minx<=1920;pair_miny<=1080;pair_maxx<=0;pair_maxy<=0;bounds_v0<=0;bounds_v1<=0;bounds_last0<=0;bounds_last1<=0;
 end else begin
  bounds_v0<=&mapvalid;bounds_v1<=bounds_v0;
  bounds_last0<=received==pairs-1'b1;bounds_last1<=bounds_last0;
  pair_minx<=(lane_minx[0]<lane_minx[1])?lane_minx[0]:lane_minx[1];
  pair_miny<=(lane_miny[0]<lane_miny[1])?lane_miny[0]:lane_miny[1];
  pair_maxx<=(lane_maxx[0]>lane_maxx[1])?lane_maxx[0]:lane_maxx[1];
  pair_maxy<=(lane_maxy[0]>lane_maxy[1])?lane_maxy[0]:lane_maxy[1];
 end
wire [7:0] span_beats={1'b0,maxx[10:4]}-{1'b0,minx[10:4]}+1'b1;
wire [11:0] span_rows={1'b0,maxy}-{1'b0,miny[10:1],1'b0}+1'b1;
wire split_needed=minx<=maxx && miny<=maxy && (span_beats>16 || span_rows>64) && tile_cols>32;
wire map_finished=running && map_state==MBOUNDS && !split_needed;
wire [6:0] row_word=(write_cols==128)?{write_row,3'd0}:
                         ((write_cols==64)?{1'b0,write_row,2'd0}:{2'b0,write_row,1'b0});
wire [6:0] read_word=row_word+write_beat+(wa?7'd1:7'd0);
genvar outlane;
generate for(outlane=0;outlane<16;outlane=outlane+1) begin: output_lane
 reg [15:0] mem[0:255];reg [15:0] q;
 always @(posedge clk) begin
  if((&resvalid) && results[2:0]==outlane/2) mem[{compute_bank,results[9:3]}]<=result[outlane%2];
  q<=mem[{write_bank,read_word}];
 end
 assign wdata[outlane*16+:16]=q;
end endgenerate
// Every ownership bit has a single writer. A stalled downstream stage retains
// its bank, so no scanner/loader/renderer can overwrite data still in use.
genvar slot,bank;
generate for(slot=0;slot<4;slot=slot+1) begin: coordinate_ownership
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin coord_reserved[slot]<=0;coord_ready[slot]<=0;end
  else if(begin_frame) begin coord_reserved[slot]<=0;coord_ready[slot]<=0;end
  else begin
   coord_reserved[slot]<=(coord_reserved[slot] || (map_start && map_slot==slot)) && !(compute_finished && compute_slot==slot);
   coord_ready[slot]<=(coord_ready[slot] || (map_finished && map_slot==slot)) && !(load_start && load_slot==slot);
  end
end
for(bank=0;bank<2;bank=bank+1) begin: cache_output_ownership
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin cache_reserved[bank]<=0;cache_ready[bank]<=0;out_reserved[bank]<=0;out_ready[bank]<=0;end
  else if(begin_frame) begin cache_reserved[bank]<=0;cache_ready[bank]<=0;out_reserved[bank]<=0;out_ready[bank]<=0;end
  else begin
   cache_reserved[bank]<=(cache_reserved[bank] || (load_start && load_bank==bank)) && !(compute_finished && compute_bank==bank);
   cache_ready[bank]<=(cache_ready[bank] || (load_finished && load_bank==bank)) && !(compute_start && compute_bank==bank);
   out_reserved[bank]<=(out_reserved[bank] || (compute_start && compute_bank==bank)) && !(write_finished && write_bank==bank);
   out_ready[bank]<=(out_ready[bank] || (compute_finished && compute_bank==bank)) && !(write_start && write_bank==bank);
  end
end endgenerate
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  running<=0;map_state<=MWAIT;load_state<=LWAIT;compute_state<=CWAIT;write_state<=WWAIT;
  src<=0;dst<=0;matrix<=0;frame_affine_crop<=0;tx<=0;ty<=0;tile_cols<=128;
  map_slot<=0;load_slot<=0;compute_slot<=0;write_slot<=0;
  load_x<=0;load_y<=0;load_beats<=0;load_rows<=0;load_row<=0;load_beat<=0;load_last<=0;
  compute_x<=0;compute_y<=0;compute_pairs<=0;compute_blank<=0;compute_last<=0;
  write_x<=0;write_y<=0;write_cols<=128;write_rows<=16;write_row<=0;write_beat<=0;write_last<=0;
  issued<=0;received<=0;sent<=0;results<=0;minx<=1920;miny<=1080;maxx<=0;maxy<=0;
  cv<=0;done<=0;fault<=0;frame_cycles<=0;clipped_tiles<=0;
 end else begin
  done<=0;cv<=render_fire;
  if(begin_frame) begin
   running<=1;map_state<=MWAIT;load_state<=LWAIT;compute_state<=CWAIT;write_state<=WWAIT;
   src<=source_base;dst<=dest_base;matrix<=pose_matrix;frame_affine_crop<=affine_crop;tx<=0;ty<=0;tile_cols<=128;
   map_slot<=0;load_slot<=0;compute_slot<=0;write_slot<=0;
   issued<=0;received<=0;sent<=0;results<=0;minx<=1920;miny<=1080;maxx<=0;maxy<=0;
   load_row<=0;load_beat<=0;write_row<=0;write_beat<=0;
   fault<=|cachefault;frame_cycles<=0;clipped_tiles<=0;
  end else if(running) begin
   frame_cycles<=frame_cycles+1'b1;
   if(|cachefault || (mapvalid[0]!=mapvalid[1]) || (resvalid[0]!=resvalid[1])) fault<=1;
   if(&mapvalid) received<=received+1'b1;
   if(bounds_v1) begin
    if(pair_minx<minx) minx<=pair_minx;
    if(pair_miny<miny) miny<=pair_miny;
    if(pair_maxx>maxx) maxx<=pair_maxx;
    if(pair_maxy>maxy) maxy<=pair_maxy;
   end
   if(&resvalid) results<=results+1'b1;
   case(map_state)
    MWAIT:if(map_start) begin
     issued<=0;received<=0;minx<=1920;miny<=1080;maxx<=0;maxy<=0;map_state<=MSCAN;
    end
    MSCAN:begin
     if(issued<pairs) issued<=issued+1'b1;
     if(bounds_v1 && bounds_last1) map_state<=MBOUNDS;
    end
    MBOUNDS:begin
     if(split_needed) begin
      tile_cols<=tile_cols>>1;map_state<=MSCAN;issued<=0;received<=0;minx<=1920;miny<=1080;maxx<=0;maxy<=0;
     end else begin
      desc_x[map_slot]<=tx;desc_y[map_slot]<=ty;desc_cols[map_slot]<=tile_cols;desc_height[map_slot]<=tile_rows;
      desc_bx[map_slot]<={minx[10:4],4'd0};desc_by[map_slot]<={miny[10:1],1'b0};
      desc_beats[map_slot]<=span_beats;desc_rows[map_slot]<=span_rows;desc_pairs[map_slot]<=pairs;
      desc_blank[map_slot]<=minx>maxx || miny>maxy || span_beats>16 || span_rows>64;
      desc_last[map_slot]<=remaining==0 && ty==1072;
      if(minx<=maxx && miny<=maxy && (span_beats>16 || span_rows>64)) clipped_tiles<=clipped_tiles+1'b1;
      if(remaining==0 && ty==1072) map_state<=MFIN;
      else begin
       map_slot<=map_slot+1'b1;map_state<=MWAIT;
       if(remaining==0) begin tx<=0;ty<=ty+16;tile_cols<=128;end
       else begin tx<=tx+tile_cols;tile_cols<=(remaining>=128)?128:((remaining>=64)?64:32);end
      end
     end
    end
    default:;
   endcase
   if(load_start) begin
    load_x<=desc_bx[load_slot];load_y<=desc_by[load_slot];load_beats<=desc_beats[load_slot];load_rows<=desc_rows[load_slot];
    load_last<=desc_last[load_slot];load_row<=0;load_beat<=0;load_state<=LRADDR;
   end else if(load_state==LRADDR && arready) load_state<=LRDATA;
   if(ra) begin
    if(rlast!=(load_beat==load_beats-1'b1)) fault<=1;
    if(load_beat==load_beats-1'b1) begin load_beat<=0;load_row<=load_row+1'b1;load_state<=LRADDR;end
    else load_beat<=load_beat+1'b1;
   end
   if(load_finished) begin
    if((load_start && desc_last[load_slot]) || (!load_start && load_last)) load_state<=LFIN;
    else begin load_slot<=load_slot+1'b1;load_state<=LWAIT;end
   end
   case(compute_state)
    CWAIT:if(compute_start) begin
     compute_x<=desc_bx[compute_slot];compute_y<=desc_by[compute_slot];compute_pairs<=desc_pairs[compute_slot];
     compute_blank<=desc_blank[compute_slot];compute_last<=desc_last[compute_slot];
     out_x[compute_bank]<=desc_x[compute_slot];out_y[compute_bank]<=desc_y[compute_slot];
     out_cols[compute_bank]<=desc_cols[compute_slot];out_height[compute_bank]<=desc_height[compute_slot];out_last[compute_bank]<=desc_last[compute_slot];
     sent<=0;results<=0;compute_state<=CRENDER;
    end
    CRENDER:begin sent<=sent+1'b1;if(sent==compute_pairs-1'b1) compute_state<=CDRAIN;end
    CDRAIN:if(compute_finished) begin
     if(compute_last) compute_state<=CFIN;
     else begin compute_slot<=compute_slot+1'b1;compute_state<=CWAIT;end
    end
    default:;
   endcase
   case(write_state)
    WWAIT:if(write_start) begin
     write_x<=out_x[write_bank];write_y<=out_y[write_bank];write_cols<=out_cols[write_bank];write_rows<=out_height[write_bank];write_last<=out_last[write_bank];
     write_row<=0;write_beat<=0;write_state<=WPREP;
    end
    WPREP:write_state<=WADDR;
    WADDR:if(awready) write_state<=WDATA;
    default:;
   endcase
   if(wa) begin
    if(wlast!=(write_beat==write_beats-1'b1)) fault<=1;
    if(write_beat==write_beats-1'b1) begin write_beat<=0;write_row<=write_row+1'b1;write_state<=WPREP;end
    else write_beat<=write_beat+1'b1;
   end
   if(write_finished) begin
    if(write_last) begin running<=0;done<=1;write_state<=WFIN;end
    else begin write_slot<=write_slot+1'b1;write_state<=WWAIT;end
   end
  end
 end
endmodule
