// Two pixels/cycle at the existing 125 MHz DDR user clock.
// Independent load, compute and store controllers overlap adjacent tiles.
// DDR addresses count 32-bit words; each beat advances by 8 words.
module correction_engine(input clk,rst_n,start,
 input [27:0] source_base,dest_base,
 output busy,output reg done,output reg fault,
 output [27:0] araddr,output [3:0] arlen,output arvalid,input arready,
 input [255:0] rdata,input rvalid,rlast,
 output [27:0] awaddr,output [3:0] awlen,output awvalid,input awready,
 output [255:0] wdata,input wready,wlast,
 output reg [31:0] frame_cycles);
localparam LWAIT=0,LDESC0=1,LDESC1=2,LDESC2=3,LRADDR=4,LRDATA=5,LFIN=6;
localparam CWAIT=0,CMAP=1,CDRAIN=2,CFIN=3;
localparam WWAIT=0,WPREP=1,WADDR=2,WDATA=3,WFIN=4;
reg running;
reg [27:0] src,dst;
reg [2:0] load_state,write_state;
reg [1:0] compute_state;
reg [9:0] load_tile,compute_tile,write_tile;
reg [10:0] compute_x,compute_y,write_x,write_y;
reg [10:0] load_x,load_y;
reg [4:0] load_beats;
reg [5:0] load_rows,load_row;
reg [3:0] load_beat;
reg [9:0] issue_count,result_count;
reg [4:0] compute_rows,write_row;
reg [2:0] write_beat;
reg [1:0] loaded,out_reserved,out_ready;
reg [10:0] bank_x[0:1],bank_y[0:1];
wire load_bank=load_tile[0],compute_bank=compute_tile[0],write_bank=write_tile[0];
wire [9:0] tile_id=compute_tile;
wire [2:0] state={1'b0,compute_state};
wire begin_frame=start && !running;
wire [47:0] desc;
tile_bounds bounds(clk,load_tile,desc);
assign busy=running;
assign araddr=src+((load_y+load_row)*28'd960)+({17'd0,load_x}>>1);
assign arlen=load_beats-1'b1;
assign arvalid=running && load_state==LRADDR;
assign awaddr=dst+((write_y+write_row)*28'd960)+({17'd0,write_x}>>1);
assign awlen=4'd7;
assign awvalid=running && write_state==WADDR;
// Also accept first data beat on the address handshake cycle.
wire load_accept=running && rvalid &&
 (load_state==LRDATA || (load_state==LRADDR && arready));
wire write_accept=running && wready &&
 (write_state==WDATA || (write_state==WADDR && awready));
wire load_finished=load_accept && load_beat==load_beats-1'b1 && load_row==load_rows-1'b1;
wire compute_start=running && compute_state==CWAIT && loaded[compute_bank] && !out_reserved[compute_bank];
wire write_start=running && write_state==WWAIT && out_ready[write_bank];
wire [4:0] write_rows=(write_y==1072)?5'd8:5'd16;
wire write_finished=write_accept && write_beat==7 && write_row==write_rows-1'b1;
wire [1:0] map_valid,cache_valid,result_valid,cache_fault;
wire [15:0] result[0:1];
wire compute_finished=running && compute_state==CDRAIN && (&result_valid) &&
 result_count==({compute_rows,6'd0}-1'b1);
// Single writer per ownership flag, independent of bus stall duration.
genvar bank;
generate for(bank=0;bank<2;bank=bank+1) begin: ownership
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin loaded[bank]<=0;out_reserved[bank]<=0;out_ready[bank]<=0;end
  else if(begin_frame) begin loaded[bank]<=0;out_reserved[bank]<=0;out_ready[bank]<=0;end
  else begin
   loaded[bank]<=(loaded[bank] || (load_finished && load_bank==bank)) &&
                 !(compute_finished && compute_bank==bank);
   out_reserved[bank]<=(out_reserved[bank] || (compute_start && compute_bank==bank)) &&
                       !(write_finished && write_bank==bank);
   out_ready[bank]<=(out_ready[bank] || (compute_finished && compute_bank==bank)) &&
                    !(write_start && write_bank==bank);
  end
end endgenerate

genvar pixel_lane;
generate for(pixel_lane=0;pixel_lane<2;pixel_lane=pixel_lane+1) begin: pixels
 wire signed [31:0] uq,vq;
 wire [15:0] p00,p10,p01,p11;
 wire [9:0] fx,fy;
 wire [15:0] outx={5'd0,compute_x}+{9'd0,issue_count[5:0],1'b0}+pixel_lane;
 wire [15:0] outy={5'd0,compute_y}+{12'd0,issue_count[9:6]};
 distortion_map mapper(.clk(clk),.rst_n(rst_n),.ce(1'b1),
  .in_valid(running && compute_state==CMAP),.in_u(outx),.in_v(outy),
  .out_valid(map_valid[pixel_lane]),.out_u(uq),.out_v(vq),.center_outside());
 tile_cache cache(.clk(clk),.rst_n(rst_n),.load_bank(load_bank),.read_bank(compute_bank),
  .load_en(load_accept),.load_y(load_row[4:0]),.load_beat(load_beat),.load_data(rdata),
  .in_valid(map_valid[pixel_lane]),.uq(uq),.vq(vq),
  .base_x(bank_x[compute_bank]),.base_y(bank_y[compute_bank]),
  .out_valid(cache_valid[pixel_lane]),.p00(p00),.p10(p10),.p01(p01),.p11(p11),
  .fx(fx),.fy(fy),.fault(cache_fault[pixel_lane]));
 bilinear565 interp(.clk(clk),.rst_n(rst_n),.ce(1'b1),.in_valid(cache_valid[pixel_lane]),
  .p00(p00),.p10(p10),.p01(p01),.p11(p11),.fx(fx),.fy(fy),
  .out_valid(result_valid[pixel_lane]),.pixel(result[pixel_lane]));
end endgenerate
wire [6:0] read_word={write_row[3:0],write_beat}+(write_accept?7'd1:7'd0);
genvar lane;
generate for(lane=0;lane<16;lane=lane+1) begin: output_lane
 reg [15:0] mem[0:255];
 reg [15:0] q;
 always @(posedge clk) begin
  if(result_valid[lane%2] && result_count[2:0]==lane/2)
   mem[{compute_bank,result_count[9:3]}]<=result[lane%2];
  q<=mem[{write_bank,read_word}];
 end
 assign wdata[lane*16+:16]=q;
end endgenerate
always @(posedge clk or negedge rst_n)
if(!rst_n) begin
 running<=0;done<=0;fault<=0;src<=0;dst<=0;frame_cycles<=0;
 load_state<=LWAIT;compute_state<=CWAIT;write_state<=WWAIT;
 load_tile<=0;compute_tile<=0;write_tile<=0;
 compute_x<=0;compute_y<=0;write_x<=0;write_y<=0;
 load_x<=0;load_y<=0;load_beats<=0;load_rows<=0;load_row<=0;load_beat<=0;
 issue_count<=0;result_count<=0;compute_rows<=16;write_row<=0;write_beat<=0;
 bank_x[0]<=0;bank_x[1]<=0;bank_y[0]<=0;bank_y[1]<=0;
end else begin
 done<=0;
 if(begin_frame) begin
  running<=1;fault<=|cache_fault;src<=source_base;dst<=dest_base;frame_cycles<=0;
  load_state<=LWAIT;compute_state<=CWAIT;write_state<=WWAIT;
  load_tile<=0;compute_tile<=0;write_tile<=0;
  compute_x<=0;compute_y<=0;write_x<=0;write_y<=0;
  load_row<=0;load_beat<=0;issue_count<=0;result_count<=0;
  compute_rows<=16;write_row<=0;write_beat<=0;
 end else if(running) begin
  frame_cycles<=frame_cycles+1'b1;
  if((|cache_fault) || (result_valid[0]!=result_valid[1])) fault<=1;
  if(&result_valid) result_count<=result_count+1'b1;
  case(load_state)
   LWAIT: if(!loaded[load_bank]) load_state<=LDESC0;
   LDESC0: load_state<=LDESC1;
   LDESC1: load_state<=LDESC2;
   LDESC2: begin
    load_x<=desc[10:0];load_y<=desc[21:11];load_beats<=desc[26:22];load_rows<=desc[33:27];
    bank_x[load_bank]<=desc[10:0];bank_y[load_bank]<=desc[21:11];
    if(desc[33:27]>32 || desc[33:27]==0 || desc[26:22]>16 || desc[26:22]==0) fault<=1;
    load_row<=0;load_beat<=0;load_state<=LRADDR;
   end
   LRADDR: if(arready) load_state<=LRDATA;
   default: ;
  endcase
  if(load_accept) begin
   if(rlast != (load_beat==load_beats-1'b1)) fault<=1;
   if(load_beat==load_beats-1'b1) begin
    load_beat<=0;
    if(load_row==load_rows-1'b1) begin
     if(load_tile==1019) load_state<=LFIN;
     else begin load_tile<=load_tile+1'b1;load_state<=LWAIT;end
    end else begin load_row<=load_row+1'b1;load_state<=LRADDR;end
   end else load_beat<=load_beat+1'b1;
  end
  case(compute_state)
   CWAIT: if(compute_start) begin
    issue_count<=0;result_count<=0;compute_rows<=(compute_y==1072)?8:16;compute_state<=CMAP;
   end
   CMAP: begin
    issue_count<=issue_count+1'b1;
    if(issue_count==({compute_rows,6'd0}-1'b1)) compute_state<=CDRAIN;
   end
   CDRAIN: if(compute_finished) begin
    if(compute_tile==1019) compute_state<=CFIN;
    else begin
     compute_tile<=compute_tile+1'b1;compute_state<=CWAIT;
     if(compute_x==1792) begin compute_x<=0;compute_y<=compute_y+16;end
     else compute_x<=compute_x+128;
    end
   end
   default: ;
  endcase
  case(write_state)
   WWAIT: if(write_start) begin write_row<=0;write_beat<=0;write_state<=WPREP;end
   WPREP: write_state<=WADDR;
   WADDR: if(awready) write_state<=WDATA;
   default: ;
  endcase
  if(write_accept) begin
   if(wlast != (write_beat==7)) fault<=1;
   if(write_beat==7) begin
    write_beat<=0;
    if(write_row==write_rows-1'b1) begin
     if(write_tile==1019) begin running<=0;done<=1;write_state<=WFIN;end
     else begin
      write_tile<=write_tile+1'b1;write_state<=WWAIT;
      if(write_x==1792) begin write_x<=0;write_y<=write_y+16;end
      else write_x<=write_x+128;
     end
    end else begin write_row<=write_row+1'b1;write_state<=WPREP;end
   end else write_beat<=write_beat+1'b1;
  end
 end
end
endmodule
