// One tile of a 4x4 contrast-limited histogram equalizer. A sparse histogram
// is collected during the active frame and converted to a 256-entry map in
// vertical blanking. The published map is never modified during a frame.
module clahe_tile(input clk,rst_n,frame_boundary,sample,
 input [7:0] sample_luma,lookup_luma,
 output reg [7:0] mapped,output reg valid,output wire glare);
 localparam [12:0] SAMPLES=13'd8100;
 localparam [12:0] CLIP=13'd512;
 localparam IDLE=0,PASS1_READ=1,PASS1_ACC=2,PASS2_READ=3,
            PASS2_ACC=4,DIV_TRIAL=5,DIV_CHECK=6,FILL=7,PREP_DIV=8,
            CLASSIFY=9,FILL_BLEND=10,FILL_WRITE=11,PASS1_CHECK=12,
            DIV_COMPARE=13,DIV_DECIDE=14,PASS2_APPLY=15;
 reg [3:0] state;
 reg [12:0] histogram[0:63];
 reg [12:0] hist_read;
 reg [5:0] bin,sample_bin,sample_bin2;
 reg pending,pending2,initializing;
 reg [12:0] sample_incremented;
 reg [12:0] excess,redistribution,sample_count;
 reg [12:0] excess_increment;
 reg [13:0] percentile_sum;
 reg [5:0] p10_bin,p90_bin;
 reg got10,got90;
 reg [1:0] strength;
 reg [7:0] map_value_q,write_address_q,write_address_qq,published_map_q;
 reg [13:0] cumulative;
 reg [13:0] pass2_increment;
 reg [20:0] dividend;
 reg [20:0] candidate_product_q;
 reg trial_pass;
 reg [7:0] quotient,candidate,previous_map,current_map;
 reg [2:0] bit_index;
 reg [1:0] fill_index;
 reg active_bank,ready,held_good;
 reg pending_glare,published_glare;
 assign glare=valid && published_glare;
 reg [7:0] lut0[0:255],lut1[0:255];
 wire [12:0] limited=hist_read>CLIP?CLIP:hist_read;
 wire [13:0] next_percentile_sum=percentile_sum+{1'b0,hist_read};
 wire [1:0] desired_strength=(p90_bin>=6'd2 && p90_bin<=6'd32 &&
                              p90_bin>=p10_bin+6'd2)?2'd2:
                             (p10_bin>=6'd18 && p10_bin<6'd52 &&
                              p90_bin>=6'd52 && p90_bin>=p10_bin+6'd8)?2'd3:2'd0;
 wire [20:0] candidate_ext={13'd0,candidate};
 wire [20:0] candidate_product=(candidate_ext<<13)-(candidate_ext<<6)-
                              (candidate_ext<<4)-(candidate_ext<<3)-(candidate_ext<<2);
 wire [7:0] accepted=trial_pass?candidate:quotient;
 // Integer identities preserve floor rounding for negative differences.
 wire [8:0] pair_sum={1'b0,previous_map}+{1'b0,current_map};
 wire [9:0] quarter_sum={1'b0,pair_sum}+{1'b0,previous_map,1'b0};
 wire [7:0] map_value=(fill_index==0)?previous_map:
                      (fill_index==1)?quarter_sum[9:2]:
                      (fill_index==2)?pair_sum[8:1]:current_map;
 wire [7:0] write_address={bin,fill_index};
 wire signed [9:0] equalize_delta=$signed({2'b0,map_value_q})-$signed({2'b0,write_address_q});
 wire signed [9:0] scaled_delta=(strength==0)?10'sd0:
                                (strength==1)?(equalize_delta>>>1):
                                (strength==2)?equalize_delta:
                                ((equalize_delta>>>1)+(equalize_delta>>>2));
 wire signed [10:0] adjusted_map=$signed({3'b0,write_address_q})+scaled_delta;
 wire [7:0] published_map=adjusted_map<0?8'd0:
                           adjusted_map>255?8'd255:adjusted_map[7:0];
 // Lift recoverable shadow levels in dark tiles without changing pure black.
 wire [7:0] shadow_floor=(write_address_q==0)?8'd0:
                          (write_address_q<8'd16)?({write_address_q[5:0],2'b0}+8'd8):
                          (write_address_q<8'd64)?(write_address_q+8'd56):
                          (8'd88+{1'b0,write_address_q[7:1]});
 always @(posedge clk)begin
  if(initializing)histogram[bin]<=0;
  else if(state==PASS2_ACC)histogram[bin]<=0;
  else if(pending2)histogram[sample_bin2]<=sample_incremented;
  if(state==PASS1_READ || state==PASS2_READ)hist_read<=histogram[bin];
  else if(sample && state==IDLE)hist_read<=histogram[sample_luma[7:2]];
  if(state==FILL_WRITE)begin
   if(active_bank)lut0[write_address_qq]<=published_map_q;
   else lut1[write_address_qq]<=published_map_q;
  end
  mapped<=active_bank?lut1[lookup_luma]:lut0[lookup_luma];
 end
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  state<=IDLE;bin<=0;sample_bin<=0;sample_bin2<=0;
  pending<=0;pending2<=0;sample_incremented<=0;initializing<=1;
  excess<=0;redistribution<=0;sample_count<=0;cumulative<=0;excess_increment<=0;
  pass2_increment<=0;dividend<=0;
  percentile_sum<=0;p10_bin<=0;p90_bin<=0;got10<=0;got90<=0;strength<=0;
  map_value_q<=0;write_address_q<=0;write_address_qq<=0;published_map_q<=0;
  quotient<=0;candidate<=0;candidate_product_q<=0;trial_pass<=0;
  previous_map<=0;current_map<=0;
  bit_index<=0;fill_index<=0;active_bank<=0;ready<=0;held_good<=0;valid<=0;pending_glare<=0;published_glare<=0;
 end else begin
  pending<=sample && state==IDLE && !initializing;
  pending2<=pending;
  if(pending)begin sample_bin2<=sample_bin;sample_incremented<=hist_read+13'd1;end
  if(sample && state==IDLE && !initializing)begin sample_bin<=sample_luma[7:2];sample_count<=sample_count+1'b1;end
  if(initializing)begin
   pending<=0;pending2<=0;
   if(bin==63)begin bin<=0;initializing<=0;end
   else bin<=bin+1'b1;
  end else if(frame_boundary)begin
   if(state!=IDLE)begin valid<=0;initializing<=1;bin<=0;state<=IDLE;ready<=0;pending<=0;pending2<=0;end
   else begin
    if(ready)begin active_bank<=~active_bank;valid<=1;ready<=0;published_glare<=pending_glare;end
    else valid<=0;
    state<=PASS1_READ;bin<=0;excess<=0;cumulative<=0;previous_map<=0;
    percentile_sum<=0;p10_bin<=0;p90_bin<=0;got10<=0;got90<=0;
    held_good<=sample_count==SAMPLES;
    sample_count<=0;
   end
  end else case(state)
   PASS1_READ:state<=PASS1_ACC;
   PASS1_ACC:begin
    percentile_sum<=next_percentile_sum;
    // Split RAM output -> clip/subtract -> accumulation across existing FSM
    // edges. Publication and all per-pixel timing remain exactly unchanged.
    excess_increment<=hist_read>CLIP?hist_read-CLIP:13'd0;
    state<=PASS1_CHECK;
   end
   PASS1_CHECK:begin
    excess<=excess+excess_increment;
    if(!got10 && percentile_sum>=14'd810)begin p10_bin<=bin;got10<=1;end
    if(!got90 && percentile_sum>=14'd7290)begin p90_bin<=bin;got90<=1;end
    if(bin==63)begin bin<=0;state<=CLASSIFY;end
    else begin bin<=bin+1'b1;state<=PASS1_READ;end
   end
   CLASSIFY:begin
    redistribution<=excess>>6;
    pending_glare<=desired_strength==2'd3;
    if(strength<desired_strength)strength<=strength+1'b1;
    else if(strength>desired_strength)strength<=strength-1'b1;
    state<=PASS2_READ;
   end
   PASS2_READ:state<=PASS2_ACC;
   PASS2_ACC:begin
    pass2_increment<={1'b0,limited}+{1'b0,redistribution};
    state<=PASS2_APPLY;
   end
   PASS2_APPLY:begin cumulative<=cumulative+pass2_increment;state<=PREP_DIV;end
   PREP_DIV:begin
    dividend<=({7'd0,cumulative}<<8)-{7'd0,cumulative};
    quotient<=0;bit_index<=7;state<=DIV_TRIAL;
   end
   DIV_TRIAL:begin candidate<=quotient|(8'd1<<bit_index);state<=DIV_CHECK;end
   DIV_CHECK:begin candidate_product_q<=candidate_product;state<=DIV_COMPARE;end
   DIV_COMPARE:begin trial_pass<=candidate_product_q<=dividend;state<=DIV_DECIDE;end
   DIV_DECIDE:begin
    quotient<=accepted;
    if(bit_index==0)begin current_map<=accepted;fill_index<=0;state<=FILL;end
    else begin bit_index<=bit_index-1'b1;state<=DIV_TRIAL;end
   end
   FILL:begin
    map_value_q<=map_value;write_address_q<=write_address;
    state<=FILL_BLEND;
   end
   FILL_BLEND:begin
    // Strong shoulder for glare-classified tiles; leave the dark end intact.
    if(strength==2'd2 && published_map<shadow_floor)
     published_map_q<=shadow_floor;
    else if(strength==2'd3 && published_map[7] && (|published_map[6:0]))
     published_map_q<=8'd96+{2'b0,published_map[7:2]};
    else published_map_q<=published_map;
    write_address_qq<=write_address_q;
    state<=FILL_WRITE;
   end
   FILL_WRITE:begin
    if(fill_index==3)begin
     previous_map<=current_map;
     if(bin==63)begin state<=IDLE;ready<=held_good;end
     else begin bin<=bin+1'b1;state<=PASS2_READ;end
    end else begin fill_index<=fill_index+1'b1;state<=FILL;end
   end
   default:state<=IDLE;
  endcase
 end
endmodule
