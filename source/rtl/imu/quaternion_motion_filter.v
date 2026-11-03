// Q14 quaternion nlerp low-pass, sign aligned and re-normalized each report.
// One local signed 17x17 multiplier is shared by norm, dot and sqrt trials.
// Division uses restoring shifts/subtracts. No screen/window coordinates here.
module quaternion_motion_filter #(
 parameter integer FILTER_SHIFT=4,
 parameter integer ADAPTIVE=1,
 parameter integer MEDIUM_SHIFT=3,
 parameter integer FAST_SHIFT=2,
 parameter integer MEDIUM_THRESHOLD=512,
 parameter integer FAST_THRESHOLD=1536
)(input clk,rst_n,in_valid,clear,
 input signed [15:0] qi,qj,qk,qr,
 output reg out_valid,busy,ready,rejected,
 output wire [63:0] current_q,filtered_q);
localparam IDLE=0,CUR_ISSUE=1,CUR_SAVE=2,DOT_ISSUE=3,DOT_SAVE=4,
 BLEND=5,NORM_ISSUE=6,NORM_SAVE=7,SQ_ISSUE=8,SQ_SAVE=9,
 DIV_PREP=10,DIV_RUN=11,DONE=12;
reg [3:0] state;
reg signed [15:0] cur[0:3],smooth[0:3],mix[0:3],unitq[0:3];
reg [1:0] op;
reg signed [16:0] ma,mb;
reg signed [33:0] product;
reg signed [35:0] sum;
wire signed [35:0] next_sum=sum+{{2{product[33]}},product};
reg negative_dot;
reg [31:0] norm;
reg [15:0] root,denominator;
reg [3:0] root_bit;
wire [15:0] trial=root | (16'd1 << root_bit);
wire [15:0] root_next=($unsigned(product)<=norm)?trial:root;
reg [29:0] numerator,quotient;
reg [16:0] remainder;
reg [4:0] div_bit;
reg div_negative;
wire [16:0] remainder_shift={remainder[15:0],numerator[div_bit]};
wire div_take=remainder_shift >= {1'b0,denominator};
wire [29:0] quotient_next=quotient | (div_take?(30'd1<<div_bit):30'd0);
wire signed [16:0] mix_extended={mix[op][15],mix[op]};
wire [16:0] mix_magnitude=mix_extended[16]?$unsigned(-mix_extended):$unsigned(mix_extended);
integer k;
function signed [15:0] blend_q;
 input signed [15:0] old_q,new_q;
 input flip;
 input [3:0] shift;
 reg signed [31:0] a,b,d;
 begin
  a=old_q;b=new_q;if(flip)b=-b;
  d=b-a;
  blend_q=a+((d+(32'sd1<<(shift-1)))>>>shift);
 end
endfunction
reg [3:0] blend_shift;
reg [1:0] next_motion_mode;
reg [1:0] debug_motion_mode;
reg [20:0] motion_metric;
reg signed [17:0] aligned_difference;
integer m;
// Motion is estimated from the aligned quaternion delta. Hysteresis keeps
// the filter from chattering between shifts when the camera is nearly still.
always @* begin
 motion_metric=0;
 for(m=0;m<4;m=m+1)begin
  aligned_difference=$signed(negative_dot ? -cur[m] : cur[m])-$signed(smooth[m]);
  motion_metric=motion_metric+(aligned_difference<0?-aligned_difference:aligned_difference);
 end
 if(!ADAPTIVE)begin
  blend_shift=FILTER_SHIFT;next_motion_mode=0;
 end else begin
  case(debug_motion_mode)
   0:begin
    if(motion_metric>=FAST_THRESHOLD)next_motion_mode=2;
    else if(motion_metric>=MEDIUM_THRESHOLD)next_motion_mode=1;
    else next_motion_mode=0;
   end
   1:begin
    if(motion_metric>=FAST_THRESHOLD)next_motion_mode=2;
    else if(motion_metric<(MEDIUM_THRESHOLD/2))next_motion_mode=0;
    else next_motion_mode=1;
   end
   default:begin
    if(motion_metric<(MEDIUM_THRESHOLD/2))next_motion_mode=0;
    else if(motion_metric<FAST_THRESHOLD)next_motion_mode=1;
    else next_motion_mode=2;
   end
  endcase
  case(debug_motion_mode)
   0:blend_shift=FILTER_SHIFT;
   1:blend_shift=MEDIUM_SHIFT;
   default:blend_shift=FAST_SHIFT;
  endcase
 end
end
always @* begin
 ma=0;mb=0;
 case(state)
  CUR_ISSUE,CUR_SAVE:begin ma=cur[op];mb=cur[op];end
  DOT_ISSUE,DOT_SAVE:begin ma=cur[op];mb=smooth[op];end
  NORM_ISSUE,NORM_SAVE:begin ma=mix[op];mb=mix[op];end
  SQ_ISSUE,SQ_SAVE:begin ma=$signed({1'b0,trial});mb=$signed({1'b0,trial});end
 endcase
end
always @(posedge clk) product <= $signed(ma)*$signed(mb);
genvar component;
generate for(component=0;component<4;component=component+1)begin: aligned_sample
 assign current_q[component*16+:16]=negative_dot?-$signed(cur[component]):cur[component];
end endgenerate
assign filtered_q={unitq[3],unitq[2],unitq[1],unitq[0]};
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  state<=IDLE;out_valid<=0;busy<=0;ready<=0;rejected<=0;
  op<=0;sum<=0;negative_dot<=0;norm<=0;root<=0;root_bit<=0;denominator<=1;
   numerator<=0;quotient<=0;remainder<=0;div_bit<=0;div_negative<=0;debug_motion_mode<=0;
  for(k=0;k<4;k=k+1)begin cur[k]<=0;smooth[k]<=0;mix[k]<=0;unitq[k]<=0;end
 end else begin
  out_valid<=0;rejected<=0;
  if(clear) begin state<=IDLE;busy<=0;ready<=0;negative_dot<=0;end
  else case(state)
   IDLE:if(in_valid) begin
    cur[0]<=qi;cur[1]<=qj;cur[2]<=qk;cur[3]<=qr;
    sum<=0;op<=0;busy<=1;state<=CUR_ISSUE;
   end
   CUR_ISSUE:state<=CUR_SAVE;
   CUR_SAVE:begin
    sum<=next_sum;
    if(op==3)begin
     if(next_sum<36'sd265751101 || next_sum>36'sd271119811)begin
      rejected<=1;busy<=0;state<=IDLE;
     end else if(!ready)begin
      for(k=0;k<4;k=k+1)mix[k]<=cur[k];
      negative_dot<=0;
      norm<=next_sum[31:0];root<=0;root_bit<=15;state<=SQ_ISSUE;
     end else begin op<=0;sum<=0;state<=DOT_ISSUE;end
    end else begin op<=op+1'b1;state<=CUR_ISSUE;end
   end
   DOT_ISSUE:state<=DOT_SAVE;
   DOT_SAVE:begin
    sum<=next_sum;
    if(op==3)begin negative_dot<=next_sum[35];state<=BLEND;end
    else begin op<=op+1'b1;state<=DOT_ISSUE;end
   end
   BLEND:begin
    for(k=0;k<4;k=k+1)mix[k]<=blend_q(smooth[k],cur[k],negative_dot,blend_shift);
    debug_motion_mode<=next_motion_mode;
    op<=0;sum<=0;state<=NORM_ISSUE;
   end
   NORM_ISSUE:state<=NORM_SAVE;
   NORM_SAVE:begin
    sum<=next_sum;
    if(op==3)begin norm<=next_sum[31:0];root<=0;root_bit<=15;state<=SQ_ISSUE;end
    else begin op<=op+1'b1;state<=NORM_ISSUE;end
   end
   SQ_ISSUE:state<=SQ_SAVE;
   SQ_SAVE:begin
    root<=root_next;
    if(root_bit==0)begin
     denominator<=root_next;op<=0;state<=DIV_PREP;
    end else begin root_bit<=root_bit-1'b1;state<=SQ_ISSUE;end
   end
   DIV_PREP:begin
    numerator<=({13'd0,mix_magnitude}<<14);
    quotient<=0;remainder<=0;div_bit<=29;div_negative<=mix[op][15];state<=DIV_RUN;
   end
   DIV_RUN:begin
    remainder<=div_take?(remainder_shift-{1'b0,denominator}):remainder_shift;
    quotient<=quotient_next;
    if(div_bit==0)begin
     unitq[op]<=div_negative?-$signed(quotient_next[15:0]):$signed(quotient_next[15:0]);
     if(op==3)state<=DONE;
     else begin op<=op+1'b1;state<=DIV_PREP;end
    end else div_bit<=div_bit-1'b1;
   end
   DONE:begin
    for(k=0;k<4;k=k+1)smooth[k]<=unitq[k];
    ready<=1;out_valid<=1;busy<=0;state<=IDLE;
   end
   default:state<=IDLE;
  endcase
 end
endmodule
