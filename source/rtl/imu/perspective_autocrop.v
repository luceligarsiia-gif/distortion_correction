// Find a frame-wide zoom by checking the four homogeneous output corner rays.
// All rays remain in the calibrated rectangle when corner Z is positive.
// Exactly one local 17x17 multiplier, shared by matrix scaling and all checks.
// Column scaling R*diag(s,s,1) composes crop with perspective before ONE sampling.
module perspective_autocrop #(
 parameter integer MIN_SCALE=2048, RELEASE_STEP=128, SEARCH_EPS=32,
 parameter integer PLANAR_TILE_GUARD=0,
 parameter integer SMOOTH_ENABLE=0, MARGIN_SHIFT=5,
 parameter integer ATTACK_SHIFT=2, RELEASE_SHIFT=4,
 parameter integer ATTACK_MAX_STEP=256, RELEASE_MAX_STEP=64,
 parameter integer SCALE_DEADBAND=32, RELEASE_HOLD_REPORTS=10
)(input clk,rst_n,in_valid,input [287:0] in_matrix,
 output reg busy,out_valid,output reg [287:0] out_matrix,
 output reg [14:0] scale_q14,output reg limited);
localparam IDLE=0,SCALE_ISSUE=1,SCALE_SAVE=2,DOT_ISSUE=3,DOT_SAVE=4,
 ZCHECK=5,PLANE_ISSUE=6,PLANE_SAVE=7,DECIDE=8,
 COST0_ISSUE=9,COST0_SAVE=10,COST1_ISSUE=11,COST1_SAVE=12;
localparam [287:0] ID={32'd16384,32'd0,32'd0,32'd0,32'd16384,32'd0,32'd0,32'd0,32'd16384};
reg [3:0] state,cell_id;
reg [1:0] corner,row,term,plane;
reg [287:0] held,work,best_matrix;
reg [14:0] level,low,high,previous;
reg searching,check_min,passed;
reg smoothing_check,have_previous;
reg [7:0] release_hold;
reg [14:0] safe_scale;
reg safe_limited;
reg signed [16:0] xyz[0:2];
reg signed [16:0] ma,mb;
reg signed [33:0] product;
reg signed [35:0] sum;
wire signed [35:0] sum_next=sum+{{2{product[33]}},product};
wire signed [35:0] dot_value=(sum_next+36'sd8192)>>>14;
wire signed [34:0] scaled=($signed({product[33],product})+35'sd8192)>>>14;
wire signed [34:0] bound=($signed({product[33],product})+35'sd16384)>>>15;
wire plane_failed=(plane==0 && $signed(xyz[0])<bound+35'sd32) ||
 (plane==1 && $signed(xyz[0])>bound-35'sd32) ||
 (plane==2 && $signed(xyz[1])<bound+35'sd32) ||
 (plane==3 && $signed(xyz[1])>bound-35'sd32);
wire [15:0] release_target={1'b0,previous}+RELEASE_STEP;
always @*begin
 ma=0;mb=0;
 case(state)
 SCALE_ISSUE,SCALE_SAVE:begin ma=$signed(held[cell_id*32+:32]);mb=$signed({2'd0,level});end
 DOT_ISSUE,DOT_SAVE:begin
  ma=$signed(work[(row*3+term)*32+:32]);
  if(term==0)mb=corner[0]?17'sd32668:-17'sd32603;
  else if(term==1)mb=corner[1]?17'sd18475:-17'sd18216;
  else mb=17'sd32768;
 end
 PLANE_ISSUE,PLANE_SAVE:begin
  ma=xyz[2];
  // Q15 calibrated source bounds, tightened before the per-pixel Q16 guard.
  case(plane)0:mb=-17'sd39500;1:mb=17'sd42016;2:mb=-17'sd22150;default:mb=17'sd23598;endcase
 end
 COST0_ISSUE,COST0_SAVE:begin
  ma=$signed(work[0*32+:32]);if(ma<0)ma=-ma;mb=17'sd7013;
 end
 COST1_ISSUE,COST1_SAVE:begin
  ma=$signed(work[3*32+:32]);if(ma<0)ma=-ma;mb=17'sd26764;
 end
 endcase
end
always @(posedge clk)product<=ma*mb;
integer i;
task restart_check;
 begin cell_id<=0;work<=held;corner<=0;row<=0;term<=0;sum<=0;state<=SCALE_ISSUE;end
endtask
task publish(input [287:0] m,input [14:0] s,input is_limited);
 begin out_matrix<=m;scale_q14<=s;previous<=s;have_previous<=1;
  limited<=is_limited;out_valid<=1;busy<=0;state<=IDLE;end
endtask
// First solve the CURRENT pose's safe bound without release-rate limiting.
// Then smooth only uniform crop, not pose/viewport coordinates. A safety clamp
// can override the slew limit on abrupt motion: never retain an unsafe scale.
task finish_safe(input [287:0] m,input [14:0] s,input is_limited);
 reg [15:0] target,candidate,distance,step_size;
 begin
  if(!SMOOTH_ENABLE || is_limited)begin
   if(is_limited)release_hold<=RELEASE_HOLD_REPORTS;
   publish(m,s,is_limited);
  end else begin
   target={1'b0,s}-({1'b0,s}>>MARGIN_SHIFT);
   if(target<MIN_SCALE)target=MIN_SCALE;
   candidate={1'b0,previous};
   if(release_hold!=0)release_hold<=release_hold-1'b1;
   if(!have_previous)begin candidate=target;release_hold<=RELEASE_HOLD_REPORTS;end
   else if(target+SCALE_DEADBAND<previous)begin
    distance=previous-target;step_size=distance>>ATTACK_SHIFT;
    if(step_size<1)step_size=1;
    if(step_size>ATTACK_MAX_STEP)step_size=ATTACK_MAX_STEP;
    candidate=previous-step_size;release_hold<=RELEASE_HOLD_REPORTS;
   end else if(target>previous+SCALE_DEADBAND && release_hold==0)begin
    distance=target-previous;step_size=distance>>RELEASE_SHIFT;
    if(step_size<1)step_size=1;
    if(step_size>RELEASE_MAX_STEP)step_size=RELEASE_MAX_STEP;
    candidate=previous+step_size;
   end
   if(candidate>s)begin candidate=s;release_hold<=RELEASE_HOLD_REPORTS;end
   if(candidate<MIN_SCALE)candidate=MIN_SCALE;
   best_matrix<=m;safe_scale<=s;safe_limited<=is_limited;
   level<=candidate[14:0];smoothing_check<=1;
   restart_check();
  end
 end
endtask
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  state<=IDLE;busy<=0;out_valid<=0;out_matrix<=ID;scale_q14<=16384;limited<=0;
  held<=ID;work<=ID;best_matrix<=ID;level<=16384;low<=MIN_SCALE;high<=16384;previous<=16384;
  searching<=0;check_min<=0;passed<=0;cell_id<=0;corner<=0;row<=0;term<=0;plane<=0;sum<=0;
  smoothing_check<=0;have_previous<=0;release_hold<=0;
  safe_scale<=16384;safe_limited<=0;
  for(i=0;i<3;i=i+1)xyz[i]<=0;
 end else begin
  out_valid<=0;
  case(state)
  IDLE:if(in_valid)begin
   held<=in_matrix;work<=in_matrix;busy<=1;searching<=0;check_min<=0;
   level<=SMOOTH_ENABLE?15'd16384:(release_target>16384?15'd16384:release_target[14:0]);
   smoothing_check<=0;
   cell_id<=0;state<=SCALE_ISSUE;
  end
  SCALE_ISSUE:begin
   if(cell_id%3==2)begin
    work[cell_id*32+:32]<=held[cell_id*32+:32];
    if(cell_id==8)begin corner<=0;row<=0;term<=0;sum<=0;state<=PLANAR_TILE_GUARD?COST0_ISSUE:DOT_ISSUE;end
    else cell_id<=cell_id+1'b1;
   end else state<=SCALE_SAVE;
  end
  SCALE_SAVE:begin work[cell_id*32+:32]<=scaled[31:0];cell_id<=cell_id+1'b1;state<=SCALE_ISSUE;end
  // Affine-only 128x16 tile row bound: shared multiplier, Q14 * Q8.
  // Brown |dY/dX|<=0.15, |dY/dY|<=1.01 in the calibrated rectangle.
  // Keep source row span <=60px, leaving bilinear/alignment/rounding margin
  // inside the existing 64-row cache. Perspective leaves this check disabled.
  COST0_ISSUE:state<=COST0_SAVE;
  COST0_SAVE:begin sum<={{2{product[33]}},product};state<=COST1_ISSUE;end
  COST1_ISSUE:state<=COST1_SAVE;
  COST1_SAVE:begin
   sum<=0;
   if(sum_next>36'sd251658240)begin passed<=0;state<=DECIDE;end
   else state<=DOT_ISSUE;
  end
  DOT_ISSUE:state<=DOT_SAVE;
  DOT_SAVE:begin
   if(term==2)begin
    xyz[row]<=dot_value[16:0];sum<=0;term<=0;
    if(row==2)state<=ZCHECK;else begin row<=row+1'b1;state<=DOT_ISSUE;end
   end else begin sum<=sum_next;term<=term+1'b1;state<=DOT_ISSUE;end
  end
  ZCHECK:begin
   if(xyz[2]<6144)begin passed<=0;state<=DECIDE;end
   else begin plane<=0;state<=PLANE_ISSUE;end
  end
  PLANE_ISSUE:state<=PLANE_SAVE;
  PLANE_SAVE:begin
   if(plane_failed)begin passed<=0;state<=DECIDE;end
   else if(plane==3)begin
    if(corner==3)begin passed<=1;state<=DECIDE;end
    else begin corner<=corner+1'b1;row<=0;term<=0;sum<=0;state<=DOT_ISSUE;end
   end else begin plane<=plane+1'b1;state<=PLANE_ISSUE;end
  end
  DECIDE:begin
   if(smoothing_check)begin
    // Recheck rounded, composed coefficients with the same corner/row guards.
    // Fall back to the solved safe matrix if quantization defeats smoothing.
    if(passed)publish(work,level,safe_limited);
    else begin publish(best_matrix,safe_scale,safe_limited);release_hold<=RELEASE_HOLD_REPORTS;end
   end else if(!searching)begin
    if(passed)finish_safe(work,level,1'b0);
    else if(level<=MIN_SCALE)finish_safe(work,level,1'b1);
    else begin
     searching<=1;check_min<=1;low<=MIN_SCALE;high<=level-1'b1;level<=MIN_SCALE;restart_check();
    end
   end else if(check_min)begin
    if(!passed)finish_safe(work,level,1'b1);
    else if(high-low<=SEARCH_EPS)finish_safe(work,level,1'b0);
    else begin
     best_matrix<=work;check_min<=0;level<=({1'b0,low}+{1'b0,high}+16'd1)>>1;restart_check();
    end
   end else if(passed)begin
    if(high-level<=SEARCH_EPS)finish_safe(work,level,1'b0);
    else begin
     low<=level;best_matrix<=work;level<=({1'b0,level}+{1'b0,high}+16'd1)>>1;restart_check();
    end
   end else begin
    if(level-1'b1-low<=SEARCH_EPS)finish_safe(best_matrix,low,1'b0);
    else begin high<=level-1'b1;level<=({1'b0,low}+{1'b0,level})>>1;restart_check();end
   end
  end
  default:state<=IDLE;
  endcase
 end
endmodule
