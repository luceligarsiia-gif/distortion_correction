// Q12 degrees. Circular filter with a fixed motion coefficient and watchdog.
// mode: 0 quiet (1/4 after 300ms dwell), 1 motion (1/2); 2 reserved.
// 50ms: stale/hold. 200ms: slew to neutral at <=1 degree per 10ms.
// Recovery slews <=2 degrees per accepted report without changing reference.
module affine_angle_filter #(
 parameter signed [31:0] DEADBAND=82,
 parameter integer CLK_HZ=27000000,STALE_MS=50,FALLBACK_MS=200
)(
 input clk,rst_n,in_valid,input signed [31:0] in_angle,
 output reg out_valid,output reg signed [31:0] out_angle,
 output reg [1:0] mode,output reg [7:0] age_ms,
 output wire stale,output reg recovering);
localparam signed [31:0] HALF_TURN=737280,FULL_TURN=1474560;
localparam integer TICKS_PER_MS=CLK_HZ/1000;
reg initialized;
reg [31:0] ms_ticks;
reg [3:0] neutral_ticks;
reg [8:0] quiet_ms;
reg signed [31:0] previous,estimate,delta,next_angle,difference,step_angle;
reg signed [31:0] motion_delta;
reg [31:0] abs_motion;
wire [7:0] interval_ms=(age_ms==0)?8'd1:age_ms;
wire [15:0] slow_limit=interval_ms*16'd8; // ~2 degrees/second
wire [15:0] wake_limit=interval_ms*16'd16; // ~4 degrees/second
reg [1:0] selected_mode;
wire [9:0] quiet_next={1'b0,quiet_ms}+{2'b0,interval_ms};
// Freshness and bounded angle catch-up are independent. A fresh accepted
// report must resume publication even while the angle is still recovering.
// Otherwise the selector freezes every recovery step until catch-up finishes.
assign stale=!initialized || age_ms>=STALE_MS;
function signed [31:0] wrap;
 input signed [31:0] a;
 begin
  if(a>HALF_TURN)wrap=a-FULL_TURN;
  else if(a< -HALF_TURN)wrap=a+FULL_TURN;
  else wrap=a;
 end
endfunction
always @* begin
 motion_delta=wrap(in_angle-previous);
 abs_motion=motion_delta<0?-motion_delta:motion_delta;
 // A stale interval must not count as stationary dwell.
 if(recovering || age_ms>=STALE_MS)selected_mode=1;
 else if(mode==0 && abs_motion<=wake_limit)selected_mode=0;
 else if(abs_motion<=slow_limit && quiet_next>=300)selected_mode=0;
 else selected_mode=1;
 delta=wrap(in_angle-estimate);
 case(selected_mode)
  0:step_angle=delta>>>2;
  default:step_angle=delta>>>1;
 endcase
 if(recovering || age_ms>=STALE_MS)begin
  if(delta>8192)step_angle=8192;
  else if(delta< -8192)step_angle=-8192;
  else step_angle=delta;
 end
 next_angle=wrap(estimate+step_angle);
 difference=wrap(next_angle-out_angle);
end
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  initialized<=0;estimate<=0;out_valid<=0;out_angle<=0;previous<=0;
  mode<=1;age_ms<=255;ms_ticks<=0;neutral_ticks<=0;quiet_ms<=0;recovering<=0;
 end
 else begin
  out_valid<=0;
  if(ms_ticks==TICKS_PER_MS-1)begin
   ms_ticks<=0;
   if(age_ms<255)age_ms<=age_ms+1'b1;
   if(age_ms>=STALE_MS)begin recovering<=initialized;quiet_ms<=0;mode<=1;end
   if(initialized && age_ms>=FALLBACK_MS)begin
    if(neutral_ticks==9)begin
     neutral_ticks<=0;out_valid<=1;
     if(out_angle>4096)begin out_angle<=out_angle-4096;estimate<=out_angle-4096;end
     else if(out_angle< -4096)begin out_angle<=out_angle+4096;estimate<=out_angle+4096;end
     else begin out_angle<=0;estimate<=0;end
    end else neutral_ticks<=neutral_ticks+1'b1;
   end else neutral_ticks<=0;
  end else ms_ticks<=ms_ticks+1'b1;
  if(in_valid)begin
   ms_ticks<=0;age_ms<=0;neutral_ticks<=0;previous<=in_angle;
   if(initialized && !recovering && age_ms<STALE_MS && abs_motion<=slow_limit)
    quiet_ms<=quiet_next>=300?9'd300:quiet_next[8:0];
   else quiet_ms<=0;
   if(!initialized)begin
    initialized<=1;estimate<=in_angle;out_angle<=in_angle;out_valid<=1;mode<=1;recovering<=0;
   end else begin
    estimate<=next_angle;mode<=selected_mode;out_valid<=1;
    recovering<=(recovering || age_ms>=STALE_MS) && (delta>8192 || delta< -8192);
    if(selected_mode!=0 || recovering || age_ms>=STALE_MS ||
       difference>DEADBAND || difference< -DEADBAND)out_angle<=next_angle;
   end
  end
 end
endmodule
