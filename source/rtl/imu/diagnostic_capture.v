// All observation uses SYS clock; no ready/backpressure into functional logic.
// Packet interval is measured BEFORE accuracy/norm filtering (0.1 ms units).
// Curves share affine_reference_q and the same camera basis/sign convention.
module diagnostic_capture #(
 parameter integer CLK_HZ=27000000,
 parameter AXIS_X=0,AXIS_Y=1,AXIS_Z=2,SIGN_X=1,SIGN_Y=-1,SIGN_Z=-1
)(input clk,rst_n,enabled,online,reference_valid,input [63:0] reference_q,
 input report_valid,input [63:0] report_q,input [1:0] accuracy,
 input [4:0] events,input filter_rejected,pose_stale,
 input predict_anomaly,
 input filtered_valid,input signed [31:0] filtered_angle,
 input display_valid,input [48:0] display_data,
 input [15:0] telemetry_lost,
 output reg out_valid,output reg [383:0] data);
localparam integer TICKS=CLK_HZ/10000;
reg [31:0] sub_ticks;
reg [6:0] bin_ticks;
reg [15:0] age,raw_age,filtered_age,dt,dt_max,dt_bin;
reg have_report,raw_have,filtered_have,display_have,reference_d,stale_d;
reg [7:0] epoch;
reg [15:0] seq;
reg [15:0] display_lost;
reg signed [31:0] raw_last,raw_min,raw_max,filtered_last,display_last;
reg bin_seen,bin_wrap,bin_event;
reg [15:0] n_nack,n_bus,n_packet,n_acc0,n_reject,n_gap,n_stale,n_diag,n_rx,n_retry;
wire active=enabled && online && reference_valid;
wire tenth_tick=sub_ticks==TICKS-1;
wire sample_tick=tenth_tick && bin_ticks==99;
reg raw_start,raw_wait;
reg [7:0] raw_cycles;
reg [63:0] raw_q,raw_ref;
wire [63:0] relative;
wire relative_valid,raw_valid;
wire signed [31:0] raw_angle;
wire raw_reset=rst_n && active;
affine_relative_quaternion raw_relative(clk,raw_reset,raw_start,raw_q,raw_ref,relative,relative_valid,);
quaternion_roll_direct #(.AXIS_X(AXIS_X),.AXIS_Y(AXIS_Y),.AXIS_Z(AXIS_Z),
 .SIGN_X(SIGN_X),.SIGN_Y(SIGN_Y),.SIGN_Z(SIGN_Z)) raw_roll(
 .clk(clk),.rst_n(raw_reset),.in_valid(relative_valid),.relative_q(relative),
 .out_valid(raw_valid),.angle(raw_angle));
function [15:0] inc;input [15:0] v;begin inc=(v==65535)?v:v+1'b1;end endfunction
wire signed [31:0] raw_difference=raw_angle-raw_last;
wire raw_cross=raw_have && (raw_difference>737280 || raw_difference< -737280);
wire gap_event=have_report && tenth_tick && age==499;
wire stale_event=active && pose_stale && !stale_d;
wire any_event=(|events) || (report_valid && accuracy==0) || filter_rejected || predict_anomaly || gap_event || stale_event;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  sub_ticks<=0;bin_ticks<=0;age<=0;raw_age<=65535;filtered_age<=65535;dt<=0;dt_max<=0;dt_bin<=0;
  have_report<=0;raw_have<=0;filtered_have<=0;display_have<=0;reference_d<=0;stale_d<=0;
  epoch<=0;seq<=0;display_lost<=0;raw_last<=0;raw_min<=0;raw_max<=0;filtered_last<=0;display_last<=0;
  bin_seen<=0;bin_wrap<=0;bin_event<=0;raw_start<=0;raw_wait<=0;raw_cycles<=0;raw_q<=0;raw_ref<=0;
  n_nack<=0;n_bus<=0;n_packet<=0;n_acc0<=0;n_reject<=0;n_gap<=0;n_stale<=0;n_diag<=0;n_rx<=0;n_retry<=0;
  out_valid<=0;data<=0;
 end else begin
  out_valid<=0;raw_start<=0;reference_d<=active;stale_d<=pose_stale;
  if(tenth_tick)begin
   sub_ticks<=0;age<=inc(age);raw_age<=inc(raw_age);filtered_age<=inc(filtered_age);
   if(bin_ticks==99)bin_ticks<=0;else bin_ticks<=bin_ticks+1'b1;
  end else sub_ticks<=sub_ticks+1'b1;
  if(events[0])n_nack<=inc(n_nack);
  if(events[1])n_bus<=inc(n_bus);
  if(events[2] || events[4])n_packet<=inc(n_packet);
  if(events[3])n_retry<=inc(n_retry);
  if(filter_rejected)n_reject<=inc(n_reject);
  if(gap_event)n_gap<=inc(n_gap);
  if(stale_event)n_stale<=inc(n_stale);
  if(predict_anomaly)n_diag<=inc(n_diag);
  if(report_valid)begin
   n_rx<=inc(n_rx);age<=0;have_report<=1;
   if(accuracy==0)n_acc0<=inc(n_acc0);
   if(have_report)begin
    dt<=age;
    if(age>dt_max)dt_max<=age;
    if(age>dt_bin || sample_tick)dt_bin<=age;
   end
   if(active)begin
    if(!raw_wait)begin raw_q<=report_q;raw_ref<=reference_q;raw_start<=1;raw_wait<=1;raw_cycles<=0;end
    else begin n_diag<=inc(n_diag);bin_event<=1;end
   end
  end
  if(raw_wait)begin
   raw_cycles<=raw_cycles+1'b1;
   if(raw_cycles==200)begin raw_wait<=0;raw_have<=0;n_diag<=inc(n_diag);bin_event<=1;end
  end
  if(raw_valid && active)begin
   raw_wait<=0;raw_have<=1;raw_age<=0;raw_last<=raw_angle;
   if(!bin_seen || sample_tick)begin raw_min<=raw_angle;raw_max<=raw_angle;end
   else if(raw_cross)begin raw_min<=raw_angle;raw_max<=raw_angle;end
   else begin
    if(raw_angle<raw_min)raw_min<=raw_angle;
    if(raw_angle>raw_max)raw_max<=raw_angle;
   end
   bin_seen<=1;
   if(raw_cross)bin_wrap<=1;
  end
  if(filtered_valid && active)begin filtered_last<=filtered_angle;filtered_have<=1;filtered_age<=0;end
  if(display_valid)begin display_last<=$signed(display_data[31:0]);display_have<=display_data[32];display_lost<=display_data[48:33];end
  if(any_event)bin_event<=1;
  if(sample_tick)begin
   out_valid<=1;seq<=seq+1'b1;data<=0;
   data[20:0]<=bin_seen?raw_min[20:0]:raw_last[20:0];
   data[41:21]<=bin_seen?raw_max[20:0]:raw_last[20:0];
   data[62:42]<=filtered_last[20:0];data[83:63]<=display_last[20:0];
   data[99:84]<=dt;data[115:100]<=dt_max;data[131:116]<=age;
   data[139:132]<={have_report,enabled,active,bin_event,bin_wrap,
       (display_have && active),(filtered_have && active && filtered_age<500),
       (raw_have && active && raw_age<500)};
   data[147:140]<=epoch;
   data[163:148]<=n_nack;data[179:164]<=n_bus;data[195:180]<=n_packet;
   data[211:196]<=n_acc0;data[227:212]<=n_reject;data[243:228]<=n_gap;
   data[259:244]<=n_stale;data[275:260]<=n_diag;data[291:276]<=n_rx;
   data[307:292]<=seq;data[323:308]<=dt_bin;data[339:324]<=telemetry_lost;data[355:340]<=n_retry;
   data[356]<=online;
   data[383:368]<=display_lost;
   // Coincident events belong to the next time bin, never disappear.
   bin_seen<=raw_valid && active;bin_wrap<=raw_valid && raw_cross;
   bin_event<=any_event || (raw_wait && raw_cycles==200) || (report_valid && active && raw_wait);
   dt_bin<=(report_valid && have_report)?age:0;
  end
  if(!active)begin raw_wait<=0;raw_start<=0;raw_have<=0;filtered_have<=0;bin_seen<=0;bin_wrap<=0;end
  if(active && !reference_d)begin epoch<=epoch+1'b1;raw_have<=0;filtered_have<=0;bin_seen<=0;bin_wrap<=0;end
 end
endmodule
