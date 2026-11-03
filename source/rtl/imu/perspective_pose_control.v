// Full 3-axis inverse rotation for output-to-input perspective sampling.
// Capture a new reference on each S5 enable (and after sensor timeout/reconnect).
// OFF publishes identity: fixed lens correction remains, no residual pose warp.
module perspective_pose_control #(
 parameter AXIS_X=0,AXIS_Y=1,AXIS_Z=2,SIGN_X=1,SIGN_Y=-1,SIGN_Z=-1,
 parameter integer CLK_HZ=27000000,STALE_MS=100
)(input clk,rst_n,enable,online,report_valid,input [63:0] current_q,
 output reg out_valid,output reg [287:0] matrix,output reg out_affine_crop,
 output reference_ready,ready,pose_stale,pose_recovering,
 output [7:0] debug_age_ms,output [1:0] debug_filter_mode,
 output signed [31:0] debug_angle_q12,
 output reg [63:0] debug_reference_q,output reg debug_reference_valid,
 output signed [31:0] debug_filtered_angle,output debug_filtered_valid);
localparam [287:0] IDENTITY={32'd16384,32'd0,32'd0,32'd0,32'd16384,32'd0,32'd0,32'd0,32'd16384};
localparam integer MS_TICKS=CLK_HZ/1000;
reg [31:0] sub_ms;
reg [7:0] age_ms;
reg timed_out,enable_d,online_d,stale_d;
wire calc_reset_n=rst_n && enable && online && !timed_out;
wire calc_valid,calc_busy,calc_rejected,calc_reference;
wire [287:0] calc_matrix;
wire crop_valid,crop_limited;wire [287:0] cropped_matrix;
wire [14:0] crop_scale_q14;
assign reference_ready=calc_reference;
assign ready=enable && online && calc_reference && !timed_out;
assign pose_stale=!online || timed_out;
assign pose_recovering=enable && !ready;
assign debug_age_ms=age_ms;
assign debug_filter_mode=crop_limited?2'd3:2'd0;
quaternion_pose_shared #(.DYNAMIC_REFERENCE(0),.JITTER_LIMIT(0),
 .AXIS_X(AXIS_X),.AXIS_Y(AXIS_Y),.AXIS_Z(AXIS_Z),
 .SIGN_X(SIGN_X),.SIGN_Y(SIGN_Y),.SIGN_Z(SIGN_Z)) projective_rotation(
 .clk(clk),.rst_n(calc_reset_n),.in_valid(report_valid),.recenter(1'b0),
 .suppress_camera_roll(1'b0),.qi($signed(current_q[15:0])),.qj($signed(current_q[31:16])),
 .qk($signed(current_q[47:32])),.qr($signed(current_q[63:48])),.dynamic_reference_q(64'd0),
 .busy(calc_busy),.out_valid(calc_valid),.reference_ready(calc_reference),.rejected(calc_rejected),.matrix(calc_matrix));
// Adaptive uniform crop is composed into the 3x3 inverse mapping before CDC.
perspective_autocrop #(.SMOOTH_ENABLE(1)) crop_to_valid_view(.clk(clk),.rst_n(calc_reset_n),.in_valid(calc_valid),
 .in_matrix(calc_matrix),.busy(),.out_valid(crop_valid),.out_matrix(cropped_matrix),
 .scale_q14(crop_scale_q14),.limited(crop_limited));
// Read-only roll diagnostics. They do not replace or constrain the 3D matrix.
wire [63:0] relative_q;wire relative_valid;
affine_relative_quaternion diagnostics_relative(.clk(clk),.rst_n(calc_reset_n),
 .in_valid(report_valid && debug_reference_valid),.current_q(current_q),.reference_q(debug_reference_q),
 .relative_q(relative_q),.out_valid(relative_valid),.ready());
quaternion_roll_direct #(.AXIS_X(AXIS_X),.AXIS_Y(AXIS_Y),.AXIS_Z(AXIS_Z),
 .SIGN_X(SIGN_X),.SIGN_Y(SIGN_Y),.SIGN_Z(SIGN_Z)) diagnostics_roll(
 .clk(clk),.rst_n(calc_reset_n),.in_valid(relative_valid),.relative_q(relative_q),
 .out_valid(debug_filtered_valid),.angle(debug_filtered_angle));
assign debug_angle_q12=debug_filtered_angle;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin sub_ms<=0;age_ms<=0;timed_out<=0;end
 else if(!enable || !online || report_valid)begin sub_ms<=0;age_ms<=0;timed_out<=0;end
 else if(sub_ms==MS_TICKS-1)begin
  sub_ms<=0;
  if(age_ms<255)age_ms<=age_ms+1'b1;
  if(age_ms>=STALE_MS-1)timed_out<=1;
 end else sub_ms<=sub_ms+1'b1;
always @(posedge clk or negedge calc_reset_n)
 if(!calc_reset_n)begin debug_reference_q<=0;debug_reference_valid<=0;end
 else if(report_valid && !debug_reference_valid)begin debug_reference_q<=current_q;debug_reference_valid<=1;end
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  out_valid<=0;matrix<=IDENTITY;out_affine_crop<=0;enable_d<=0;online_d<=0;stale_d<=1;
 end else begin
  out_valid<=0;enable_d<=enable;online_d<=online;stale_d<=pose_stale;
  if(!enable || !online || timed_out)begin
   matrix<=IDENTITY;out_affine_crop<=enable;
   if(enable!=enable_d || online!=online_d || pose_stale!=stale_d)out_valid<=1;
  end else if(enable!=enable_d || online!=online_d || pose_stale!=stale_d)begin
   matrix<=IDENTITY;out_affine_crop<=1;out_valid<=1;
  end else if(crop_valid)begin
   matrix<=cropped_matrix;out_affine_crop<=1;out_valid<=1;
  end
 end
endmodule
