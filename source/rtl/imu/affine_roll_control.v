// Fixed-reference planar rotation from the filtered camera quaternion.
// Keep the existing circular angle filter after quaternion smoothing.
// COMPOSE_RESIDUAL=0: S5 planar rotation only, with a fixed image center.
// COMPOSE_RESIDUAL=1: inherited small residual composition M_residual * A.
// Normal mode always passes the original residual through unchanged.
module affine_roll_control #(
 parameter AXIS_X=0,AXIS_Y=1,AXIS_Z=2,SIGN_X=1,SIGN_Y=-1,SIGN_Z=-1,
 parameter integer COMPOSE_RESIDUAL=1, CLK_HZ=27000000
)(input clk,rst_n,enable,online,filter_valid,input [63:0] filtered_q,current_q,
 input residual_valid,input [287:0] residual_matrix,
 output reg out_valid,output reg [287:0] matrix,output reg out_affine_crop,
 output reference_ready,output reg ready,
 output signed [31:0] debug_angle_q12,output [1:0] debug_filter_mode,
 output [7:0] debug_age_ms,output pose_stale,pose_recovering,
 output [63:0] debug_reference_q,output debug_reference_valid,
 output signed [31:0] debug_filtered_angle,output debug_filtered_valid);
localparam [287:0] IDENTITY={32'd16384,32'd0,32'd0,32'd0,32'd16384,32'd0,32'd0,32'd0,32'd16384};
// Link loss resets the reference; a connected but stale pose uses the watchdog.
wire affine_reset_n=rst_n && enable && online;
reg have_reference;
reg [63:0] frozen_reference;
always @(posedge clk or negedge affine_reset_n)
 if(!affine_reset_n)begin have_reference<=0;frozen_reference<=0;end
 else if(filter_valid && online && !have_reference)begin have_reference<=1;frozen_reference<=filtered_q;end
wire [63:0] relative_q;wire relative_valid;
affine_relative_quaternion reference_pose(
 .clk(clk),.rst_n(affine_reset_n),.in_valid(filter_valid && online && have_reference),
 .current_q(filtered_q),.reference_q(frozen_reference),
 .relative_q(relative_q),.out_valid(relative_valid),.ready(reference_ready));
wire angle_done;wire signed [31:0] angle;
// Read-only diagnostic taps; no feedback into the correction datapath.
assign debug_reference_q=frozen_reference;
assign debug_reference_valid=have_reference;
assign debug_filtered_angle=angle;
assign debug_filtered_valid=angle_done;
quaternion_roll_direct #(.AXIS_X(AXIS_X),.AXIS_Y(AXIS_Y),.AXIS_Z(AXIS_Z),
 .SIGN_X(SIGN_X),.SIGN_Y(SIGN_Y),.SIGN_Z(SIGN_Z)) direct_roll(
 .clk(clk),.rst_n(affine_reset_n),.in_valid(relative_valid),
 .relative_q(relative_q),.out_valid(angle_done),.angle(angle));
wire smooth_valid;wire signed [31:0] smooth_angle;
affine_angle_filter #(.CLK_HZ(CLK_HZ)) smooth(
 .clk(clk),.rst_n(affine_reset_n),.in_valid(angle_done && online),.in_angle(angle),
 .out_valid(smooth_valid),.out_angle(smooth_angle),.mode(debug_filter_mode),
 .age_ms(debug_age_ms),.stale(pose_stale),.recovering(pose_recovering));
assign debug_angle_q12=smooth_angle;
wire coeff_done;wire signed [31:0] cosine,sine;
affine_sincos trig(clk,affine_reset_n,smooth_valid,smooth_angle,coeff_done,cosine,sine);
reg [287:0] residual_latest,held,assembled;
reg signed [31:0] c,s;
reg pending,working,enable_d,online_d,stale_d;
reg [3:0] cell_id;
reg half,consume;
reg signed [16:0] ma,mb;
reg signed [33:0] product,first;
wire signed [34:0] sum={{1{first[33]}},first}+{{1{product[33]}},product}+35'sd8192;
integer row,col;
always @* begin
 row=cell_id/3;col=cell_id%3;
 ma=$signed(held[(row*3+(half?1:0))*32+:32]);
 if(col==0) mb=half?s:c;
 else mb=half?c:-s;
end
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  out_valid<=0;matrix<=IDENTITY;out_affine_crop<=0;ready<=0;residual_latest<=IDENTITY;
  held<=IDENTITY;assembled<=IDENTITY;c<=16384;s<=0;
  pending<=0;working<=0;enable_d<=0;online_d<=0;stale_d<=1;cell_id<=0;half<=0;consume<=0;product<=0;first<=0;
 end else begin
  out_valid<=0;enable_d<=enable;online_d<=online;stale_d<=pose_stale;
  if(residual_valid)begin
   residual_latest<=residual_matrix;
   if(!enable || !ready)pending<=1;
  end
  if(coeff_done)begin ready<=1;pending<=1;end
  if(!online) begin
   matrix<=IDENTITY;ready<=0;working<=0;pending<=0;residual_latest<=IDENTITY;
   if(online_d || enable!=enable_d) begin out_valid<=1;out_affine_crop<=enable;end
  end else if(COMPOSE_RESIDUAL==0 && enable)begin
   // Do not let the slowly changing pitch/yaw residual translate the picture
   // while the user is asking S5 to compensate planar roll. The center ray
   // remains (0,0,1), independently of those residual inputs.
   working<=0;pending<=0;
   if(coeff_done)begin
    matrix<={32'd16384,32'd0,32'd0,32'd0,cosine,sine,32'd0,-sine,cosine};
    out_valid<=1;out_affine_crop<=1;ready<=!pose_stale;
   end else if(enable!=enable_d)begin
    matrix<=IDENTITY;out_valid<=1;out_affine_crop<=1;ready<=0;
   end else if(pose_stale!=stale_d)begin
    // Publish health transitions atomically with the held pose to flush history.
    out_valid<=1;out_affine_crop<=1;ready<=!pose_stale;
   end
  end else if(!enable || !ready) begin
   working<=0;
   if(!enable) ready<=0;
   if(residual_valid || enable!=enable_d) begin
    matrix<=residual_valid?residual_matrix:residual_latest;out_valid<=1;out_affine_crop<=enable;
   end
  end else if(!working && pending) begin
   held<=residual_latest;assembled<=residual_latest;c<=cosine;s<=sine;
   working<=1;cell_id<=0;half<=0;consume<=0;pending<=coeff_done;
  end else if(working) begin
   if(cell_id%3==2) begin
    if(cell_id==8)begin matrix<=assembled;out_valid<=1;out_affine_crop<=1;working<=0;end
    else cell_id<=cell_id+1'b1;
   end else if(!consume)begin product<=ma*mb;consume<=1;end
   else begin
    consume<=0;
    if(!half)begin first<=product;half<=1;end
    else begin assembled[cell_id*32+:32]<=sum>>>14;half<=0;cell_id<=cell_id+1'b1;end
   end
  end
 end
endmodule
