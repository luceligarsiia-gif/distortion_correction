module pose_distortion_map #(parameter EXTERNAL_RECIPROCAL=0)(input clk,rst_n,in_valid,input [15:0] in_u,in_v,
 input [287:0] matrix,input affine_crop,output out_valid,output signed [31:0] out_u,out_v,
 output wire [12:0] reciprocal_address,input wire [17:0] reciprocal_data);
wire ray_valid,bad;wire signed [31:0] nx,ny,uq,vq;
pose_ray_map #(.EXTERNAL_RECIPROCAL(EXTERNAL_RECIPROCAL)) ray(
 .clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_u(in_u),.in_v(in_v),
 .matrix(matrix),.affine_crop(affine_crop),.out_valid(ray_valid),.ray_invalid(bad),
 .nx_q28(nx),.ny_q28(ny),.reciprocal_address(reciprocal_address),.reciprocal_data(reciprocal_data));
reg [30:0] invalid_pipe;
always @(posedge clk or negedge rst_n)
 if(!rst_n) invalid_pipe<=0;else invalid_pipe<={invalid_pipe[29:0],bad};
distortion_map #(.NORMALIZED_INPUT(1)) lens(.clk(clk),.rst_n(rst_n),.ce(1'b1),
 .in_valid(ray_valid),.in_u(16'd0),.in_v(16'd0),.normalized_x(nx),.normalized_y(ny),
 .out_valid(out_valid),.out_u(uq),.out_v(vq),.center_outside());
assign out_u=invalid_pipe[30]?-32'sd2048:uq;
assign out_v=invalid_pipe[30]?-32'sd2048:vq;
endmodule
