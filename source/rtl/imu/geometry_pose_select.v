// Atomic pose publication: affine uses one constant, frame-locked fixed crop.
// Perspective retains its existing adaptive crop. No affine zoom controller.
// Perspective retains the upstream controller's own crop matrix unchanged.
module geometry_pose_select(input clk,rst_n,input [1:0] mode,
 input affine_valid,input [287:0] affine_matrix,input affine_crop,affine_ready,
 input perspective_valid,input [287:0] perspective_matrix,input perspective_ready,
 output reg out_valid,output reg [287:0] out_matrix,output reg out_crop,out_active,
 input fixed_crop);
 localparam [287:0] IDENTITY={32'd16384,32'd0,32'd0,32'd0,32'd16384,32'd0,32'd0,32'd0,32'd16384};
 // Original fixed mode used 9830/16384 (~0.6). Use 9011/16384 (~0.55)
 // for all-angle coverage of the existing calibrated rectangle AND 64-row
 // tile cache. The factor never depends on pose; added zoom is ~1.818x.
 localparam signed [31:0] AFFINE_FIXED_SCALE_Q14=32'sd9011;
 wire [287:0] fixed_matrix;
 function signed [31:0] scale_fixed;
  input signed [31:0] v;
  reg signed [63:0] w,product;
  begin
   w={{32{v[31]}},v};
   // 9011 = 8192+512+256+64-16+4-1; no DSP multiplier.
   product=(w<<<13)+(w<<<9)+(w<<<8)+(w<<<6)-(w<<<4)+(w<<<2)-w;
   scale_fixed=(product+64'sd8192)>>>14;
  end
 endfunction
 genvar k;
 generate for(k=0;k<9;k=k+1)begin: fixed_columns
  if(k%3==2)assign fixed_matrix[k*32+:32]=affine_matrix[k*32+:32];
  else assign fixed_matrix[k*32+:32]=scale_fixed($signed(affine_matrix[k*32+:32]));
 end endgenerate
 reg [1:0] mode_d;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  mode_d<=3;out_valid<=0;out_matrix<=IDENTITY;out_crop<=0;out_active<=0;
 end else begin
  mode_d<=mode;out_valid<=0;
  if(mode!=mode_d)begin
   out_matrix<=IDENTITY;out_crop<=mode==1;out_active<=0;out_valid<=1;
  end else if(mode==1)begin
   if(!affine_ready)begin
    if(affine_valid || out_active)begin out_valid<=1;out_active<=0;out_crop<=1;end
   end else if(affine_valid && affine_crop)begin
    // Compose the constant scale before the existing CDC/frame history.
    out_matrix<=fixed_matrix;out_crop<=1;out_active<=1;out_valid<=1;
   end
  end else if(mode==2 && perspective_valid)begin
   out_matrix<=perspective_matrix;out_crop<=0;out_active<=perspective_ready;out_valid<=1;
  end
 end
endmodule
