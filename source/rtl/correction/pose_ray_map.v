// Output-pixel -> rotated camera ray -> normalized source pinhole coordinate.
// Q16 internal rays, Q14 rotation. Reciprocal LUT error < 1/16384 relative.
// Distortion follows this module; image pixels are interpolated only once.
module pose_ray_map #(parameter EXTERNAL_RECIPROCAL=0)(input clk,rst_n,in_valid,input [15:0] in_u,in_v,
 input [287:0] matrix,input affine_crop,
 output reg out_valid,ray_invalid,output reg signed [31:0] nx_q28,ny_q28,
 output wire [12:0] reciprocal_address,input wire [17:0] reciprocal_data);
localparam signed [31:0] CX=3926176,CY=2194230;
// All modes use the original 0.8 base rays. Fixed/adaptive crop is composed
// into the per-frame matrix. affine_crop remains a mode/history/diagnostic tag.
reg signed [31:0] invfx,invfy;
reg dv;reg signed [31:0] dx,dy;
wire nv;wire signed [31:0] nx,ny;
pose_mul32 #(.SHIFT(24)) normx(clk,rst_n,dv,dx,invfx,nv,nx);
pose_mul32 #(.SHIFT(24)) normy(clk,rst_n,dv,dy,invfy,,ny);
wire rv;wire signed [31:0] rx0,rx1,ry0,ry1,rz0,rz1;
pose_mul32 mx0(clk,rst_n,nv,nx,$signed(matrix[0*32+:32]),rv,rx0);
pose_mul32 mx1(clk,rst_n,nv,ny,$signed(matrix[1*32+:32]),,rx1);
pose_mul32 my0(clk,rst_n,nv,nx,$signed(matrix[3*32+:32]),,ry0);
pose_mul32 my1(clk,rst_n,nv,ny,$signed(matrix[4*32+:32]),,ry1);
pose_mul32 mz0(clk,rst_n,nv,nx,$signed(matrix[6*32+:32]),,rz0);
pose_mul32 mz1(clk,rst_n,nv,ny,$signed(matrix[7*32+:32]),,rz1);
reg rayv,normv,romv;
reg signed [31:0] rx,ry,rz,rxn,ryn,rxr,ryr;
reg [31:0] zn;
reg signed [5:0] exponent,rom_exp,exp_pipe[0:3];
reg bad_norm,bad_rom,bad_pipe[0:3];
integer msb,i,j;
always @* begin
 msb=0;
 for(j=0;j<31;j=j+1) if(rz[j]) msb=j;
end
wire [17:0] recip;
assign reciprocal_address=zn[29:17];
generate if(EXTERNAL_RECIPROCAL)begin: shared_lookup
 assign recip=reciprocal_data;
end else begin: standalone_lookup
 // Retain a self-contained default for standalone module/test reuse.
 reg [17:0] reciprocal[0:8191];
 reg [17:0] value;
 initial $readmemh("source/rtl/correction/reciprocal_q18.mem",reciprocal);
 always @(posedge clk or negedge rst_n)
  if(!rst_n)value<=0;else value<=reciprocal[reciprocal_address];
 assign recip=value;
end endgenerate
wire pv;wire signed [31:0] px,py;
pose_mul32 #(.SHIFT(18)) projectx(clk,rst_n,romv,rxr,$signed({14'd0,recip}),pv,px);
pose_mul32 #(.SHIFT(18)) projecty(clk,rst_n,romv,ryr,$signed({14'd0,recip}),,py);
wire signed [31:0] sx=(exp_pipe[3]<0)?(px<<<(-exp_pipe[3])):(px>>>exp_pipe[3]);
wire signed [31:0] sy=(exp_pipe[3]<0)?(py<<<(-exp_pipe[3])):(py>>>exp_pipe[3]);
// The crop controller and pixel mapper share the calibrated normal rectangle.
// Keep the same original safe domain for the fixed Brown polynomial.
wire outside_normal=sx< -79037 || sx>84067 || sy< -44346 || sy>47239;
wire invalid=bad_pipe[3] || outside_normal;
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  dv<=0;dx<=0;dy<=0;invfx<=278634;invfy<=278561;rayv<=0;normv<=0;romv<=0;rx<=0;ry<=0;rz<=0;
  rxn<=0;ryn<=0;rxr<=0;ryr<=0;zn<=0;exponent<=0;rom_exp<=0;
  bad_norm<=1;bad_rom<=1;out_valid<=0;ray_invalid<=1;nx_q28<=0;ny_q28<=0;
  for(i=0;i<4;i=i+1) begin exp_pipe[i]<=0;bad_pipe[i]<=1;end
 end else begin
  dv<=in_valid;dx<=$signed({4'd0,in_u,12'd0})-CX;dy<=$signed({4'd0,in_v,12'd0})-CY;
  invfx<=32'sd278634;
  invfy<=32'sd278561;
  rayv<=rv;rx<=rx0+rx1+($signed(matrix[2*32+:32])<<<2);
  ry<=ry0+ry1+($signed(matrix[5*32+:32])<<<2);
  rz<=rz0+rz1+($signed(matrix[8*32+:32])<<<2);
  normv<=rayv;rxn<=rx;ryn<=ry;zn<=rz<<(30-msb);exponent<=msb-16;
  bad_norm<=rz<8192 || rz>262144;
  romv<=normv;rxr<=rxn;ryr<=ryn;rom_exp<=exponent;bad_rom<=bad_norm;
  exp_pipe[0]<=rom_exp;bad_pipe[0]<=bad_rom;
  for(i=1;i<4;i=i+1) begin exp_pipe[i]<=exp_pipe[i-1];bad_pipe[i]<=bad_pipe[i-1];end
  out_valid<=pv;ray_invalid<=invalid;
  nx_q28<=invalid?0:(sx<<<12);ny_q28<=invalid?0:(sy<<<12);
 end
endmodule
