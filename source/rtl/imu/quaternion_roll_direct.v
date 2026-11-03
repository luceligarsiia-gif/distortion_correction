// Direct roll extraction from a relative Q14 quaternion.
// Mapping to camera axes is followed by atan2(2*(x*y+w*z),
// 1-2*(y*y+z*z)). This removes the old quaternion-to-matrix step.
module quaternion_roll_direct #(
 parameter integer AXIS_X=0,AXIS_Y=1,AXIS_Z=2,
 parameter integer SIGN_X=1,SIGN_Y=-1,SIGN_Z=-1
)(input clk,rst_n,input in_valid,input [63:0] relative_q,
 output reg out_valid,output reg signed [31:0] angle);
reg busy,consume,atan_start,waiting;
reg [2:0] op;
reg signed [15:0] qx,qy,qz,qw;
reg signed [33:0] product;
reg signed [33:0] p_xy,p_wz,p_yy,p_zz;
wire signed [15:0] sx=$signed(relative_q[AXIS_X*16+:16]);
wire signed [15:0] sy=$signed(relative_q[AXIS_Y*16+:16]);
wire signed [15:0] sz=$signed(relative_q[AXIS_Z*16+:16]);
// Quaternion vector is an axial vector: include basis determinant so odd
// permutations/reflections also match B * R * transpose(B).
localparam integer PARITY=((AXIS_X==0 && AXIS_Y==1) ||
 (AXIS_X==1 && AXIS_Y==2) || (AXIS_X==2 && AXIS_Y==0))?1:-1;
localparam integer DET=PARITY*SIGN_X*SIGN_Y*SIGN_Z;
wire signed [15:0] cam_x=(DET*SIGN_X<0)?-sx:sx;
wire signed [15:0] cam_y=(DET*SIGN_Y<0)?-sy:sy;
wire signed [15:0] cam_z=(DET*SIGN_Z<0)?-sz:sz;
reg signed [15:0] mul_a,mul_b;
always @* begin
 case(op)
  0:begin mul_a=qx;mul_b=qy;end
  1:begin mul_a=qw;mul_b=qz;end
  2:begin mul_a=qy;mul_b=qy;end
  default:begin mul_a=qz;mul_b=qz;end
 endcase
end
wire signed [34:0] num_wide=({p_xy[33],p_xy}+{p_wz[33],p_wz})<<<1;
wire signed [34:0] den_wide=35'sd268435456-(({p_yy[33],p_yy}+{p_zz[33],p_zz})<<<1);
// Preserve the old projected-direction guard (1/64 in normalized units).
wire direction_valid=(num_wide>4194304 || num_wide< -4194304 ||
 den_wide>4194304 || den_wide< -4194304);
wire atan_done;wire signed [31:0] atan_angle;
affine_atan2 atan(.clk(clk),.rst_n(rst_n),.start(atan_start),
 .x_in(den_wide[31:0]),.y_in(num_wide[31:0]),.done(atan_done),.angle(atan_angle));
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  busy<=0;consume<=0;atan_start<=0;waiting<=0;op<=0;out_valid<=0;angle<=0;
  qx<=0;qy<=0;qz<=0;qw<=0;product<=0;p_xy<=0;p_wz<=0;p_yy<=0;p_zz<=0;
 end else begin
  out_valid<=0;atan_start<=0;
  if(!busy && !waiting && in_valid)begin
   qx<=cam_x;qy<=cam_y;qz<=cam_z;qw<=$signed(relative_q[48+:16]);
   busy<=1;consume<=0;op<=0;
  end else if(busy)begin
   if(op==4)begin
    busy<=0;
    if(direction_valid)begin atan_start<=1;waiting<=1;end
   end else if(!consume)begin product<=mul_a*mul_b;consume<=1;end
   else begin
    consume<=0;
    case(op)
     0:p_xy<=product;
     1:p_wz<=product;
     2:p_yy<=product;
     3:p_zz<=product;
    endcase
    op<=op+1'b1;
   end
  end
  if(atan_done)begin angle<=atan_angle;out_valid<=1;waiting<=0;end
 end
endmodule

// S5-only relative quaternion. Same conjugate(current) * reference ordering
// and Q14 rounding as quaternion_pose_shared, without building a 3x3 matrix.
// A registered 17x17 product is shared by norm checks and Hamilton products.
module affine_relative_quaternion(
 input clk,rst_n,in_valid,input [63:0] current_q,reference_q,
 output wire [63:0] relative_q,output reg out_valid,ready);
reg busy,consume;
reg [1:0] phase;
reg [4:0] op;
reg signed [15:0] cur[0:3],refq[0:3],rel[0:3];
reg signed [16:0] ma,mb;
reg signed [33:0] product;
reg signed [35:0] sum;
wire signed [35:0] next_sum=sum+{{2{product[33]}},product};
wire signed [35:0] rounded=(next_sum+36'sd8192)>>>14;
integer ai,bi,sgn,k;
always @* begin
 ai=0;bi=0;sgn=1;
 if(phase<2)begin ai=op;bi=op;end
 else case(op)
  0:begin ai=3;bi=0;end 1:begin ai=0;bi=3;sgn=-1;end
  2:begin ai=1;bi=2;sgn=-1;end 3:begin ai=2;bi=1;end
  4:begin ai=3;bi=1;end 5:begin ai=0;bi=2;end
  6:begin ai=1;bi=3;sgn=-1;end 7:begin ai=2;bi=0;sgn=-1;end
  8:begin ai=3;bi=2;end 9:begin ai=0;bi=1;sgn=-1;end
  10:begin ai=1;bi=0;end 11:begin ai=2;bi=3;sgn=-1;end
  12:begin ai=3;bi=3;end 13:begin ai=0;bi=0;end
  14:begin ai=1;bi=1;end default:begin ai=2;bi=2;end
 endcase
 ma=$signed(cur[ai]);mb=$signed(cur[bi]);
 if(phase==1)begin ma=$signed(refq[ai]);mb=$signed(refq[bi]);end
 else if(phase==2)begin
  ma=$signed(cur[ai]);if(sgn<0)ma=-ma;
  mb=$signed(refq[bi]);
 end
end
assign relative_q={rel[3],rel[2],rel[1],rel[0]};
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  busy<=0;consume<=0;phase<=0;op<=0;sum<=0;product<=0;out_valid<=0;ready<=0;
  for(k=0;k<4;k=k+1)begin cur[k]<=0;refq[k]<=0;rel[k]<=0;end
 end else begin
  out_valid<=0;
  if(!busy && in_valid)begin
   for(k=0;k<4;k=k+1)begin cur[k]<=current_q[k*16+:16];refq[k]<=reference_q[k*16+:16];end
   busy<=1;phase<=0;op<=0;sum<=0;consume<=0;
  end else if(busy)begin
   if(!consume)begin product<=ma*mb;consume<=1;end
   else begin
    consume<=0;
    if(phase<2)begin
     sum<=next_sum;
     if(op==3)begin
      if(next_sum<36'sd265751101 || next_sum>36'sd271119811)busy<=0;
      else begin phase<=phase+1'b1;op<=0;sum<=0;end
     end else op<=op+1'b1;
    end else begin
     if(op[1:0]==3)begin
      rel[op[3:2]]<=rounded[15:0];sum<=0;
      if(op==15)begin busy<=0;out_valid<=1;ready<=1;end
      else op<=op+1'b1;
     end else begin sum<=next_sum;op<=op+1'b1;end
    end
   end
  end
 end
endmodule
