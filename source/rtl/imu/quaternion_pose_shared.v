// BNO080 world-from-sensor Q14. Output camera-current-from-camera-reference.
// Exactly ONE signed 17x17 '*' operator, scheduled across all quaternion products.
module quaternion_pose_shared #(
 parameter integer DYNAMIC_REFERENCE=0, JITTER_LIMIT=0,
 parameter integer AXIS_X=0,AXIS_Y=1,AXIS_Z=2,
 parameter integer SIGN_X=1,SIGN_Y=1,SIGN_Z=1
)(input clk,rst_n,in_valid,recenter,suppress_camera_roll,
 input signed [15:0] qi,qj,qk,qr,
 input [63:0] dynamic_reference_q,
 output reg busy,out_valid,reference_ready,rejected,
 output wire [287:0] matrix);
reg [287:0] matrix_sensor;
reg signed [15:0] cur[0:3],refq[0:3],rel[0:3];
reg signed [31:0] terms[0:8],rot[0:8];
reg [2:0] phase;
reg [4:0] op;
reg consume;
reg suppress_roll_held;
reg signed [35:0] sum;
reg signed [33:0] product;
reg signed [16:0] ma,mb;
integer ai,bi,sgn,k;
reg signed [35:0] next_sum;
wire signed [35:0] relative_value=(next_sum+36'sd8192)>>>14;
function signed [15:0] limited_component;
 input signed [35:0] value;
 begin
  if(JITTER_LIMIT!=0 && value>JITTER_LIMIT) limited_component=JITTER_LIMIT;
  else if(JITTER_LIMIT!=0 && value< -JITTER_LIMIT) limited_component=-JITTER_LIMIT;
  else limited_component=value[15:0];
 end
endfunction
function integer axis(input integer n);
begin case(n) 0:axis=AXIS_X;1:axis=AXIS_Y;default:axis=AXIS_Z;endcase end
endfunction
function integer sign_axis(input integer n);
begin case(n) 0:sign_axis=SIGN_X;1:sign_axis=SIGN_Y;default:sign_axis=SIGN_Z;endcase end
endfunction
always @* begin
 ai=0;bi=0;sgn=1;
 if(phase==0 || phase==5) begin ai=op;bi=op;end
 else if(phase==1) begin
  case(op)
   0:begin ai=3;bi=0;end 1:begin ai=0;bi=3;sgn=-1;end
   2:begin ai=1;bi=2;sgn=-1;end 3:begin ai=2;bi=1;end
   4:begin ai=3;bi=1;end 5:begin ai=0;bi=2;end
   6:begin ai=1;bi=3;sgn=-1;end 7:begin ai=2;bi=0;sgn=-1;end
   8:begin ai=3;bi=2;end 9:begin ai=0;bi=1;sgn=-1;end
   10:begin ai=1;bi=0;end 11:begin ai=2;bi=3;sgn=-1;end
   12:begin ai=3;bi=3;end 13:begin ai=0;bi=0;end
   14:begin ai=1;bi=1;end default:begin ai=2;bi=2;end
  endcase
 end else begin
  case(op)
   0:begin ai=0;bi=0;end 1:begin ai=1;bi=1;end 2:begin ai=2;bi=2;end
   3:begin ai=0;bi=1;end 4:begin ai=0;bi=2;end 5:begin ai=1;bi=2;end
   6:begin ai=3;bi=0;end 7:begin ai=3;bi=1;end default:begin ai=3;bi=2;end
  endcase
 end
 ma=$signed(cur[ai]);mb=$signed(cur[bi]);
 if(phase==1) begin ma=(sgn<0)?-$signed(cur[ai]):$signed(cur[ai]);mb=$signed(refq[bi]);end
 else if(phase==2 || phase==5) begin ma=$signed(rel[ai]);mb=$signed(rel[bi]);end
 next_sum=sum+{{2{product[33]}},product};
end
always @(posedge clk or negedge rst_n) begin
 if(!rst_n) begin
  busy<=0;out_valid<=0;reference_ready<=0;rejected<=0;phase<=0;op<=0;consume<=0;sum<=0;product<=0;suppress_roll_held<=0;
  matrix_sensor<=0;
  for(k=0;k<4;k=k+1) begin cur[k]<=0;refq[k]<=0;rel[k]<=0;end
  for(k=0;k<9;k=k+1) begin terms[k]<=0;rot[k]<=0;end
 end else begin
  out_valid<=0;rejected<=0;
  if(recenter) begin reference_ready<=0;busy<=0;consume<=0;end
  else if(!busy && in_valid) begin
   cur[0]<=qi;cur[1]<=qj;cur[2]<=qk;cur[3]<=qr;
   suppress_roll_held<=suppress_camera_roll;
   if(DYNAMIC_REFERENCE!=0)
    for(k=0;k<4;k=k+1)refq[k]<=$signed(dynamic_reference_q[k*16+:16]);
   busy<=1;phase<=0;op<=0;consume<=0;sum<=0;
  end else if(busy) begin
   if(phase<=2 || phase==5) begin
    if(!consume) begin product<=ma*mb;consume<=1;end
    else begin
     consume<=0;
     if(phase==0) begin
      sum<=next_sum;
      if(op==3) begin
       // Unit quaternion norm tolerance +/-1%; reject zero/invalid sensor reports.
       if(next_sum<36'sd265751101 || next_sum>36'sd271119811) begin busy<=0;rejected<=1;end
       else if(DYNAMIC_REFERENCE!=0)begin reference_ready<=1;phase<=1;op<=0;sum<=0;end
       else if(!reference_ready) begin
        for(k=0;k<4;k=k+1) refq[k]<=cur[k];
        reference_ready<=1;matrix_sensor<=0;
        matrix_sensor[0*32+:32]<=32'sd16384;matrix_sensor[4*32+:32]<=32'sd16384;matrix_sensor[8*32+:32]<=32'sd16384;
        out_valid<=1;busy<=0;
       end else begin phase<=1;op<=0;sum<=0;end
      end else op<=op+1'b1;
     end else if(phase==1) begin
      if(op[1:0]==3) begin
       if(op==15)rel[3]<=relative_value[15:0];
       // S5 planar control already compensates camera-Z rotation. Preserve
       // bounded pitch/yaw residual without adding a second lagging roll.
       else if(JITTER_LIMIT!=0 && suppress_roll_held && op[3:2]==AXIS_Z)rel[op[3:2]]<=0;
       else rel[op[3:2]]<=limited_component(relative_value);
       sum<=0;
       if(op==15) begin phase<=(JITTER_LIMIT!=0)?5:2;op<=0;end else op<=op+1'b1;
      end else begin sum<=next_sum;op<=op+1'b1;end
     end else if(phase==5)begin
      sum<=next_sum;
      if(op==2)begin
       // Small-angle unit real component: sqrt(1-|v|^2) ~= 1-|v|^2/2.
       // With limit 256, total angle <=3.11 degrees; neglected term <1 Q14 LSB.
       rel[3]<=16'sd16384-((next_sum+36'sd16384)>>>15);
       phase<=2;op<=0;sum<=0;
      end else op<=op+1'b1;
     end else begin
      terms[op]<=product;
      if(op==8) phase<=3;else op<=op+1'b1;
     end
    end
   end else if(phase==3) begin
    rot[0]<=16384-((terms[1]+terms[2]+4096)>>>13);
    rot[1]<=(terms[3]-terms[8]+4096)>>>13;
    rot[2]<=(terms[4]+terms[7]+4096)>>>13;
    rot[3]<=(terms[3]+terms[8]+4096)>>>13;
    rot[4]<=16384-((terms[0]+terms[2]+4096)>>>13);
    rot[5]<=(terms[5]-terms[6]+4096)>>>13;
    rot[6]<=(terms[4]-terms[7]+4096)>>>13;
    rot[7]<=(terms[5]+terms[6]+4096)>>>13;
    rot[8]<=16384-((terms[0]+terms[1]+4096)>>>13);
    phase<=4;
   end else begin
    for(k=0;k<9;k=k+1)
     matrix_sensor[k*32+:32]<=rot[k];
    out_valid<=1;busy<=0;
   end
  end
 end
end
genvar cell_index;
generate for(cell_index=0;cell_index<9;cell_index=cell_index+1) begin: camera_basis
 localparam integer ROW=cell_index/3,COL=cell_index%3;
 localparam integer RI=(ROW==0)?AXIS_X:((ROW==1)?AXIS_Y:AXIS_Z);
 localparam integer CI=(COL==0)?AXIS_X:((COL==1)?AXIS_Y:AXIS_Z);
 localparam integer RS=(ROW==0)?SIGN_X:((ROW==1)?SIGN_Y:SIGN_Z);
 localparam integer CS=(COL==0)?SIGN_X:((COL==1)?SIGN_Y:SIGN_Z);
 wire [31:0] coefficient=matrix_sensor[(RI*3+CI)*32+:32];
 wire invert_sign=RS[31]^CS[31];
 assign matrix[cell_index*32+:32]=invert_sign?((~coefficient)+32'd1):coefficient;
end endgenerate
endmodule
