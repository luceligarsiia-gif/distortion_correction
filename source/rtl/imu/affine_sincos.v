// Q12-degree input, Q28 internal vector, Q14 coefficients. II=21 clocks.
module affine_sincos(input clk,rst_n,start,input signed [31:0] angle,
 output reg done,output reg signed [31:0] cosine,sine);
reg busy;reg [4:0] step;
reg signed [31:0] x,y,z;
reg signed [31:0] xx,yy,zz;
function signed [31:0] atan_q12;
 input [4:0] n;
 begin case(n)
 0:atan_q12=184320;1:atan_q12=108810;2:atan_q12=57492;3:atan_q12=29184;
 4:atan_q12=14649;5:atan_q12=7331;6:atan_q12=3667;7:atan_q12=1833;
 8:atan_q12=917;9:atan_q12=458;10:atan_q12=229;11:atan_q12=115;
 12:atan_q12=57;13:atan_q12=29;14:atan_q12=14;15:atan_q12=7;
 16:atan_q12=4;17:atan_q12=2;18:atan_q12=1;default:atan_q12=0;
 endcase end
endfunction
always @* begin
 if(z>=0) begin xx=x-(y>>>step);yy=y+(x>>>step);zz=z-atan_q12(step);end
 else begin xx=x+(y>>>step);yy=y-(x>>>step);zz=z+atan_q12(step);end
end
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin busy<=0;done<=0;step<=0;x<=0;y<=0;z<=0;cosine<=16384;sine<=0;end
 else begin
  done<=0;
  if(start && !busy) begin
   busy<=1;step<=0;y<=0;
   if(angle>368640) begin x<=-163008219;z<=angle-737280;end
   else if(angle< -368640) begin x<=-163008219;z<=angle+737280;end
   else begin x<=163008219;z<=angle;end
  end else if(busy) begin
   x<=xx;y<=yy;z<=zz;
   if(step==19) begin cosine<=(xx+8192)>>>14;sine<=(yy+8192)>>>14;busy<=0;done<=1;end
   else step<=step+1'b1;
  end
 end
endmodule
