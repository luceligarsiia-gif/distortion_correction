// Planar angle Q12 degrees, retained through filtering and sine/cosine.
// Extend the Q14 matrix vector before vectoring to preserve sub-pixel angles.
module affine_atan2(input clk,rst_n,start,input signed [31:0] x_in,y_in,
 output reg done,output reg signed [31:0] angle);
reg busy;reg [4:0] step;
reg signed [47:0] x,y,xx,yy;
reg signed [31:0] z,zz;
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
wire signed [47:0] wide_x=$signed({{16{x_in[31]}},x_in})<<<14;
wire signed [47:0] wide_y=$signed({{16{y_in[31]}},y_in})<<<14;
always @* begin
 xx=x;yy=y;zz=z;
 if(y>0)begin xx=x+(y>>>step);yy=y-(x>>>step);zz=z+atan_q12(step);end
 else if(y<0)begin xx=x-(y>>>step);yy=y+(x>>>step);zz=z-atan_q12(step);end
end
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin busy<=0;done<=0;step<=0;x<=0;y<=0;z<=0;angle<=0;end
 else begin
  done<=0;
  if(start && !busy)begin
   busy<=1;step<=0;
   if(x_in<0)begin x<=-wide_x;y<=-wide_y;z<=(y_in>=0)?737280:-737280;end
   else begin x<=wide_x;y<=wide_y;z<=0;end
  end else if(busy)begin
   x<=xx;y<=yy;z<=zz;
   if(step==19)begin
    if(zz>737280) angle<=zz-1474560;
    else if(zz< -737280) angle<=zz+1474560;
    else angle<=zz;
    busy<=0;done<=1;
   end else step<=step+1'b1;
  end
 end
endmodule
