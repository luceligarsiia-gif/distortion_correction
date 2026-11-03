// 4x4 local, contrast-limited histogram equalization on luminance. The tile
// maps lag capture by one frame; pixels and syncs retain a fixed six-clock
// latency regardless of enable state. RGB changes by the same luminance delta.
module clahe_rgb(input clk,rst_n,requested_enabled,
 input [7:0] in_r,in_g,in_b,input in_vs,in_hs,in_de,
 output reg [7:0] out_r,out_g,out_b,output reg out_vs,out_hs,out_de,
 output reg enabled,output wire ready,output wire glare_present);
 reg enable_meta,enable_sync,vs_d,de_d;
 wire frame_boundary=vs_d && !in_vs;
 reg [11:0] x,y;
 wire [15:0] luma_sum=({8'd0,in_r}*16'd77)+({8'd0,in_g}*16'd150)+({8'd0,in_b}*16'd29);
 wire [7:0] luma=luma_sum[15:8];
 reg [7:0] lookup_luma_q;
 wire [1:0] sample_col=x<480?0:x<960?1:x<1440?2:3;
 wire [1:0] sample_row=y<270?0:y<540?1:y<810?2:3;
 wire sampling=in_de && x[3:0]==4'd8 && x<1920 && y<1080;
 reg sample_q;
 reg [1:0] sample_col_q,sample_row_q;
 wire [1:0] left_col=x<720?0:x<1200?1:x<1680?2:3;
 wire [1:0] right_col=x<240?0:x>=1680?3:left_col+1'b1;
 wire [1:0] top_row=y<405?0:y<675?1:y<945?2:3;
 wire [1:0] bottom_row=y<135?0:y>=945?3:top_row+1'b1;
 wire [11:0] x_relative=x<240 || x>=1680?0:x<720?x-12'd240:x<1200?x-12'd720:x-12'd1200;
 wire [11:0] y_relative=y<135 || y>=945?0:y<405?y-12'd135:y<675?y-12'd405:y-12'd675;
 wire [13:0] x_product=x_relative*5'd17;
 wire [12:0] y_product=y_relative*5'd30;
 wire [4:0] fraction_x=x_product[12:8];
 wire [4:0] fraction_y=y_product[12:8];
 wire [7:0] tile_map[0:15];
 wire [15:0] tile_valid;
 wire [15:0] tile_glare;
 assign glare_present=|tile_glare;
 reg [15:0] tile_sample_q;
 reg [7:0] sample_luma_q;
 genvar n;
 generate for(n=0;n<16;n=n+1)begin: tiles
  always @(posedge clk or negedge rst_n)
   if(!rst_n)tile_sample_q[n]<=0;
   else tile_sample_q[n]<=sample_q && sample_col_q==n%4 && sample_row_q==n/4;
  clahe_tile tile(.clk(clk),.rst_n(rst_n),.frame_boundary(frame_boundary),
   .sample(tile_sample_q[n]),
   .sample_luma(sample_luma_q),.lookup_luma(lookup_luma_q),
   .mapped(tile_map[n]),.valid(tile_valid[n]),.glare(tile_glare[n]));
 end endgenerate
 assign ready=&tile_valid;
function [12:0] times5;
 input [7:0] a; input [4:0] b;
 reg [12:0] p0,p1,p2,p3,p4;
 begin
  p0=b[0]?{5'd0,a}:13'd0;p1=b[1]?{4'd0,a,1'b0}:13'd0;
  p2=b[2]?{3'd0,a,2'd0}:13'd0;p3=b[3]?{2'd0,a,3'd0}:13'd0;
  p4=b[4]?{1'd0,a,4'd0}:13'd0;times5=(p0+p1)+(p2+p3)+p4;
 end
endfunction
function [7:0] blend;
 input [7:0] a,b;input [4:0] fraction;
 reg [13:0] value;
 begin value=({1'b0,a,5'd0}-{1'b0,times5(a,fraction)})+
              {1'b0,times5(b,fraction)}+14'd16;
  blend=value[12:5];
 end
endfunction
 function [7:0] add_delta;
  input [7:0] channel;input signed [9:0] delta;
  reg signed [10:0] sum;
  begin sum=$signed({3'b0,channel})+delta;
   add_delta=sum<0?0:sum>255?255:sum[7:0];end
 endfunction
 reg [23:0] rgb1,rgb1_d,rgb2,rgb3,rgb4;
 reg [7:0] y1,y1_d,y2,y3,y4;
 reg [2:0] ctl1,ctl1_d,ctl2,ctl3,ctl4;
 reg [1:0] lc1,rc1,tr1,br1,lc1_d,rc1_d,tr1_d,br1_d;
 reg [4:0] fx1,fy1,fx1_d,fy1_d,fx2,fy2,fy3;
 reg en1,en1_d,en2,en3,en4;
 reg [7:0] tl2,tr2,bl2,br2 /* synthesis syn_preserve=1 syn_keep=1 */;
 reg [7:0] top3,bottom3,map4;
 wire signed [9:0] delta=$signed({2'b0,map4})-$signed({2'b0,y4});
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  enable_meta<=0;enable_sync<=0;enabled<=0;vs_d<=1;de_d<=0;x<=0;y<=0;
  lookup_luma_q<=0;sample_q<=0;sample_col_q<=0;sample_row_q<=0;sample_luma_q<=0;
  rgb1<=0;rgb1_d<=0;rgb2<=0;rgb3<=0;rgb4<=0;
  y1<=0;y1_d<=0;y2<=0;y3<=0;y4<=0;
  ctl1<=0;ctl1_d<=0;ctl2<=0;ctl3<=0;ctl4<=0;
  lc1<=0;rc1<=0;tr1<=0;br1<=0;
  lc1_d<=0;rc1_d<=0;tr1_d<=0;br1_d<=0;
  fx1<=0;fy1<=0;fx1_d<=0;fy1_d<=0;fx2<=0;fy2<=0;fy3<=0;
  en1<=0;en1_d<=0;en2<=0;en3<=0;en4<=0;
  tl2<=0;tr2<=0;bl2<=0;br2<=0;top3<=0;bottom3<=0;map4<=0;
  out_r<=0;out_g<=0;out_b<=0;out_vs<=0;out_hs<=0;out_de<=0;
 end else begin
  enable_meta<=requested_enabled;enable_sync<=enable_meta;
  vs_d<=in_vs;de_d<=in_de;
  if(frame_boundary)begin x<=0;y<=0;enabled<=enable_sync;end
  else begin
   if(in_de)x<=x+1'b1;else x<=0;
   if(de_d && !in_de)y<=y+1'b1;
  end
  lookup_luma_q<=luma;sample_luma_q<=lookup_luma_q;
  sample_q<=sampling;sample_col_q<=sample_col;sample_row_q<=sample_row;
  rgb1<={in_r,in_g,in_b};y1<=luma;ctl1<={in_vs,in_hs,in_de};
  lc1<=left_col;rc1<=right_col;tr1<=top_row;br1<=bottom_row;
  fx1<=fraction_x;fy1<=fraction_y;en1<=enabled && ready;
  rgb1_d<=rgb1;y1_d<=y1;ctl1_d<=ctl1;
  lc1_d<=lc1;rc1_d<=rc1;tr1_d<=tr1;br1_d<=br1;
  fx1_d<=fx1;fy1_d<=fy1;en1_d<=en1;
  rgb2<=rgb1_d;y2<=y1_d;ctl2<=ctl1_d;fx2<=fx1_d;fy2<=fy1_d;en2<=en1_d;
  tl2<=tile_map[{tr1_d,lc1_d}];tr2<=tile_map[{tr1_d,rc1_d}];
  bl2<=tile_map[{br1_d,lc1_d}];br2<=tile_map[{br1_d,rc1_d}];
  rgb3<=rgb2;y3<=y2;ctl3<=ctl2;fy3<=fy2;en3<=en2;
  top3<=blend(tl2,tr2,fx2);bottom3<=blend(bl2,br2,fx2);
  rgb4<=rgb3;y4<=y3;ctl4<=ctl3;en4<=en3;
  map4<=blend(top3,bottom3,fy3);
  out_r<=en4?add_delta(rgb4[23:16],delta):rgb4[23:16];
  out_g<=en4?add_delta(rgb4[15:8],delta):rgb4[15:8];
  out_b<=en4?add_delta(rgb4[7:0],delta):rgb4[7:0];
  {out_vs,out_hs,out_de}<=ctl4;
 end
endmodule
