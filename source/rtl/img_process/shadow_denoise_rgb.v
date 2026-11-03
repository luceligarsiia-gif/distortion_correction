// Causal 2x2 range-gated spatial denoising before shadow enhancement.
// Only past pixels of THIS frame are used; no line/frame wait is introduced.
// Two raw-line banks avoid read/write-address collisions. Five pixel stages.
module shadow_denoise_rgb #(parameter integer WIDTH=1920,HEIGHT=1080)(
 input clk,rst_n,requested_enabled,
 input [7:0] in_r,in_g,in_b,input in_vs,in_hs,in_de,
 output reg [7:0] out_r,out_g,out_b,output reg out_vs,out_hs,out_de,
 output reg enabled);
 reg enable_meta,enable_sync,vs_d,de_d;
 reg [11:0] x,y;reg bank;
 wire frame_boundary=vs_d && !in_vs;
 wire inside_image=x<WIDTH && y<HEIGHT;
 reg [23:0] line0[0:WIDTH-1] /* synthesis syn_ramstyle="block_ram" */;
 reg [23:0] line1[0:WIDTH-1] /* synthesis syn_ramstyle="block_ram" */;
 reg [23:0] up0_q,up1_q;
 // Read only the bank opposite to the write bank; do not reset RAM contents.
 always @(posedge clk)begin
  if(in_de && inside_image)begin
   if(bank)begin line1[x]<={in_r,in_g,in_b};up0_q<=line0[x];end
   else begin line0[x]<={in_r,in_g,in_b};up1_q<=line1[x];end
  end
 end
 reg [23:0] raw_left,up_left_hold;
 reg [23:0] c1,l1,c2,l2,u2,d2,c3,l3,u3,d3;
 reg [2:0] ctl1,ctl2,ctl3,ctl4;
 reg bank1,left_ok1,up_ok1,inside1,en1;
 reg left_ok2,up_ok2,filter2;
 reg [7:0] luma1;reg [3:0] threshold2;
 wire [9:0] luma_quarter={2'b0,in_r}+{1'b0,in_g,1'b0}+{2'b0,in_b};
 wire [23:0] selected_up=bank1?up0_q:up1_q;
 reg [9:0] r4,g4,b4;
 function [8:0] difference;
  input [7:0] a,b;
  begin difference=a>=b?({1'b0,a}-{1'b0,b}):({1'b0,b}-{1'b0,a});end
 endfunction
 function [23:0] accepted_neighbor;
  input use_neighbor;input [23:0] neighbor,center;input [3:0] threshold;
  begin
   if(use_neighbor && difference(neighbor[23:16],center[23:16])<=threshold &&
      difference(neighbor[15:8],center[15:8])<=threshold &&
      difference(neighbor[7:0],center[7:0])<=threshold)
    accepted_neighbor=neighbor;
   else accepted_neighbor=center;
  end
 endfunction
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  enable_meta<=0;enable_sync<=0;enabled<=0;vs_d<=1;de_d<=0;x<=0;y<=0;bank<=0;
  raw_left<=0;up_left_hold<=0;c1<=0;l1<=0;c2<=0;l2<=0;u2<=0;d2<=0;
  c3<=0;l3<=0;u3<=0;d3<=0;ctl1<=0;ctl2<=0;ctl3<=0;ctl4<=0;
  bank1<=0;left_ok1<=0;up_ok1<=0;inside1<=0;en1<=0;luma1<=0;
  left_ok2<=0;up_ok2<=0;filter2<=0;threshold2<=0;r4<=0;g4<=0;b4<=0;
  out_r<=0;out_g<=0;out_b<=0;out_vs<=0;out_hs<=0;out_de<=0;
 end else begin
  enable_meta<=requested_enabled;enable_sync<=enable_meta;vs_d<=in_vs;de_d<=in_de;
  if(frame_boundary)begin x<=0;y<=0;bank<=0;enabled<=enable_sync;end
  else begin
   if(in_de)begin if(x!=12'hfff)x<=x+1'b1;end else x<=0;
   if(de_d && !in_de)begin if(y!=12'hfff)y<=y+1'b1;bank<=~bank;end
  end
  if(in_de)raw_left<={in_r,in_g,in_b};else raw_left<=0;
  if(ctl1[0])up_left_hold<=selected_up;else up_left_hold<=0;
  // S1 runs in parallel with the synchronous previous-line read.
  c1<={in_r,in_g,in_b};l1<=raw_left;ctl1<={in_vs,in_hs,in_de};
  bank1<=bank;left_ok1<=x!=0;up_ok1<=y!=0;inside1<=inside_image;
  en1<=enabled;luma1<=luma_quarter[9:2];
  // S2 aligns the four RAW samples, and registers the dark-pixel tolerance.
  c2<=c1;l2<=l1;u2<=selected_up;d2<=up_left_hold;ctl2<=ctl1;
  left_ok2<=left_ok1;up_ok2<=up_ok1;
  filter2<=en1 && inside1 && ctl1[0] && luma1<144;
  threshold2<=luma1<8?4'd6:luma1<64?4'd12:luma1<112?4'd8:4'd4;
  // S3: use one RGB range gate, preserving both luma and chromatic edges.
  c3<=c2;ctl3<=ctl2;
  l3<=accepted_neighbor(filter2 && left_ok2,l2,c2,threshold2);
  u3<=accepted_neighbor(filter2 && up_ok2,u2,c2,threshold2);
  d3<=accepted_neighbor(filter2 && left_ok2 && up_ok2,d2,c2,threshold2);
  // S4: fixed denominator, balanced sums, no multiplier/divider or DSP.
  r4<=(({2'd0,c3[23:16]}+{2'd0,l3[23:16]})+
       ({2'd0,u3[23:16]}+{2'd0,d3[23:16]}))+10'd2;
  g4<=(({2'd0,c3[15:8]}+{2'd0,l3[15:8]})+
       ({2'd0,u3[15:8]}+{2'd0,d3[15:8]}))+10'd2;
  b4<=(({2'd0,c3[7:0]}+{2'd0,l3[7:0]})+
       ({2'd0,u3[7:0]}+{2'd0,d3[7:0]}))+10'd2;
  ctl4<=ctl3;
  // S5: round-to-nearest output and exactly aligned video controls.
  out_r<=r4[9:2];out_g<=g4[9:2];out_b<=b4[9:2];
  {out_vs,out_hs,out_de}<=ctl4;
 end
endmodule
