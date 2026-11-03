// Synchronous glyph ROM and aligned RGB/HS/VS/DE: three pixel-clock stages.
// Text is generated from the actual distortion_map fixed-point defaults.
module parameter_osd #(
 parameter FONT_FILE="source/rtl/display/parameter_font.mem",
 parameter TEXT_FILE="source/rtl/display/parameter_text.mem"
)(input clk,rst_n,requested_visible,
 input [7:0] in_r,in_g,in_b,input in_vs,in_hs,in_de,
 output reg [7:0] out_r,out_g,out_b,output reg out_vs,out_hs,out_de);
reg [11:0] x,y;
reg de_d,vs_d,visible;
wire inside_box=x>=32 && x<608 && y>=112 && y<528;
wire [11:0] lx=x-32,ly=y-112;
reg [4:0] text_col,glyph_x;
wire [4:0] col=text_col,row=ly[8:5];
wire [4:0] gx=glyph_x;
wire [8:0] text_addr=row*24+col;
reg [6:0] text_rom[0:511];
reg [23:0] font[0:4095];
initial begin $readmemh(TEXT_FILE,text_rom);$readmemh(FONT_FILE,font);end
wire [6:0] glyph_id=text_rom[text_addr];
reg [23:0] glyph;
reg [4:0] gx_d;
reg box_d,heading_d;
reg [23:0] pixel_d;
reg [2:0] controls_d;
reg ink_q,box_q,heading_q;reg [23:0] pixel_q;reg [2:0] controls_q;
always @(posedge clk) glyph<=font[{glyph_id,ly[4:0]}];
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  x<=0;y<=0;de_d<=0;vs_d<=0;visible<=0;text_col<=0;glyph_x<=0;
  gx_d<=0;box_d<=0;heading_d<=0;pixel_d<=0;controls_d<=0;
  ink_q<=0;box_q<=0;heading_q<=0;pixel_q<=0;controls_q<=0;
  out_r<=0;out_g<=0;out_b<=0;out_vs<=0;out_hs<=0;out_de<=0;
 end else begin
  de_d<=in_de;vs_d<=in_vs;
  if(in_vs || !in_de || x<32 || x>=608)begin text_col<=0;glyph_x<=0;end
  else if(glyph_x==23)begin glyph_x<=0;text_col<=text_col+1'b1;end
  else glyph_x<=glyph_x+1'b1;
  // Only change visibility at VS rising edge, never halfway through a frame.
  if(in_vs && !vs_d) visible<=requested_visible;
  if(in_vs) begin x<=0;y<=0;end
  else begin
   if(in_de)x<=x+1'b1;else x<=0;
   if(de_d&&!in_de)y<=y+1'b1;
  end
  gx_d<=gx;box_d<=inside_box && visible;
  heading_d<=row==0 || row==2 || row==7;
  pixel_d<={in_r,in_g,in_b};controls_d<={in_vs,in_hs,in_de};
  ink_q<=glyph[23-gx_d];box_q<=box_d;heading_q<=heading_d;pixel_q<=pixel_d;controls_q<=controls_d;
  {out_r,out_g,out_b}<=pixel_q;
  {out_vs,out_hs,out_de}<=controls_q;
  if(controls_q[0] && box_q) begin
   if(ink_q) {out_r,out_g,out_b}<=heading_q?24'h72dfdf:24'he8f4ff;
   else {out_r,out_g,out_b}<=24'h141a22;
  end
 end
endmodule

