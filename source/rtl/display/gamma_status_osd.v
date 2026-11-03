// Coherent source-clock mailbox; shown only on output VS rising edge.
// Text reports HDMI input enhancement, not a new geometric error metric.
// Three stages: character, glyph bit, palette. RGB/VS/HS/DE align.
module gamma_status_osd(input src_clk,src_rst_n,input [63:0] status,
 input clk,rst_n,input [7:0] in_r,in_g,in_b,input in_vs,in_hs,in_de,
 output reg [7:0] out_r,out_g,out_b,output reg out_vs,out_hs,out_de);
reg [63:0] held,shown;
reg request,ack;
reg [1:0] ack_sync /* synthesis syn_preserve=1 syn_srlstyle="registers" */;
reg [1:0] req_sync /* synthesis syn_preserve=1 syn_srlstyle="registers" */;
reg vs_d,de_d;
reg [11:0] x,y;
always @(posedge src_clk or negedge src_rst_n)
 if(!src_rst_n)begin held<=0;request<=0;ack_sync<=0;end
 else begin
  ack_sync<={ack_sync[0],ack};
  if(ack_sync[1]==request)begin held<=status;request<=~request;end
 end
wire [11:0] lx=x-32,ly=y-24;
wire box=x>=32 && x<624 && y>=24 && y<88;
wire signed [5:0] level=$signed(shown[13:8]);
wire [5:0] magnitude=level<0?-level:level;
reg [3:0] avg_h,avg_t,avg_o,level_t,level_o;
wire signed [5:0] target=$signed(shown[46:41]);
wire [5:0] target_magnitude=target<0?-target:target;
reg [3:0] target_t,target_o,p50h,p50t,p50o,p90h,p90t,p90o,sath,satt,sato;
reg [47:0] scene_text;
// Decimal conversion is registered separately from character/glyph selection.
// Status changes in blanking, so these settle before the displayed text.
always @(posedge clk)begin
 avg_h<=shown[7:0]/100;avg_t<=(shown[7:0]/10)%10;avg_o<=shown[7:0]%10;
 level_t<=magnitude/10;level_o<=magnitude%10;
 target_t<=target_magnitude/10;target_o<=target_magnitude%10;
 p50h<=shown[25:18]/100;p50t<=(shown[25:18]/10)%10;p50o<=shown[25:18]%10;
 p90h<=shown[33:26]/100;p90t<=(shown[33:26]/10)%10;p90o<=shown[33:26]%10;
 sath<=shown[40:34]/100;satt<=(shown[40:34]/10)%10;sato<=shown[40:34]%10;
end
always @*begin
 case(shown[17:16])
 0:scene_text="CLAHE ";1:scene_text="DIM   ";2:scene_text="DARK  ";default:scene_text="BRIGHT";
 endcase
 if(!shown[14])scene_text="WAIT  ";
end
reg [7:0] ch;
always @*begin
 ch=" ";
 if(!ly[5])case(lx[9:4])
  0:ch="H";1:ch="Y";2:ch="B";3:ch="R";4:ch="D";5:ch=":";
  6:ch="O";7:ch=shown[15]?"N":"F";8:ch=shown[15]?" ":"F";
  10:ch="W";11:ch=":";12:ch=shown[14]?(level<0?"-":"+"):"?";
  13:ch=shown[14]?"0"+level_t:"-";
  14:ch=shown[14]?"0"+level_o:"-";
  17:ch="N";18:ch="R";19:ch=":";20:ch="O";21:ch=shown[47]?"N":"F";22:ch=shown[47]?" ":"F";
  24:ch="S";25:ch="C";26:ch=":";
  27:ch=scene_text[47:40];28:ch=scene_text[39:32];29:ch=scene_text[31:24];
  30:ch=scene_text[23:16];31:ch=scene_text[15:8];32:ch=scene_text[7:0];
  default:;
 endcase
 else case(lx[9:4])
  0:ch="I";1:ch="N";3:ch="A";4:ch="V";5:ch="G";6:ch=":";
  7:ch=shown[14]?"0"+avg_h:"-";
  8:ch=shown[14]?"0"+avg_t:"-";
  9:ch=shown[14]?"0"+avg_o:"-";
  11:ch="P";12:ch="5";13:ch="0";14:ch=":";
  15:ch=shown[14]?"0"+p50h:"-";16:ch=shown[14]?"0"+p50t:"-";17:ch=shown[14]?"0"+p50o:"-";
  19:ch="P";20:ch="9";21:ch="0";22:ch=":";
  23:ch=shown[14]?"0"+p90h:"-";24:ch=shown[14]?"0"+p90t:"-";25:ch=shown[14]?"0"+p90o:"-";
  27:ch="S";28:ch="A";29:ch="T";30:ch=":";
  31:ch=shown[14]?"0"+sath:"-";32:ch=shown[14]?"0"+satt:"-";33:ch=shown[14]?"0"+sato:"-";34:ch="%";
  default:;
 endcase
end
function [63:0] font;
 input [7:0] c;
 begin case(c)
  "0":font=64'h0038444c54644438;"1":font=64'h0010301010101038;
  "2":font=64'h003844040830407c;"3":font=64'h0078040438040478;
  "4":font=64'h00081828487c0808;"5":font=64'h007c404078040478;
  "6":font=64'h0038404078444438;"7":font=64'h007c040810202020;
  "8":font=64'h0038444438444438;"9":font=64'h003844443c040438;
  "A":font=64'h003844447c444444;"F":font=64'h007c404078404040;
  "G":font=64'h003c40404c444438;"I":font=64'h0038101010101038;
  "L":font=64'h004040404040407c;"M":font=64'h00446c5444444444;
  "N":font=64'h00446464544c4c44;"O":font=64'h0038444444444438;
  "V":font=64'h0044444444442810;":":font=64'h0000101000101000;
  "+":font=64'h000010107c101000;"-":font=64'h000000007c000000;
  "?":font=64'h0038440408100010;default:font=0;
  "B":font=64'h0078444478444478;"C":font=64'h0038444040404438;
  "D":font=64'h0078444444444478;"E":font=64'h007c40407840407c;
  "H":font=64'h004444447c444444;"K":font=64'h0044485060504844;
  "P":font=64'h0078444478404040;"R":font=64'h0078444478504844;
  "S":font=64'h003c404038040478;"T":font=64'h007c101010101010;
  "Y":font=64'h0044442810101010;
  "W":font=64'h0044444454546c44;"%":font=64'h0062640810204c8c;
 endcase end
endfunction
// Register character and glyph selection before palette output.
reg [7:0] ch1;
reg [5:0] glyph_index1;
reg box1;
reg [23:0] rgb1;
reg [2:0] controls1;
wire [63:0] glyph=font(ch1);
wire ink=glyph[glyph_index1];
reg ink2,box2;reg [23:0] rgb2;reg [2:0] controls2;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  shown<=0;ack<=0;req_sync<=0;vs_d<=0;de_d<=0;x<=0;y<=0;
  out_r<=0;out_g<=0;out_b<=0;out_vs<=0;out_hs<=0;out_de<=0;
  ch1<=0;glyph_index1<=0;box1<=0;rgb1<=0;controls1<=0;
  ink2<=0;box2<=0;rgb2<=0;controls2<=0;
 end else begin
  req_sync<={req_sync[0],request};vs_d<=in_vs;de_d<=in_de;
  if(in_vs && !vs_d && req_sync[1]!=ack)begin shown<=held;ack<=req_sync[1];end
  if(in_vs)begin x<=0;y<=0;end
  else begin if(in_de)x<=x+1'b1;else x<=0;if(de_d && !in_de)y<=y+1'b1;end
  ch1<=ch;glyph_index1<={~ly[4:2],~lx[3:1]};box1<=box;
  rgb1<={in_r,in_g,in_b};controls1<={in_vs,in_hs,in_de};
  ink2<=ink;box2<=box1;rgb2<=rgb1;controls2<=controls1;
  {out_r,out_g,out_b}<=rgb2;{out_vs,out_hs,out_de}<=controls2;
  if(controls2[0] && box2){out_r,out_g,out_b}<=ink2?24'h72dfdf:24'h141a22;
 end
endmodule
