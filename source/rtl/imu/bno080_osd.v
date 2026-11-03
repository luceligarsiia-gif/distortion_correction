`timescale 1ns/1ps
// Four-line BNO080 / affine / throughput status at (32,1008); 16x16 characters.
// Line 1: BNO080 link/status.
// Line 2: Roll/Pitch/Yaw in degrees with 0.1 degree resolution.
// The source-domain mailbox is captured only at VS so one displayed frame uses
// one coherent status/angle sample.
//
// status layout (88 bits):
// [87] online, [86] configured, [85:84] accuracy, [83:80] error,
// [79:64] report count,
// [63:47] roll  {sign,hundreds,tens,ones,tenths},
// [46:30] pitch {sign,hundreds,tens,ones,tenths},
// [29:13] yaw   {sign,hundreds,tens,ones,tenths},
// [12] angle_valid, [11:0] reserved.
module bno080_osd(input src_clk,src_rst_n,input [87:0] status,
 input [31:0] debug_status,
 input perf_clk,perf_rst_n,input [63:0] perf_status,
 input clk,rst_n,input [7:0] in_r,in_g,in_b,input in_vs,in_hs,in_de,
 output reg [7:0] out_r,out_g,out_b,output reg out_vs,out_hs,out_de);
reg [87:0] held,shown;
reg [31:0] debug_held,debug_shown;
reg [63:0] perf_held,perf_shown;
reg perf_request,perf_ack;
(* async_reg="true" *) reg [1:0] perf_ack_sync,perf_req_sync;
always @(posedge perf_clk or negedge perf_rst_n)
 if(!perf_rst_n)begin perf_held<=0;perf_request<=0;perf_ack_sync<=0;end
 else begin
  perf_ack_sync<={perf_ack_sync[0],perf_ack};
  if(perf_ack_sync[1]==perf_request)begin perf_held<=perf_status;perf_request<=~perf_request;end
 end
reg request,ack;
reg [1:0] ack_sync /* synthesis syn_preserve=1 syn_srlstyle="registers" */;
reg [1:0] req_sync /* synthesis syn_preserve=1 syn_srlstyle="registers" */;
reg vs_d,de_d;
reg [11:0] x,y;
always @(posedge src_clk or negedge src_rst_n)
 if(!src_rst_n) begin held<=0;debug_held<=0;request<=0;ack_sync<=0;end
 else begin
  ack_sync<={ack_sync[0],ack};
  if(ack_sync[1]==request) begin held<=status;debug_held<=debug_status;request<=~request;end
 end
wire [11:0] lx=x-32,ly=y-1008;
wire box=x>=32 && x<720 && y>=1008 && y<1072;
// Registered decimal conversion keeps division out of the glyph/pixel path.
function [11:0] decimal3;
 input [15:0] n;
 reg [15:0] v;
 begin v=n>999?16'd999:n;
  decimal3[11:8]=v/100;decimal3[7:4]=(v/10)%10;decimal3[3:0]=v%10;
 end
endfunction
reg [11:0] age_digits,new_digits,cor_digits,skip_digits,error_digits,ms_digits,clip_digits;
always @(posedge clk)begin
 age_digits<=decimal3({8'd0,debug_shown[24:17]});
 new_digits<=decimal3({8'd0,perf_shown[7:0]});
 cor_digits<=decimal3({8'd0,perf_shown[15:8]});
 skip_digits<=decimal3({8'd0,perf_shown[23:16]});
 error_digits<=decimal3({8'd0,perf_shown[31:24]});
 ms_digits<=decimal3(perf_shown[47:32]);
 clip_digits<=decimal3({8'd0,perf_shown[55:48]});
end
reg [7:0] ch;
function [7:0] hexchar;
 input [3:0] n;
 begin hexchar=(n<10)?8'd48+n:8'd65+(n-10);end
endfunction
function [7:0] decchar;
 input [3:0] n;
 begin decchar=(n<10)?8'd48+n:"?";end
endfunction

always @* begin
 ch=" ";
 if(ly[5:4]==0) case(lx[9:4])
  0:ch="B";1:ch="N";2:ch="O";3:ch="0";4:ch="8";5:ch="0";
  7:ch=shown[87]?"O":"W";8:ch=shown[87]?"K":"A";
  9:ch=shown[87]?" ":"I";10:ch=shown[87]?" ":"T";
  12:ch="A";13:ch=hexchar({2'b0,shown[85:84]});
  15:ch="N";16:ch=hexchar(shown[79:76]);17:ch=hexchar(shown[75:72]);
  18:ch=hexchar(shown[71:68]);19:ch=hexchar(shown[67:64]);
  21:ch="E";22:ch=hexchar(shown[83:80]);23:ch=shown[86]?"+":"-";
  default:ch=" ";
 endcase
 else if(ly[5:4]==1) case(lx[9:4])
  // R+012.3 P-004.5 Y+090.8
  0:ch="R";
  1:ch=shown[12]?(shown[63]?"-":"+"):"-";
  2:ch=shown[12]?decchar(shown[62:59]):"-";
  3:ch=shown[12]?decchar(shown[58:55]):"-";
  4:ch=shown[12]?decchar(shown[54:51]):"-";
  5:ch=".";
  6:ch=shown[12]?decchar(shown[50:47]):"-";
  8:ch="P";
  9:ch=shown[12]?(shown[46]?"-":"+"):"-";
 10:ch=shown[12]?decchar(shown[45:42]):"-";
 11:ch=shown[12]?decchar(shown[41:38]):"-";
 12:ch=shown[12]?decchar(shown[37:34]):"-";
 13:ch=".";
 14:ch=shown[12]?decchar(shown[33:30]):"-";
 16:ch="Y";
 17:ch=shown[12]?(shown[29]?"-":"+"):"-";
 18:ch=shown[12]?decchar(shown[28:25]):"-";
 19:ch=shown[12]?decchar(shown[24:21]):"-";
 20:ch=shown[12]?decchar(shown[20:17]):"-";
 21:ch=".";
 22:ch=shown[12]?decchar(shown[16:13]):"-";
  default:ch=" ";
 endcase
 else if(ly[5:4]==2)case(lx[9:4])
  0:ch="R";1:ch="O";2:ch="T";
  3:ch=debug_shown[16]?"-":"+";
  4:ch=decchar(debug_shown[15:12]);5:ch=decchar(debug_shown[11:8]);
  6:ch=decchar(debug_shown[7:4]);7:ch=".";8:ch=decchar(debug_shown[3:0]);
  10:ch="A";11:ch="G";12:ch="E";
  13:ch=decchar(age_digits[11:8]);14:ch=decchar(age_digits[7:4]);15:ch=decchar(age_digits[3:0]);
  16:ch="M";17:ch="S";19:ch="F";20:ch=":";
  21:ch=debug_shown[26:25]==0?"Q":(debug_shown[26:25]==2?"F":"N");
  23:ch=debug_shown[31:30]==2?"P":"A";24:ch=debug_shown[31:30]==2?"R":"F";
  25:ch=debug_shown[31:30]==2?"S":"F";26:ch=debug_shown[31:30]==2?"P":" ";27:ch=":";
   28:ch=!debug_shown[29]?"O":(debug_shown[28]?"S":(shown[85:84]==0?"L":(debug_shown[27]?"R":"O")));
   29:ch=!debug_shown[29]?"F":(debug_shown[28]?"T":(shown[85:84]==0?"O":(debug_shown[27]?"E":"K")));
   30:ch=!debug_shown[29]?"F":(debug_shown[28]?"L":(shown[85:84]==0?"W":(debug_shown[27]?"C":" ")));
  default:ch=" ";
 endcase
 else case(lx[9:4])
  0:ch="N";1:ch="E";2:ch="W";
  3:ch=decchar(new_digits[11:8]);4:ch=decchar(new_digits[7:4]);5:ch=decchar(new_digits[3:0]);
  7:ch="C";8:ch="O";9:ch="R";
  10:ch=decchar(cor_digits[11:8]);11:ch=decchar(cor_digits[7:4]);12:ch=decchar(cor_digits[3:0]);
  14:ch="S";15:ch="K";16:ch="P";
  17:ch=decchar(skip_digits[11:8]);18:ch=decchar(skip_digits[7:4]);19:ch=decchar(skip_digits[3:0]);
  21:ch="E";22:ch="R";23:ch="R";
  24:ch=decchar(error_digits[11:8]);25:ch=decchar(error_digits[7:4]);26:ch=decchar(error_digits[3:0]);
  28:ch="T";29:ch=decchar(ms_digits[11:8]);30:ch=decchar(ms_digits[7:4]);31:ch=decchar(ms_digits[3:0]);
  32:ch="M";33:ch="S";
  35:ch="C";36:ch="L";37:ch="P";
  38:ch=decchar(clip_digits[11:8]);39:ch=decchar(clip_digits[7:4]);40:ch=decchar(clip_digits[3:0]);
  default:ch=" ";
 endcase
 if(ly[5:4]==2 && debug_shown[31:30]==2 && debug_shown[29] &&
    !debug_shown[28] && !debug_shown[27] && debug_shown[26:25]==3)begin
  case(lx[9:4])28:ch="L";29:ch="I";30:ch="M";default:;endcase
 end
end
function [63:0] font;
 input [7:0] c;
 begin case(c)
  "0":font=64'h0038444c54644438;"1":font=64'h0010301010101038;
  "2":font=64'h003844040830407c;"3":font=64'h0078040438040478;
  "4":font=64'h00081828487c0808;"5":font=64'h007c404078040478;
  "6":font=64'h0038404078444438;"7":font=64'h007c040810202020;
  "8":font=64'h0038444438444438;"9":font=64'h003844443c040438;
  "A":font=64'h003844447c444444;"B":font=64'h0078444478444478;
  "C":font=64'h003c40404040403c;"D":font=64'h0078444444444478;
  "E":font=64'h007c40407840407c;"F":font=64'h007c404078404040;
  "G":font=64'h003c40405c44443c;"L":font=64'h004040404040407c;
  "M":font=64'h00446c5454444444;"Q":font=64'h0038444444544834;
  "S":font=64'h003c404038040478;":":font=64'h0000181800181800;
  "I":font=64'h0038101010101038;"J":font=64'h001c080808484830;
  "K":font=64'h0044485060504844;"N":font=64'h00446464544c4c44;
  "O":font=64'h0038444444444438;"P":font=64'h0078444478404040;
  "R":font=64'h0078444478504844;"T":font=64'h007c101010101010;
  "W":font=64'h0044444454545428;"Y":font=64'h0044442810101010;
  "+":font=64'h000010107c101000;"-":font=64'h000000007c000000;
  ".":font=64'h0000000000001818;"?":font=64'h0038040810100010;
  default:font=0;
 endcase end
endfunction
wire [5:0] bit_index={~ly[3:1],~lx[3:1]};
// Separate status/character selection from glyph lookup on the HDMI clock.
// RGB and VS/HS/DE take the same extra stage, preserving pixel placement.
reg [7:0] ch_d;
reg [5:0] bit_index_d;
reg box_d,good_d;
reg [23:0] rgb_d;
reg [2:0] sync_d;
wire [63:0] glyph=font(ch_d);
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  shown<=0;debug_shown<=0;perf_shown<=0;perf_ack<=0;perf_req_sync<=0;
  ack<=0;req_sync<=0;vs_d<=0;de_d<=0;x<=0;y<=0;
  out_r<=0;out_g<=0;out_b<=0;out_vs<=0;out_hs<=0;out_de<=0;
  ch_d<=0;bit_index_d<=0;box_d<=0;good_d<=0;rgb_d<=0;sync_d<=0;
 end else begin
  req_sync<={req_sync[0],request};vs_d<=in_vs;de_d<=in_de;
  perf_req_sync<={perf_req_sync[0],perf_request};
  if(in_vs && !vs_d && req_sync[1]!=ack) begin shown<=held;debug_shown<=debug_held;ack<=req_sync[1];end
  if(in_vs && !vs_d && perf_req_sync[1]!=perf_ack)begin perf_shown<=perf_held;perf_ack<=perf_req_sync[1];end
  if(in_vs) begin x<=0;y<=0;end
  else begin
   if(in_de) x<=x+1'b1;else x<=0;
   if(de_d && !in_de) y<=y+1'b1;
  end
  ch_d<=ch;bit_index_d<=bit_index;box_d<=in_de && box;
  good_d<=shown[87] && !(debug_shown[29] && (debug_shown[28] || shown[85:84]==0));
  rgb_d<={in_r,in_g,in_b};sync_d<={in_vs,in_hs,in_de};
  {out_r,out_g,out_b}<=rgb_d;
  {out_vs,out_hs,out_de}<=sync_d;
  if(box_d)
   if(glyph[bit_index_d]) {out_r,out_g,out_b}<=good_d?24'h60ffb0:24'hffbc50;
   else {out_r,out_g,out_b}<=24'h101820;
 end
endmodule
