// Chinese labels: before / after. 256x64 at (1640,24) in 1920x1080.
// Two registered pixel stages, with identical HS/VS/DE latency.
// Font lookup is separated from glyph-bit selection and RGB composition.
module video_mode_osd(input clk,rst_n,corrected,input [7:0] in_r,in_g,in_b,
 input affine_enabled,affine_ready,
 input [7:0] delay_gray,
 input in_vs,in_hs,in_de,output reg [7:0] out_r,out_g,out_b,
 output reg out_vs,out_hs,out_de,input fixed_crop);
reg [11:0] x,y;
reg de_d;
(* async_reg="true" *) reg [1:0] ae_sync,ar_sync;
reg [1:0] affine_shown;
(* async_reg="true" *) reg [7:0] delay_meta,delay_sync;
reg [7:0] delay_index_shown;
wire [7:0] delay_index;
genvar bit_id;
generate for(bit_id=0;bit_id<8;bit_id=bit_id+1)begin: gray_decode
 assign delay_index[bit_id]=^(delay_sync >> bit_id);
end endgenerate
reg [3:0] delay_h,delay_t,delay_o;
always @(posedge clk)begin
 delay_h<=delay_index_shown/100;
 delay_t<=(delay_index_shown/10)%10;delay_o<=delay_index_shown%10;
end
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin ae_sync<=0;ar_sync<=0;affine_shown<=0;delay_meta<=15;delay_sync<=15;delay_index_shown<=10;end
 else begin
  ae_sync<={ae_sync[0],affine_enabled};ar_sync<={ar_sync[0],affine_ready};
  delay_meta<=delay_gray;delay_sync<=delay_meta;
  if(in_vs)begin
   affine_shown<=!ae_sync[1]?0:((ar_sync[1] && corrected)?2:1);
   delay_index_shown<=delay_index;
  end
 end
wire affine_box=x>=1640 && x<1896 && y>=96 && y<128;
wire [11:0] ax=x-1640,ay=y-96;
reg [7:0] ch;
always @* begin
 ch=" ";
 case(ax[7:4])
 0:ch="A";1:ch="F";2:ch="F";3:ch="I";4:ch="N";5:ch="E";6:ch=":";
 8:ch=(affine_shown==1)?"W":"O";
 9:ch=(affine_shown==1)?"A":((affine_shown==2)?"N":"F");
 10:ch=(affine_shown==1)?"I":((affine_shown==0)?"F":" ");
 11:ch=(affine_shown==1)?"T":" ";
 13:ch="F";14:ch="I";15:ch="X";
 default:ch=" ";endcase
end
function [63:0] font;
 input [7:0] c;
 begin case(c)
 "A":font=64'h182442427E424200;"F":font=64'h7E40407C40404000;
 "R":font=64'h7C42427C48444200;
 "D":font=64'h7844424242447800;"X":font=64'h4242241824424200;
 "I":font=64'h3E08080808083E00;"N":font=64'h4262524A46424200;
 "E":font=64'h7E40407C40407E00;":":font=64'h0018180018180000;
 "O":font=64'h3C42424242423C00;"W":font=64'h4242425A5A664200;
 "T":font=64'h7F08080808080800;default:font=0;
 "S":font=64'h3C42403C02423C00;"Y":font=64'h4242241818181800;
 "C":font=64'h3C42404040423C00;"M":font=64'h42665A5A42424200;
 "0":font=64'h3C42464A52623C00;"1":font=64'h0818080808081C00;
 "2":font=64'h3C42020C30407E00;"3":font=64'h3C42021C02423C00;
 "4":font=64'h040C14247E040400;"5":font=64'h7E407C0202423C00;
 "6":font=64'h1C20407C42423C00;"7":font=64'h7E02040810101000;
 "8":font=64'h3C42423C42423C00;"9":font=64'h3C42423E02043800;
 endcase end
endfunction
wire [63:0] aglyph=font(ch);
wire aink=aglyph[{~ay[4:2],~ax[3:1]}];
wire delay_box=affine_shown!=0 && x>=1640 && x<1896 && y>=136 && y<168;
wire [11:0] dx=x-1640,dy=y-136;
reg [7:0] delay_ch;
always @* begin
 delay_ch=" ";
 case(dx[7:4])
  0:delay_ch="S";1:delay_ch="Y";2:delay_ch="N";3:delay_ch="C";4:delay_ch=":";
  5:delay_ch="0"+delay_h;6:delay_ch="0"+delay_t;
  7:delay_ch="0"+delay_o;8:delay_ch="M";9:delay_ch="S";
  default:delay_ch=" ";
 endcase
end
wire [63:0] dglyph=font(delay_ch);
wire dink=dglyph[{~dy[4:2],~dx[3:1]}];
wire [11:0] local_x=x-12'd1640, local_y=y-12'd24;
wire inside_box=x>=1640 && x<1896 && y>=24 && y<88;
reg [127:0] glyph_row;
always @* begin
 case({corrected,local_y[5:1]})
 6'd0: glyph_row=128'h00000000000000000000000000000000;
 6'd1: glyph_row=128'h00000000000000000000000000000000;
 6'd2: glyph_row=128'h00000000000000000000000000000000;
 6'd3: glyph_row=128'h00000000000000000000000000000000;
 6'd4: glyph_row=128'h00000000000000000000000000000000;
 6'd5: glyph_row=128'h00000000000000000000000000000000;
 6'd6: glyph_row=128'h00000000000000000000000000000000;
 6'd7: glyph_row=128'h00000004000000000000000000000000;
 6'd8: glyph_row=128'h0000000c001f00000000402000000000;
 6'd9: glyph_row=128'h0000000c0fff1fffff80e07000000000;
 6'd10: glyph_row=128'h0000000fee401fffff8030e000000000;
 6'd11: glyph_row=128'h0000001fe0400003003fffffc0000000;
 6'd12: glyph_row=128'h0000001b00400003003fffffc0000000;
 6'd13: glyph_row=128'h0000001300c000030000000000000000;
 6'd14: glyph_row=128'h000000131fff8303000ffe6300000000;
 6'd15: glyph_row=128'h000000031fff8303000ffe6300000000;
 6'd16: glyph_row=128'h0000000301b00303000c066300000000;
 6'd17: glyph_row=128'h0000003fe3180303ff0c066300000000;
 6'd18: glyph_row=128'h0000003fe70c0303ff0ffe6300000000;
 6'd19: glyph_row=128'h000000030e070303000ffe6300000000;
 6'd20: glyph_row=128'h00000002380fc303000c066300000000;
 6'd21: glyph_row=128'h00000003320c8303000c066300000000;
 6'd22: glyph_row=128'h00000007020c0303000ffe6300000000;
 6'd23: glyph_row=128'h00000007820c0303000ffe6300000000;
 6'd24: glyph_row=128'h00000006c60c0303000c066300000000;
 6'd25: glyph_row=128'h0000000c660c0303000c060300000000;
 6'd26: glyph_row=128'h0000000c4c0c0303000c060300000000;
 6'd27: glyph_row=128'h000000181c0c3fffffcc7e3f00000000;
 6'd28: glyph_row=128'h00000018380c3fffffcc7c3e00000000;
 6'd29: glyph_row=128'h00000000300c00000000000000000000;
 6'd30: glyph_row=128'h00000000000000000000000000000000;
 6'd31: glyph_row=128'h00000000000000000000000000000000;
 6'd32: glyph_row=128'h00000000000000000000000000000000;
 6'd33: glyph_row=128'h00000000000000000000000000000000;
 6'd34: glyph_row=128'h00000000000000000000000000000000;
 6'd35: glyph_row=128'h00000000000000000000000000000000;
 6'd36: glyph_row=128'h00000000000000000000000000000000;
 6'd37: glyph_row=128'h00000000000000000000000000000000;
 6'd38: glyph_row=128'h00000000000000000000000000000000;
 6'd39: glyph_row=128'h00000004000000000000000000000000;
 6'd40: glyph_row=128'h0000000c001f00000000007e00000000;
 6'd41: glyph_row=128'h0000000c0fff1fffff87ffff00000000;
 6'd42: glyph_row=128'h0000000fee401fffff87ffc000000000;
 6'd43: glyph_row=128'h0000001fe04000030006000000000000;
 6'd44: glyph_row=128'h0000001b004000030006000000000000;
 6'd45: glyph_row=128'h0000001300c000030006000000000000;
 6'd46: glyph_row=128'h000000131fff83030007ffffc0000000;
 6'd47: glyph_row=128'h000000031fff83030007ffffc0000000;
 6'd48: glyph_row=128'h0000000301b003030006000000000000;
 6'd49: glyph_row=128'h0000003fe3180303ff06000000000000;
 6'd50: glyph_row=128'h0000003fe70c0303ff06000000000000;
 6'd51: glyph_row=128'h000000030e07030300067ffe00000000;
 6'd52: glyph_row=128'h00000002380fc30300067ffe00000000;
 6'd53: glyph_row=128'h00000003320c83030006600600000000;
 6'd54: glyph_row=128'h00000007020c03030006600600000000;
 6'd55: glyph_row=128'h00000007820c0303000e600600000000;
 6'd56: glyph_row=128'h00000006c60c0303000c600600000000;
 6'd57: glyph_row=128'h0000000c660c0303000c600600000000;
 6'd58: glyph_row=128'h0000000c4c0c03030018600600000000;
 6'd59: glyph_row=128'h000000181c0c3fffffd87ffe00000000;
 6'd60: glyph_row=128'h00000018380c3ffffff07ffe00000000;
 6'd61: glyph_row=128'h00000000300c00000010600600000000;
 6'd62: glyph_row=128'h00000000000000000000000000000000;
 6'd63: glyph_row=128'h00000000000000000000000000000000;
 default: glyph_row=0;
 endcase
end
reg [63:0] aglyph1,dglyph1 /* synthesis syn_preserve=1 syn_keep=1 */;
reg [127:0] glyph1 /* synthesis syn_preserve=1 syn_keep=1 */;
reg [5:0] aindex1,dindex1;
reg [6:0] index1;
reg inside1,affine_box1,delay_box1,marker1,corrected1;
reg [23:0] rgb1;
reg vs1,hs1,de1;
wire ink1=glyph1[index1];
wire aink1=aglyph1[aindex1],dink1=dglyph1[dindex1];
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  x<=0;y<=0;de_d<=0;out_r<=0;out_g<=0;out_b<=0;out_vs<=0;out_hs<=0;out_de<=0;
  aglyph1<=0;dglyph1<=0;glyph1<=0;aindex1<=0;dindex1<=0;index1<=0;
  inside1<=0;affine_box1<=0;delay_box1<=0;marker1<=0;corrected1<=0;
  rgb1<=0;vs1<=0;hs1<=0;de1<=0;
 end else begin
  de_d<=in_de;
  if(in_vs) begin x<=0;y<=0;end
  else begin
   if(in_de) x<=x+1'b1; else x<=0;
   if(de_d && !in_de) y<=y+1'b1;
  end
  aglyph1<=aglyph;dglyph1<=dglyph;glyph1<=glyph_row;
  aindex1<={~ay[4:2],~ax[3:1]};dindex1<={~dy[4:2],~dx[3:1]};
  index1<=7'd127-local_x[7:1];
  inside1<=inside_box;affine_box1<=affine_box;delay_box1<=delay_box;
  marker1<=local_x<8;corrected1<=corrected;
  rgb1<={in_r,in_g,in_b};vs1<=in_vs;hs1<=in_hs;de1<=in_de;
  out_vs<=vs1;out_hs<=hs1;out_de<=de1;
  {out_r,out_g,out_b}<=rgb1;
  if(de1 && inside1) begin
   if(marker1) begin out_r<=corrected1?8'd32:8'd255;out_g<=corrected1?8'd220:8'd160;out_b<=8'd48;end
   else if(ink1) begin out_r<=255;out_g<=255;out_b<=255;end
   else begin out_r<=24;out_g<=28;out_b<=36;end
  end
  if(de1 && affine_box1) begin
   {out_r,out_g,out_b}<=aink1?24'hFFFFFF:24'h181C24;
  end
  if(de1 && delay_box1)begin
   {out_r,out_g,out_b}<=dink1?24'h72DFDF:24'h181C24;
  end
 end
endmodule
