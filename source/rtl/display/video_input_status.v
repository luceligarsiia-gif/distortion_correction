// Measure the incoming signal, not the fixed HDMI output timing. No clock is
// derived from VS/DE. Shared reset asserts in both CDC domains.
module video_input_status #(
 parameter integer CLK_HZ=125000000, EXPECT_WIDTH=1920, EXPECT_HEIGHT=1080
)(input pix_clk,clk,rst_n,input in_vs,in_de,
 input [15:0] capture_sequence,input [63:0] perf_status,
 input video_valid,engine_busy,engine_fault,
 output reg capture_ok,output [63:0] status);
reg vs_d,de_d,have_boundary,bad_width;
reg [11:0] pixels,lines,width;
reg [15:0] frame_sequence,frame_gray;
reg dimensions_valid;reg [24:0] dimensions;
wire edge_vs=in_vs && !vs_d;
always @(posedge pix_clk or negedge rst_n)
 if(!rst_n)begin
  vs_d<=0;de_d<=0;have_boundary<=0;bad_width<=0;pixels<=0;lines<=0;width<=0;
  frame_sequence<=0;frame_gray<=0;dimensions_valid<=0;dimensions<=0;capture_ok<=0;
 end else begin
  vs_d<=in_vs;de_d<=in_de;dimensions_valid<=0;
  if(in_de)begin if(pixels!=4095)pixels<=pixels+1'b1;end
  if(de_d && !in_de)begin
   if(lines==0)width<=pixels;
   if(pixels!=EXPECT_WIDTH)bad_width<=1;
   if(lines!=4095)lines<=lines+1'b1;
   pixels<=0;
  end
  if(edge_vs)begin
   frame_sequence<=frame_sequence+1'b1;
   frame_gray<=(frame_sequence+16'd1)^((frame_sequence+16'd1)>>1);
   if(have_boundary)begin
    dimensions<={!bad_width && width==EXPECT_WIDTH && lines==EXPECT_HEIGHT,width,lines};
    dimensions_valid<=1;
    capture_ok<=!bad_width && width==EXPECT_WIDTH && lines==EXPECT_HEIGHT;
   end
   have_boundary<=1;pixels<=0;lines<=0;width<=0;bad_width<=0;
  end
 end
wire dim_valid;wire [24:0] dim_data;
diagnostic_cdc #(.WIDTH(25)) dimensions_cdc(.src_clk(pix_clk),.rst_n(rst_n),
 .src_valid(dimensions_valid),.src_data(dimensions),.busy(),.lost(),
 .dst_clk(clk),.dst_valid(dim_valid),.dst_data(dim_data));
(* async_reg="true" *) reg [15:0] gray_meta,gray_sync;
reg [15:0] sequence_now;
integer k;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin gray_meta<=0;gray_sync<=0;end
 else begin gray_meta<=frame_gray;gray_sync<=gray_meta;end
always @*begin
 sequence_now[15]=gray_sync[15];
 for(k=14;k>=0;k=k-1)sequence_now[k]=sequence_now[k+1]^gray_sync[k];
end
reg [31:0] ticks,age;
reg [15:0] sequence_prev,window_sequence,window_capture;
reg [7:0] input_fps,capture_fps;
reg [24:0] latest_dimensions;
wire link_ok=age<CLK_HZ/4;
wire [15:0] input_delta=sequence_now-window_sequence;
wire [15:0] capture_delta=capture_sequence-window_capture;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  ticks<=0;age<=CLK_HZ/4;sequence_prev<=0;window_sequence<=0;window_capture<=0;
  input_fps<=0;capture_fps<=0;latest_dimensions<=0;
 end else begin
  sequence_prev<=sequence_now;
  if(sequence_now!=sequence_prev)age<=0;
  else if(age<CLK_HZ/4)age<=age+1'b1;
  if(dim_valid)latest_dimensions<=dim_data;
  if(ticks==CLK_HZ-1)begin
   ticks<=0;window_sequence<=sequence_now;window_capture<=capture_sequence;
   input_fps<=input_delta>255?8'd255:input_delta[7:0];
   capture_fps<=capture_delta>255?8'd255:capture_delta[7:0];
  end else ticks<=ticks+1'b1;
 end
// [63:56] flags; [55:48] display new FPS; [47:40] corrected FPS;
// [39:32] captured FPS; [31:24] input VS rate; [23:12] width; [11:0] height.
assign status={3'd0,engine_fault,engine_busy,video_valid,
 link_ok && latest_dimensions[24],link_ok,perf_status[7:0],perf_status[15:8],
 capture_fps,(link_ok?input_fps:8'd0),latest_dimensions[23:0]};
endmodule

// One aligned overlay stage. Always visible so absent input remains diagnosable.
module video_input_status_osd(input src_clk,clk,rst_n,input [63:0] status,
 input [7:0] in_r,in_g,in_b,input in_vs,in_hs,in_de,
 output reg [7:0] out_r,out_g,out_b,output reg out_vs,out_hs,out_de);
wire sample_valid;wire [63:0] sample;
wire busy;
diagnostic_cdc #(.WIDTH(64)) transfer(.src_clk(src_clk),.rst_n(rst_n),
 .src_valid(!busy),.src_data(status),.busy(busy),.lost(),
 .dst_clk(clk),.dst_valid(sample_valid),.dst_data(sample));
reg [63:0] latest,shown;
reg vs_d,de_d;reg [11:0] x,y;
wire [11:0] lx=x-640,ly=y-24;
wire box=x>=640 && x<1440 && y>=24 && y<88;
wire [5:0] col=lx[9:4];
function [15:0] decimal4;
 input [11:0] n;
 reg [3:0] th,h,t,o;
 begin th=n/1000;h=(n/100)%10;t=(n/10)%10;o=n%10;decimal4={th,h,t,o};end
endfunction
function [11:0] decimal3;
 input [7:0] n;
 reg [3:0] h,t,o;
 begin h=n/100;t=(n/10)%10;o=n%10;decimal3={h,t,o};end
endfunction
reg [15:0] wd,ht;reg [11:0] fps,cap,cor,newfps;
always @(posedge clk)begin
 wd<=decimal4(shown[23:12]);ht<=decimal4(shown[11:0]);
 fps<=decimal3(shown[31:24]);cap<=decimal3(shown[39:32]);
 cor<=decimal3(shown[47:40]);newfps<=decimal3(shown[55:48]);
end
localparam [399:0] LINE0="IN:0000X0000 FPS:000 CAP:000 COR:000 NEW:000      ";
localparam [399:0] LINE1="LINK:0 FORMAT:0 VIDEO:0 BUSY:0 ERR:0  C:0 DE      ";
reg [7:0] ch;
always @*begin
 ch=" ";
 if(col<50)begin
  if(!ly[5])begin
   ch=LINE0[(49-col)*8+:8];
   if(col>=3 && col<=6)ch="0"+wd[(6-col)*4+:4];
   if(col>=8 && col<=11)ch="0"+ht[(11-col)*4+:4];
   if(col>=17 && col<=19)ch="0"+fps[(19-col)*4+:4];
   if(col>=25 && col<=27)ch="0"+cap[(27-col)*4+:4];
   if(col>=33 && col<=35)ch="0"+cor[(35-col)*4+:4];
   if(col>=41 && col<=43)ch="0"+newfps[(43-col)*4+:4];
  end else begin
   ch=LINE1[(49-col)*8+:8];
   if(col==5)ch="0"+shown[56];if(col==14)ch="0"+shown[57];
   if(col==22)ch="0"+shown[58];if(col==29)ch="0"+shown[59];if(col==35)ch="0"+shown[60];
   if(col==40)ch="0"+shown[63:61];
  end
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
 "C":font=64'h0038444040404438;"D":font=64'h0078444444444478;
 "E":font=64'h007c40407840407c;"F":font=64'h007c404078404040;
 "G":font=64'h003c40404c444438;"I":font=64'h0038101010101038;
 "K":font=64'h0044485060504844;"L":font=64'h004040404040407c;
 "M":font=64'h00446c5444444444;"N":font=64'h00446464544c4c44;
 "O":font=64'h0038444444444438;"P":font=64'h0078444478404040;
 "R":font=64'h0078444478504844;"S":font=64'h003c404038040478;
 "T":font=64'h007c101010101010;"U":font=64'h0044444444444438;
 "V":font=64'h0044444444442810;"W":font=64'h00444444546c4444;
 "X":font=64'h0044442810284444;"Y":font=64'h0044442810101010;
 ":":font=64'h0000101000101000;default:font=0;
 endcase end
endfunction
wire [63:0] glyph=font(ch);
wire ink=glyph[{~ly[4:2],~lx[3:1]}];
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  latest<=0;shown<=0;vs_d<=0;de_d<=0;x<=0;y<=0;
  out_r<=0;out_g<=0;out_b<=0;out_vs<=0;out_hs<=0;out_de<=0;
 end else begin
  if(sample_valid)latest<=sample;
  vs_d<=in_vs;de_d<=in_de;
  if(in_vs && !vs_d)shown<=latest;
  if(in_vs)begin x<=0;y<=0;end
  else begin if(in_de)x<=x+1'b1;else x<=0;if(de_d && !in_de)y<=y+1'b1;end
  {out_r,out_g,out_b}<={in_r,in_g,in_b};{out_vs,out_hs,out_de}<={in_vs,in_hs,in_de};
  if(in_de && box){out_r,out_g,out_b}<=ink?(shown[57]?24'h72dfdf:24'hffb050):24'h141a22;
 end
endmodule
