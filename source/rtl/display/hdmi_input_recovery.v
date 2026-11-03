// 1080p progressive DE-based frame recovery. 4096 pixel clocks exceeds normal
// horizontal blanking (280 at 60p, 2480 at CEA 30p) and is below vertical blanking. No free
// running frame timer: one pulse per blank gap, rearmed only by active pixels.
// This cannot repair a damaged pixel clock/DE bus or decode YCbCr422.
module hdmi_input_recovery #(
 parameter integer GAP_CLOCKS=4096, VS_CLOCKS=32
)(input clk,rst_n,input in_vs,in_hs,in_de,input [7:0] in_r,in_g,in_b,
 input mode_clk,mode_rst_n,input [2:0] requested_color,
 output reg [2:0] active_color,
 output reg out_vs,out_hs,out_de,output reg [7:0] out_r,out_g,out_b);
// One common input register bank keeps colour and controls together. External
// setup/hold still needs to be checked against the receiver's data sheet.
reg [23:0] rgb0;reg de0,hs0;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin rgb0<=0;de0<=0;hs0<=0;end
 else begin rgb0<={in_r,in_g,in_b};de0<=in_de;hs0<=in_hs;end
wire mode_valid;wire [2:0] mode_data;wire mode_busy;
diagnostic_cdc #(.WIDTH(3)) color_transfer(.src_clk(mode_clk),.rst_n(mode_rst_n),
 .src_valid(!mode_busy),.src_data(requested_color),.busy(mode_busy),.lost(),
 .dst_clk(clk),.dst_valid(mode_valid),.dst_data(mode_data));
reg [2:0] pending_color;
reg [15:0] blank_count;reg gap_fired;reg [15:0] pulse_left;
wire recovered_vs=(pulse_left==0); // active low, as expected by writer/Gamma
wire gap_event=!de0 && !gap_fired && blank_count==GAP_CLOCKS-1;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin blank_count<=0;gap_fired<=0;pulse_left<=0;pending_color<=0;active_color<=0;end
 else begin
  if(mode_valid)pending_color<=mode_data<=4?mode_data:3'd0;
  if(pulse_left!=0)pulse_left<=pulse_left-1'b1;
  if(de0)begin blank_count<=0;gap_fired<=0;end
  else if(!gap_fired)begin
   if(gap_event)begin
    gap_fired<=1;pulse_left<=VS_CLOCKS;active_color<=pending_color;
   end else blank_count<=blank_count+1'b1;
  end
 end

// 0: RGB bypass; 1: Cr/Y/Cb BT709 limited; 2: Y/Cb/Cr BT709 limited;
// 3: Cr/Y/Cb BT601 limited; 4: Y/Cb/Cr BT601 limited.
// RGB stays the default; EDID is not evidence of actual bus colour packing.
reg signed [9:0] yy,cb,cr;
always @*begin
 yy=$signed({2'b00,rgb0[15:8]})-10'sd16;
 cb=$signed({2'b00,rgb0[7:0]})-10'sd128;
 cr=$signed({2'b00,rgb0[23:16]})-10'sd128;
 if(active_color==2 || active_color==4)begin
  yy=$signed({2'b00,rgb0[23:16]})-10'sd16;
  cb=$signed({2'b00,rgb0[15:8]})-10'sd128;
  cr=$signed({2'b00,rgb0[7:0]})-10'sd128;
 end
end
// Bit-exact constant products use logic, avoiding an APM output-to-adder
// critical path. Pixel/control pipeline length and all colour modes are kept.
wire signed [20:0] ey={{11{yy[9]}},yy};
wire signed [20:0] eu={{11{cb[9]}},cb};
wire signed [20:0] ev={{11{cr[9]}},cr};
wire mode601=active_color==3 || active_color==4;
wire signed [20:0] py298=((ey<<<8)+(ey<<<5))+((ey<<<3)+(ey<<<1));
wire signed [20:0] pv409=((ev<<<8)+(ev<<<7))+((ev<<<4)+(ev<<<3))+ev;
wire signed [20:0] pv459=(ev<<<9)-((ev<<<5)+(ev<<<4))-((ev<<<2)+ev);
wire signed [20:0] pu100=((eu<<<6)+(eu<<<5))+(eu<<<2);
wire signed [20:0] pu55=(eu<<<6)-((eu<<<3)+eu);
wire signed [20:0] pv208=(ev<<<8)-((ev<<<5)+(ev<<<4));
wire signed [20:0] pv136=(ev<<<7)+(ev<<<3);
wire signed [20:0] pu516=(eu<<<9)+(eu<<<2);
wire signed [20:0] pu541=((eu<<<9)+(eu<<<5))-((eu<<<1)+eu);
reg signed [20:0] y1,rc1,gc1,gc2,bc1;
reg [23:0] raw1,raw2;reg bypass1,bypass2;
reg [2:0] ctl1,ctl2;
reg signed [21:0] r2,g2,b2;
function [7:0] clamp8;
 input signed [21:0] n;
 begin if(n<0)clamp8=0;else if(n>22'sd65280)clamp8=255;else clamp8=n[15:8];end
endfunction
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  y1<=0;rc1<=0;gc1<=0;gc2<=0;bc1<=0;raw1<=0;raw2<=0;bypass1<=1;bypass2<=1;
  ctl1<=3'b100;ctl2<=3'b100;r2<=0;g2<=0;b2<=0;
  out_vs<=1;out_hs<=0;out_de<=0;out_r<=0;out_g<=0;out_b<=0;
 end else begin
  y1<=py298;
  rc1<=mode601?pv409:pv459;
  gc1<=mode601?pu100:pu55;
  gc2<=mode601?pv208:pv136;
  bc1<=mode601?pu516:pu541;
  raw1<=rgb0;bypass1<=active_color==0;ctl1<={recovered_vs,hs0,de0};
  r2<=$signed(y1)+$signed(rc1)+22'sd128;
  g2<=$signed(y1)-$signed(gc1)-$signed(gc2)+22'sd128;
  b2<=$signed(y1)+$signed(bc1)+22'sd128;
  raw2<=raw1;bypass2<=bypass1;ctl2<=ctl1;
  {out_r,out_g,out_b}<=bypass2?raw2:{clamp8(r2),clamp8(g2),clamp8(b2)};
  {out_vs,out_hs,out_de}<=ctl2;
 end
endmodule

// S3+S4 pressed together within 100ms and held 2s cycles colour once. Delay
// individual presses for chord recognition (adds 100ms to hold thresholds).
// A chord never reaches the existing Gamma/SYNC button controllers.
module input_color_buttons #(
 parameter integer CHORD_WINDOW=2700000,HOLD_CLOCKS=54000000
)(input clk,rst_n,s3_n,s4_n,output key3_n,key4_n,output reg [2:0] color_mode);
reg [1:0] a,b;reg [2:0] state;reg [31:0] count;
localparam IDLE=0,DECIDE=1,SINGLE=2,CHORD=3,RELEASE=4;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin a<=3;b<=3;end else begin a<={s4_n,s3_n};b<=a;end
assign key3_n=state==SINGLE?b[0]:1'b1;
assign key4_n=state==SINGLE?b[1]:1'b1;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin state<=IDLE;count<=0;color_mode<=0;end
 else case(state)
 IDLE:begin count<=0;if(b!=3)state<=DECIDE;end
 DECIDE:if(b==0)begin state<=CHORD;count<=0;end
        else if(b==3)begin state<=IDLE;count<=0;end
        else if(count==CHORD_WINDOW-1)begin state<=SINGLE;count<=0;end
        else count<=count+1'b1;
 SINGLE:if(b==3)state<=IDLE;
 CHORD:if(b!=0)state<=RELEASE;
       else if(count==HOLD_CLOCKS-1)begin color_mode<=color_mode==4?3'd0:color_mode+1'b1;state<=RELEASE;end
       else count<=count+1'b1;
 RELEASE:if(b==3)state<=IDLE;
 default:state<=IDLE;
 endcase
endmodule
