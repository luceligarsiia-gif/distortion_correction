// Final read-only overlay, RGB/VS/HS/DE all delayed exactly FIVE clocks.
// 1024-bin single-clock ring; display the last 512 completed 10ms bins.
// Snapshot pointer once/frame. Extra half-ring prevents normal read/write
// collision. Sequence tags hide uninitialized RAM and lost telemetry bins.
module diagnostic_scope_osd(input clk,rst_n,visible,
 input sample_valid,input [383:0] sample,
 input [7:0] in_r,in_g,in_b,input in_vs,in_hs,in_de,
 output reg [7:0] out_r,out_g,out_b,output reg out_vs,out_hs,out_de);
reg [95:0] history[0:1023];
reg [95:0] point_ram,point;
reg [383:0] latest,shown;
wire [47:0] interval_digits;
reg [47:0] shown_digits;
diagnostic_decimal decimal_values(clk,rst_n,sample_valid,
 {sample[131:116],sample[115:100],sample[99:84]},interval_digits);
reg [15:0] latest_end,shown_end;
reg have,shown_have,shown_visible;
reg [9:0] filled,shown_filled;
reg [11:0] x,y,x0,y0,x1,y1;
reg de_d;
reg [23:0] rgb0,rgb1;
reg [2:0] ctl0,ctl1;
reg [15:0] tag0,tag1;
wire [11:0] local_x=x-32;
wire [15:0] read_tag=shown_end-16'd512+{7'd0,local_x[9:1]};
function [9:0] ordinate;
 input signed [20:0] a;
 reg signed [31:0] d;
 begin d=a;d=d>>>12;if(d>180)d=180;if(d< -180)d=-180;ordinate=700-d;end
endfunction
wire [15:0] span_dt=sample[323:308]>sample[99:84]?sample[323:308]:sample[99:84];
wire [15:0] plot_dt=sample[131:116]>span_dt?sample[131:116]:span_dt;
wire [15:0] dt_ms=plot_dt/10;
wire [5:0] dt_height=(dt_ms>50)?6'd50:dt_ms[5:0];
always @(posedge clk)begin
 if(sample_valid)history[sample[301:292]]<=
  {18'd0,sample[307:292],sample[147:140],sample[139:132],dt_height,
   ordinate($signed(sample[20:0])),ordinate($signed(sample[41:21])),
   ordinate($signed(sample[62:42])),ordinate($signed(sample[83:63]))};
 point_ram<=history[read_tag[9:0]];
 point<=point_ram;
end
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  latest<=0;shown<=0;latest_end<=0;shown_end<=0;have<=0;shown_have<=0;shown_visible<=0;filled<=0;shown_filled<=0;shown_digits<=0;
  x<=0;y<=0;x0<=0;y0<=0;x1<=0;y1<=0;de_d<=0;rgb0<=0;rgb1<=0;ctl0<=0;ctl1<=0;tag0<=0;tag1<=0;
 end else begin
  if(sample_valid)begin
   latest<=sample;latest_end<=sample[307:292]+1'b1;have<=1;
   if(filled<512)filled<=filled+1'b1;
  end
  de_d<=in_de;
  if(in_vs)begin
   x<=0;y<=0;shown<=latest;shown_end<=latest_end;shown_have<=have;shown_visible<=visible;shown_filled<=filled;
   shown_digits<=interval_digits;
  end else begin
   if(in_de)x<=x+1'b1;else x<=0;
   if(de_d && !in_de)y<=y+1'b1;
  end
  x0<=x;y0<=y;rgb0<={in_r,in_g,in_b};ctl0<={in_vs,in_hs,in_de};tag0<=read_tag;
  x1<=x0;y1<=y0;rgb1<=rgb0;ctl1<=ctl0;tag1<=tag0;
 end
wire [11:0] lx=x1-32;
wire box=x1>=32 && x1<1056 && y1>=480 && y1<992;
wire point_valid=shown_have && {1'b0,lx[9:1]}>=10'd512-shown_filled && point[77:62]==tag1;
wire epoch_ok=point[61:54]==shown[147:140];
wire [7:0] flags=point[53:46];
wire [9:0] raw_top=point[29:20],raw_bottom=point[39:30];
wire [9:0] green_y=point[19:10],yellow_y=point[9:0];
wire [5:0] dth=point[45:40];
reg [9:0] previous_raw,previous_green,previous_yellow;
reg [15:0] previous_tag;
reg [7:0] previous_flags;
reg previous_ok;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin previous_raw<=700;previous_green<=700;previous_yellow<=700;previous_tag<=0;previous_flags<=0;previous_ok<=0;end
 else if(!ctl1[0] || x1<32)previous_ok<=0;
 else if(lx[0])begin
  previous_raw<=raw_top;previous_green<=green_y;previous_yellow<=yellow_y;
  previous_tag<=tag1;previous_flags<=flags;previous_ok<=point_valid && epoch_ok;
 end
function segment;
 input [11:0] row;input [9:0] a,b;
 reg [10:0] delta;
 begin
  delta={1'b0,a}-{1'b0,b};
  segment=(delta<180 || delta>1868) &&
          ((row>=a && row<=b) || (row>=b && row<=a));
 end
endfunction
wire connect=previous_ok && previous_tag+16'd1==tag1;
reg [7:0] character;
reg [15:0] value;
reg [6:0] col;
reg [3:0] group_id,offset;
// 8x8 glyph, doubled vertically. Numeric counts are explicitly hexadecimal.
function [63:0] font;
 input [7:0] c;
 begin case(c)
 "0":font=64'h0038444c54644438;"1":font=64'h0010301010101038;
 "2":font=64'h003844040830407c;"3":font=64'h0078040438040478;
 "4":font=64'h00081828487c0808;"5":font=64'h007c404078040438;
 "6":font=64'h0038404078444438;"7":font=64'h007c040810202020;
 "8":font=64'h0038444438444438;"9":font=64'h003844443c040438;
 "A":font=64'h003844447c444444;"B":font=64'h0078444478444478;
 "C":font=64'h003c40404040403c;"D":font=64'h0078444444444478;
 "E":font=64'h007c40407840407c;"F":font=64'h007c404078404040;
 "G":font=64'h003c40404c444438;"H":font=64'h004444447c444444;
 "I":font=64'h0038101010101038;"J":font=64'h001c080808484830;
 "K":font=64'h0044485060504844;"L":font=64'h004040404040407c;
 "M":font=64'h00446c5444444444;"N":font=64'h00446464544c4c44;
 "O":font=64'h0038444444444438;"P":font=64'h0078444478404040;
 "Q":font=64'h0038444444544834;"R":font=64'h0078444478504844;
 "S":font=64'h003c404038040478;"T":font=64'h007c101010101010;
 "U":font=64'h0044444444444438;"V":font=64'h0044444444442810;
 "W":font=64'h0044444454545428;"X":font=64'h0044442810284444;
 "Y":font=64'h0044442810101010;"Z":font=64'h007c04081020407c;
 ":":font=64'h0000101000101000;".":font=64'h0000000000001010;
 "+":font=64'h000010107c101000;"-":font=64'h000000007c000000;
 "x":font=64'h0000004428102844;
 "/":font=64'h0004080810202040;"=":font=64'h00007c007c000000;
 default:font=0;
 endcase end
endfunction
function [7:0] hexchar;input [3:0] n;begin hexchar=n<10?"0"+n:"A"+n-10;end endfunction
localparam [8*128-1:0] TITLE={"RAW WHITE / FILTER GREEN / DISPLAY YELLOW    5.12S WINDOW  10MS/BIN  +180 TOP  -180 BOTTOM  1DEG/PIX",{28{8'd32}}};
localparam [8*128-1:0] SUBTITLE={"DT CYAN: 0..50MS  1MS/PIX   RED=EVENT/GAP  MAGENTA=ANGLE WRAP   COUNTERS HEX, DT/MAX/AGE DEC x0.1MS",{29{8'd32}}};
// Footer fixed 12-character cells: label + four hex digits.
localparam [8*96-1:0] FOOT0="NACK:       BUS:        PKT:        ACC0:       REJ:        GAP:        STL:        DIAG:       ";
localparam [8*96-1:0] FOOT1="RX:         RETRY:      DT:         MAX:        AGE:        LOST:       ACTIVE:     ONLINE:     ";
localparam [8*32-1:0] EXTRA="FRAME LOST:         S4 HIDE      ";
always @*begin
 col=lx[9:3];character=" ";value=0;group_id=0;offset=0;
 if(col<12)begin group_id=0;offset=col;end
 else if(col<24)begin group_id=1;offset=col-12;end
 else if(col<36)begin group_id=2;offset=col-24;end
 else if(col<48)begin group_id=3;offset=col-36;end
 else if(col<60)begin group_id=4;offset=col-48;end
 else if(col<72)begin group_id=5;offset=col-60;end
 else if(col<84)begin group_id=6;offset=col-72;end
 else begin group_id=7;offset=col-84;end
 if(y1<496)character=TITLE[(127-col)*8+:8];
 else if(y1<512)character=SUBTITLE[(127-col)*8+:8];
 else if(y1>=960 && col>=96)begin
  character=EXTRA[(127-col)*8+:8];
  if(col>=108 && col<112)character=hexchar(shown[368+(111-col)*4+:4]);
 end else if(y1>=960 && col<96)begin
  if(y1<976)begin
   character=FOOT0[(95-col)*8+:8];
   case(group_id)
    0:value=shown[163:148];1:value=shown[179:164];2:value=shown[195:180];3:value=shown[211:196];
    4:value=shown[227:212];5:value=shown[243:228];6:value=shown[259:244];7:value=shown[275:260];
   endcase
  end else begin
   character=FOOT1[(95-col)*8+:8];
   case(group_id)
    0:value=shown[291:276];1:value=shown[355:340];2:value=shown_digits[15:0];3:value=shown_digits[31:16];
    4:value=shown_digits[47:32];5:value=shown[339:324];6:value={15'd0,shown[137]};7:value={15'd0,shown[356]};
   endcase
  end
  if(offset>=8)character=hexchar(value[(11-offset)*4+:4]);
 end
end
// Register independent paint predicates before priority/color selection.
// Do not combine BRAM output, segment tests and RGB mux in one pixel cycle.
reg [6:0] paint;
always @*begin
 paint=0;
 if(y1>=512 && y1<960)begin
  if(lx[6:0]==0 || y1==520 || y1==610 || y1==700 || y1==790 || y1==880 || y1==900 || y1==940 || y1==950)paint[0]=1;
  if(point_valid)begin
   if(epoch_ok && flags[0] && y1>=520 && y1<=880)begin
    if(!flags[3] && y1>=raw_top && y1<=raw_bottom)paint[1]=1;
    else if(flags[3] && (y1==raw_top || y1==raw_bottom))paint[1]=1;
    if(connect && previous_flags[0] && !flags[3] && segment(y1,raw_top,previous_raw))paint[1]=1;
   end
   // Separate subcolumns keep coincident traces visible.
   if(epoch_ok && flags[1] && lx[0]==0 && (y1==green_y || (connect && previous_flags[1] && segment(y1,green_y,previous_green))))paint[2]=1;
   if(epoch_ok && flags[2] && lx[0]==1 && (y1==yellow_y || (connect && previous_flags[2] && segment(y1,yellow_y,previous_yellow))))paint[3]=1;
   if(flags[7] && y1==950-dth)paint[4]=1;
   if(flags[4] && y1>=888 && y1<892)paint[5]=1;
   if(flags[3] && y1>=884 && y1<888)paint[6]=1;
  end
 end
end
reg [23:0] rgb2,text_color2;
reg [6:0] paint2;
reg overlay2;
reg [2:0] ctl2;
reg text2;
reg [7:0] char2;
reg [5:0] glyph_index2;
wire [63:0] glyph2=font(char2);
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin rgb2<=0;ctl2<=0;text2<=0;char2<=0;glyph_index2<=0;text_color2<=0;paint2<=0;overlay2<=0;end
 else begin
  rgb2<=rgb1;paint2<=paint;overlay2<=shown_visible && ctl1[0] && box;
  ctl2<=ctl1;text2<=shown_visible && ctl1[0] && box && (y1<512 || y1>=960);
  char2<=character;glyph_index2<={~y1[3:1],~lx[2:0]};text_color2<=24'hc0c8d0;
  if(y1<496)begin
   if(lx<80)text_color2<=24'hffffff;
   else if(lx>=96 && lx<208)text_color2<=24'h50ff70;
   else if(lx>=224 && lx<352)text_color2<=24'hffe050;
  end
 end
reg [23:0] color2,rgb3;
reg [2:0] ctl3;
always @*begin
 color2=24'h101820;
 if(paint2[0])color2=24'h304050;
 if(paint2[1])color2=24'hffffff;
 if(paint2[2])color2=24'h50ff70;
 if(paint2[3])color2=24'hffe050;
 if(paint2[4])color2=24'h50e0ff;
 if(paint2[5])color2=24'hff4040;
 if(paint2[6])color2=24'hff60ff;
end
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin rgb3<=0;ctl3<=0;end
 else begin
  rgb3<=text2 && glyph2[glyph_index2]?text_color2:(overlay2?color2:rgb2);
  ctl3<=ctl2;
 end
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin out_r<=0;out_g<=0;out_b<=0;out_vs<=0;out_hs<=0;out_de<=0;end
 else begin
  {out_r,out_g,out_b}<=rgb3;
  {out_vs,out_hs,out_de}<=ctl3;
 end
endmodule

// Decimal conversion off the pixel path: 16 cycles, three clamped 4-digit values.
module diagnostic_decimal(input clk,rst_n,start,input [47:0] values,output reg [47:0] digits);
reg [31:0] work[0:2];reg [4:0] step;reg busy;
reg [31:0] adjusted[0:2];
integer i,j;
always @*begin
 for(i=0;i<3;i=i+1)begin
  adjusted[i]=work[i];
  for(j=0;j<4;j=j+1)if(work[i][16+j*4+:4]>=5)adjusted[i][16+j*4+:4]=work[i][16+j*4+:4]+4'd3;
 end
end
integer k;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin busy<=0;step<=0;digits<=0;for(k=0;k<3;k=k+1)work[k]<=0;end
 else if(start && !busy)begin
  for(k=0;k<3;k=k+1)work[k]<={16'd0,(values[k*16+:16]>9999?16'd9999:values[k*16+:16])};
  busy<=1;step<=0;
 end else if(busy)begin
  for(k=0;k<3;k=k+1)work[k]<=adjusted[k]<<1;
  step<=step+1'b1;
  if(step==15)begin
   busy<=0;for(k=0;k<3;k=k+1)digits[k*16+:16]<=adjusted[k][30:15];
  end
 end
endmodule
