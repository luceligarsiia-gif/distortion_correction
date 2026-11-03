// Two 256 x 64 RGB565 tiles. Load and compute use different tile banks.
// Three read stages: RAM -> 8-lane sample selection -> parity/border output.
// Base x must be a multiple of 16; base y must be even.
// Each 256-bit DDR beat writes eight even and eight odd x samples.
module tile_cache_pose #(parameter USE_ADDRESS_HINTS=0)(input clk,rst_n,
 input load_bank,read_bank,
 input load_en,input [5:0] load_y,input [3:0] load_beat,input [255:0] load_data,
 input in_valid,input signed [31:0] uq,vq,input [10:0] base_x,base_y,
 output reg out_valid,output reg [15:0] p00 /* synthesis syn_preserve=1 syn_keep=1 */,
 output reg [15:0] p10 /* synthesis syn_preserve=1 syn_keep=1 */,
 output reg [15:0] p01 /* synthesis syn_preserve=1 syn_keep=1 */,
 output reg [15:0] p11 /* synthesis syn_preserve=1 syn_keep=1 */,
 output reg [9:0] fx,fy,output reg fault, input [17:0] address_hint);
wire signed [31:0] ix=uq>>>10, iy=vq>>>10;
wire signed [31:0] lx=ix-$signed({1'b0,base_x}), ly=iy-$signed({1'b0,base_y});
wire [8:0] xe=lx[0]?lx[8:0]+9'd1:lx[8:0];
wire [8:0] xo=lx[0]?lx[8:0]:lx[8:0]+9'd1;
wire [6:0] ye=ly[0]?ly[6:0]+7'd1:ly[6:0];
wire [6:0] yo=ly[0]?ly[6:0]:ly[6:0]+7'd1;
// Even-coordinate rounding is performed before coordinate RAM storage.
// Only narrow modulo subtraction remains on the RAM-to-RAM read path.
// Preserve the original path for callers with non-aligned bases.
wire aligned=base_x[3:0]==0 && !base_y[0];
wire [3:0] hint_xe=address_hint[7:4]-base_x[7:4];
wire [3:0] hint_xo=address_hint[3:0]-base_x[7:4];
wire [4:0] hint_ye=address_hint[17:13]-base_y[5:1];
wire [4:0] hint_yo=address_hint[12:8]-base_y[5:1];
wire [9:0] ae=USE_ADDRESS_HINTS && aligned?{read_bank,hint_ye,hint_xe}:{read_bank,ye[5:1],xe[7:4]};
wire [9:0] ao=USE_ADDRESS_HINTS && aligned?{read_bank,hint_ye,hint_xo}:{read_bank,ye[5:1],xo[7:4]};
wire [9:0] be=USE_ADDRESS_HINTS && aligned?{read_bank,hint_yo,hint_xe}:{read_bank,yo[5:1],xe[7:4]};
wire [9:0] bo=USE_ADDRESS_HINTS && aligned?{read_bank,hint_yo,hint_xo}:{read_bank,yo[5:1],xo[7:4]};
reg [127:0] m00[0:1023],m10[0:1023],m01[0:1023],m11[0:1023];
wire [9:0] wa={load_bank,load_y[5:1],load_beat};
wire [127:0] even_data,odd_data;
genvar i;
generate for(i=0;i<8;i=i+1) begin: unpack
 assign even_data[i*16+:16]=load_data[i*32+:16];
 assign odd_data[i*16+:16]=load_data[i*32+16+:16];
end endgenerate
reg [127:0] q00,q10,q01,q11;
always @(posedge clk) begin
 if(load_en && !load_y[0]) begin m00[wa]<=even_data;m10[wa]<=odd_data;end
 if(load_en && load_y[0]) begin m01[wa]<=even_data;m11[wa]<=odd_data;end
 if(in_valid) begin q00<=m00[ae];q10<=m10[ao];q01<=m01[be];q11<=m11[bo];end
end
reg v1,v2,xodd,yodd,xodd2,yodd2;
reg [15:0] a2,b2,c2,d2 /* synthesis syn_preserve=1 syn_keep=1 */;
reg [3:0] valid_neighbors2;
reg [9:0] fx2,fy2;
reg [2:0] le,lo;
reg [3:0] valid_neighbors;
reg [9:0] fx1,fy1;
wire [15:0] a=q00[le*16+:16],b=q10[lo*16+:16],c=q01[le*16+:16],d=q11[lo*16+:16];
wire [15:0] top0=yodd2?c2:a2, top1=yodd2?d2:b2, bot0=yodd2?a2:c2, bot1=yodd2?b2:d2;
always @(posedge clk or negedge rst_n)
if(!rst_n) begin v1<=0;v2<=0;out_valid<=0;fault<=0;valid_neighbors<=0;valid_neighbors2<=0;
 xodd2<=0;yodd2<=0;a2<=0;b2<=0;c2<=0;d2<=0;fx2<=0;fy2<=0;
 xodd<=0;yodd<=0;le<=0;lo<=0;fx1<=0;fy1<=0;fx<=0;fy<=0;p00<=0;p10<=0;p01<=0;p11<=0;end
else begin
 v1<=in_valid;v2<=v1;out_valid<=v2;
 if(in_valid) begin
 xodd<=lx[0];yodd<=ly[0];le<=xe[3:1];lo<=xo[3:1];fx1<=uq[9:0];fy1<=vq[9:0];
 valid_neighbors[0]<=ix>=0 && ix<1920 && iy>=0 && iy<1080;
 valid_neighbors[1]<=ix+1>=0 && ix+1<1920 && iy>=0 && iy<1080;
 valid_neighbors[2]<=ix>=0 && ix<1920 && iy+1>=0 && iy+1<1080;
 valid_neighbors[3]<=ix+1>=0 && ix+1<1920 && iy+1>=0 && iy+1<1080;
 // Calibration/bounds mismatch must invalidate the frame, not silently wrap RAM.
 if((ix>=0 && ix<1920 && iy>=0 && iy<1080 && (lx<0 || lx>=256 || ly<0 || ly>=64)) ||
    (ix+1>=0 && ix+1<1920 && iy>=0 && iy<1080 && (lx+1<0 || lx+1>=256 || ly<0 || ly>=64)) ||
    (ix>=0 && ix<1920 && iy+1>=0 && iy+1<1080 && (lx<0 || lx>=256 || ly+1<0 || ly+1>=64)) ||
    (ix+1>=0 && ix+1<1920 && iy+1>=0 && iy+1<1080 && (lx+1<0 || lx+1>=256 || ly+1<0 || ly+1>=64))) fault<=1;
 end
 if(v1)begin
  a2<=a;b2<=b;c2<=c;d2<=d;xodd2<=xodd;yodd2<=yodd;
  valid_neighbors2<=valid_neighbors;fx2<=fx1;fy2<=fy1;
 end
 if(v2) begin
 fx<=fx2;fy<=fy2;
 p00<=valid_neighbors2[0]?(xodd2?top1:top0):16'd0;
 p10<=valid_neighbors2[1]?(xodd2?top0:top1):16'd0;
 p01<=valid_neighbors2[2]?(xodd2?bot1:bot0):16'd0;
 p11<=valid_neighbors2[3]?(xodd2?bot0:bot1):16'd0;
 end
end
endmodule
