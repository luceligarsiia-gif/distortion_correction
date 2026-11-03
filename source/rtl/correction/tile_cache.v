// Two 256 x 32 RGB565 tiles. Load and compute use different tile banks.
// Base x must be a multiple of 16; base y must be even.
// Each 256-bit DDR beat writes eight even and eight odd x samples.
module tile_cache(input clk,rst_n,
 input load_bank,read_bank,
 input load_en,input [4:0] load_y,input [3:0] load_beat,input [255:0] load_data,
 input in_valid,input signed [31:0] uq,vq,input [10:0] base_x,base_y,
 output reg out_valid,output reg [15:0] p00,p10,p01,p11,
 output reg [9:0] fx,fy,output reg fault);
wire signed [31:0] ix=uq>>>10, iy=vq>>>10;
wire signed [31:0] lx=ix-$signed({1'b0,base_x}), ly=iy-$signed({1'b0,base_y});
wire [8:0] xe=lx[0]?lx[8:0]+9'd1:lx[8:0];
wire [8:0] xo=lx[0]?lx[8:0]:lx[8:0]+9'd1;
wire [5:0] ye=ly[0]?ly[5:0]+6'd1:ly[5:0];
wire [5:0] yo=ly[0]?ly[5:0]:ly[5:0]+6'd1;
wire [8:0] ae={read_bank,ye[4:1],xe[7:4]}, ao={read_bank,ye[4:1],xo[7:4]};
wire [8:0] be={read_bank,yo[4:1],xe[7:4]}, bo={read_bank,yo[4:1],xo[7:4]};
reg [127:0] m00[0:511],m10[0:511],m01[0:511],m11[0:511];
wire [8:0] wa={load_bank,load_y[4:1],load_beat};
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
reg v1,xodd,yodd;
reg [2:0] le,lo;
reg [3:0] valid_neighbors;
reg [9:0] fx1,fy1;
wire [15:0] a=q00[le*16+:16],b=q10[lo*16+:16],c=q01[le*16+:16],d=q11[lo*16+:16];
wire [15:0] top0=yodd?c:a, top1=yodd?d:b, bot0=yodd?a:c, bot1=yodd?b:d;
always @(posedge clk or negedge rst_n)
if(!rst_n) begin v1<=0;out_valid<=0;fault<=0;valid_neighbors<=0;
 xodd<=0;yodd<=0;le<=0;lo<=0;fx1<=0;fy1<=0;fx<=0;fy<=0;p00<=0;p10<=0;p01<=0;p11<=0;end
else begin
 v1<=in_valid;out_valid<=v1;
 if(in_valid) begin
 xodd<=lx[0];yodd<=ly[0];le<=xe[3:1];lo<=xo[3:1];fx1<=uq[9:0];fy1<=vq[9:0];
 valid_neighbors[0]<=ix>=0 && ix<1920 && iy>=0 && iy<1080;
 valid_neighbors[1]<=ix+1>=0 && ix+1<1920 && iy>=0 && iy<1080;
 valid_neighbors[2]<=ix>=0 && ix<1920 && iy+1>=0 && iy+1<1080;
 valid_neighbors[3]<=ix+1>=0 && ix+1<1920 && iy+1>=0 && iy+1<1080;
 // Calibration/bounds mismatch must invalidate the frame, not silently wrap RAM.
 if((ix>=0 && ix<1920 && (lx<0 || lx>=256)) ||
    (ix+1>=0 && ix+1<1920 && (lx+1<0 || lx+1>=256)) ||
    (iy>=0 && iy<1080 && (ly<0 || ly>=32)) ||
    (iy+1>=0 && iy+1<1080 && (ly+1<0 || ly+1>=32))) fault<=1;
 end
 if(v1) begin
 fx<=fx1;fy<=fy1;
 p00<=valid_neighbors[0]?(xodd?top1:top0):16'd0;
 p10<=valid_neighbors[1]?(xodd?top0:top1):16'd0;
 p01<=valid_neighbors[2]?(xodd?bot1:bot0):16'd0;
 p11<=valid_neighbors[3]?(xodd?bot0:bot1):16'd0;
 end
end
endmodule
