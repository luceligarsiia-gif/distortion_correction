// HDMI camera 1920x1080; calibrated 2026-10-24 with CALIB_FIX_K3.
// Calibration constants originate from prepare_calibration.py; K_out=K_in.
// FX/FY/CX/CY: 12 fractional bits. INVF and k/p: 28 fractional bits.
// Radial model: 1 + K1*r^2 + K2*r^4; K3 is fixed to zero.
// 31 enabled-edge stages: input at edge N -> output at edge N+30.
// ce=0 freezes ALL pipeline state. Consumer accepts output only when ce && out_valid.
// Three-stage exact multiplication is factored into signed_mul32_pipe.
// The following rounding register is the unchanged fourth product stage.
// Squared operands share the signed high*unsigned low partial product:
// (hi*2^16+lo)^2 = lo^2 + 2*(hi*lo)*2^16 + hi^2*2^32.
// Full products are reconstructed before rounding; the 31-stage latency,
// complete signed input domain and one-input-per-clock rate are preserved.
module distortion_map #(
 parameter NORMALIZED_INPUT=0,
 parameter integer IMAGE_WIDTH=1920, IMAGE_HEIGHT=1080,
 parameter signed [31:0] FX=32'sd3156864,
 parameter signed [31:0] FY=32'sd3157686,
 parameter signed [31:0] CX=32'sd3926176,
 parameter signed [31:0] CY=32'sd2194230,
 parameter signed [31:0] INVFX=32'sd348292,
 parameter signed [31:0] INVFY=32'sd348202,
 parameter signed [31:0] K1=-32'sd82217307,
 parameter signed [31:0] K2=32'sd22613731,
 parameter signed [31:0] P1=32'sd61012,
 parameter signed [31:0] P2=32'sd423309
)(
 input wire clk, rst_n, ce, in_valid,
 input wire [15:0] in_u, in_v,
 output wire out_valid,
 output wire signed [31:0] out_u, out_v,
 output wire center_outside,
 input signed [31:0] normalized_x,normalized_y
);
 function signed [31:0] add32;
 input signed [31:0] a,b;
 reg signed [32:0] wide;
 begin
 wide={a[31],a}+{b[31],b};
 // synthesis translate_off
 if ((^wide !== 1'bx) && (wide[32] != wide[31])) begin
 $display("FAIL: add overflow"); $finish;
 end
 // synthesis translate_on
 add32=wide[31:0];
 end
 endfunction
 function signed [31:0] round2;
 input signed [31:0] a;
 reg signed [32:0] wide;
 begin
 wide={a[31],a};
 round2=(wide+33'sd2) >>> 2;
 end
 endfunction
 reg [30:0] valid_pipe;
 reg signed [31:0] normx0,normx1,normx2,normy0,normy1,normy2;
reg signed [31:0] x_2_delay0;
reg signed [31:0] x_2_delay1;
reg signed [31:0] x_2_delay2;
reg signed [31:0] y_2_delay0;
reg signed [31:0] y_2_delay1;
reg signed [31:0] y_2_delay2;
reg signed [31:0] ax_4_delay0;
reg signed [31:0] ax_4_delay1;
reg signed [31:0] ax_4_delay2;
reg signed [31:0] ay_4_delay0;
reg signed [31:0] ay_4_delay1;
reg signed [31:0] ay_4_delay2;
reg signed [31:0] x_4_delay0;
reg signed [31:0] x_4_delay1;
reg signed [31:0] x_4_delay2;
reg signed [31:0] y_4_delay0;
reg signed [31:0] y_4_delay1;
reg signed [31:0] y_4_delay2;
reg signed [31:0] tx_4_delay0;
reg signed [31:0] tx_4_delay1;
reg signed [31:0] tx_4_delay2;
reg signed [31:0] rad1_5_delay0;
reg signed [31:0] rad1_5_delay1;
reg signed [31:0] rad1_5_delay2;
reg signed [31:0] x_5_delay0;
reg signed [31:0] x_5_delay1;
reg signed [31:0] x_5_delay2;
reg signed [31:0] y_5_delay0;
reg signed [31:0] y_5_delay1;
reg signed [31:0] y_5_delay2;
reg signed [31:0] t1x_7_delay0;
reg signed [31:0] t1x_7_delay1;
reg signed [31:0] t1x_7_delay2;
reg signed [31:0] t2x_7_delay0;
reg signed [31:0] t2x_7_delay1;
reg signed [31:0] t2x_7_delay2;
reg signed [31:0] t1y_7_delay0;
reg signed [31:0] t1y_7_delay1;
reg signed [31:0] t1y_7_delay2;
reg signed [31:0] t2y_7_delay0;
reg signed [31:0] t2y_7_delay1;
reg signed [31:0] t2y_7_delay2;
 reg signed [31:0] dx_0;
 reg signed [31:0] dy_0;
 reg signed [31:0] x_1;
 reg signed [31:0] x_2;
 reg signed [31:0] x_3;
 reg signed [31:0] x_4;
 reg signed [31:0] x_5;
 reg signed [31:0] x_6;
 reg signed [31:0] y_1;
 reg signed [31:0] y_2;
 reg signed [31:0] y_3;
 reg signed [31:0] y_4;
 reg signed [31:0] y_5;
 reg signed [31:0] y_6;
 reg signed [31:0] x2_2;
 reg signed [31:0] y2_2;
 reg signed [31:0] xy_2;
 reg signed [31:0] r2_3;
 reg signed [31:0] tx_3;
 reg signed [31:0] tx_4;
 reg signed [31:0] xx_3;
 reg signed [31:0] yy_3;
 reg signed [31:0] r4_4;
 reg signed [31:0] ax_4;
 reg signed [31:0] ay_4;
 reg signed [31:0] kr1_4;
 reg signed [31:0] kr2_5;
 reg signed [31:0] rad1_5;
 reg signed [31:0] t1x_5;
 reg signed [31:0] t1x_6;
 reg signed [31:0] t1x_7;
 reg signed [31:0] t2x_5;
 reg signed [31:0] t2x_6;
 reg signed [31:0] t2x_7;
 reg signed [31:0] t2x_8;
 reg signed [31:0] t1y_5;
 reg signed [31:0] t1y_6;
 reg signed [31:0] t1y_7;
 reg signed [31:0] t2y_5;
 reg signed [31:0] t2y_6;
 reg signed [31:0] t2y_7;
 reg signed [31:0] t2y_8;
 reg signed [31:0] rad_6;
 reg signed [31:0] xr_7;
 reg signed [31:0] yr_7;
 reg signed [31:0] xd0_8;
 reg signed [31:0] yd0_8;
 reg signed [31:0] xd_9 /* synthesis syn_preserve=1 syn_keep=1 */;
 reg signed [31:0] yd_9 /* synthesis syn_preserve=1 syn_keep=1 */;
 reg signed [31:0] px_10;
 reg signed [31:0] py_10;
 reg signed [31:0] us_11;
 reg signed [31:0] vs_11;
 reg signed [31:0] uq_12;
 reg signed [31:0] vq_12;
 assign out_valid=valid_pipe[30];
 assign out_u=uq_12;
 assign out_v=vq_12;
 assign center_outside=(out_u < 0) || (out_v < 0)
   || (out_u > ((IMAGE_WIDTH-1)*1024)) || (out_v > ((IMAGE_HEIGHT-1)*1024));
 wire signed [63:0] x_1_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_x_1(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(dx_0),.b(INVFX),.out_valid(),.product(x_1_pipe_prod));
wire signed [63:0] y_1_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_y_1(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(dy_0),.b(INVFY),.out_valid(),.product(y_1_pipe_prod));
wire signed [63:0] x2_2_pipe_prod;
signed_mul32_pipe #(.SQUARE(1)) multiply_x2_2(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(x_1),.b(x_1),.out_valid(),.product(x2_2_pipe_prod));
wire signed [63:0] y2_2_pipe_prod;
signed_mul32_pipe #(.SQUARE(1)) multiply_y2_2(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(y_1),.b(y_1),.out_valid(),.product(y2_2_pipe_prod));
wire signed [63:0] xy_2_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_xy_2(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(x_1),.b(y_1),.out_valid(),.product(xy_2_pipe_prod));
wire signed [63:0] r4_4_pipe_prod;
signed_mul32_pipe #(.SQUARE(1)) multiply_r4_4(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(r2_3),.b(r2_3),.out_valid(),.product(r4_4_pipe_prod));
wire signed [63:0] kr1_4_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_kr1_4(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(K1),.b(r2_3),.out_valid(),.product(kr1_4_pipe_prod));
wire signed [63:0] kr2_5_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_kr2_5(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(K2),.b(r4_4),.out_valid(),.product(kr2_5_pipe_prod));
wire signed [63:0] t1x_5_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_t1x_5(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(P1),.b(tx_4),.out_valid(),.product(t1x_5_pipe_prod));
wire signed [63:0] t2x_5_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_t2x_5(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(P2),.b(ax_4),.out_valid(),.product(t2x_5_pipe_prod));
wire signed [63:0] t1y_5_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_t1y_5(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(P1),.b(ay_4),.out_valid(),.product(t1y_5_pipe_prod));
wire signed [63:0] t2y_5_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_t2y_5(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(P2),.b(tx_4),.out_valid(),.product(t2y_5_pipe_prod));
wire signed [63:0] xr_7_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_xr_7(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(x_6),.b(rad_6),.out_valid(),.product(xr_7_pipe_prod));
wire signed [63:0] yr_7_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_yr_7(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(y_6),.b(rad_6),.out_valid(),.product(yr_7_pipe_prod));
wire signed [63:0] px_10_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_px_10(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(FX),.b(xd_9),.out_valid(),.product(px_10_pipe_prod));
wire signed [63:0] py_10_pipe_prod;
signed_mul32_pipe #(.SQUARE(0)) multiply_py_10(
 .clk(clk),.rst_n(rst_n),.ce(ce),.in_valid(1'b1),
 .a(FY),.b(yd_9),.out_valid(),.product(py_10_pipe_prod));
always @(posedge clk or negedge rst_n) begin
 if (!rst_n) begin
 valid_pipe <= 0;
 normx0<=0;normx1<=0;normx2<=0;normy0<=0;normy1<=0;normy2<=0;
x_2_delay0 <= 0;
x_2_delay1 <= 0;
x_2_delay2 <= 0;
y_2_delay0 <= 0;
y_2_delay1 <= 0;
y_2_delay2 <= 0;
ax_4_delay0 <= 0;
ax_4_delay1 <= 0;
ax_4_delay2 <= 0;
ay_4_delay0 <= 0;
ay_4_delay1 <= 0;
ay_4_delay2 <= 0;
x_4_delay0 <= 0;
x_4_delay1 <= 0;
x_4_delay2 <= 0;
y_4_delay0 <= 0;
y_4_delay1 <= 0;
y_4_delay2 <= 0;
tx_4_delay0 <= 0;
tx_4_delay1 <= 0;
tx_4_delay2 <= 0;
rad1_5_delay0 <= 0;
rad1_5_delay1 <= 0;
rad1_5_delay2 <= 0;
x_5_delay0 <= 0;
x_5_delay1 <= 0;
x_5_delay2 <= 0;
y_5_delay0 <= 0;
y_5_delay1 <= 0;
y_5_delay2 <= 0;
t1x_7_delay0 <= 0;
t1x_7_delay1 <= 0;
t1x_7_delay2 <= 0;
t2x_7_delay0 <= 0;
t2x_7_delay1 <= 0;
t2x_7_delay2 <= 0;
t1y_7_delay0 <= 0;
t1y_7_delay1 <= 0;
t1y_7_delay2 <= 0;
t2y_7_delay0 <= 0;
t2y_7_delay1 <= 0;
t2y_7_delay2 <= 0;
 dx_0 <= 0;
 dy_0 <= 0;
 x_1 <= 0;
 x_2 <= 0;
 x_3 <= 0;
 x_4 <= 0;
 x_5 <= 0;
 x_6 <= 0;
 y_1 <= 0;
 y_2 <= 0;
 y_3 <= 0;
 y_4 <= 0;
 y_5 <= 0;
 y_6 <= 0;
 x2_2 <= 0;
 y2_2 <= 0;
 xy_2 <= 0;
 r2_3 <= 0;
 tx_3 <= 0;
 tx_4 <= 0;
 xx_3 <= 0;
 yy_3 <= 0;
 r4_4 <= 0;
 ax_4 <= 0;
 ay_4 <= 0;
 kr1_4 <= 0;
 kr2_5 <= 0;
 rad1_5 <= 0;
 t1x_5 <= 0;
 t1x_6 <= 0;
 t1x_7 <= 0;
 t2x_5 <= 0;
 t2x_6 <= 0;
 t2x_7 <= 0;
 t2x_8 <= 0;
 t1y_5 <= 0;
 t1y_6 <= 0;
 t1y_7 <= 0;
 t2y_5 <= 0;
 t2y_6 <= 0;
 t2y_7 <= 0;
 t2y_8 <= 0;
 rad_6 <= 0;
 xr_7 <= 0;
 yr_7 <= 0;
 xd0_8 <= 0;
 yd0_8 <= 0;
 xd_9 <= 0;
 yd_9 <= 0;
 px_10 <= 0;
 py_10 <= 0;
 us_11 <= 0;
 vs_11 <= 0;
 uq_12 <= 0;
 vq_12 <= 0;
 end else if (ce) begin
 valid_pipe <= {valid_pipe[29:0],in_valid};
 // Stage 1
 dx_0 <= NORMALIZED_INPUT ? normalized_x : add32($signed({4'b0,in_u,12'b0}), -CX);
 normx0<=dx_0;normx1<=normx0;normx2<=normx1;
 dy_0 <= NORMALIZED_INPUT ? normalized_y : add32($signed({4'b0,in_v,12'b0}), -CY);
 normy0<=dy_0;normy1<=normy0;normy2<=normy1;
 // Original stage 2: four registered steps
 x_1 <= NORMALIZED_INPUT ? normx2 : ((x_1_pipe_prod+(64'sd1 << (12-1))) >>> 12);
 y_1 <= NORMALIZED_INPUT ? normy2 : ((y_1_pipe_prod+(64'sd1 << (12-1))) >>> 12);
 // Original stage 3: four registered steps
 x2_2 <= (x2_2_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 y2_2 <= (y2_2_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 xy_2 <= (xy_2_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 x_2_delay0 <= x_1;
 x_2_delay1 <= x_2_delay0;
 x_2_delay2 <= x_2_delay1;
 x_2 <= x_2_delay2;
 y_2_delay0 <= y_1;
 y_2_delay1 <= y_2_delay0;
 y_2_delay2 <= y_2_delay1;
 y_2 <= y_2_delay2;
 // Stage 4
 r2_3 <= add32(x2_2,y2_2);
 tx_3 <= add32(xy_2,xy_2);
 xx_3 <= add32(x2_2,x2_2);
 yy_3 <= add32(y2_2,y2_2);
 x_3 <= x_2;
 y_3 <= y_2;
 // Original stage 5: four registered steps
 r4_4 <= (r4_4_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 ax_4_delay0 <= add32(r2_3,xx_3);
 ax_4_delay1 <= ax_4_delay0;
 ax_4_delay2 <= ax_4_delay1;
 ax_4 <= ax_4_delay2;
 ay_4_delay0 <= add32(r2_3,yy_3);
 ay_4_delay1 <= ay_4_delay0;
 ay_4_delay2 <= ay_4_delay1;
 ay_4 <= ay_4_delay2;
 kr1_4 <= (kr1_4_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 x_4_delay0 <= x_3;
 x_4_delay1 <= x_4_delay0;
 x_4_delay2 <= x_4_delay1;
 x_4 <= x_4_delay2;
 y_4_delay0 <= y_3;
 y_4_delay1 <= y_4_delay0;
 y_4_delay2 <= y_4_delay1;
 y_4 <= y_4_delay2;
 tx_4_delay0 <= tx_3;
 tx_4_delay1 <= tx_4_delay0;
 tx_4_delay2 <= tx_4_delay1;
 tx_4 <= tx_4_delay2;
 // Original stage 6: four registered steps
 kr2_5 <= (kr2_5_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 rad1_5_delay0 <= add32(32'sd268435456,kr1_4);
 rad1_5_delay1 <= rad1_5_delay0;
 rad1_5_delay2 <= rad1_5_delay1;
 rad1_5 <= rad1_5_delay2;
 t1x_5 <= (t1x_5_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 t2x_5 <= (t2x_5_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 t1y_5 <= (t1y_5_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 t2y_5 <= (t2y_5_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 x_5_delay0 <= x_4;
 x_5_delay1 <= x_5_delay0;
 x_5_delay2 <= x_5_delay1;
 x_5 <= x_5_delay2;
 y_5_delay0 <= y_4;
 y_5_delay1 <= y_5_delay0;
 y_5_delay2 <= y_5_delay1;
 y_5 <= y_5_delay2;
 // Stage 7
 rad_6 <= add32(rad1_5,kr2_5);
 x_6 <= x_5;
 y_6 <= y_5;
 t1x_6 <= t1x_5;
 t2x_6 <= t2x_5;
 t1y_6 <= t1y_5;
 t2y_6 <= t2y_5;
 // Original stage 8: four registered steps
 xr_7 <= (xr_7_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 yr_7 <= (yr_7_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 t1x_7_delay0 <= t1x_6;
 t1x_7_delay1 <= t1x_7_delay0;
 t1x_7_delay2 <= t1x_7_delay1;
 t1x_7 <= t1x_7_delay2;
 t2x_7_delay0 <= t2x_6;
 t2x_7_delay1 <= t2x_7_delay0;
 t2x_7_delay2 <= t2x_7_delay1;
 t2x_7 <= t2x_7_delay2;
 t1y_7_delay0 <= t1y_6;
 t1y_7_delay1 <= t1y_7_delay0;
 t1y_7_delay2 <= t1y_7_delay1;
 t1y_7 <= t1y_7_delay2;
 t2y_7_delay0 <= t2y_6;
 t2y_7_delay1 <= t2y_7_delay0;
 t2y_7_delay2 <= t2y_7_delay1;
 t2y_7 <= t2y_7_delay2;
 // Stage 9
 xd0_8 <= add32(xr_7,t1x_7);
 yd0_8 <= add32(yr_7,t1y_7);
 t2x_8 <= t2x_7;
 t2y_8 <= t2y_7;
 // Stage 10
 xd_9 <= add32(xd0_8,t2x_8);
 yd_9 <= add32(yd0_8,t2y_8);
 // Original stage 11: four registered steps
 px_10 <= (px_10_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 py_10 <= (py_10_pipe_prod+(64'sd1 << (28-1))) >>> 28;
 // Stage 12
 us_11 <= add32(px_10,CX);
 vs_11 <= add32(py_10,CY);
 // Stage 13
 uq_12 <= round2(us_11);
 vq_12 <= round2(vs_11);
 end
 end
endmodule

