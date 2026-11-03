// RGB565 bilinear interpolation, Q10 fractions, one pixel per enabled clock.
// Three enabled stages: horizontal Q10 -> vertical Q20 -> rounded RGB565.
// ce=0 freezes data, fractional Y and validity together. Invalid neighbors
// are supplied as zero by the tile cache; this module does not clamp them.
// Exact factorization: a*(1024-f)+b*f = (a<<10)+(b-a)*f.
// No intermediate rounding: six variable products per channel become three.
module bilinear565(input clk,rst_n,ce,in_valid,
 input [15:0] p00,p10,p01,p11,input [9:0] fx,fy,
 output out_valid,output [15:0] pixel);
reg [2:0] vp;
reg [9:0] fy1;
wire signed [10:0] weight_x=$signed({1'b0,fx});
wire signed [10:0] weight_y=$signed({1'b0,fy1});
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin vp<=0; fy1<=0; end
 else if(ce) begin vp<={vp[1:0],in_valid}; fy1<=fy; end
assign out_valid=vp[2];
genvar c;
generate for(c=0;c<3;c=c+1) begin: ch
 localparam B=(c==1)?6:5;
 localparam L=(c==0)?0:((c==1)?5:11);
 reg [B+9:0] h0,h1;
 reg [B+19:0] sum;
 reg [B-1:0] q;
 // Explicit zero/sign extension preserves negative differences while all
 // stored weighted sums remain unsigned and nonnegative.
 wire signed [B:0] dx0=$signed({1'b0,p10[L+:B]})-$signed({1'b0,p00[L+:B]});
 wire signed [B:0] dx1=$signed({1'b0,p11[L+:B]})-$signed({1'b0,p01[L+:B]});
 wire signed [B+11:0] mx0=dx0*weight_x;
 wire signed [B+11:0] mx1=dx1*weight_x;
 wire signed [B+11:0] horizontal0=$signed({2'b0,p00[L+:B],10'b0})+mx0;
 wire signed [B+11:0] horizontal1=$signed({2'b0,p01[L+:B],10'b0})+mx1;
 wire signed [B+10:0] dy=$signed({1'b0,h1})-$signed({1'b0,h0});
 wire signed [B+21:0] my=dy*weight_y;
 wire signed [B+21:0] vertical=$signed({2'b0,h0,10'b0})+my;
 wire [B+20:0] rounded={1'b0,sum}+27'd524288;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin h0<=0;h1<=0;sum<=0;q<=0;end
 else if(ce) begin
 h0<=horizontal0[B+9:0];
 h1<=horizontal1[B+9:0];
 sum<=vertical[B+19:0];
 q<=rounded[20+:B];
 end
 assign pixel[L+:B]=q;
end endgenerate
endmodule
