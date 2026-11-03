// Exact signed 32x32 product. Three enabled edges, initiation interval one.
// Input at edge N -> product/valid at N+2; ce=0 freezes every stage.
// The low halves are UNSIGNED; the high halves are SIGNED. This distinction
// is essential for negative operands and the full -2^31..2^31-1 domain.
// General multiplication uses three 18x18-or-smaller DSP products:
// cross=(ahi+alo)*(bhi+blo)-ahi*bhi-alo*blo.
// Squares use the cheaper direct cross=2*ahi*alo (also three products).
module signed_mul32_pipe #(parameter SQUARE=0)(
 input wire clk,rst_n,ce,in_valid,
 input wire signed [31:0] a,b,
 output wire out_valid,output reg signed [63:0] product);
 reg [2:0] valid_pipe;
 reg [31:0] low_product;
 reg signed [31:0] high_product;
 reg signed [63:0] base;
 reg signed [32:0] cross;
 wire signed [15:0] ahi=a[31:16],bhi=b[31:16];
 wire signed [17:0] asum=$signed({2'b0,a[15:0]})+$signed({{2{ahi[15]}},ahi});
 wire signed [17:0] bsum=$signed({2'b0,b[15:0]})+$signed({{2{bhi[15]}},bhi});
 assign out_valid=valid_pipe[2];
 generate if(SQUARE)begin: square_product
  reg signed [31:0] half_cross;
  always @(posedge clk or negedge rst_n)
   if(!rst_n)begin low_product<=0;high_product<=0;half_cross<=0;cross<=0;end
   else if(ce)begin
    low_product<=a[15:0]*a[15:0];
    high_product<=ahi*ahi;
    half_cross<=ahi*$signed({1'b0,a[15:0]});
    cross<=$signed({half_cross[31],half_cross})<<<1;
   end
 end else begin: general_product
  reg signed [35:0] middle;
  wire signed [35:0] cross_wide=middle-$signed({4'b0,low_product})-
                               $signed({{4{high_product[31]}},high_product});
  always @(posedge clk or negedge rst_n)
   if(!rst_n)begin low_product<=0;high_product<=0;middle<=0;cross<=0;end
   else if(ce)begin
    low_product<=a[15:0]*b[15:0];
    high_product<=ahi*bhi;
    middle<=asum*bsum;
    cross<=cross_wide[32:0]; // proven range [-2^32,2^32-1]
   end
 end endgenerate
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin valid_pipe<=0;base<=0;product<=0;end
  else if(ce)begin
   valid_pipe<={valid_pipe[1:0],in_valid};
   base<=$signed({high_product,low_product});
   product<=base+($signed({{31{cross[32]}},cross})<<<16);
  end
endmodule
