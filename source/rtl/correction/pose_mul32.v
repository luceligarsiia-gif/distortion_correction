// Rounded signed product, four enabled edges and one input per clock.
// The reusable exact core provides the full product at N+2; this wrapper
// applies the original one-time round/shift at N+3. No saturation is added.
module pose_mul32 #(parameter SHIFT=14)(input clk,rst_n,in_valid,
 input signed [31:0] a,b,output reg out_valid,output reg signed [31:0] result);
 wire valid_product;wire signed [63:0] product;
 signed_mul32_pipe multiply(.clk(clk),.rst_n(rst_n),.ce(1'b1),
  .in_valid(in_valid),.a(a),.b(b),.out_valid(valid_product),.product(product));
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin out_valid<=0;result<=0;end
  else begin out_valid<=valid_product;result<=(product+(64'sd1<<(SHIFT-1)))>>>SHIFT;end
endmodule
