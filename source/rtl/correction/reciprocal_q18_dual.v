// One immutable reciprocal table, two simultaneous synchronous read ports.
// Both geometry lanes keep one lookup per clock and one-cycle read latency.
// No async reset on the ROM output registers: infer a shared dual-port DRM.
// Consumers qualify startup data with their existing reset/valid pipelines.
module reciprocal_q18_dual #(parameter FILE="source/rtl/correction/reciprocal_q18.mem")(
 input wire clk,input wire [12:0] address0,address1,
 output reg [17:0] data0,data1);
 reg [17:0] table_data[0:8191] /* synthesis syn_romstyle="block_rom" */;
 initial $readmemh(FILE,table_data);
 always @(posedge clk)begin data0<=table_data[address0];data1<=table_data[address1];end
endmodule
