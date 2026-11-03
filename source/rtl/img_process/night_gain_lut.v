// Exactly the original 129x256 Q8 gain table, one synchronous lookup cycle.
// A 33024-word array rounds up to a much larger native ROM allocation.
// Split it into 32768 words plus the final 256-word curve instead. The two
// reads and bank-select register occur in parallel; no pixel stage is added.
// prepare_tables.tcl derives both files from the original table, without
// modifying any value. Custom FILE overrides retain the legacy read path.
module night_gain_lut #(
 parameter FILE="source/rtl/img_process/night_gain_q8.mem",
 parameter BULK_FILE="source/rtl/img_process/night_gain_bulk_q8.mem",
 parameter LAST_FILE="source/rtl/img_process/night_gain_last_q8.mem"
)(input wire clk,input wire [7:0] strength,luma,output wire [11:0] gain);
 generate if(FILE=="source/rtl/img_process/night_gain_q8.mem")begin: compact
  reg [11:0] bulk[0:32767] /* synthesis syn_romstyle="block_rom" */;
  reg [11:0] last_curve[0:255] /* synthesis syn_romstyle="block_rom" */;
  reg [11:0] bulk_q,last_q;
  reg select_last_q;
  initial begin $readmemh(BULK_FILE,bulk);$readmemh(LAST_FILE,last_curve);end
  always @(posedge clk)begin
   bulk_q<=bulk[{strength[6:0],luma}];
   last_q<=last_curve[luma];
   select_last_q<=strength[7];
  end
  // Valid strength domain is 0..128, unchanged from local_gamma_tiles.
  assign gain=select_last_q?last_q:bulk_q;
 end else begin: custom_table
  reg [11:0] legacy[0:33023] /* synthesis syn_romstyle="block_rom" */;
  reg [11:0] gain_q;
  initial $readmemh(FILE,legacy);
  always @(posedge clk)gain_q<=legacy[{strength,luma}];
  assign gain=gain_q;
 end endgenerate
endmodule
