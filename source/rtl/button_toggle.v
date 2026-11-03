// S3 / USER_BUTTON0: active low. 20 ms press AND release debounce at 125 MHz.
module button_toggle #(parameter DEBOUNCE_CYCLES=2500000)(
 input clk,rst_n,key_n,output reg requested_corrected);
// Preserve physical flip-flops: do not infer a LUT shift register for this CDC.
reg key_meta /* synthesis syn_preserve=1 syn_keep=1 syn_srlstyle="registers" */;
reg key_sync /* synthesis syn_preserve=1 syn_keep=1 syn_srlstyle="registers" */;
reg stable_level;
reg [21:0] debounce_count;
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin key_meta<=1;key_sync<=1;end
 else begin key_meta<=key_n;key_sync<=key_meta;end
always @(posedge clk or negedge rst_n)
 if(!rst_n) begin stable_level<=1;debounce_count<=0;requested_corrected<=1;end
 else if(key_sync==stable_level) debounce_count<=0;
 else if(debounce_count==DEBOUNCE_CYCLES-1) begin
  stable_level<=key_sync;debounce_count<=0;
  if(!key_sync) requested_corrected<=~requested_corrected;
 end else debounce_count<=debounce_count+1'b1;
endmodule
