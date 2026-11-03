// S3: debounced short release toggles raw/corrected; long holds have no action.
module button_correction_control #(
 parameter integer DEBOUNCE_CYCLES=2500000,HOLD_CYCLES=250000000
)(input clk,rst_n,key_n,output reg requested_corrected);
reg key_meta /* synthesis syn_preserve=1 syn_keep=1 syn_srlstyle="registers" */;
reg key_sync /* synthesis syn_preserve=1 syn_keep=1 syn_srlstyle="registers" */;
reg stable_level,long_fired;
reg [21:0] debounce_count;
reg [27:0] hold_count;
wire long_event=!stable_level && !long_fired && hold_count==HOLD_CYCLES-1;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin key_meta<=1;key_sync<=1;end
 else begin key_meta<=key_n;key_sync<=key_meta;end
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  requested_corrected<=1;stable_level<=1;long_fired<=0;debounce_count<=0;hold_count<=0;
 end else begin
  if(key_sync==stable_level)debounce_count<=0;
  else if(debounce_count==DEBOUNCE_CYCLES-1)begin
   stable_level<=key_sync;debounce_count<=0;
   if(!key_sync)begin hold_count<=0;long_fired<=0;end
   else if(!long_fired && !long_event)requested_corrected<=~requested_corrected;
  end else debounce_count<=debounce_count+1'b1;
  if(!stable_level && !long_fired)begin
   if(hold_count==HOLD_CYCLES-1)begin long_fired<=1;end
   else hold_count<=hold_count+1'b1;
  end
 end
endmodule
