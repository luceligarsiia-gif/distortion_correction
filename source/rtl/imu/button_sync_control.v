// S4 short release: panel visibility. Hold 1s: SYNC increment, then repeat.
// Affine always uses fixed crop. S4 never changes the crop factor.
module button_sync_control #(
 parameter integer DEBOUNCE_CYCLES=540000,HOLD_CYCLES=27000000,
 parameter integer REPEAT_CYCLES=2700000, DOUBLE_CYCLES=9450000,
 parameter [7:0] DEFAULT_DELAY_INDEX=10
)(input clk,rst_n,key_n,output reg parameters_hidden,
 output reg [7:0] delay_index,delay_gray,output wire fixed_crop);
 assign fixed_crop=1'b1;
 reg key_meta /* synthesis syn_preserve=1 syn_keep=1 syn_srlstyle="registers" */;
 reg key_sync /* synthesis syn_preserve=1 syn_keep=1 syn_srlstyle="registers" */;
 reg stable_level,long_fired;
 reg [19:0] debounce_count;
 reg [24:0] hold_count;
 wire [7:0] next_index=delay_index+8'd1;
 wire long_event=!stable_level && !key_sync &&
                 hold_count==(long_fired?REPEAT_CYCLES-1:HOLD_CYCLES-1);
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin key_meta<=1;key_sync<=1;end
 else begin key_meta<=key_n;key_sync<=key_meta;end
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  parameters_hidden<=1;delay_index<=DEFAULT_DELAY_INDEX;
  delay_gray<=DEFAULT_DELAY_INDEX^(DEFAULT_DELAY_INDEX>>1);
  stable_level<=1;long_fired<=0;debounce_count<=0;hold_count<=0;
 end else begin
  if(key_sync==stable_level)debounce_count<=0;
  else if(debounce_count==DEBOUNCE_CYCLES-1)begin
   stable_level<=key_sync;debounce_count<=0;
   if(!key_sync)begin hold_count<=0;long_fired<=0;end
   else if(!long_fired && !long_event)parameters_hidden<=~parameters_hidden;
  end else debounce_count<=debounce_count+1'b1;
  if(!stable_level && !key_sync)begin
   if(long_event)begin
    long_fired<=1;hold_count<=0;
    delay_index<=next_index;delay_gray<=next_index^(next_index>>1);
   end else hold_count<=hold_count+1'b1;
  end
 end
endmodule
