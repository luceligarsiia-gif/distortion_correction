// S5 short release: affine ON/OFF. Hold 1s: perspective ON/OFF, once per hold.
// Modes are exclusive: 0=fixed lens only, 1=original affine, 2=3-axis perspective.
module button_geometry_control #(
 parameter integer DEBOUNCE_CYCLES=540000,LONG_CYCLES=27000000
)(input clk,rst_n,key_n,output reg [1:0] mode);
reg meta,sync /* synthesis syn_preserve=1 syn_keep=1 syn_srlstyle="registers" */;
reg stable;
reg [31:0] debounce,held;
reg long_fired;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin meta<=1;sync<=1;end else begin meta<=key_n;sync<=meta;end
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin stable<=1;debounce<=0;held<=0;long_fired<=0;mode<=0;end
 else begin
  if(sync==stable)debounce<=0;
  else if(debounce==DEBOUNCE_CYCLES-1)begin
   stable<=sync;debounce<=0;
   if(!sync)begin held<=0;long_fired<=0;end
   else begin
    // Include the exact threshold/release collision in the long-press case.
    if(!long_fired && held<LONG_CYCLES-1)mode<=(mode==1)?2'd0:2'd1;
    held<=0;long_fired<=0;
   end
  end else debounce<=debounce+1'b1;
  if(!stable)begin
   if(!long_fired)begin
    if(held==LONG_CYCLES-1)begin mode<=(mode==2)?2'd0:2'd2;long_fired<=1;end
    else held<=held+1'b1;
   end
  end
 end
endmodule
