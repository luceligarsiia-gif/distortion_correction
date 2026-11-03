// Raw-input statistics select a whole-frame algorithm. No video is buffered.
// W=0 is the original CLAHE branch, W=16 the original NIGHT_GAMMA branch.
// Hysteresis and a two-unit/frame slew prevent a brightness jump at the seam.
module hybrid_light_control(
 input clk,rst_n,frame_boundary,stats_valid,night_ready,glare_present,
 input [7:0] luma_avg,p50,p90,input [6:0] saturation_percent,
 output reg [4:0] night_weight,output reg [1:0] scene);
 reg night_selected;
 wire enter_night=p50<8'd144 && p90<8'd192 && luma_avg<8'd168;
 wire hold_night=p50<8'd152 && p90<8'd200 && luma_avg<8'd176;
 wire choose_night=stats_valid && night_ready && !glare_present &&
                   (night_selected?hold_night:enter_night);
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin night_weight<=0;night_selected<=0;scene<=0;end
 else if(frame_boundary)begin
  night_selected<=choose_night;
  if(!stats_valid || !night_ready)night_weight<=0;
  else if(choose_night)begin
   if(night_weight<16)night_weight<=night_weight+5'd2;
  end else if(night_weight!=0)night_weight<=night_weight-5'd2;
  if(choose_night)scene<=(p50<80 && p90<176)?2'd2:2'd1;
  else scene<=(stats_valid && p50>192 && saturation_percent>=10)?2'd3:2'd0;
 end
endmodule
