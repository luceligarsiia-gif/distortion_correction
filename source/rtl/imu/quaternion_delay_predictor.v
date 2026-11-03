// Fixed-latency quaternion extrapolator for the affine correction path.
// The input is already the normalized, adaptively filtered quaternion. Three
// report periods (30 ms at the configured 100 Hz report rate) are predicted;
// the extrapolated quaternion is renormalized before it is released.
// A report timeout, invalid report event, or bad norm disables prediction and
// holds the last valid output until a fresh good sample arrives. A rejected
// extrapolation publishes its captured, validated input instead of dropping it.
module quaternion_delay_predictor #(
 parameter integer CLK_HZ=27000000,
 parameter integer PREDICT_MS=30,
 parameter integer STALE_MS=50,
 parameter integer PREDICT_STEPS=(PREDICT_MS/10)
)(input clk,rst_n,enable,online,input in_valid,
 input [63:0] filtered_q,input bad_event,
 output reg out_valid,output reg [63:0] predicted_q,
 output reg stale,output reg prediction_active,output reg anomaly);
localparam integer STALE_CYCLES=CLK_HZ/1000*STALE_MS;
localparam integer STEPS=(PREDICT_STEPS<1)?1:((PREDICT_STEPS>4)?4:PREDICT_STEPS);
localparam [39:0] NORM_ONE=40'd268435456;
localparam [39:0] NORM_LOW=40'd214748365;
localparam [39:0] NORM_HIGH=40'd322122547;
localparam IDLE=0,NORM=1,SCALE=2;
reg [1:0] state;
reg have_sample,hold_fault;
reg [31:0] age_ticks;
reg [63:0] previous_q,measured_q;
reg prediction_fault;
reg signed [31:0] extrapolated[0:3];
reg signed [39:0] norm_sum;
reg [1:0] op;
// A 16-bit component plus up to four 17-bit differences fits signed 20 bits.
// Share one 20x20 operator between norm accumulation and component scaling.
reg signed [15:0] registered_scale;
reg [15:0] scaled_parts[0:2];
wire signed [19:0] multiplier_a=extrapolated[op][19:0];
wire signed [19:0] multiplier_b=(state==NORM)?multiplier_a:{{4{registered_scale[15]}},registered_scale};
wire signed [39:0] product=multiplier_a*multiplier_b;
reg signed [40:0] norm_error;
reg signed [31:0] scale_q14;
integer i;

function signed [15:0] clamp16;
 input signed [63:0] v;
 begin
  if(v>32767)clamp16=16'sh7fff;
  else if(v< -32768)clamp16=-16'sd32768;
  else clamp16=v[15:0];
 end
endfunction
function signed [31:0] extrapolate;
 input signed [15:0] current,previous;
 reg signed [31:0] a,d;
 begin
  a=current;d=a-$signed(previous);
  case(STEPS)
   1:extrapolate=a+d;
   2:extrapolate=a+(d<<<1);
   3:extrapolate=a+d+(d<<<1);
   default:extrapolate=a+(d<<<2);
  endcase
 end
endfunction
wire signed [39:0] norm_next=norm_sum+$signed(product);
wire norm_good=(norm_next>=NORM_LOW)&&(norm_next<=NORM_HIGH);

always @* begin
 norm_error=$signed({1'b0,40'd805306368})-$signed({1'b0,norm_next});
 scale_q14=norm_error>>>15;
 if(scale_q14<8192)scale_q14=8192;
 if(scale_q14>24576)scale_q14=24576;
end

always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  state<=IDLE;have_sample<=0;hold_fault<=0;age_ticks<=0;previous_q<=0;
  measured_q<=64'h4000000000000000;prediction_fault<=0;
  registered_scale<=16384;scaled_parts[0]<=0;scaled_parts[1]<=0;scaled_parts[2]<=0;
  norm_sum<=0;op<=0;out_valid<=0;predicted_q<=64'h4000000000000000;
  stale<=1;prediction_active<=0;anomaly<=0;
  for(i=0;i<4;i=i+1)extrapolated[i]<=0;
 end else begin
  out_valid<=0;anomaly<=0;
  if(!enable || !online)begin
   state<=IDLE;have_sample<=0;hold_fault<=0;age_ticks<=0;stale<=1;prediction_active<=0;prediction_fault<=0;
   predicted_q<=64'h4000000000000000;
  end else begin
   if(age_ticks<STALE_CYCLES)age_ticks<=age_ticks+1'b1;
   if(have_sample && age_ticks>=STALE_CYCLES)begin
    if(!stale)anomaly<=1;
    stale<=1;prediction_active<=0;
   end
   if(bad_event)begin
    hold_fault<=1;prediction_active<=0;anomaly<=1;
    if(state!=IDLE)prediction_fault<=1;
   end
   if(in_valid && state==IDLE)begin
    age_ticks<=0;previous_q<=filtered_q;
    if(!have_sample || hold_fault || stale || bad_event)begin
     // First sample after startup/fault is a measured pose, never a guess.
     predicted_q<=filtered_q;have_sample<=1;stale<=0;hold_fault<=0;
     prediction_active<=0;out_valid<=1;
    end else begin
     measured_q<=filtered_q;prediction_fault<=0;
     for(i=0;i<4;i=i+1)begin
      extrapolated[i]<=extrapolate($signed(filtered_q[i*16+:16]),$signed(previous_q[i*16+:16]));
     end
     norm_sum<=0;op<=0;state<=NORM;
    end
   end else if(in_valid && state!=IDLE)begin
    // The normal filter is much slower than this small predictor. If a future
    // integration changes that assumption, drop the prediction safely.
    anomaly<=1;hold_fault<=1;prediction_active<=0;prediction_fault<=1;
   end
   if(state==NORM)begin
    norm_sum<=norm_next;
    if(op==3)begin
     if(norm_good && !prediction_fault && !bad_event && !in_valid)begin
      registered_scale<=scale_q14[15:0];op<=0;state<=SCALE;
     end else begin
      // Only the prediction failed: its captured input already passed the
      // quaternion filter. Publish that measured pose, never a newer live bus
      // value or an invalid raw report. The next sample restarts prediction.
      state<=IDLE;predicted_q<=measured_q;out_valid<=1;hold_fault<=1;
      prediction_active<=0;anomaly<=1;
     end
    end else op<=op+1'b1;
   end
   if(state==SCALE)begin
    if(op==3)begin
     state<=IDLE;out_valid<=1;
     if(!prediction_fault && !bad_event && !in_valid)begin
      predicted_q<={clamp16($signed(product)>>>14),scaled_parts[2],scaled_parts[1],scaled_parts[0]};
      prediction_active<=1;
     end else begin
      predicted_q<=measured_q;hold_fault<=1;prediction_active<=0;anomaly<=1;
     end
    end else begin scaled_parts[op]<=clamp16($signed(product)>>>14);op<=op+1'b1;end
   end
  end
 end
endmodule
