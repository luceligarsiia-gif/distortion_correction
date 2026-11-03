// Night gamma: bounded shared-RGB Q8 gain, six stages, one pixel per clock.
// One extra pixel stage separates gain weighting from limiting. No new frame
// or line buffer is introduced; RGB and all timing controls travel together.
module adaptive_gamma_rgb #(
 parameter GAIN_FILE="source/rtl/img_process/gamma_gain_q8.mem",
 parameter HEADROOM_FILE="source/rtl/img_process/gamma_headroom_q8.mem",
 parameter TONE_FILE="source/rtl/img_process/night_gain_q8.mem",
 parameter integer MIN_SAMPLES=1024,
 parameter integer WIDTH=1920,HEIGHT=1080,COLS=8,ROWS=9,
 parameter integer TILE_X_SHIFT=8,TILE_Y_SHIFT=7,MIN_TILE_SAMPLES=16
)(input clk,rst_n,requested_enabled,
 input [7:0] in_r,in_g,in_b,input in_vs,in_hs,in_de,
 output reg [7:0] out_r,out_g,out_b,output reg out_vs,out_hs,out_de,
 output reg gamma_enabled,output wire signed [5:0] gamma_level,
 output wire [7:0] luma_avg,output wire stats_valid,
 input [15:0] time_ms,
 output wire signed [5:0] target_level,
 output wire [1:0] scene,
 output wire [7:0] p10,p50,p90,
 output wire [6:0] saturation_percent,
 output wire local_ready);
reg enable_meta /* synthesis syn_preserve=1 syn_srlstyle="registers" */;
reg enable_sync /* synthesis syn_preserve=1 syn_srlstyle="registers" */;
reg vs_d;
wire frame_boundary=vs_d && !in_vs;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin enable_meta<=0;enable_sync<=0;vs_d<=1;gamma_enabled<=0;end
 else begin
  enable_meta<=requested_enabled;enable_sync<=enable_meta;vs_d<=in_vs;
  if(frame_boundary)gamma_enabled<=enable_sync;
 end
// Exact baseline luminance, expressed as shifts/adds to avoid DSP inference.
wire [15:0] red_ext={8'd0,in_r},green_ext={8'd0,in_g},blue_ext={8'd0,in_b};
wire [15:0] raw_luma_sum=((red_ext<<6)+(red_ext<<3)+(red_ext<<2)+red_ext)+
 ((green_ext<<7)+(green_ext<<4)+(green_ext<<2)+(green_ext<<1))+
 ((blue_ext<<4)+(blue_ext<<3)+(blue_ext<<2)+blue_ext);
wire [7:0] raw_luma=raw_luma_sum[15:8];
wire [7:0] raw_max=(in_r>=in_g && in_r>=in_b)?in_r:((in_g>=in_b)?in_g:in_b);
wire [7:0] dark_left,dark_right,high_left,high_right;
wire [4:0] fraction_x;
wire local_valid;
assign local_ready=local_valid;
wire [3:0] dark_peak,high_peak;
local_gamma_tiles #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.COLS(COLS),.ROWS(ROWS),
 .TILE_X_SHIFT(TILE_X_SHIFT),.TILE_Y_SHIFT(TILE_Y_SHIFT),.MIN_TILE_SAMPLES(MIN_TILE_SAMPLES))
 local_control(.clk(clk),.rst_n(rst_n),.frame_boundary(frame_boundary),.in_de(in_de),.luma(raw_luma),
 .dark_left(dark_left),.dark_right(dark_right),.high_left(high_left),.high_right(high_right),
 .fraction_x(fraction_x),.map_valid(local_valid),.dark_peak(dark_peak),.high_peak(high_peak));
// Status D/H are the map's maximum shadow/highlight strength, 0..8.
// The whole-image histogram/scene remain diagnostic, not a local enable gate.
assign gamma_level=$signed({2'd0,dark_peak});
assign target_level=$signed({2'd0,high_peak});

function [12:0] times5;
 input [7:0] a; input [4:0] b;
 reg [12:0] p0,p1,p2,p3,p4;
 begin
  p0=b[0]?{5'd0,a}:13'd0;p1=b[1]?{4'd0,a,1'b0}:13'd0;
  p2=b[2]?{3'd0,a,2'd0}:13'd0;p3=b[3]?{2'd0,a,3'd0}:13'd0;
  p4=b[4]?{1'd0,a,4'd0}:13'd0;times5=(p0+p1)+(p2+p3)+p4;
 end
endfunction
function [7:0] blend;
 input [7:0] a,b;input [4:0] fraction;
 reg [13:0] value;
 begin value=({1'b0,a,5'd0}-{1'b0,times5(a,fraction)})+
              {1'b0,times5(b,fraction)}+14'd16;
  blend=value[12:5];
 end
endfunction
// 129 strength curves, stored by night_gain_lut without padded spare rows.
reg [15:0] headroom_rom[0:255];
initial $readmemh(HEADROOM_FILE,headroom_rom);
reg [15:0] luma1 /* synthesis syn_preserve=1 syn_keep=1 syn_srlstyle="registers" */;
reg [7:0] max1,left1,right1,strength2;
reg [7:0] luma2;
reg [4:0] fraction1;

reg [15:0] limit2;
reg [23:0] pixel1,pixel2,pixel3;
reg [2:0] controls1,controls2,controls3,controls4;
reg bypass1,bypass2,bypass3;
reg [11:0] gain3;
wire [11:0] lookup3a;
night_gain_lut #(.FILE(TONE_FILE)) gain_table(.clk(clk),.strength(strength2),
 .luma(luma2),.gain(lookup3a));
reg [15:0] limit3a;
reg bypass3a;
reg [23:0] pixel3a;
reg [2:0] controls3a;
wire [11:0] selected_gain3=bypass3?12'd256:gain3;
reg [19:0] r4,g4,b4;
wire [20:0] rounded_r={1'b0,r4}+21'd128;
wire [20:0] rounded_g={1'b0,g4}+21'd128;
wire [20:0] rounded_b={1'b0,b4}+21'd128;
// Stage 2: synchronous headroom ROM; Stage 3: synchronous gain bank.
always @(posedge clk)begin
 limit2<=headroom_rom[max1];
end
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  luma1<=0;max1<=0;left1<=0;right1<=0;fraction1<=0;strength2<=0;
  luma2<=0;pixel1<=0;pixel2<=0;pixel3<=0;
  controls1<=0;controls2<=0;controls3<=0;controls4<=0;
  bypass1<=1;bypass2<=1;bypass3<=1;gain3<=256;
  limit3a<=0;bypass3a<=1;pixel3a<=0;controls3a<=0;
  r4<=0;g4<=0;b4<=0;out_r<=0;out_g<=0;out_b<=0;out_vs<=0;out_hs<=0;out_de<=0;
 end else begin
  // Stage 1: raw RGB, luma, coordinates and coefficient endpoints.
  luma1<=raw_luma_sum;max1<=raw_max;pixel1<={in_r,in_g,in_b};controls1<={in_vs,in_hs,in_de};
  fraction1<=fraction_x;
  left1<=raw_luma>=192?high_left:dark_left;
  right1<=raw_luma>=192?high_right:dark_right;
  bypass1<=!gamma_enabled || !local_valid;
  strength2<=blend(left1,right1,fraction1);luma2<=luma1[15:8];
  pixel2<=pixel1;controls2<=controls1;bypass2<=bypass1;
  // Stage 3: gain bank read, with matching pixel, controls and headroom.
  limit3a<=limit2;
  pixel3a<=pixel2;controls3a<=controls2;bypass3a<=bypass2;
  // Stage 4: rounded/capped shared RGB gain, with matching video signals.
  gain3<=({4'd0,lookup3a}>limit3a)?limit3a:lookup3a;
  pixel3<=pixel3a;controls3<=controls3a;bypass3<=bypass3a;
  // Stage 5: reuse the baseline three RGB multipliers, one pixel/clock.
  r4<=pixel3[23:16]*selected_gain3;g4<=pixel3[15:8]*selected_gain3;b4<=pixel3[7:0]*selected_gain3;
  controls4<=controls3;
  // Stage 6: rounded/saturated RGB and exactly matching timing controls.
  out_r<=rounded_r[20:16]!=0?8'd255:rounded_r[15:8];
  out_g<=rounded_g[20:16]!=0?8'd255:rounded_g[15:8];
  out_b<=rounded_b[20:16]!=0?8'd255:rounded_b[15:8];
  {out_vs,out_hs,out_de}<=controls4;
 end
reg stats_frame_boundary;
reg [15:0] stats_time_ms;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin stats_frame_boundary<=0;stats_time_ms<=0;end
 else begin stats_frame_boundary<=frame_boundary;stats_time_ms<=time_ms;end
gamma_histogram_control #(.MIN_SAMPLES(MIN_SAMPLES),.GAIN_FILE(GAIN_FILE)) statistics(
 .clk(clk),.rst_n(rst_n),.frame_boundary(stats_frame_boundary),.in_de(controls1[0]),
 .luma(luma1[15:8]),.maximum(max1),.time_ms(stats_time_ms),
 .luma_avg(luma_avg),.stats_valid(stats_valid),.target_level(),
 .scene(scene),.p10(p10),.p50(p50),.p90(p90),.saturation_percent(saturation_percent));
endmodule

// Registered Gray milliseconds from the stable 27MHz oscillator. This clock
// is independent of HDMI resolution/frame rate. Full-cycle binary wrap is
// safe because all expiry intervals are well below 32768ms.
module gamma_ms_cdc #(parameter integer CLOCKS_PER_MS=27000)(
 input src_clk,rst_n,dst_clk,output wire [15:0] time_ms);
reg [14:0] divider;
reg [15:0] binary_time,gray_time;
(* async_reg="true" *) reg [15:0] gray_meta,gray_sync;
wire [15:0] next_time=binary_time+16'd1;
always @(posedge src_clk or negedge rst_n)
 if(!rst_n)begin divider<=0;binary_time<=0;gray_time<=0;end
 else if(divider==CLOCKS_PER_MS-1)begin
  divider<=0;binary_time<=next_time;gray_time<=next_time^(next_time>>1);
 end else divider<=divider+1'b1;
always @(posedge dst_clk or negedge rst_n)
 if(!rst_n)begin gray_meta<=0;gray_sync<=0;end
 else begin gray_meta<=gray_time;gray_sync<=gray_meta;end
genvar bit_id;
generate for(bit_id=0;bit_id<16;bit_id=bit_id+1)begin: decode_time
 assign time_ms[bit_id]=^(gray_sync>>bit_id);
end endgenerate
endmodule

// Ping-pong 64-bin histograms, one sample per 16 active pixels. Histogram
// RMW is two clocks; sampling is at least 16 clocks apart within a line.
// The first 16 pixels of each line are skipped to avoid a short inter-line
// RMW hazard and the sample phase advances by five on each new line.
// Statistics never stall the RGB pipeline. All time constants use sys-clock
// milliseconds supplied through a Gray-code CDC, not a nominal pixel clock.
module gamma_histogram_control #(
 parameter integer MIN_SAMPLES=1024,
 parameter GAIN_FILE="source/rtl/img_process/gamma_gain_q8.mem"
)(input clk,rst_n,frame_boundary,in_de,input [7:0] luma,maximum,
 input [15:0] time_ms,
 output reg [7:0] luma_avg,output reg stats_valid,
 output reg signed [5:0] target_level,output reg [1:0] scene,
 output wire [7:0] p10,p50,p90,output wire [6:0] saturation_percent);
localparam NORMAL=0,DARK=1,BACKLIT=2,BRIGHT=3;
localparam IDLE=0,HREAD=1,HACC=2,DTRIAL=3,DMUL=4,DCHECK=5,
 SMOOTH=6,CLASSIFY=7,PREPARE=8,LREAD=9,LMUL=10,LCHECK=11,LDECIDE=12,HADD=13;
reg [3:0] state;
reg [17:0] hist0[0:63],hist1[0:63];
reg initializing,collecting,write_bank,scan_bank,frame_good,overflowed;
reg [5:0] clear_bin,scan_bin,sample_bin;
reg sample_pending,sample_bank,de_d;
reg [17:0] hist_read0,hist_read1;
wire [17:0] scan_value=scan_bank?hist_read1:hist_read0;
wire hist_we0=initializing || (sample_pending && !sample_bank) || (state==HACC && !scan_bank);
wire hist_we1=initializing || (sample_pending && sample_bank) || (state==HACC && scan_bank);
wire [5:0] hist_wa0=initializing?clear_bin:((sample_pending && !sample_bank)?sample_bin:scan_bin);
wire [5:0] hist_wa1=initializing?clear_bin:((sample_pending && sample_bank)?sample_bin:scan_bin);
wire [17:0] hist_wd0=(!initializing && sample_pending && !sample_bank)?hist_read0+18'd1:18'd0;
wire [17:0] hist_wd1=(!initializing && sample_pending && sample_bank)?hist_read1+18'd1:18'd0;
reg [3:0] phase,x_mod;
reg [4:0] line_warmup;
reg [17:0] count,dark_count,sat_count,held_count,held_dark,held_sat,previous_count;
wire count_changed=previous_count!=0 &&
 (({1'b0,count}+({1'b0,previous_count}>>3)<{1'b0,previous_count}) ||
  ({1'b0,count}>{1'b0,previous_count}+({1'b0,previous_count}>>3)));
reg [25:0] sum,held_sum,dividend,div_product;
reg [18:0] cumulative;
reg got10,got50,got90;
reg [7:0] raw10,raw50,raw90,raw_average,raw_dark,raw_sat;
reg [11:0] smooth10,smooth50,smooth90,smooth_dark,smooth_sat;
assign p10=smooth10[11:4];assign p50=smooth50[11:4];assign p90=smooth90[11:4];
assign saturation_percent=smooth_sat[10:4];
reg [7:0] quotient,candidate;
reg [2:0] bit_index;
reg [1:0] division_kind,pending_scene,classified;
reg [15:0] last_good_ms,last_smooth_ms,pending_since;
reg have_good;
wire [15:0] good_age=time_ms-last_good_ms,smooth_age=time_ms-last_smooth_ms;
wire [15:0] pending_age=time_ms-pending_since;
wire sample_fire=collecting && !initializing && !frame_boundary && in_de &&
 de_d && line_warmup==16 && x_mod==phase && count!=18'h3ffff;
wire [18:0] next_cumulative=cumulative+{1'b0,scan_value};
reg [18:0] evaluated_cumulative;
wire [22:0] c10=({4'd0,evaluated_cumulative}<<3)+({4'd0,evaluated_cumulative}<<1);
wire [22:0] n9=({5'd0,held_count}<<3)+{5'd0,held_count};
wire [7:0] accepted=div_product<=dividend?candidate:quotient;
wire [7:0] bin_center={scan_bin,2'b10};
reg [9:0] decision_rom[0:8191];
initial $readmemh(GAIN_FILE,decision_rom);
reg [9:0] decision_gain;
reg [4:0] trial_level,level_cap;
reg negative_trial;
reg [7:0] desired_median,evaluation_luma;
wire [4:0] decision_row=negative_trial?(trial_level-1'b1):(trial_level+5'd15);
reg [17:0] prediction;
reg trial_enough_q;
wire [7:0] predicted_median=(prediction+18'd128)>>8;
wire trial_enough=negative_trial?predicted_median<=desired_median:predicted_median>=desired_median;
wire signed [5:0] trial_signed=negative_trial?-$signed({1'b0,trial_level}):$signed({1'b0,trial_level});
wire signed [6:0] target_distance=$signed(trial_signed)-$signed(target_level);

// Small synchronous memories; neither reset nor the policy logic resets all
// bins in parallel. Each inactive bank is cleared as it is scanned.
always @(posedge clk)begin
 if(hist_we0)hist0[hist_wa0]<=hist_wd0;
 if(hist_we1)hist1[hist_wa1]<=hist_wd1;
 hist_read0<=hist0[write_bank?scan_bin:luma[7:2]];
 hist_read1<=hist1[write_bank?luma[7:2]:scan_bin];
 decision_gain<=decision_rom[{decision_row,evaluation_luma}];
end
function [11:0] half_filter;
 input [11:0] previous;input [7:0] value;
 reg signed [13:0] delta;
 begin delta=$signed({1'b0,value,4'd0})-$signed({1'b0,previous});
 half_filter=$signed({1'b0,previous})+(delta>>>1);end
endfunction

// Entry/exit hysteresis. A white scene is not darkened solely for being white:
// require a significant near-saturation fraction as well as a high median.
always @*begin
 classified=NORMAL;
 if((p90>=208 && p10<56 && smooth_dark[11:4]>=20) ||
    (scene==BACKLIT && p90>=192 && p10<64 && smooth_dark[11:4]>=15))classified=BACKLIT;
 else if((p50<80 && p90<176) || (scene==DARK && p50<96 && p90<192))classified=DARK;
 else if((p50>192 && saturation_percent>=10) ||
         (scene==BRIGHT && p50>176 && saturation_percent>=5))classified=BRIGHT;
end
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  initializing<=1;clear_bin<=0;collecting<=0;write_bank<=0;scan_bank<=0;
  state<=IDLE;scan_bin<=0;sample_bin<=0;sample_pending<=0;sample_bank<=0;
  de_d<=0;phase<=0;x_mod<=0;line_warmup<=0;
  count<=0;dark_count<=0;sat_count<=0;sum<=0;overflowed<=0;previous_count<=0;
  held_count<=0;held_dark<=0;held_sat<=0;held_sum<=0;frame_good<=0;
  cumulative<=0;evaluated_cumulative<=0;got10<=0;got50<=0;got90<=0;raw10<=0;raw50<=0;raw90<=0;
  raw_average<=0;raw_dark<=0;raw_sat<=0;smooth10<=0;smooth50<=0;smooth90<=0;
  smooth_dark<=0;smooth_sat<=0;luma_avg<=0;stats_valid<=0;target_level<=0;scene<=NORMAL;
  quotient<=0;candidate<=0;bit_index<=0;division_kind<=0;dividend<=0;div_product<=0;
  pending_scene<=NORMAL;pending_since<=0;last_good_ms<=0;last_smooth_ms<=0;have_good<=0;
  trial_level<=1;level_cap<=0;negative_trial<=0;desired_median<=0;evaluation_luma<=0;prediction<=0;trial_enough_q<=0;
 end else begin
  de_d<=in_de;sample_pending<=sample_fire;
  if(sample_fire)begin sample_bin<=luma[7:2];sample_bank<=write_bank;end
  if(!in_de)begin line_warmup<=0;x_mod<=0;end
  else begin
   x_mod<=x_mod+1'b1;
   if(!de_d)phase<=phase+4'd5;
   if(line_warmup<16)line_warmup<=line_warmup+1'b1;
  end
  if(initializing)begin
   sample_pending<=0;
   if(clear_bin==63)begin initializing<=0;clear_bin<=0;end
   else clear_bin<=clear_bin+1'b1;
  end else if(frame_boundary)begin
   // A frame arriving before scan completion is malformed. Reinitialize both
   // banks and discard that epoch instead of reusing partially cleared bins.
   if(state!=IDLE)begin
    initializing<=1;clear_bin<=0;collecting<=0;sample_pending<=0;
    stats_valid<=0;have_good<=0;target_level<=0;scene<=NORMAL;pending_scene<=NORMAL;state<=IDLE;
   end else if(collecting)begin
    held_count<=count;held_dark<=dark_count;held_sat<=sat_count;held_sum<=sum;
    scan_bank<=write_bank;write_bank<=~write_bank;scan_bin<=0;
    cumulative<=0;got10<=0;got50<=0;got90<=0;raw10<=0;raw50<=0;raw90<=0;
    frame_good<=count>=MIN_SAMPLES && !overflowed && !count_changed;
    previous_count<=count>=MIN_SAMPLES?count:18'd0;
    state<=HREAD;
    if(count<MIN_SAMPLES || overflowed || count_changed)begin stats_valid<=0;pending_scene<=scene;pending_since<=time_ms;end
   end else collecting<=1;
   count<=0;dark_count<=0;sat_count<=0;sum<=0;overflowed<=0;
   phase<=0;line_warmup<=0;x_mod<=0;
  end else begin
   if(sample_fire)begin
    count<=count+1'b1;sum<=sum+{18'd0,luma};
    if(luma<48)dark_count<=dark_count+1'b1;
    if(maximum>=250)sat_count<=sat_count+1'b1;
   end
   if(collecting && in_de && count==18'h3ffff)overflowed<=1;
   case(state)
    HREAD:state<=HADD;
    HADD:begin evaluated_cumulative<=next_cumulative;state<=HACC;end
    HACC:begin
     cumulative<=evaluated_cumulative;
     if(!got10 && c10>={5'd0,held_count})begin raw10<=bin_center;got10<=1;end
     if(!got50 && (evaluated_cumulative<<1)>={1'b0,held_count})begin raw50<=bin_center;got50<=1;end
     if(!got90 && c10>=n9)begin raw90<=bin_center;got90<=1;end
     if(scan_bin==63)begin
      if(frame_good && evaluated_cumulative=={1'b0,held_count})begin
       dividend<=held_sum;division_kind<=0;quotient<=0;bit_index<=7;state<=DTRIAL;
      end else begin state<=IDLE;stats_valid<=0;end
     end else begin scan_bin<=scan_bin+1'b1;state<=HREAD;end
    end
    DTRIAL:begin candidate<=quotient | (8'd1<<bit_index);state<=DMUL;end
    DMUL:begin div_product<=held_count*candidate;state<=DCHECK;end
    DCHECK:begin
     quotient<=accepted;
     if(bit_index!=0)begin bit_index<=bit_index-1'b1;state<=DTRIAL;end
     else if(division_kind==0)begin
      raw_average<=accepted;division_kind<=1;dividend<=held_dark*26'd100;
      quotient<=0;bit_index<=7;state<=DTRIAL;
     end else if(division_kind==1)begin
      raw_dark<=accepted;division_kind<=2;dividend<=held_sat*26'd100;
      quotient<=0;bit_index<=7;state<=DTRIAL;
     end else begin raw_sat<=accepted;state<=SMOOTH;end
    end
    SMOOTH:begin
     luma_avg<=raw_average;last_good_ms<=time_ms;have_good<=1;stats_valid<=1;
     if(!stats_valid)begin
      smooth10<={raw10,4'd0};smooth50<={raw50,4'd0};smooth90<={raw90,4'd0};
      smooth_dark<={raw_dark,4'd0};smooth_sat<={raw_sat,4'd0};last_smooth_ms<=time_ms;
      pending_since<=time_ms;pending_scene<=scene;
     end else if(smooth_age>=50)begin
      smooth10<=half_filter(smooth10,raw10);smooth50<=half_filter(smooth50,raw50);
      smooth90<=half_filter(smooth90,raw90);smooth_dark<=half_filter(smooth_dark,raw_dark);
      smooth_sat<=half_filter(smooth_sat,raw_sat);
      last_smooth_ms<=smooth_age<100?last_smooth_ms+16'd50:time_ms;
     end
     state<=CLASSIFY;
    end
    CLASSIFY:begin
     if(classified==scene)begin pending_scene<=scene;pending_since<=time_ms;end
     else if(classified!=pending_scene)begin pending_scene<=classified;pending_since<=time_ms;end
     else if(pending_age>=150)begin scene<=classified;pending_since<=time_ms;end
     state<=PREPARE;
    end
    PREPARE:begin
     trial_level<=1;negative_trial<=scene==BRIGHT;
     // Absolute darkest pixels have no reliable recoverable detail. Limit
     // noise amplification further when the whole scene is near black.
     level_cap<=scene==DARK?((p90<32)?5'd4:5'd8):scene==BACKLIT?5'd6:5'd4;
     evaluation_luma<=(scene==BACKLIT && p50>=112)?p10:p50;
     if(scene==DARK)desired_median<=p50<72?p50+8'd24:8'd96;
     else if(scene==BACKLIT)begin
      if(p50>=112)desired_median<=p10<96?p10+8'd16:8'd112;
      else desired_median<=p50<96?p50+8'd16:8'd112;
     end else desired_median<=p50>220?p50-8'd12:8'd208;
     if(scene==NORMAL || (scene==DARK && (p50>=96 || p50<8)) ||
        (scene==BACKLIT && ((p50>=112 && p10<8) || p50<8)) || (scene==BRIGHT && p50<=208))begin
      target_level<=0;state<=IDLE;
     end else state<=LREAD;
    end
    LREAD:state<=LMUL;
    LMUL:begin prediction<=evaluation_luma*decision_gain;state<=LDECIDE;end
    LDECIDE:begin trial_enough_q<=trial_enough;state<=LCHECK;end
    LCHECK:begin
     if(trial_enough_q || trial_level>=level_cap)begin
      // One-level target deadband, except zero/sign/cap changes.
      if(target_level==0 || target_level[5]!=trial_signed[5] ||
         target_distance>1 || target_distance< -1 ||
         (!target_level[5] && target_level>$signed({1'b0,level_cap})))target_level<=trial_signed;
      state<=IDLE;
     end else begin trial_level<=trial_level+1'b1;state<=LREAD;end
    end
    default:state<=IDLE;
   endcase
  end
  // Covers missing VS and repeated short/invalid frames. The external time
  // reference also advances while the input pixel clock stops; expiry is
  // recognized on clock recovery. No stale statistics are advertised.
  if(have_good && good_age>=250 && state!=SMOOTH)begin
   stats_valid<=0;have_good<=0;target_level<=0;scene<=NORMAL;
   pending_scene<=NORMAL;pending_since<=time_ms;
  end
 end
endmodule
