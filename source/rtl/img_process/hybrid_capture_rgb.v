// Causal dark denoising followed by the parallel CLAHE/night blend.
// Total 5+8=13 pixel stages, one pixel/clock, no line/frame waiting.
module hybrid_capture_rgb(input clk,rst_n,requested_enabled,
 input [7:0] in_r,in_g,in_b,input in_vs,in_hs,in_de,input [15:0] time_ms,
 output wire [7:0] out_r,out_g,out_b,output wire out_vs,out_hs,out_de,
 output wire enabled,ready,output wire [4:0] night_weight,
 output wire [1:0] scene,output wire [7:0] luma_avg,p50,p90,
 output wire [6:0] saturation_percent,output wire denoise_active);
 wire [7:0] dr,dg,db;wire dv,dh,dd;
 shadow_denoise_rgb denoise(.clk(clk),.rst_n(rst_n),.requested_enabled(requested_enabled),
  .in_r(in_r),.in_g(in_g),.in_b(in_b),.in_vs(in_vs),.in_hs(in_hs),.in_de(in_de),
  .out_r(dr),.out_g(dg),.out_b(db),.out_vs(dv),.out_hs(dh),.out_de(dd),.enabled(denoise_active));
 // Frame-latched permission makes denoising and tone enable epochs coherent.
 hybrid_tone_rgb tone(.clk(clk),.rst_n(rst_n),.requested_enabled(denoise_active),
  .in_r(dr),.in_g(dg),.in_b(db),.in_vs(dv),.in_hs(dh),.in_de(dd),.time_ms(time_ms),
  .out_r(out_r),.out_g(out_g),.out_b(out_b),.out_vs(out_vs),.out_hs(out_hs),.out_de(out_de),
  .enabled(enabled),.ready(ready),.night_weight(night_weight),.scene(scene),
  .luma_avg(luma_avg),.p50(p50),.p90(p90),.saturation_percent(saturation_percent));
endmodule
