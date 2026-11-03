// Two six-stage algorithms operate on the SAME raw input, never in series.
// Two shift/add blend stages give a fixed eight-stage streaming video path.
module hybrid_tone_rgb(input clk,rst_n,requested_enabled,
 input [7:0] in_r,in_g,in_b,input in_vs,in_hs,in_de,
 input [15:0] time_ms,
 output reg [7:0] out_r,out_g,out_b,output reg out_vs,out_hs,out_de,
 output wire enabled,ready,output wire [4:0] night_weight,
 output wire [1:0] scene,output wire [7:0] luma_avg,p50,p90,
 output wire [6:0] saturation_percent);
 wire [7:0] cr,cg,cb,nr,ng,nb;
 wire cv,ch,cd,nv,nh,nd,clahe_enabled,clahe_ready,night_enabled,night_ready,stats_valid;
 wire glare_present;
 wire [7:0] p10;
 wire signed [5:0] dark_peak,high_peak;
 clahe_rgb clahe_path(.clk(clk),.rst_n(rst_n),.requested_enabled(requested_enabled),
  .in_r(in_r),.in_g(in_g),.in_b(in_b),.in_vs(in_vs),.in_hs(in_hs),.in_de(in_de),
  .out_r(cr),.out_g(cg),.out_b(cb),.out_vs(cv),.out_hs(ch),.out_de(cd),
  .enabled(clahe_enabled),.ready(clahe_ready),.glare_present(glare_present));
 adaptive_gamma_rgb night_path(.clk(clk),.rst_n(rst_n),.requested_enabled(requested_enabled),
  .in_r(in_r),.in_g(in_g),.in_b(in_b),.in_vs(in_vs),.in_hs(in_hs),.in_de(in_de),
  .out_r(nr),.out_g(ng),.out_b(nb),.out_vs(nv),.out_hs(nh),.out_de(nd),
  .gamma_enabled(night_enabled),.gamma_level(dark_peak),.target_level(high_peak),
  .time_ms(time_ms),.luma_avg(luma_avg),.stats_valid(stats_valid),.scene(),
  .p10(p10),.p50(p50),.p90(p90),.saturation_percent(saturation_percent),
  .local_ready(night_ready));
 assign enabled=clahe_enabled && night_enabled;
 assign ready=clahe_ready && night_ready && stats_valid;
 reg vs_d,policy_boundary;
 wire frame_boundary=vs_d && !in_vs;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin vs_d<=1;policy_boundary<=0;end
  else begin vs_d<=in_vs;policy_boundary<=frame_boundary;end
 hybrid_light_control selection(.clk(clk),.rst_n(rst_n),.frame_boundary(policy_boundary),
  .stats_valid(stats_valid),.night_ready(night_ready),.luma_avg(luma_avg),
  .p50(p50),.p90(p90),.saturation_percent(saturation_percent),.glare_present(glare_present),
  .night_weight(night_weight),.scene(scene));
 // APMs are reserved for the inherited geometry engine and night RGB gain.
 function [12:0] mul5;
  input [7:0] value;input [4:0] factor;
  reg [12:0] a,b,c,d,e;
  begin
   a=factor[0]?{5'd0,value}:13'd0;
   b=factor[1]?{4'd0,value,1'b0}:13'd0;
   c=factor[2]?{3'd0,value,2'b0}:13'd0;
   d=factor[3]?{2'd0,value,3'b0}:13'd0;
   e=factor[4]?{1'd0,value,4'b0}:13'd0;
   mul5=(a+b)+(c+d)+e;
  end
 endfunction
 reg [4:0] weight_pipe[0:5];
 wire [4:0] weight=weight_pipe[5];
 wire [4:0] inverse_weight=5'd16-weight;
 reg [12:0] rn7,rc7,gn7,gc7,bn7,bc7;
 reg [2:0] controls7;
 wire [13:0] red_sum={1'b0,rn7}+{1'b0,rc7}+14'd8;
 wire [13:0] green_sum={1'b0,gn7}+{1'b0,gc7}+14'd8;
 wire [13:0] blue_sum={1'b0,bn7}+{1'b0,bc7}+14'd8;
 integer i;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
  for(i=0;i<6;i=i+1)weight_pipe[i]<=0;
  rn7<=0;rc7<=0;gn7<=0;gc7<=0;bn7<=0;bc7<=0;controls7<=0;
  out_r<=0;out_g<=0;out_b<=0;out_vs<=0;out_hs<=0;out_de<=0;
 end else begin
  weight_pipe[0]<=night_weight;
  for(i=1;i<6;i=i+1)weight_pipe[i]<=weight_pipe[i-1];
  rn7<=mul5(nr,weight);rc7<=mul5(cr,inverse_weight);
  gn7<=mul5(ng,weight);gc7<=mul5(cg,inverse_weight);
  bn7<=mul5(nb,weight);bc7<=mul5(cb,inverse_weight);
  controls7<={cv,ch,cd};
  out_r<=red_sum[11:4];out_g<=green_sum[11:4];out_b<=blue_sum[11:4];
  {out_vs,out_hs,out_de}<=controls7;
 end
endmodule
