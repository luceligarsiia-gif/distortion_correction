// A0 is an unreliable accuracy estimate, not a malformed quaternion. Only
// planar affine may use it in explicitly indicated LOW mode, after the normal
// norm check/normalization downstream. Disable prediction until quality returns.
// Perspective and lens-only residual processing retain their accuracy gate.
module affine_report_policy(input clk,rst_n,affine_enabled,online,configured,
 input report_valid,input [1:0] accuracy,input norm_rejected,
 output accept_report,output prediction_bad);
reg low_quality;
assign accept_report=report_valid && online && configured &&
                     (accuracy!=0 || affine_enabled);
assign prediction_bad=norm_rejected ||
 (affine_enabled && (low_quality || (report_valid && accuracy==0)));
always @(posedge clk or negedge rst_n)
 if(!rst_n)low_quality<=0;
 else if(!online || !configured)low_quality<=0;
 else if(report_valid)low_quality<=accuracy==0;
endmodule
