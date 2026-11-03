// Two masters, independent read/write arbitration, one outstanding burst/channel.
// Master A (live capture/display) has priority. Selection locks before VALID
// reaches DDR and remains locked until the final data beat. No R backpressure
// exists on this Pango port. WREADY is the controller's data-consume request.
module native_ddr_arbiter(input clk,rst_n,
 input [27:0] a_araddr,input [3:0] a_arlen,input a_arvalid,output a_arready,
 output a_rvalid,output a_rlast,
 input [27:0] b_araddr,input [3:0] b_arlen,input b_arvalid,output b_arready,
 output b_rvalid,output b_rlast,
 output [27:0] araddr,output [3:0] arlen,output arvalid,input arready,
 input rvalid,rlast,
 input [27:0] a_awaddr,input [3:0] a_awlen,input a_awvalid,output a_awready,
 input [255:0] a_wdata,input [31:0] a_wstrb,output a_wready,output a_wlast,
 input [27:0] b_awaddr,input [3:0] b_awlen,input b_awvalid,output b_awready,
 input [255:0] b_wdata,output b_wready,output b_wlast,
 output [27:0] awaddr,output [3:0] awlen,output awvalid,input awready,
 output [255:0] wdata,output [31:0] wstrb,input wready,wlast);
reg [1:0] rs,ws;
reg ro,wo;
always @(posedge clk or negedge rst_n)
if(!rst_n) begin rs<=0;ws<=0;ro<=0;wo<=0;end
else begin
 case(rs)
 0: if(a_arvalid || b_arvalid) begin ro<=!a_arvalid;rs<=1;end
 1: if(arvalid && arready) rs<=(rvalid && rlast)?0:2;
 2: if(rvalid && rlast) rs<=0;
 default: rs<=0;
 endcase
 case(ws)
 0: if(a_awvalid || b_awvalid) begin wo<=!a_awvalid;ws<=1;end
 1: if(awvalid && awready) ws<=(wready && wlast)?0:2;
 2: if(wready && wlast) ws<=0;
 default: ws<=0;
 endcase
end
assign araddr=ro?b_araddr:a_araddr;
assign arlen=ro?b_arlen:a_arlen;
assign arvalid=rs==1 && (ro?b_arvalid:a_arvalid);
assign a_arready=rs==1 && !ro && arready;
assign b_arready=rs==1 && ro && arready;
wire ractive=rs==2 || (arvalid && arready);
assign a_rvalid=ractive && !ro && rvalid;
assign b_rvalid=ractive && ro && rvalid;
assign a_rlast=a_rvalid && rlast;
assign b_rlast=b_rvalid && rlast;
assign awaddr=wo?b_awaddr:a_awaddr;
assign awlen=wo?b_awlen:a_awlen;
assign awvalid=ws==1 && (wo?b_awvalid:a_awvalid);
assign a_awready=ws==1 && !wo && awready;
assign b_awready=ws==1 && wo && awready;
wire wactive=ws==2 || (awvalid && awready);
assign wdata=wo?b_wdata:a_wdata;
assign wstrb=wo?32'hffffffff:a_wstrb;
assign a_wready=wactive && !wo && wready;
assign b_wready=wactive && wo && wready;
assign a_wlast=a_wready && wlast;
assign b_wlast=b_wready && wlast;
endmodule
