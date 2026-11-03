// DDR-backed full-frame correction wrapper.
//
// Addresses are 32-bit words. Raw frames use four slots below 0x1000000;
// corrected frames use three slots starting at 0x1000000. Live capture and
// display have arbiter priority, so correction cannot change HDMI timing.
module fram_buf_correction #(
    parameter MEM_ROW_WIDTH       = 15,
    parameter MEM_COLUMN_WIDTH    = 10,
    parameter MEM_BANK_WIDTH      = 3,
    parameter CTRL_ADDR_WIDTH     = MEM_ROW_WIDTH + MEM_BANK_WIDTH + MEM_COLUMN_WIDTH,
    parameter MEM_DQ_WIDTH        = 32,
    parameter H_NUM               = 12'd1920,
    parameter V_NUM               = 12'd1080,
    parameter PIX_WIDTH           = 16,
    parameter STREAM_CAPTURE = 1,
    parameter MAX_FRAME_CYCLES    = 32'd250000000
)(
    input                          requested_corrected,
    input [287:0] pose_matrix,
    input affine_crop,
    input pose_valid,pose_history_active,
    input [7:0] pose_delay_ms,
    output [63:0] debug_perf_status,
    output [15:0] debug_source_sequence,
    output debug_video_valid,
    output debug_display_valid,output [31:0] debug_display_angle,output debug_display_active,
    output reg                     display_corrected,
    output                         correction_available,
    output                         correction_fault,
    output                         correction_busy,
    input                          vin_clk,
    input                          wr_fsync,
    input                          wr_en,
    input      [PIX_WIDTH-1:0]     wr_data,
    output                         init_done,
    input                          ddr_clk,
    input                          ddr_rstn,
    input                          vout_clk,
    input                          rd_fsync,
    input                          rd_en,
    output                         vout_de,
    output     [PIX_WIDTH-1:0]     vout_data,
    output     [CTRL_ADDR_WIDTH-1:0] axi_awaddr,
    output     [3:0]               axi_awid,
    output     [3:0]               axi_awlen,
    output     [2:0]               axi_awsize,
    output     [1:0]               axi_awburst,
    input                          axi_awready,
    output                         axi_awvalid,
    output     [MEM_DQ_WIDTH*8-1:0] axi_wdata,
    output     [MEM_DQ_WIDTH-1:0]  axi_wstrb,
    input                          axi_wlast,
    output                         axi_wvalid,
    input                          axi_wready,
    input      [3:0]               axi_bid,
    output     [CTRL_ADDR_WIDTH-1:0] axi_araddr,
    output     [3:0]               axi_arid,
    output     [3:0]               axi_arlen,
    output     [2:0]               axi_arsize,
    output     [1:0]               axi_arburst,
    output                         axi_arvalid,
    input                          axi_arready,
    output                         axi_rready,
    input      [MEM_DQ_WIDTH*8-1:0] axi_rdata,
    input                          axi_rvalid,
    input                          axi_rlast,
    input      [3:0]               axi_rid
);
    wire [27:0] a_awaddr;
    wire [27:0] a_araddr;
    wire [27:0] b_awaddr;
    wire [27:0] b_araddr;
    wire [3:0] a_awlen;
    wire [3:0] a_arlen;
    wire [3:0] b_awlen;
    wire [3:0] b_arlen;
    wire a_awvalid;
    wire a_awready;
    wire a_wready;
    wire a_wlast;
    wire a_arvalid;
    wire a_arready;
    wire a_rvalid;
    wire a_rlast;
    wire b_awvalid;
    wire b_awready;
    wire b_wready;
    wire b_wlast;
    wire b_arvalid;
    wire b_arready;
    wire b_rvalid;
    wire b_rlast;
    wire [255:0] a_wdata;
    wire [255:0] b_wdata;
    wire [31:0] a_wstrb;

    wire source_valid;
    wire corrected_valid;
    wire [15:0] source_sequence;
    wire [1:0] source_slot;
    wire [1:0] write_slot;
    wire [1:0] display_slot;
    wire [1:0] corrected_slot;
    wire engine_start;
    wire engine_busy;
    wire engine_done;
    wire engine_fault;
    wire [15:0] engine_sequence;
    wire [27:0] source_base;
    wire source_hold_valid;
    wire [1:0] source_hold_slot;
    wire [27:0] dest_base;
    wire [31:0] completed_frames;
    wire [31:0] dropped_frames;
    wire [31:0] frame_cycles;
    wire [31:0] engine_clipped_tiles;
    wire [15:0] display_data;
    wire frame_valid;
    wire active_corrected;
    assign debug_source_sequence=source_sequence;
    assign debug_video_valid=frame_valid;
    affine_perf_monitor performance(
        .clk(ddr_clk),.rst_n(ddr_rstn),.enabled(requested_corrected),
        .completed_frames(completed_frames),.dropped_frames(dropped_frames),
        .engine_start(engine_start),.engine_busy(engine_busy),.engine_done(engine_done),
        .engine_sequence(engine_sequence),.clipped_tiles(engine_clipped_tiles),
        .active_corrected(active_corrected),.frame_valid(frame_valid),
        .display_slot(display_slot),.status(debug_perf_status));

    reg timeout_fault;
    always @(posedge ddr_clk or negedge ddr_rstn) begin
        if (!ddr_rstn)
            timeout_fault <= 1'b0;
        else if (engine_start)
            timeout_fault <= 1'b0;
        else if (engine_busy && frame_cycles >= MAX_FRAME_CYCLES)
            timeout_fault <= 1'b1;
    end

    assign correction_available = corrected_valid;
    assign correction_fault = engine_fault || timeout_fault;
    assign correction_busy = engine_busy;

    reg [1:0] ready_sync;
    reg [1:0] mode_sync;
    reg video_ready;
    always @(posedge vout_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            ready_sync <= 2'b00;
            mode_sync <= 2'b11;
            video_ready <= 1'b0;
            display_corrected <= 1'b1;
        end else begin
            ready_sync <= {ready_sync[0], frame_valid};
            mode_sync <= {mode_sync[0], active_corrected};
            if (rd_fsync) begin
                video_ready <= ready_sync[1];
                display_corrected <= mode_sync[1];
            end
        end
    end

    assign vout_data = video_ready ? display_data : {PIX_WIDTH{1'b0}};
    assign axi_awid = 4'd0;
    assign axi_arid = 4'd0;
    assign axi_awsize = 3'b110;
    assign axi_arsize = 3'b110;
    assign axi_awburst = 2'b01;
    assign axi_arburst = 2'b01;
    assign axi_rready = 1'b1;
    assign axi_wvalid = axi_wready;

    fram_buf_capture_display #(
        .MEM_ROW_WIDTH    (MEM_ROW_WIDTH),
        .MEM_COLUMN_WIDTH (MEM_COLUMN_WIDTH),
        .MEM_BANK_WIDTH   (MEM_BANK_WIDTH),
        .CTRL_ADDR_WIDTH  (CTRL_ADDR_WIDTH),
        .MEM_DQ_WIDTH     (MEM_DQ_WIDTH),
        .H_NUM            (H_NUM),
        .V_NUM            (V_NUM),
        .PIX_WIDTH        (PIX_WIDTH)
    ) live_path (
        .capture_row_done(capture_row_done),
        .source_hold_valid(source_hold_valid),.source_hold_slot(source_hold_slot),
        .requested_corrected(requested_corrected),
        .active_corrected(active_corrected),
        .frame_valid(frame_valid),
        .corrected_valid(corrected_valid),
        .corrected_slot(corrected_slot),
        .display_slot(display_slot),
        .source_valid(source_valid),
        .source_slot(source_slot),
        .source_sequence(source_sequence),
        .write_slot(write_slot),
        .vin_clk(vin_clk),
        .wr_fsync(wr_fsync),
        .wr_en(wr_en),
        .wr_data(wr_data),
        .init_done(init_done),
        .ddr_clk(ddr_clk),
        .ddr_rstn(ddr_rstn),
        .vout_clk(vout_clk),
        .rd_fsync(rd_fsync),
        .rd_en(rd_en),
        .vout_de(vout_de),
        .vout_data(display_data),
        .axi_awaddr(a_awaddr),
        .axi_awid(),
        .axi_awlen(a_awlen),
        .axi_awsize(),
        .axi_awburst(),
        .axi_awready(a_awready),
        .axi_awvalid(a_awvalid),
        .axi_wdata(a_wdata),
        .axi_wstrb(a_wstrb),
        .axi_wlast(a_wlast),
        .axi_wvalid(),
        .axi_wready(a_wready),
        .axi_bid(axi_bid),
        .axi_araddr(a_araddr),
        .axi_arid(),
        .axi_arlen(a_arlen),
        .axi_arsize(),
        .axi_arburst(),
        .axi_arvalid(a_arvalid),
        .axi_arready(a_arready),
        .axi_rready(),
        .axi_rdata(axi_rdata),
        .axi_rvalid(a_rvalid),
        .axi_rlast(a_rlast),
        .axi_rid(axi_rid)
    );

    wire capture_row_done;
    wire offer_valid,read_complete,read_bad;
    wire [1:0] offer_slot;
    wire [15:0] offer_sequence;
    wire [10:0] read_rows;
    capture_progress #(.STREAM_CAPTURE(STREAM_CAPTURE),.HEIGHT(V_NUM)) progress(
      .clk(ddr_clk),.rst_n(ddr_rstn),.write_slot(write_slot),.row_done(capture_row_done),
      .source_valid(source_valid),.source_slot(source_slot),.read_slot(source_base[23:22]),
      .offer_valid(offer_valid),.offer_slot(offer_slot),.offer_sequence(offer_sequence),
      .read_rows(read_rows),.read_complete(read_complete),.read_bad(read_bad));
    wire source_pose_valid,matched_crop;
    correction_scheduler #(.STREAM_CAPTURE(STREAM_CAPTURE)) scheduler (
        .source_complete(read_complete),.source_bad(read_bad || timeout_fault),
        .enable(requested_corrected),
        .source_hold_valid(source_hold_valid),.source_hold_slot(source_hold_slot),
        .clk(ddr_clk),
        .rst_n(ddr_rstn),
        .source_valid(offer_valid && source_pose_valid),
        .source_sequence(offer_sequence),
        .source_slot(offer_slot),
        .write_slot(write_slot),
        .display_slot(display_slot),
        .engine_busy(engine_busy),
        .engine_done(engine_done),
        .engine_fault(correction_fault),
        .engine_start(engine_start),
        .source_base(source_base),
        .dest_base(dest_base),
        .corrected_valid(corrected_valid),
        .corrected_slot(corrected_slot),
        .completed_frames(completed_frames),
        .dropped_frames(dropped_frames),
        .engine_sequence(engine_sequence)
    );

    reg [1:0] pose_write_slot;
    wire [287:0] matched_pose;
    always @(posedge ddr_clk or negedge ddr_rstn)
        if(!ddr_rstn)pose_write_slot<=0;
        else pose_write_slot<=write_slot;
    frame_pose_match #(.EARLY_QUERY(STREAM_CAPTURE)) pose_alignment(
        .clk(ddr_clk),.rst_n(ddr_rstn),.sample_valid(pose_valid),
        .sample_matrix(pose_matrix),.sample_crop(affine_crop),.sample_active(pose_history_active),
        .frame_start(pose_write_slot!=write_slot),.frame_slot(write_slot),.delay_ms(pose_delay_ms),
        .source_valid(offer_valid),.source_slot(offer_slot),.read_slot(source_base[23:22]),
        .source_ready(source_pose_valid),.read_matrix(matched_pose),.read_crop(matched_crop));

    correction_engine_pose #(.STREAM_CAPTURE(STREAM_CAPTURE)) engine (
        .source_rows(read_rows),.source_bad(read_bad || timeout_fault),
        .pose_matrix(matched_pose),
        .affine_crop(matched_crop),
        .clipped_tiles(engine_clipped_tiles),
        .clk(ddr_clk),
        .rst_n(ddr_rstn),
        .start(engine_start),
        .source_base(source_base),
        .dest_base(dest_base),
        .busy(engine_busy),
        .done(engine_done),
        .fault(engine_fault),
        .araddr(b_araddr),
        .arlen(b_arlen),
        .arvalid(b_arvalid),
        .arready(b_arready),
        .rdata(axi_rdata),
        .rvalid(b_rvalid),
        .rlast(b_rlast),
        .awaddr(b_awaddr),
        .awlen(b_awlen),
        .awvalid(b_awvalid),
        .awready(b_awready),
        .wdata(b_wdata),
        .wready(b_wready),
        .wlast(b_wlast),
        .frame_cycles(frame_cycles)
    );

    // Observation only: renderer and display buffer ownership are unchanged.
    diagnostic_display_angle display_angle_monitor(
        .clk(ddr_clk),.rst_n(ddr_rstn),.engine_start(engine_start),.dest_slot(dest_base[23:22]),
        .matrix(matched_pose),.crop(matched_crop),
        .active_corrected(active_corrected),.frame_valid(frame_valid),.display_slot(display_slot),
        .out_valid(debug_display_valid),.angle(debug_display_angle),.angle_valid(debug_display_active));

    native_ddr_arbiter arbiter (
        .clk(ddr_clk),
        .rst_n(ddr_rstn),
        .a_araddr(a_araddr),
        .a_arlen(a_arlen),
        .a_arvalid(a_arvalid),
        .a_arready(a_arready),
        .a_rvalid(a_rvalid),
        .a_rlast(a_rlast),
        .b_araddr(b_araddr),
        .b_arlen(b_arlen),
        .b_arvalid(b_arvalid),
        .b_arready(b_arready),
        .b_rvalid(b_rvalid),
        .b_rlast(b_rlast),
        .araddr(axi_araddr),
        .arlen(axi_arlen),
        .arvalid(axi_arvalid),
        .arready(axi_arready),
        .rvalid(axi_rvalid),
        .rlast(axi_rlast),
        .a_awaddr(a_awaddr),
        .a_awlen(a_awlen),
        .a_awvalid(a_awvalid),
        .a_awready(a_awready),
        .a_wdata(a_wdata),
        .a_wstrb(a_wstrb),
        .a_wready(a_wready),
        .a_wlast(a_wlast),
        .b_awaddr(b_awaddr),
        .b_awlen(b_awlen),
        .b_awvalid(b_awvalid),
        .b_awready(b_awready),
        .b_wdata(b_wdata),
        .b_wready(b_wready),
        .b_wlast(b_wlast),
        .awaddr(axi_awaddr),
        .awlen(axi_awlen),
        .awvalid(axi_awvalid),
        .awready(axi_awready),
        .wdata(axi_wdata),
        .wstrb(axi_wstrb),
        .wready(axi_wready),
        .wlast(axi_wlast)
    );
endmodule
