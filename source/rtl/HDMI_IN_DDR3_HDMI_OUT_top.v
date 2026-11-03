`timescale 1ns / 1ps
module HDMI_IN_DDR3_HDMI_OUT_top#(
	parameter MEM_ROW_ADDR_WIDTH   = 15         ,
	parameter MEM_COL_ADDR_WIDTH   = 10         ,
	parameter MEM_BADDR_WIDTH      = 3          ,
	parameter MEM_DQ_WIDTH         =  32        ,
	parameter MEM_DQS_WIDTH        =  32/8       ,
    parameter IMU_AXIS_X=0, IMU_AXIS_Y=1, IMU_AXIS_Z=2,
    parameter IMU_SIGN_X=1, IMU_SIGN_Y=-1, IMU_SIGN_Z=-1,
    parameter HDMI_CORRECTION_ENABLE = 1'b1
)(
	input                                sys_clk              ,//27Mhz
    input                                clk_p ,
    input                                clk_n ,
    input                                user_button0_n ,
    input                                user_button1_n ,
    input                                user_button2_n ,

    // BNO080 on expansion connector J3.
    inout                                imu_scl,
    inout                                imu_sda,
    input                                imu_int_n,
    inout                                imu_rst_n,

//DDR
    output                               mem_rst_n                 ,
    output                               mem_ck                    ,
    output                               mem_ck_n                  ,
    output                               mem_cke                   ,
    output                               mem_cs_n                  ,
    output                               mem_ras_n                 ,
    output                               mem_cas_n                 ,
    output                               mem_we_n                  ,
    output                               mem_odt                   ,
    output      [MEM_ROW_ADDR_WIDTH-1:0] mem_a                     ,
    output      [MEM_BADDR_WIDTH-1:0]    mem_ba                    ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs                   ,
    inout       [MEM_DQ_WIDTH/8-1:0]     mem_dqs_n                 ,
    inout       [MEM_DQ_WIDTH-1:0]       mem_dq                    ,
    output      [MEM_DQ_WIDTH/8-1:0]     mem_dm                    ,
    output reg                           heart_beat_led            ,
    output                               ddr_init_done             ,
    output                               init_over_rx              ,
//MS72xx       
    output                               rstn_out                  ,
    output                               hd_scl                ,
    inout                                hd_sda                ,
    output                               hdmi_int_led              ,//HDMI_OUT初始化完成

    //HDMI_in
    input             pixclk_in    ,                            
    input             vs_in    , 
    input             hs_in    , 
    input             de_in    ,
    input     [7:0]   r_in    , 
    input     [7:0]   g_in    , 
    input     [7:0]   b_in    , 
//HDMI_OUT
    output                               pix_clk                   ,//pixclk                           
    output    reg                           vs_out                    , 
    output    reg                           hs_out                    , 
    output    reg                           de_out                    ,
    output    reg    [7:0]                  r_out                     , 
    output    reg    [7:0]                  g_out                     , 
    output    reg    [7:0]                  b_out         
);
    wire core_clk;
    wire affine_disabled,affine_enabled,affine_ready,affine_reference_ready;
    wire signed [31:0] affine_debug_angle;
    wire [1:0] affine_filter_mode;
    wire [7:0] affine_age_ms;
    wire affine_stale,affine_recovering;
    wire [63:0] affine_perf_status;
    wire [1:0] geometry_mode;
    wire affine_fixed_crop;
    wire affine_mode_enabled=geometry_mode==1;
    wire perspective_enabled=geometry_mode==2;
    assign affine_enabled=geometry_mode!=0;
    assign affine_disabled=!affine_enabled;
    reg [1:0] imu_reset_sync;
/////////////////////////////////////////////////////////////////////////////////////
// ENABLE_DDR
    parameter CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH;//28
    parameter TH_1S = 27'd33000000;
/////////////////////////////////////////////////////////////////////////////////////
    reg  [15:0]                 rstn_1ms            ;
    wire[15:0]                  o_rgb565            ;

//axi bus   
    wire [CTRL_ADDR_WIDTH-1:0]  axi_awaddr                 ;
    wire                        axi_awuser_ap              ;
    wire [3:0]                  axi_awuser_id              ;
    wire [3:0]                  axi_awlen                  ;
    wire                        axi_awready                ;
    wire                        axi_awvalid                ;
    wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata                  ;
    wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb                  ;
    wire                        axi_wready                 ;
    wire [3:0]                  axi_wusero_id              ;
    wire                        axi_wusero_last            ;
    wire [CTRL_ADDR_WIDTH-1:0]  axi_araddr                 ;
    wire                        axi_aruser_ap              ;
    wire [3:0]                  axi_aruser_id              ;
    wire [3:0]                  axi_arlen                  ;
    wire                        axi_arready                ;
    wire                        axi_arvalid                ;
    wire [MEM_DQ_WIDTH*8-1:0]   axi_rdata                   /* synthesis syn_keep = 1 */;
    wire                        axi_rvalid                  /* synthesis syn_keep = 1 */;
    wire [3:0]                  axi_rid                    ;
    wire                        axi_rlast                  ;
    reg  [26:0]                 cnt                        ;
    reg  [15:0]                 cnt_1                      ;
/////////////////////////////////////////////////////////////////////////////////////
//PLL
pll pll_gen_clk (
    .clkin1   (  sys_clk    ),//27MHz
    .clkout0  (  pix_clk    ),//148.5

    .lock (  locked     )
);


cfg_pll cfg_pll_inst (
  .clkout0(cfg_clk),    // output
  .lock(),          // output
  .clkin1(sys_clk)       // input
);




ms72xx_ctl ms72xx_ctl(
    .clk         (  cfg_clk    ), //input       clk,
    .rst_n       (  rstn_out   ), //input       rstn,
           
    .init_over_rx(  rx_init_done),                 
    .init_over   (  init_over  ), //output      init_over,
    .iic_scl     (  hd_scl    ), //output      iic_scl,
    .iic_sda     (  hd_sda    )  //inout       iic_sda
);
    assign   init_over_rx = rx_init_done;
   assign    hdmi_int_led    =    init_over; 
    
    always @(posedge cfg_clk)
    begin
    	if(!locked)
    	    rstn_1ms <= 16'd0;
    	else
    	begin
    		if(rstn_1ms == 16'h2710)
    		    rstn_1ms <= rstn_1ms;
    		else
    		    rstn_1ms <= rstn_1ms + 1'b1;
    	end
    end
    



    reg    rstn_d0;
    reg    rstn_d1;




    assign rstn_out = (rstn_1ms == 16'h2710);


    reg [1:0] input_reset_sync;
    always @(posedge pixclk_in or negedge ddr_init_done)
        if(!ddr_init_done)input_reset_sync<=0;
        else input_reset_sync<={input_reset_sync[0],1'b1};
    wire gamma_requested;
    wire color_key3_n,color_key4_n;
    wire [2:0] requested_input_color,active_input_color;
    input_color_buttons color_buttons(.clk(sys_clk),.rst_n(locked),
        .s3_n(user_button0_n),.s4_n(user_button1_n),
        .key3_n(color_key3_n),.key4_n(color_key4_n),.color_mode(requested_input_color));
    wire [7:0] recovered_r,recovered_g,recovered_b;
    wire recovered_vs,recovered_hs,recovered_de;
    hdmi_input_recovery input_recovery(.clk(pixclk_in),.rst_n(input_reset_sync[1]),
        .in_vs(vs_in),.in_hs(hs_in),.in_de(de_in),.in_r(r_in),.in_g(g_in),.in_b(b_in),
        .mode_clk(sys_clk),.mode_rst_n(locked),.requested_color(requested_input_color),
        .active_color(active_input_color),
        .out_vs(recovered_vs),.out_hs(recovered_hs),.out_de(recovered_de),
        .out_r(recovered_r),.out_g(recovered_g),.out_b(recovered_b));
    wire [7:0] pre_gamma_r,pre_gamma_g,pre_gamma_b;
    wire pre_gamma_vs,pre_gamma_hs,pre_gamma_de;
    wire input_capture_ok;
    wire [63:0] input_video_status;
    wire [15:0] input_capture_sequence;
    wire input_video_valid;
    wire histogram_enabled,histogram_ready;
    wire denoise_active;
    wire [15:0] hybrid_time_ms;
    wire [4:0] hybrid_night_weight;
    wire [1:0] hybrid_scene;
    wire [7:0] hybrid_avg,hybrid_p50,hybrid_p90;
    wire [6:0] hybrid_sat;
    gamma_ms_cdc hybrid_clock(.src_clk(sys_clk),.rst_n(locked),
        .dst_clk(pixclk_in),.time_ms(hybrid_time_ms));
    // Parallel raw-input branches; one frame-latched blend before RGB565/DDR.
    hybrid_capture_rgb pre_correction_histogram(
        .clk(pixclk_in),.rst_n(input_reset_sync[1]),.requested_enabled(gamma_requested),
        .in_r(recovered_r),.in_g(recovered_g),.in_b(recovered_b),
        .in_vs(recovered_vs),.in_hs(recovered_hs),.in_de(recovered_de),
        .out_r(pre_gamma_r),.out_g(pre_gamma_g),.out_b(pre_gamma_b),
        .out_vs(pre_gamma_vs),.out_hs(pre_gamma_hs),.out_de(pre_gamma_de),
        .enabled(histogram_enabled),.ready(histogram_ready),
        .time_ms(hybrid_time_ms),.night_weight(hybrid_night_weight),
        .scene(hybrid_scene),.luma_avg(hybrid_avg),.p50(hybrid_p50),.denoise_active(denoise_active),
        .p90(hybrid_p90),.saturation_percent(hybrid_sat));
    wire [15:0] hdmi_data_in = {pre_gamma_r[7:3],pre_gamma_g[7:2],pre_gamma_b[7:3]};


wire    vs_reg;
wire    hs_reg;
wire    rd_en ;



//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Keep live capture/display at DDR priority A. The correction engine uses
// priority B and is selected only after a complete, fault-free frame exists.
    wire correction_available /* synthesis syn_keep=1 */;
    wire correction_fault /* synthesis syn_keep=1 */;
    wire correction_busy /* synthesis syn_keep=1 */;
    wire correction_displayed /* synthesis syn_keep=1 */;
    wire requested_corrected /* synthesis syn_keep=1 */;
    wire button_requested_corrected;
    (* async_reg="true" *) reg [1:0] affine_enable_ddr;
    always @(posedge core_clk or negedge ddr_init_done)
        if(!ddr_init_done) affine_enable_ddr<=0;
        else affine_enable_ddr<={affine_enable_ddr[0],affine_enabled};
    assign requested_corrected=button_requested_corrected || affine_enable_ddr[1];
    video_input_status input_monitor(.pix_clk(pixclk_in),.clk(core_clk),.rst_n(ddr_init_done),
        .in_vs(recovered_vs),.in_de(recovered_de),.capture_sequence(input_capture_sequence),
        .perf_status(affine_perf_status),.video_valid(input_video_valid),
        .engine_busy(correction_busy),.engine_fault(correction_fault),
        .capture_ok(input_capture_ok),.status(input_video_status));

    // S3 short release selects raw/corrected; hold 2s toggles tone enhancement.
    button_gamma_control mode_button (
        .clk                 (core_clk),
        .rst_n               (ddr_init_done),
        .key_n               (color_key3_n),
        .requested_corrected (button_requested_corrected),
        .gamma_enabled       (gamma_requested)
    );

    wire [287:0] pose_matrix_ddr;
    wire pose_affine_crop_ddr;
    wire pose_valid_ddr,pose_history_active_ddr;
    wire [7:0] sync_delay_index,sync_delay_gray;
    (* async_reg="true" *) reg [7:0] delay_gray_meta,delay_gray_sync;
    always @(posedge core_clk or negedge ddr_init_done)
        if(!ddr_init_done)begin delay_gray_meta<=8'd15;delay_gray_sync<=8'd15;end
        else begin delay_gray_meta<=sync_delay_gray;delay_gray_sync<=delay_gray_meta;end
    wire [7:0] delay_ms_ddr;
    genvar sync_bit;
    generate for(sync_bit=0;sync_bit<8;sync_bit=sync_bit+1)begin: sync_decode
        assign delay_ms_ddr[sync_bit]=^(delay_gray_sync >> sync_bit);
    end endgenerate
    wire diag_display_pulse,diag_display_active;
    wire [31:0] diag_display_angle;
    fram_buf_correction fram_buf(
        .debug_display_valid(diag_display_pulse),.debug_display_angle(diag_display_angle),.debug_display_active(diag_display_active),
        .debug_perf_status(affine_perf_status),
        .debug_source_sequence(input_capture_sequence),.debug_video_valid(input_video_valid),
        .pose_matrix(pose_matrix_ddr),
        .affine_crop(pose_affine_crop_ddr),
        .pose_valid(pose_valid_ddr),.pose_history_active(pose_history_active_ddr),
        .pose_delay_ms(delay_ms_ddr),
        .requested_corrected (requested_corrected),
        .display_corrected   (correction_displayed),
        .correction_available(correction_available),
        .correction_fault    (correction_fault),
        .correction_busy     (correction_busy),
        .ddr_clk        (  core_clk             ),//input                         ddr_clk,
        .ddr_rstn       (  ddr_init_done        ),//input                         ddr_rstn,
        //data_in                                  
        .vin_clk        (  pixclk_in         ),//input                         vin_clk,
        .wr_fsync       (  ~pre_gamma_vs     ),//input                         wr_fsync,
        .wr_en          (  pre_gamma_de ),// diagnostics must not throttle video capture
        .wr_data        (  hdmi_data_in             ),//input  [15 : 0]  wr_data,
        //data_out
        .vout_clk       (  pix_clk              ),//input                         vout_clk,
        .rd_fsync       (  vs_reg               ),//input                         rd_fsync,
        .rd_en          (  rd_en                ),//input                         rd_en,
        .vout_de        (  de_o               ),//output                        vout_de,
        .vout_data      (  o_rgb565             ),//output [PIX_WIDTH- 1'b1 : 0]  vout_data,
        .init_done      (  init_done            ),//output reg                    init_done,
        //axi bus
        .axi_awaddr     (  axi_awaddr           ),// output[27:0]
        .axi_awid       (  axi_awuser_id        ),// output[3:0]
        .axi_awlen      (  axi_awlen            ),// output[3:0]
        .axi_awsize     (                       ),// output[2:0]
        .axi_awburst    (                       ),// output[1:0]
        .axi_awready    (  axi_awready          ),// input
        .axi_awvalid    (  axi_awvalid          ),// output               
        .axi_wdata      (  axi_wdata            ),// output[255:0]
        .axi_wstrb      (  axi_wstrb            ),// output[31:0]
        .axi_wlast      (  axi_wusero_last      ),// input
        .axi_wvalid     (                       ),// output
        .axi_wready     (  axi_wready           ),// input
        .axi_bid        (  4'd0                 ),// input[3:0]
        .axi_araddr     (  axi_araddr           ),// output[27:0]
        .axi_arid       (  axi_aruser_id        ),// output[3:0]
        .axi_arlen      (  axi_arlen            ),// output[3:0]
        .axi_arsize     (                       ),// output[2:0]
        .axi_arburst    (                       ),// output[1:0]
        .axi_arvalid    (  axi_arvalid          ),// output
        .axi_arready    (  axi_arready          ),// input
        .axi_rready     (                       ),// output
        .axi_rdata      (  axi_rdata            ),// input[255:0]
        .axi_rvalid     (  axi_rvalid           ),// input
        .axi_rlast      (  axi_rlast            ),// input
        .axi_rid        (  axi_rid              ) // input[3:0]         
    );

    wire [7:0] display_r = {o_rgb565[15:11], 3'd0};
    wire [7:0] display_g = {o_rgb565[10:5], 2'd0};
    wire [7:0] display_b = {o_rgb565[4:0], 3'd0};

    wire [7:0] mode_r;
    wire [7:0] mode_g;
    wire [7:0] mode_b;
    wire mode_vs;
    wire mode_hs;
    wire mode_de;

    video_mode_osd mode_osd (
        .fixed_crop(affine_fixed_crop && affine_mode_enabled),
        .affine_enabled(affine_enabled),.affine_ready(affine_ready && !affine_stale),
        .delay_gray(sync_delay_gray),
        .clk       (pix_clk),
        .rst_n     (ddr_init_done),
        .corrected (correction_displayed),
        .in_r      (display_r),
        .in_g      (display_g),
        .in_b      (display_b),
        .in_vs     (vs_reg),
        .in_hs     (hs_reg),
        .in_de     (de_o),
        .out_r     (mode_r),
        .out_g     (mode_g),
        .out_b     (mode_b),
        .out_vs    (mode_vs),
        .out_hs    (mode_hs),
        .out_de    (mode_de)
    );

    // Keep the existing parameter panel after the mode indicator.
    wire [7:0] parameter_r;
    wire [7:0] parameter_g;
    wire [7:0] parameter_b;
    wire parameter_vs;
    wire parameter_hs;
    wire parameter_de;

    wire parameters_hidden_sys;
    (* async_reg="true" *) reg [1:0] parameters_hidden_sync;
    always @(posedge pix_clk or negedge ddr_init_done)
        if(!ddr_init_done)parameters_hidden_sync<=2'b11;
        else parameters_hidden_sync<={parameters_hidden_sync[0],parameters_hidden_sys};
    wire parameters_hidden=parameters_hidden_sync[1];
    // S4 short release: parameter panel. Affine crop stays fixed.
    // Hold 1s: +1ms SYNC, repeat every 100ms. Colour chord front-end retained.
    button_sync_control parameter_button (
        .clk                 (sys_clk),
        .rst_n               (imu_reset_sync[1]),
        .key_n               (color_key4_n),
        .parameters_hidden   (parameters_hidden_sys),
        .delay_index(sync_delay_index),.delay_gray(sync_delay_gray),.fixed_crop(affine_fixed_crop)
    );

    parameter_osd parameters (
        .clk               (pix_clk),
        .rst_n             (ddr_init_done),
        .requested_visible (!parameters_hidden),
        .in_r              (mode_r),
        .in_g              (mode_g),
        .in_b              (mode_b),
        .in_vs             (mode_vs),
        .in_hs             (mode_hs),
        .in_de             (mode_de),
        .out_r             (parameter_r),
        .out_g             (parameter_g),
        .out_b             (parameter_b),
        .out_vs            (parameter_vs),
        .out_hs            (parameter_hs),
        .out_de            (parameter_de)
    );

    wire [7:0] gamma_osd_r,gamma_osd_g,gamma_osd_b;
    wire gamma_osd_vs,gamma_osd_hs,gamma_osd_de;
    gamma_status_osd gamma_display(
        .src_clk(pixclk_in),.src_rst_n(input_reset_sync[1]),
        .status({16'd0,denoise_active,6'd0,hybrid_sat,hybrid_p90,hybrid_p50,hybrid_scene,
                 histogram_enabled,histogram_ready,1'b0,hybrid_night_weight,hybrid_avg}),
        .clk(pix_clk),.rst_n(ddr_init_done),
        .in_r(parameter_r),.in_g(parameter_g),.in_b(parameter_b),
        .in_vs(parameter_vs),.in_hs(parameter_hs),.in_de(parameter_de),
        .out_r(gamma_osd_r),.out_g(gamma_osd_g),.out_b(gamma_osd_b),
        .out_vs(gamma_osd_vs),.out_hs(gamma_osd_hs),.out_de(gamma_osd_de));

    wire [7:0] input_osd_r,input_osd_g,input_osd_b;
    wire input_osd_vs,input_osd_hs,input_osd_de;
    wire [2:0] input_color_ddr;
    diagnostic_cdc #(.WIDTH(3)) input_color_status_cdc(.src_clk(pixclk_in),.rst_n(ddr_init_done),
        .src_valid(1'b1),.src_data(active_input_color),.busy(),.lost(),
        .dst_clk(core_clk),.dst_valid(),.dst_data(input_color_ddr));
    video_input_status_osd input_display(.src_clk(core_clk),.clk(pix_clk),.rst_n(ddr_init_done),
        .status({input_color_ddr,input_video_status[60:0]}),
        .in_r(gamma_osd_r),.in_g(gamma_osd_g),.in_b(gamma_osd_b),
        .in_vs(gamma_osd_vs),.in_hs(gamma_osd_hs),.in_de(gamma_osd_de),
        .out_r(input_osd_r),.out_g(input_osd_g),.out_b(input_osd_b),
        .out_vs(input_osd_vs),.out_hs(input_osd_hs),.out_de(input_osd_de));
    // BNO080 remains the final overlay stage.
    always @(posedge sys_clk or negedge locked)
        if (!locked)
            imu_reset_sync <= 2'b00;
        else
            imu_reset_sync <= {imu_reset_sync[0], 1'b1};

    wire signed [15:0] imu_qi;
    wire signed [15:0] imu_qj;
    wire signed [15:0] imu_qk;
    wire signed [15:0] imu_qr;
    wire [1:0] imu_accuracy;
    wire imu_valid;
    wire imu_online;
    wire imu_configured;
    wire [15:0] imu_reports;
    wire [3:0] imu_error;
    wire [4:0] imu_debug_events;

    bno080_i2c imu (
        .clk          (sys_clk),
        .rst_n        (imu_reset_sync[1]),
        .int_n        (imu_int_n),
        .scl          (imu_scl),
        .sda          (imu_sda),
        .sensor_rst_n (imu_rst_n),
        .qi           (imu_qi),
        .qj           (imu_qj),
        .qk           (imu_qk),
        .qr           (imu_qr),
        .accuracy     (imu_accuracy),
        .valid        (imu_valid),
        .online       (imu_online),
        .configured   (imu_configured),
        .reports      (imu_reports),
        .last_error   (imu_error),.debug_events(imu_debug_events)
    );

    
    // Follow deliberate camera motion; smooth only the short-term 3-axis residual.
    wire [63:0] imu_current_q,imu_filtered_q;
    wire imu_filter_valid;
    wire imu_filter_rejected;
    wire imu_accept_report,imu_prediction_bad;
    affine_report_policy affine_quality(
        .clk(sys_clk),.rst_n(imu_reset_sync[1]),.affine_enabled(affine_mode_enabled),
        .online(imu_online),.configured(imu_configured),.report_valid(imu_valid),
        .accuracy(imu_accuracy),.norm_rejected(imu_filter_rejected),
        .accept_report(imu_accept_report),.prediction_bad(imu_prediction_bad));
    quaternion_motion_filter #(.FILTER_SHIFT(4),.ADAPTIVE(1),.MEDIUM_SHIFT(3),.FAST_SHIFT(2),
        .MEDIUM_THRESHOLD(512),.FAST_THRESHOLD(1536)) motion_filter(
        .clk(sys_clk),.rst_n(imu_reset_sync[1]),.clear(!imu_online),
        .in_valid(imu_accept_report),
        .qi(imu_qi),.qj(imu_qj),.qk(imu_qk),.qr(imu_qr),
        .current_q(imu_current_q),.filtered_q(imu_filtered_q),
        .out_valid(imu_filter_valid),.busy(),.ready(),.rejected(imu_filter_rejected));
    wire [63:0] imu_predicted_q;
    wire imu_predict_valid,imu_predict_stale,imu_prediction_active,imu_predict_anomaly;
    quaternion_delay_predictor #(.CLK_HZ(27000000),.PREDICT_MS(30),.STALE_MS(50)) motion_predictor(
        .clk(sys_clk),.rst_n(imu_reset_sync[1]),.enable(affine_mode_enabled),.online(imu_online),
        .in_valid(imu_filter_valid),.filtered_q(imu_filtered_q),
        .bad_event(imu_prediction_bad),
        .out_valid(imu_predict_valid),.predicted_q(imu_predicted_q),.stale(imu_predict_stale),
        .prediction_active(imu_prediction_active),.anomaly(imu_predict_anomaly));
    wire [287:0] imu_pose_matrix;
    // Preserve S5 short affine toggle; add S5 hold-1s perspective toggle.
    button_geometry_control geometry_button(.clk(sys_clk),.rst_n(imu_reset_sync[1]),
        .key_n(user_button2_n),.mode(geometry_mode));
    wire imu_pose_valid,imu_pose_busy,imu_reference_ready,imu_pose_rejected;
    quaternion_pose_shared #(.DYNAMIC_REFERENCE(1),.JITTER_LIMIT(256),.AXIS_X(IMU_AXIS_X),.AXIS_Y(IMU_AXIS_Y),.AXIS_Z(IMU_AXIS_Z),
        .SIGN_X(IMU_SIGN_X),.SIGN_Y(IMU_SIGN_Y),.SIGN_Z(IMU_SIGN_Z)) pose_calc(
        .clk(sys_clk),.rst_n(imu_reset_sync[1]),
        .in_valid(imu_filter_valid),.recenter(!imu_online),
        .suppress_camera_roll(affine_mode_enabled),
        .qi($signed(imu_current_q[15:0])),.qj($signed(imu_current_q[31:16])),
        .qk($signed(imu_current_q[47:32])),.qr($signed(imu_current_q[63:48])),
        .dynamic_reference_q(imu_filtered_q),
        .busy(imu_pose_busy),.out_valid(imu_pose_valid),.reference_ready(imu_reference_ready),
        .rejected(imu_pose_rejected),.matrix(imu_pose_matrix));
    // Both domains use the common PLL reset. Do not independently reset one end of a handshake.
    localparam [287:0] IMU_IDENTITY_POSE={32'd16384,32'd0,32'd0,32'd0,32'd16384,32'd0,32'd0,32'd0,32'd16384};
    reg imu_online_d;
    always @(posedge sys_clk or negedge imu_reset_sync[1])
        if(!imu_reset_sync[1]) imu_online_d<=0;else imu_online_d<=imu_online;
    wire path_affine_reference_ready,path_affine_ready,path_affine_stale,path_affine_recovering;
    wire signed [31:0] path_affine_debug_angle;
    wire [1:0] path_affine_filter_mode;wire [7:0] path_affine_age_ms;
    wire [287:0] affine_pose;
    wire affine_valid;
    wire affine_crop_out;
    wire [63:0] affine_diag_reference_q;
    wire affine_diag_reference_valid,affine_diag_filtered_valid;
    wire signed [31:0] affine_diag_filtered_angle;
    affine_roll_control #(.AXIS_X(IMU_AXIS_X),.AXIS_Y(IMU_AXIS_Y),.AXIS_Z(IMU_AXIS_Z),
        .SIGN_X(IMU_SIGN_X),.SIGN_Y(IMU_SIGN_Y),.SIGN_Z(IMU_SIGN_Z),.COMPOSE_RESIDUAL(0)) affine_control(
        .clk(sys_clk),.rst_n(imu_reset_sync[1]),.enable(affine_mode_enabled),.online(imu_online),
        .filter_valid(imu_predict_valid),.filtered_q(imu_predicted_q),.current_q(imu_current_q),
        .residual_valid(imu_pose_valid),.residual_matrix(imu_pose_matrix),
        .out_valid(affine_valid),.matrix(affine_pose),.out_affine_crop(affine_crop_out),
        .reference_ready(path_affine_reference_ready),.ready(path_affine_ready),
        .debug_angle_q12(path_affine_debug_angle),.debug_filter_mode(path_affine_filter_mode),
        .debug_age_ms(path_affine_age_ms),.pose_stale(path_affine_stale),.pose_recovering(path_affine_recovering),
        .debug_reference_q(affine_diag_reference_q),.debug_reference_valid(affine_diag_reference_valid),
        .debug_filtered_angle(affine_diag_filtered_angle),.debug_filtered_valid(affine_diag_filtered_valid));
    wire [287:0] perspective_pose,composed_pose;
    wire perspective_valid,perspective_ready,perspective_reference_ready,perspective_stale,perspective_recovering;
    wire [63:0] perspective_reference_q,diag_reference_q;
    wire perspective_reference_valid,perspective_filtered_valid,diag_reference_valid,diag_filtered_valid;
    wire signed [31:0] perspective_angle,perspective_filtered_angle,diag_filtered_angle;
    wire [1:0] perspective_filter_mode;wire [7:0] perspective_age_ms;
    wire composed_valid,composed_affine_crop,composed_active;
    perspective_pose_control #(.AXIS_X(IMU_AXIS_X),.AXIS_Y(IMU_AXIS_Y),.AXIS_Z(IMU_AXIS_Z),
        .SIGN_X(IMU_SIGN_X),.SIGN_Y(IMU_SIGN_Y),.SIGN_Z(IMU_SIGN_Z)) perspective_control(
        .clk(sys_clk),.rst_n(imu_reset_sync[1]),.enable(perspective_enabled),.online(imu_online),
        .report_valid(imu_filter_valid),.current_q(imu_current_q),
        .out_valid(perspective_valid),.matrix(perspective_pose),.out_affine_crop(),
        .reference_ready(perspective_reference_ready),.ready(perspective_ready),
        .debug_angle_q12(perspective_angle),.debug_filter_mode(perspective_filter_mode),
        .debug_age_ms(perspective_age_ms),.pose_stale(perspective_stale),.pose_recovering(perspective_recovering),
        .debug_reference_q(perspective_reference_q),.debug_reference_valid(perspective_reference_valid),
        .debug_filtered_angle(perspective_filtered_angle),.debug_filtered_valid(perspective_filtered_valid));
    geometry_pose_select geometry_select(.clk(sys_clk),.rst_n(imu_reset_sync[1]),.mode(geometry_mode),
        .fixed_crop(affine_fixed_crop),
        .affine_valid(affine_valid),.affine_matrix(affine_pose),.affine_crop(affine_crop_out),
        .affine_ready(path_affine_ready && !path_affine_stale && imu_online),
        .perspective_valid(perspective_valid),.perspective_matrix(perspective_pose),
        .perspective_ready(perspective_ready && !perspective_stale && imu_online),
        .out_valid(composed_valid),.out_matrix(composed_pose),.out_crop(composed_affine_crop),.out_active(composed_active));
    assign affine_reference_ready=perspective_enabled?perspective_reference_ready:path_affine_reference_ready;
    assign affine_ready=perspective_enabled?perspective_ready:(affine_mode_enabled && path_affine_ready);
    assign affine_debug_angle=perspective_enabled?perspective_angle:path_affine_debug_angle;
    assign affine_filter_mode=perspective_enabled?perspective_filter_mode:path_affine_filter_mode;
    assign affine_age_ms=perspective_enabled?perspective_age_ms:path_affine_age_ms;
    assign affine_stale=perspective_enabled?perspective_stale:(affine_mode_enabled && path_affine_stale);
    assign affine_recovering=perspective_enabled?perspective_recovering:(affine_mode_enabled && path_affine_recovering);
    assign diag_reference_q=perspective_enabled?perspective_reference_q:affine_diag_reference_q;
    assign diag_reference_valid=perspective_enabled?perspective_reference_valid:affine_diag_reference_valid;
    assign diag_filtered_angle=perspective_enabled?perspective_filtered_angle:affine_diag_filtered_angle;
    assign diag_filtered_valid=perspective_enabled?perspective_filtered_valid:affine_diag_filtered_valid;
    pose_cdc pose_transfer(.src_clk(sys_clk),.src_rst_n(imu_reset_sync[1]),
        .src_valid(composed_valid),.src_matrix(composed_pose),.src_affine_crop(composed_affine_crop),
        .src_history_active(composed_active),
        .dst_clk(core_clk),.dst_rst_n(imu_reset_sync[1]),.dst_matrix(pose_matrix_ddr),
        .dst_affine_crop(pose_affine_crop_ddr),.dst_history_active(pose_history_active_ddr),.dst_valid(pose_valid_ddr));

    wire signed [15:0] imu_roll_deg10;
    wire signed [15:0] imu_pitch_deg10;
    wire signed [15:0] imu_yaw_deg10;
    wire imu_euler_valid;
    wire imu_euler_busy;
    reg  imu_angles_ready;

    quat_to_euler_shared imu_euler (
        .clk         (sys_clk),
        .rst_n       (imu_reset_sync[1]),
        .in_valid    (imu_valid),
        .qi          (imu_qi),
        .qj          (imu_qj),
        .qk          (imu_qk),
        .qr          (imu_qr),
        .roll_deg10  (imu_roll_deg10),
        .pitch_deg10 (imu_pitch_deg10),
        .yaw_deg10   (imu_yaw_deg10),
        .out_valid   (imu_euler_valid),
        .busy        (imu_euler_busy)
    );

    always @(posedge sys_clk or negedge imu_reset_sync[1]) begin
        if (!imu_reset_sync[1])
            imu_angles_ready <= 1'b0;
        else if (!imu_online)
            imu_angles_ready <= 1'b0;
        else if (imu_euler_valid)
            imu_angles_ready <= 1'b1;
    end

    wire [16:0] imu_roll_disp;
    wire [16:0] imu_pitch_disp;
    wire [16:0] imu_yaw_disp;
    wire signed [31:0] affine_tenths_wide=(affine_debug_angle*32'sd10)>>>12;
    wire [16:0] affine_angle_disp;
    angle_format fmt_affine(.angle_deg10(affine_tenths_wide[15:0]),.packed(affine_angle_disp));
    angle_format fmt_roll  (.angle_deg10(imu_roll_deg10),  .packed(imu_roll_disp));
    angle_format fmt_pitch (.angle_deg10(imu_pitch_deg10), .packed(imu_pitch_disp));
    angle_format fmt_yaw   (.angle_deg10(imu_yaw_deg10),   .packed(imu_yaw_disp));

    wire [7:0] imu_osd_r;
    wire [7:0] imu_osd_g;
    wire [7:0] imu_osd_b;
    wire imu_osd_vs;
    wire imu_osd_hs;
    wire imu_osd_de;

    bno080_osd imu_display (
        .debug_status({geometry_mode,affine_enabled,affine_stale,affine_recovering,
                       affine_filter_mode,affine_age_ms,affine_angle_disp}),
        .perf_clk(core_clk),.perf_rst_n(ddr_init_done),.perf_status(affine_perf_status),
        .src_clk   (sys_clk),
        .src_rst_n (imu_reset_sync[1]),
        .status    ({imu_online, imu_configured, imu_accuracy, imu_error,
                     imu_reports, imu_roll_disp, imu_pitch_disp,
                     imu_yaw_disp, imu_angles_ready, 12'd0}),
        .clk       (pix_clk),
        .rst_n     (ddr_init_done),
        .in_r      (input_osd_r),
        .in_g      (input_osd_g),
        .in_b      (input_osd_b),
        .in_vs     (input_osd_vs),
        .in_hs     (input_osd_hs),
        .in_de     (input_osd_de),
        .out_r     (imu_osd_r),
        .out_g     (imu_osd_g),
        .out_b     (imu_osd_b),
        .out_vs    (imu_osd_vs),
        .out_hs    (imu_osd_hs),
        .out_de    (imu_osd_de)
    );

    // Diagnostics observe the existing algorithms. No diagnostic ready signal
    // controls the sensor, frame scheduler, filter or correction engine.
    wire diag_reset_n=imu_reset_sync[1] && ddr_init_done;
    wire diag_frame_sys_valid;
    wire [48:0] diag_frame_sys_data;
    wire [15:0] diag_frame_lost,diag_scope_lost;
    diagnostic_cdc #(.WIDTH(49)) diag_frame_transfer(
        .src_clk(core_clk),.rst_n(diag_reset_n),.src_valid(diag_display_pulse),
        .src_data({diag_frame_lost,diag_display_active,diag_display_angle}),.busy(),.lost(diag_frame_lost),
        .dst_clk(sys_clk),.dst_valid(diag_frame_sys_valid),.dst_data(diag_frame_sys_data));
    wire diag_sample_valid,diag_pixel_valid;
    wire [383:0] diag_sample,diag_pixel_sample;
    diagnostic_capture #(.AXIS_X(IMU_AXIS_X),.AXIS_Y(IMU_AXIS_Y),.AXIS_Z(IMU_AXIS_Z),
        .SIGN_X(IMU_SIGN_X),.SIGN_Y(IMU_SIGN_Y),.SIGN_Z(IMU_SIGN_Z)) diag_capture(
        .clk(sys_clk),.rst_n(diag_reset_n),.enabled(affine_enabled),.online(imu_online),
        .reference_valid(diag_reference_valid),.reference_q(diag_reference_q),
        .report_valid(imu_valid),.report_q({imu_qr,imu_qk,imu_qj,imu_qi}),.accuracy(imu_accuracy),
        .events(imu_debug_events),.filter_rejected(imu_filter_rejected),.pose_stale(affine_stale),
        .predict_anomaly(imu_predict_anomaly),
        .filtered_valid(diag_filtered_valid),.filtered_angle(diag_filtered_angle),
        .display_valid(diag_frame_sys_valid),.display_data(diag_frame_sys_data),
        .telemetry_lost(diag_scope_lost),.out_valid(diag_sample_valid),.data(diag_sample));
    diagnostic_cdc #(.WIDTH(384)) diag_scope_transfer(
        .src_clk(sys_clk),.rst_n(diag_reset_n),.src_valid(diag_sample_valid),.src_data(diag_sample),
        .busy(),.lost(diag_scope_lost),.dst_clk(pix_clk),.dst_valid(diag_pixel_valid),.dst_data(diag_pixel_sample));
    wire [7:0] diag_r,diag_g,diag_b;
    wire diag_vs,diag_hs,diag_de;
    // S4 single-click controls only the distortion parameter panel.
    // Disable the scope overlay; retain its aligned five-clock RGB/control
    // bypass so Gamma, frame-rate and sensor overlays keep their timing.
    diagnostic_scope_osd diag_display(
        .clk(pix_clk),.rst_n(diag_reset_n),.visible(1'b0),
        .sample_valid(diag_pixel_valid),.sample(diag_pixel_sample),
        .in_r(imu_osd_r),.in_g(imu_osd_g),.in_b(imu_osd_b),.in_vs(imu_osd_vs),.in_hs(imu_osd_hs),.in_de(imu_osd_de),
        .out_r(diag_r),.out_g(diag_g),.out_b(diag_b),.out_vs(diag_vs),.out_hs(diag_hs),.out_de(diag_de));

    // Keep a common final output register for pixels and all timing signals.
    always @(posedge pix_clk) begin
        vs_out <= diag_vs;
        hs_out <= diag_hs;
        de_out <= diag_de;
        r_out  <= diag_r;
        g_out  <= diag_g;
        b_out  <= diag_b;
    end
/////////////////////////////////////////////////////////////////////////////////////
//产生visa时序 
wire                        hs         ;
wire                        vs         ;
wire                        de         ;
//MODE_1080p
parameter V_TOTAL = 12'd1125;  //场扫描周期
parameter V_FP = 12'd4;        //场显示前沿
parameter V_BP = 12'd36;       //场显示后沿
parameter V_SYNC = 12'd5;      //场同步
parameter V_ACT = 12'd1080;    //场有效数据
parameter H_TOTAL = 12'd2200;  //行扫描周期
parameter H_FP = 12'd88;       //行显示前沿
parameter H_BP = 12'd148;      //行显示后沿
parameter H_SYNC = 12'd44;     //行同步
parameter H_ACT = 12'd1920;    //行数据有效
parameter HV_OFFSET = 12'd0;   
parameter   X_WIDTH = 4'd12;
parameter   Y_WIDTH = 4'd12; 
wire [X_WIDTH - 1'b1:0]     act_x      ;
wire [Y_WIDTH - 1'b1:0]     act_y      ;  
sync_vg #(
    .X_BITS               (  X_WIDTH              ), 
    .Y_BITS               (  Y_WIDTH              ),
    .V_TOTAL              (  V_TOTAL              ),//                        
    .V_FP                 (  V_FP                 ),//                        
    .V_BP                 (  V_BP                 ),//                        
    .V_SYNC               (  V_SYNC               ),//                        
    .V_ACT                (  V_ACT                ),//                        
    .H_TOTAL              (  H_TOTAL              ),//                        
    .H_FP                 (  H_FP                 ),//                        
    .H_BP                 (  H_BP                 ),//                        
    .H_SYNC               (  H_SYNC               ),//                        
    .H_ACT                (  H_ACT                ) //                        

) sync_vg                                         
(                                                 
    .clk                  (  pix_clk               ),//input                   clk,                                 
    .rstn                 (  ddr_init_done                 ),//input                   rstn,                            
    .vs_out               (  vs_reg                   ),//output reg              vs_out,                                                                                                                                      
    .hs_out               (  hs_reg                   ),//output reg              hs_out,            
    .de_out               (  rd_en                   ),//output reg              de_out,             
    .x_act                (  act_x                ),//output reg [X_BITS-1:0] x_out,             
    .y_act                (  act_y                ) //output reg [Y_BITS:0]   y_out,             
); 


////////////////////////////////////////////////////////////////////////////////////////////

wire clk_125Mhz ;

GTP_INBUFGDS #(
    .IOSTANDARD("DEFAULT"),
    .TERM_DIFF("ON")
) u_gtp (
    .O(clk_125Mhz), // OUTPUT  
    .I(clk_p), // INPUT  
    .IB(clk_n) // INPUT  
);

//ddr    
        ddr3_test u_ddr3_test_h (
             .ref_clk                   (clk_125Mhz            ),
             .resetn                    (rstn_out           ),// input
             .ddr_init_done             (ddr_init_done      ),// output

             .pll_lock                  (pll_lock           ),// output

             .core_clk                  (core_clk),                                  // output

             .phy_pll_lock              (phy_pll_lock),                          // output
             .gpll_lock                 (gpll_lock),                                // output
             .rst_gpll_lock             (rst_gpll_lock),                        // output
             .ddrphy_cpd_lock           (ddrphy_cpd_lock),                    // output
             //.ddr_init_done             (ddr_init_done),                        // output


             .axi_awaddr                (axi_awaddr         ),// input [27:0]
             .axi_awuser_ap             (1'b0               ),// input
             .axi_awuser_id             (axi_awuser_id      ),// input [3:0]
             .axi_awlen                 (axi_awlen          ),// input [3:0]
             .axi_awready               (axi_awready        ),// output
             .axi_awvalid               (axi_awvalid        ),// input
             .axi_wdata                 (axi_wdata          ),
             .axi_wstrb                 (axi_wstrb          ),// input [31:0]
             .axi_wready                (axi_wready         ),// output
             .axi_wusero_id             (                   ),// output [3:0]
             .axi_wusero_last           (axi_wusero_last    ),// output
             .axi_araddr                (axi_araddr         ),// input [27:0]
             .axi_aruser_ap             (1'b0               ),// input
             .axi_aruser_id             (axi_aruser_id      ),// input [3:0]
             .axi_arlen                 (axi_arlen          ),// input [3:0]
             .axi_arready               (axi_arready        ),// output
             .axi_arvalid               (axi_arvalid        ),// input
             .axi_rdata                 (axi_rdata          ),// output [255:0]
             .axi_rid                   (axi_rid            ),// output [3:0]
             .axi_rlast                 (axi_rlast          ),// output
             .axi_rvalid                (axi_rvalid         ),// output

             .apb_clk                   (1'b0               ),// input
             .apb_rst_n                 (1'b1               ),// input
             .apb_sel                   (1'b0               ),// input
             .apb_enable                (1'b0               ),// input
             .apb_addr                  (8'b0               ),// input [7:0]
             .apb_write                 (1'b0               ),// input
             .apb_ready                 (                   ), // output
             .apb_wdata                 (16'b0              ),// input [15:0]
             .apb_rdata                 (                   ),// output [15:0]
//             .apb_int                   (                   ),// output

             .mem_rst_n                 (mem_rst_n          ),// output
             .mem_ck                    (mem_ck             ),// output
             .mem_ck_n                  (mem_ck_n           ),// output
             .mem_cke                   (mem_cke            ),// output
             .mem_cs_n                  (mem_cs_n           ),// output
             .mem_ras_n                 (mem_ras_n          ),// output
             .mem_cas_n                 (mem_cas_n          ),// output
             .mem_we_n                  (mem_we_n           ),// output
             .mem_odt                   (mem_odt            ),// output
             .mem_a                     (mem_a              ),// output [14:0]
             .mem_ba                    (mem_ba             ),// output [2:0]
             .mem_dqs                   (mem_dqs            ),// inout [3:0]
             .mem_dqs_n                 (mem_dqs_n          ),// inout [3:0]
             .mem_dq                    (mem_dq             ),// inout [31:0]
             .mem_dm                    (mem_dm             ),// output [3:0]
             //debug

  .dbg_gate_start(1'b0),                      // input
  .dbg_cpd_start(1'b0),                        // input
  .dbg_ddrphy_rst_n(1'b1),                  // input
  .dbg_gpll_scan_rst(1'b0),                // input
  .samp_position_dyn_adj(1'b0),        // input
  .init_samp_position_even(32'd0),    // input [31:0]
  .init_samp_position_odd(32'd0),      // input [31:0]
  .wrcal_position_dyn_adj(1'b0),      // input
  .init_wrcal_position(32'd0),            // input [31:0]
  .force_read_clk_ctrl(1'b0),            // input
  .init_slip_step(16'd0),                      // input [15:0]
  .init_read_clk_ctrl(12'd0),              // input [11:0]
  .debug_calib_ctrl(),                  // output [33:0]
  .dbg_slice_status(),                  // output [67:0]
  .dbg_slice_state(),                    // output [87:0]
  .debug_data(),                              // output [275:0]
  .dbg_dll_upd_state(),                // output [1:0]
  .debug_gpll_dps_phase(),          // output [8:0]
  .dbg_rst_dps_state(),                // output [2:0]
  .dbg_tran_err_rst_cnt(),          // output [5:0]
  .dbg_ddrphy_init_fail(),          // output
  .debug_cpd_offset_adj(1'b0),          // input
  .debug_cpd_offset_dir(1'b0),          // input
  .debug_cpd_offset(10'd0),                  // input [9:0]
  .debug_dps_cnt_dir0(),              // output [9:0]
  .debug_dps_cnt_dir1(),              // output [9:0]
  .ck_dly_en(1'b0),                                // input
  .init_ck_dly_step(8'h0),                  // input [7:0]
  .ck_dly_set_bin(),                      // output [7:0]
  .align_error(),                            // output
  .debug_rst_state(),                    // output [3:0]
  .debug_cpd_state()                     // output [3:0]
       );

//心跳信号
     always@(posedge core_clk) begin
        if (!ddr_init_done)
            cnt <= 27'd0;
        else if ( cnt >= TH_1S )
            cnt <= 27'd0;
        else
            cnt <= cnt + 27'd1;
     end

     always @(posedge core_clk)
        begin
        if (!ddr_init_done)
            heart_beat_led <= 1'd1;
        else if ( cnt >= TH_1S )
            heart_beat_led <= ~heart_beat_led;
    end
                 
/////////////////////////////////////////////////////////////////////////////////////
endmodule
