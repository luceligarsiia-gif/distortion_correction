`timescale 1ns/1ps
// BNO080 Rotation Vector quaternion (I,J,K,Real), signed Q14,
// converted to Roll/Pitch/Yaw in 0.1 degree units.
//
// The equations match the common BNO08x quaternion->Euler form:
//   roll  = atan2(2*(qj*qk + qi*qr), -qi^2-qj^2+qk^2+qr^2)
//   pitch = asin (2*(qr*qj - qi*qk) / |q|^2)
//   yaw   = atan2(2*(qi*qj + qk*qr),  qi^2-qj^2-qk^2+qr^2)
//
// For pitch, asin(s) is evaluated as atan2(s, sqrt(1-s^2)).  The sqrt is
// avoided by reusing the magnitude from the roll CORDIC.  This also keeps
// the result correct when the received quaternion norm is slightly away from 1.
module quat_to_euler_shared(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               in_valid,
    input  wire signed [15:0] qi,
    input  wire signed [15:0] qj,
    input  wire signed [15:0] qk,
    input  wire signed [15:0] qr,
    output reg  signed [15:0] roll_deg10,
    output reg  signed [15:0] pitch_deg10,
    output reg  signed [15:0] yaw_deg10,
    output reg                out_valid,
    output reg                busy
);
    localparam W = 40;

    // One local 17x17 signed multiplier, reused for ten quaternion products
    // and three limbs of the 40x17 CORDIC gain product. No pixel-path sharing.
    reg signed [15:0] sample_i, sample_j, sample_k, sample_r;
    reg [3:0] product_index;
    reg [1:0] gain_index;
    reg signed [16:0] mul_a, mul_b;
    reg signed [33:0] mul_product;
    reg signed [56:0] gain_acc;
    reg signed [39:0] pitch_y_scaled;
    wire signed [56:0] gain_term =
        $signed({{23{mul_product[33]}},mul_product}) <<< (gain_index * 16);

    // Q14 * Q14 -> Q28.  Widen before sums/differences so no intermediate
    // overflow is introduced by Verilog expression sizing.
    reg signed [31:0] p_wx;
    reg signed [31:0] p_wy;
    reg signed [31:0] p_wz;
    reg signed [31:0] p_xy;
    reg signed [31:0] p_xz;
    reg signed [31:0] p_yz;
    reg signed [31:0] p_xx;
    reg signed [31:0] p_yy;
    reg signed [31:0] p_zz;
    reg signed [31:0] p_ww;

    wire signed [W-1:0] wx = {{(W-32){p_wx[31]}}, p_wx};
    wire signed [W-1:0] wy = {{(W-32){p_wy[31]}}, p_wy};
    wire signed [W-1:0] wz = {{(W-32){p_wz[31]}}, p_wz};
    wire signed [W-1:0] xy = {{(W-32){p_xy[31]}}, p_xy};
    wire signed [W-1:0] xz = {{(W-32){p_xz[31]}}, p_xz};
    wire signed [W-1:0] yz = {{(W-32){p_yz[31]}}, p_yz};
    wire signed [W-1:0] xx = {{(W-32){p_xx[31]}}, p_xx};
    wire signed [W-1:0] yy = {{(W-32){p_yy[31]}}, p_yy};
    wire signed [W-1:0] zz = {{(W-32){p_zz[31]}}, p_zz};
    wire signed [W-1:0] ww = {{(W-32){p_ww[31]}}, p_ww};

    wire signed [W-1:0] roll_y_calc  = (yz + wx) <<< 1;
    wire signed [W-1:0] roll_x_calc  = ww + zz - xx - yy;
    wire signed [W-1:0] pitch_y_calc = (wy - xz) <<< 1;
    wire signed [W-1:0] yaw_y_calc   = (xy + wz) <<< 1;
    wire signed [W-1:0] yaw_x_calc   = ww + xx - yy - zz;

    reg signed [W-1:0] roll_y, roll_x, pitch_y, yaw_y, yaw_x;
    reg signed [W-1:0] roll_mag;

    // CORDIC gain K ~= 1.646760258, represented as Q15 53961.
    // roll_mag is already multiplied by K by the first CORDIC, so pitch_y
    // is multiplied by the same K before the pitch atan2.
    // Gain multiplication is split into unsigned low/middle 16-bit limbs
    // and a signed high 8-bit limb, preserving the original full-width result.

    reg cordic_start;
    reg signed [W-1:0] cordic_x, cordic_y;
    wire cordic_busy, cordic_done;
    wire signed [15:0] cordic_angle;
    wire signed [W-1:0] cordic_magnitude;

    cordic_atan2_nomul #(.W(W)) u_cordic (
        .clk(clk), .rst_n(rst_n), .start(cordic_start),
        .x_in(cordic_x), .y_in(cordic_y),
        .busy(cordic_busy), .done(cordic_done),
        .angle_deg10(cordic_angle), .magnitude(cordic_magnitude)
    );

    localparam ST_IDLE        = 4'd0;
    localparam ST_ROLL_START  = 4'd1;
    localparam ST_ROLL_WAIT   = 4'd2;
    localparam ST_PITCH_START = 4'd3;
    localparam ST_PITCH_WAIT  = 4'd4;
    localparam ST_YAW_START   = 4'd5;
    localparam ST_YAW_WAIT    = 4'd6;
    localparam ST_PRODUCT_ISSUE = 4'd7;
    localparam ST_PRODUCT_SAVE  = 4'd8;
    localparam ST_PREPARE       = 4'd9;
    localparam ST_GAIN_ISSUE    = 4'd10;
    localparam ST_GAIN_SAVE     = 4'd11;
    reg [3:0] state;

    always @* begin
        mul_a = 17'sd0;
        mul_b = 17'sd0;
        if (state == ST_GAIN_ISSUE || state == ST_GAIN_SAVE) begin
            case (gain_index)
                0: mul_a = $signed({1'b0,pitch_y[15:0]});
                1: mul_a = $signed({1'b0,pitch_y[31:16]});
                default: mul_a = $signed({{9{pitch_y[39]}},pitch_y[39:32]});
            endcase
            mul_b = 17'sd53961;
        end else begin
            case (product_index)
                0: begin mul_a=sample_r; mul_b=sample_i; end
                1: begin mul_a=sample_r; mul_b=sample_j; end
                2: begin mul_a=sample_r; mul_b=sample_k; end
                3: begin mul_a=sample_i; mul_b=sample_j; end
                4: begin mul_a=sample_i; mul_b=sample_k; end
                5: begin mul_a=sample_j; mul_b=sample_k; end
                6: begin mul_a=sample_i; mul_b=sample_i; end
                7: begin mul_a=sample_j; mul_b=sample_j; end
                8: begin mul_a=sample_k; mul_b=sample_k; end
                9: begin mul_a=sample_r; mul_b=sample_r; end
            endcase
        end
    end
    always @(posedge clk) mul_product <= $signed(mul_a) * $signed(mul_b);



    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            roll_deg10  <= 16'sd0;
            pitch_deg10 <= 16'sd0;
            yaw_deg10   <= 16'sd0;
            out_valid   <= 1'b0;
            busy        <= 1'b0;
            roll_y      <= {W{1'b0}};
            roll_x      <= {W{1'b0}};
            pitch_y     <= {W{1'b0}};
            yaw_y       <= {W{1'b0}};
            yaw_x       <= {W{1'b0}};
            roll_mag    <= {W{1'b0}};
            cordic_start<= 1'b0;
            cordic_x    <= {W{1'b0}};
            cordic_y    <= {W{1'b0}};
            sample_i <= 0; sample_j <= 0; sample_k <= 0; sample_r <= 0;
            product_index <= 0; gain_index <= 0; gain_acc <= 0;
            pitch_y_scaled <= 0;
            p_wx <= 0; p_wy <= 0; p_wz <= 0; p_xy <= 0; p_xz <= 0;
            p_yz <= 0; p_xx <= 0; p_yy <= 0; p_zz <= 0; p_ww <= 0;
            state       <= ST_IDLE;
        end else begin
            out_valid    <= 1'b0;
            cordic_start <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (in_valid) begin
                        // Capture one coherent quaternion sample.  New samples are
                        // ignored while the short conversion is in progress.
                        sample_i <= qi; sample_j <= qj;
                        sample_k <= qk; sample_r <= qr;
                        product_index <= 0;
                        busy <= 1'b1;
                        state <= ST_PRODUCT_ISSUE;
                    end
                end

                ST_PRODUCT_ISSUE: state <= ST_PRODUCT_SAVE;
                ST_PRODUCT_SAVE: begin
                    case (product_index)
                        0: p_wx <= mul_product[31:0];
                        1: p_wy <= mul_product[31:0];
                        2: p_wz <= mul_product[31:0];
                        3: p_xy <= mul_product[31:0];
                        4: p_xz <= mul_product[31:0];
                        5: p_yz <= mul_product[31:0];
                        6: p_xx <= mul_product[31:0];
                        7: p_yy <= mul_product[31:0];
                        8: p_zz <= mul_product[31:0];
                        9: p_ww <= mul_product[31:0];
                    endcase
                    if (product_index == 9) state <= ST_PREPARE;
                    else begin
                        product_index <= product_index + 1'b1;
                        state <= ST_PRODUCT_ISSUE;
                    end
                end
                ST_PREPARE: begin
                    roll_y <= roll_y_calc; roll_x <= roll_x_calc;
                    pitch_y <= pitch_y_calc;
                    yaw_y <= yaw_y_calc; yaw_x <= yaw_x_calc;
                    state <= ST_ROLL_START;
                end
                ST_GAIN_ISSUE: state <= ST_GAIN_SAVE;
                ST_GAIN_SAVE: begin
                    gain_acc <= gain_acc + gain_term;
                    if (gain_index == 2) begin
                        pitch_y_scaled <= (gain_acc + gain_term) >>> 15;
                        state <= ST_PITCH_START;
                    end else begin
                        gain_index <= gain_index + 1'b1;
                        state <= ST_GAIN_ISSUE;
                    end
                end

                ST_ROLL_START: begin
                    cordic_x     <= roll_x;
                    cordic_y     <= roll_y;
                    cordic_start <= 1'b1;
                    state        <= ST_ROLL_WAIT;
                end

                ST_ROLL_WAIT: begin
                    if (cordic_done) begin
                        roll_deg10 <= cordic_angle;
                        roll_mag   <= cordic_magnitude;
                        gain_index <= 0;
                        gain_acc <= 0;
                        state <= ST_GAIN_ISSUE;
                    end
                end

                ST_PITCH_START: begin
                    cordic_x     <= roll_mag;
                    cordic_y     <= pitch_y_scaled;
                    cordic_start <= 1'b1;
                    state        <= ST_PITCH_WAIT;
                end

                ST_PITCH_WAIT: begin
                    if (cordic_done) begin
                        pitch_deg10 <= cordic_angle;
                        state       <= ST_YAW_START;
                    end
                end

                ST_YAW_START: begin
                    cordic_x     <= yaw_x;
                    cordic_y     <= yaw_y;
                    cordic_start <= 1'b1;
                    state        <= ST_YAW_WAIT;
                end

                ST_YAW_WAIT: begin
                    if (cordic_done) begin
                        yaw_deg10 <= cordic_angle;
                        out_valid <= 1'b1;
                        busy      <= 1'b0;
                        state     <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
