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
module quat_to_euler(
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

    // Q14 * Q14 -> Q28.  Widen before sums/differences so no intermediate
    // overflow is introduced by Verilog expression sizing.
    wire signed [31:0] p_wx = $signed(qr) * $signed(qi);
    wire signed [31:0] p_wy = $signed(qr) * $signed(qj);
    wire signed [31:0] p_wz = $signed(qr) * $signed(qk);
    wire signed [31:0] p_xy = $signed(qi) * $signed(qj);
    wire signed [31:0] p_xz = $signed(qi) * $signed(qk);
    wire signed [31:0] p_yz = $signed(qj) * $signed(qk);
    wire signed [31:0] p_xx = $signed(qi) * $signed(qi);
    wire signed [31:0] p_yy = $signed(qj) * $signed(qj);
    wire signed [31:0] p_zz = $signed(qk) * $signed(qk);
    wire signed [31:0] p_ww = $signed(qr) * $signed(qr);

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
    wire signed [56:0] pitch_gain_product = $signed(pitch_y) * 17'sd53961;
    wire signed [W-1:0] pitch_y_scaled = pitch_gain_product >>> 15;

    reg cordic_start;
    reg signed [W-1:0] cordic_x, cordic_y;
    wire cordic_busy, cordic_done;
    wire signed [15:0] cordic_angle;
    wire signed [W-1:0] cordic_magnitude;

    cordic_atan2 #(.W(W)) u_cordic (
        .clk(clk), .rst_n(rst_n), .start(cordic_start),
        .x_in(cordic_x), .y_in(cordic_y),
        .busy(cordic_busy), .done(cordic_done),
        .angle_deg10(cordic_angle), .magnitude(cordic_magnitude)
    );

    localparam ST_IDLE        = 3'd0;
    localparam ST_ROLL_START  = 3'd1;
    localparam ST_ROLL_WAIT   = 3'd2;
    localparam ST_PITCH_START = 3'd3;
    localparam ST_PITCH_WAIT  = 3'd4;
    localparam ST_YAW_START   = 3'd5;
    localparam ST_YAW_WAIT    = 3'd6;
    reg [2:0] state;

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
                        roll_y  <= roll_y_calc;
                        roll_x  <= roll_x_calc;
                        pitch_y <= pitch_y_calc;
                        yaw_y   <= yaw_y_calc;
                        yaw_x   <= yaw_x_calc;
                        busy    <= 1'b1;
                        state   <= ST_ROLL_START;
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
                        state      <= ST_PITCH_START;
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
