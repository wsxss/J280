module j280_top #(
    parameter integer PID_DIV       = 1000,
    parameter integer PID_INT_LIM   = 5000,

    parameter integer ANG_OUT_MAX   = 100,
    parameter integer ANG_D_MAX     = 100,
    parameter integer POS_OUT_MAX   = 150,
    parameter integer POS_D_MAX     = 200,

    parameter signed [15:0] CENTER_ANGLE = 16'sd3122,

    parameter integer SWING_AMP   = 32,
    parameter integer SWING_VMIN  = 2,
    parameter integer KICK_TICKS  = 80,
    parameter signed [15:0] CATCH_WINDOW = 16'sd480,
    parameter integer CATCH_VEL   = 32,
    parameter signed [15:0] FALL_OUT = 16'sd650,
    parameter integer COAST_MS    = 500,

    parameter integer ANG_KP        = 810,
    parameter integer ANG_KI        = 0,
    parameter integer ANG_KD        = 2000,

    parameter integer POS_KP        = 800,
    parameter integer POS_KI        = 0,
    parameter integer POS_KD        = 4500,

    parameter signed [15:0] POS_STEP  = 16'sd400,
    parameter integer       POS_SLEW  = 10,
    parameter signed [15:0] POS_RANGE = 16'sd12000,

    parameter signed [15:0] TRAJ_HALF = 16'sd540,
    parameter integer       TRAJ_MS   = 5000,

    parameter integer       FAULT_MS  = 3000
)(
    input  wire        clk_50m,
    input  wire        rst_n,

    input  wire [2:0]  key_sw,

    output wire        adc_clk,
    output wire        adc_oe_n,
    input  wire [9:0]  adc_data_in,

    output wire        pwm,
    output wire        in1,
    output wire        in2,

    input  wire        enc_a,
    input  wire        enc_b,

    output wire        uart_tx,

    output wire [3:0]  led
);

    wire [11:0] theta_out;

    adc adc_inst (
        .clk_50m     (clk_50m   ),
        .rst_n       (rst_n     ),
        .adc_clk     (adc_clk   ),
        .adc_oe_n    (adc_oe_n  ),
        .adc_data_in (adc_data_in),
        .theta_out   (theta_out )
    );

    wire signed [15:0] actual_angle = $signed({4'b0, theta_out});

    reg [17:0] cnt_angle;
    reg        pulse_5ms;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            cnt_angle <= 18'd0;
            pulse_5ms <= 1'b0;
        end else if (cnt_angle == 18'd249_999) begin
            cnt_angle <= 18'd0;
            pulse_5ms <= 1'b1;
        end else begin
            cnt_angle <= cnt_angle + 18'd1;
            pulse_5ms <= 1'b0;
        end
    end

    reg [21:0] cnt_pos;
    reg        pulse_50ms;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            cnt_pos    <= 22'd0;
            pulse_50ms <= 1'b0;
        end else if (cnt_pos == 22'd2_499_999) begin
            cnt_pos    <= 22'd0;
            pulse_50ms <= 1'b1;
        end else begin
            cnt_pos    <= cnt_pos + 22'd1;
            pulse_50ms <= 1'b0;
        end
    end

    wire key_press;

    key_filter #(
        .CNT_MAX (1_000_000)
    ) key_inst (
        .clk       (clk_50m),
        .rst_n     (rst_n  ),
        .key_in    (key_sw[0]),
        .key_press (key_press)
    );

    reg arm_en;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            arm_en <= 1'b0;
        else if (key_press)
            arm_en <= ~arm_en;
    end

    wire key2_press;
    key_filter #(
        .CNT_MAX (1_000_000)
    ) key2_inst (
        .clk       (clk_50m  ),
        .rst_n     (rst_n    ),
        .key_in    (key_sw[1]),
        .key_press (key2_press)
    );

    wire key3_press;
    key_filter #(
        .CNT_MAX (1_000_000)
    ) key3_inst (
        .clk       (clk_50m  ),
        .rst_n     (rst_n    ),
        .key_in    (key_sw[2]),
        .key_press (key3_press)
    );

    wire signed [15:0] dev     = actual_angle - CENTER_ANGLE;
    wire signed [15:0] dev_abs = dev[15] ? -dev : dev;

    reg signed [15:0] theta_prev;
    reg signed [15:0] dtheta;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            theta_prev <= 16'sd0;
            dtheta     <= 16'sd0;
        end else if (pulse_5ms) begin
            theta_prev <= actual_angle;
            dtheta     <= actual_angle - theta_prev;
        end
    end
    wire signed [15:0] dtheta_abs = dtheta[15] ? -dtheta : dtheta;

    wire in_window     = (dev_abs <= CATCH_WINDOW);
    wire slow_enough   = (dtheta_abs <= CATCH_VEL);

    reg bal_active;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            bal_active <= 1'b0;
        else if (!arm_en)
            bal_active <= 1'b0;
        else if (bal_active) begin
            if (dev_abs > FALL_OUT)
                bal_active <= 1'b0;
        end else if (in_window && slow_enough) begin
            bal_active <= 1'b1;
        end
    end

    reg bal_d;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            bal_d <= 1'b0;
        else
          bal_d <= bal_active;
    end
    wire catch = bal_active && !bal_d;

    wire cos_pos  = (dev_abs <= 16'sd1130);
    wire swing_up = (dtheta[15] == 1'b0) ? cos_pos : ~cos_pos;

    reg [6:0] kick_cnt;
    reg       kick_dir;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            kick_cnt <= 7'd0;
            kick_dir <= 1'b0;
        end else if (pulse_5ms && !bal_active) begin
            if (kick_cnt >= KICK_TICKS - 1) begin
                kick_cnt <= 7'd0;
                kick_dir <= ~kick_dir;
            end else begin
                kick_cnt <= kick_cnt + 7'd1;
            end
        end
    end

    wire swing_still = (dtheta_abs < SWING_VMIN);
    wire signed [15:0] u_kick  = kick_dir ? (SWING_AMP >>> 1) : -(SWING_AMP >>> 1);
    wire signed [15:0] u_pump  = swing_up ? SWING_AMP : -SWING_AMP;
    wire signed [15:0] u_brake = swing_up ? -(SWING_AMP >>> 1) : (SWING_AMP >>> 1);

    localparam SW_PUMP = 1'b0, SW_COAST = 1'b1;
    reg sw_state;
    reg [21:0] coast_cnt;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            sw_state  <= SW_PUMP;
            coast_cnt <= 22'd0;
        end else if (!arm_en || bal_active) begin
            sw_state  <= SW_PUMP;
            coast_cnt <= 22'd0;
        end else if (bal_d) begin
            sw_state  <= SW_COAST;
            coast_cnt <= 22'd0;
        end else if (sw_state == SW_PUMP) begin
            if (in_window) begin
                sw_state  <= SW_COAST;
                coast_cnt <= 22'd0;
            end
        end else begin
            if (pulse_5ms) begin
                if (!in_window) begin
                    if (coast_cnt >= (COAST_MS / 5)) begin
                        sw_state  <= SW_PUMP;
                        coast_cnt <= 22'd0;
                    end else begin
                        coast_cnt <= coast_cnt + 22'd1;
                    end
                end else begin
                    coast_cnt <= 22'd0;
                end
            end
        end
    end

    wire signed [15:0] u_swing =
          (sw_state == SW_PUMP) ? (swing_still ? u_kick : u_pump)
        : (in_window ? (slow_enough ? 16'sd0 : u_brake) : 16'sd0);

    wire [15:0] enc_pos;

    encoder encoder_inst (
        .clk_50m (clk_50m),
        .rst_n   (rst_n  ),
        .enc_a   (enc_a  ),
        .enc_b   (enc_b  ),
        .pos     (enc_pos)
    );

    reg signed [15:0] target_pos;
    reg signed [15:0] target_cmd;
    wire signed [15:0] actual_pos = $signed(enc_pos);

    localparam integer TRAJ_T  = TRAJ_MS / 5;
    localparam integer TRAJ_DV = (TRAJ_HALF * 4 * 65536) / (TRAJ_T * TRAJ_T);

    reg               traj_loop;
    reg               traj_busy;
    reg               traj_dir;
    reg [15:0]        traj_t;
    reg signed [31:0] traj_v;
    reg signed [39:0] traj_p;
    reg signed [15:0] traj_base;

    wire signed [15:0] traj_mag   = (traj_p + traj_v) >>> 16;
    wire signed [15:0] traj_delta = traj_dir ? -traj_mag : traj_mag;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            target_pos <= 16'sd10000;
            target_cmd <= 16'sd10000;
            traj_loop  <= 1'b0;
            traj_busy  <= 1'b0;
            traj_dir   <= 1'b0;
            traj_t     <= 16'd0;
            traj_v     <= 32'sd0;
            traj_p     <= 40'sd0;
            traj_base  <= 16'sd10000;
        end else if (!bal_active || catch) begin
            target_pos <= $signed(enc_pos);
            target_cmd <= $signed(enc_pos);
            traj_loop  <= 1'b0;
            traj_busy  <= 1'b0;
            traj_t     <= 16'd0;
            traj_v     <= 32'sd0;
            traj_p     <= 40'sd0;
        end else if (key2_press) begin
            if (traj_busy) begin
                traj_loop  <= 1'b0;
                traj_busy  <= 1'b0;
                target_cmd <= target_pos + POS_STEP;
            end else begin
                if (target_cmd > (POS_RANGE - POS_STEP))
                    target_cmd <= POS_RANGE;
                else
                    target_cmd <= target_cmd + POS_STEP;
            end
        end else if (key3_press) begin
            if (traj_loop) begin
                traj_loop  <= 1'b0;
                traj_busy  <= 1'b0;
                target_cmd <= target_pos;
            end else begin
                traj_loop  <= 1'b1;
                traj_dir   <= 1'b0;
                traj_base  <= target_pos;
                traj_p     <= 40'sd0;
                traj_v     <= 32'sd0;
                traj_t     <= 16'd0;
                traj_busy  <= 1'b1;
            end
        end else if (traj_busy) begin
            if (pulse_5ms) begin
                if (traj_t >= TRAJ_T - 1) begin
                    target_pos <= traj_base + traj_delta;
                    target_cmd <= traj_base + traj_delta;
                    if (traj_loop) begin
                        traj_dir  <= ~traj_dir;
                        traj_base <= traj_base + traj_delta;
                        traj_p    <= 40'sd0;
                        traj_v    <= 32'sd0;
                        traj_t    <= 16'd0;

                    end else begin
                        traj_busy <= 1'b0;
                    end
                end else begin
                    traj_p <= traj_p + traj_v;
                    if (traj_t < (TRAJ_T >> 1))
                        traj_v <= traj_v + TRAJ_DV;
                    else
                        traj_v <= traj_v - TRAJ_DV;
                    traj_t     <= traj_t + 16'd1;
                    target_pos <= traj_base + traj_delta;
                end
            end
        end else begin
            if (pulse_50ms) begin
                if (target_pos + POS_SLEW < target_cmd)
                    target_pos <= target_pos + POS_SLEW;
                else if (target_pos - POS_SLEW > target_cmd)
                    target_pos <= target_pos - POS_SLEW;
                else
                    target_pos <= target_cmd;
            end
        end
    end

    wire signed [31:0] pos_target32 = {{16{target_pos[15]}}, target_pos};
    wire signed [31:0] pos_actual32 = {{16{actual_pos[15]}}, actual_pos};
    wire signed [31:0] pos_out_p_core, pos_out_d_core;
    wire               pos_done;

    pid #(
        .KP        (POS_KP      ),
        .KI        (POS_KI      ),
        .KD        (POS_KD      ),
        .DIV       (PID_DIV     ),
        .DEAD_ZONE (0           ),
        .INT_LIM   (PID_INT_LIM ),
        .OUT_MAX   (POS_OUT_MAX ),
        .D_MAX     (POS_D_MAX   )
    ) pid_pos_inst (
        .clk     (clk_50m       ),
        .rst_n   (rst_n         ),
        .tick    (pulse_50ms    ),
        .clr_int (1'b0          ),
        .target  (pos_target32  ),
        .actual  (pos_actual32  ),
        .out     (              ),
        .out_p   (pos_out_p_core),
        .out_d   (pos_out_d_core),
        .done    (pos_done      )
    );

    reg signed [15:0] pos_pid_out;
    reg signed [15:0] pos_d_out;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            pos_pid_out <= 16'sd0;
            pos_d_out   <= 16'sd0;
        end else if (!bal_active || catch) begin
            pos_pid_out <= 16'sd0;
            pos_d_out   <= 16'sd0;
        end else if (pos_done) begin
            pos_pid_out <= pos_out_p_core[15:0];
            pos_d_out   <= pos_out_d_core[15:0];
        end
    end

    reg signed [15:0] pos_out_filtered;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            pos_out_filtered <= 16'sd0;
        else if (!bal_active || catch)
            pos_out_filtered <= 16'sd0;
        else if (pulse_50ms)
            pos_out_filtered <= (pos_out_filtered * 2 + pos_pid_out) >>> 2;
    end

    wire signed [15:0] target_angle = CENTER_ANGLE + pos_out_filtered + pos_d_out;

    wire signed [31:0] ang_target32 = {{16{target_angle[15]}}, target_angle};
    wire signed [31:0] ang_actual32 = {{16{actual_angle[15]}}, actual_angle};
    wire signed [31:0] ang_out_core;
    wire               ang_done;

    pid #(
        .KP        (ANG_KP      ),
        .KI        (ANG_KI      ),
        .KD        (ANG_KD      ),
        .DIV       (PID_DIV     ),
        .DEAD_ZONE (0           ),
        .INT_LIM   (PID_INT_LIM ),
        .OUT_MAX   (ANG_OUT_MAX ),
        .D_MAX     (ANG_D_MAX   )
    ) pid_angle_inst (
        .clk     (clk_50m      ),
        .rst_n   (rst_n        ),
        .tick    (pulse_5ms    ),
        .clr_int (~bal_active  ),
        .target  (ang_target32 ),
        .actual  (ang_actual32 ),
        .out     (ang_out_core ),
        .out_p   (             ),
        .out_d   (             ),
        .done    (ang_done     )
    );

    reg signed [15:0] angle_pid_out;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            angle_pid_out <= 16'sd0;
        else if (ang_done)
            angle_pid_out <= ang_out_core[15:0];
    end

    wire signed [15:0] u_raw = bal_active ? angle_pid_out : u_swing;

    reg        fault;
    reg [15:0] sat_cnt;

    wire signed [15:0] u_abs_chk = u_raw[15] ? -u_raw : u_raw;
    wire               u_sat     = (u_abs_chk >= ANG_OUT_MAX);

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            fault   <= 1'b0;
            sat_cnt <= 16'd0;
        end else if (!arm_en) begin
            fault   <= 1'b0;
            sat_cnt <= 16'd0;
        end else if (!fault) begin
            if (pulse_5ms) begin
                if (u_sat) begin
                    if (sat_cnt >= (FAULT_MS / 5))
                        fault <= 1'b1;
                    else
                        sat_cnt <= sat_cnt + 16'd1;
                end else begin
                    sat_cnt <= 16'd0;
                end
            end
        end
    end

    wire signed [15:0] u_ctrl = (!arm_en || fault) ? 16'sd0 : u_raw;

    wire               pid_neg   = u_ctrl[15];
    wire signed [15:0] abs_out   = pid_neg ? -u_ctrl : u_ctrl;
    wire               motor_run = arm_en && (u_ctrl != 16'sd0);
    wire [1:0]         motor_dir  = motor_run ? (pid_neg ? 2'b10 : 2'b01) : 2'b00;
    wire [15:0]        motor_duty = motor_run ? abs_out[15:0] : 16'd0;

    motor_ctrl motor_inst (
        .clk_50m    (clk_50m   ),
        .rst_n      (rst_n     ),
        .motor_a_sel(motor_dir ),
        .duty_out   (motor_duty),
        .ain1       (in1       ),
        .ain2       (in2       ),
        .pwm        (pwm       )
    );

    wire [15:0] status = fault ? 16'd10002 : (arm_en ? 16'd10001 : 16'd10000);

    uart_report #(
        .UART_BPS  (9600),
        .REPORT_MS (100)
    ) uart_inst (
        .clk   (clk_50m              ),
        .rst_n (rst_n                ),
        .v1    (theta_out + 16'd135  ),
        .v2    (enc_pos              ),
        .v3    (status               ),
        .tx    (uart_tx              )
    );

    assign led = 4'b1111;

endmodule
