module pid #(
    parameter integer KP        = 800,
    parameter integer KI        = 0,
    parameter integer KD        = 2000,
    parameter integer DIV       = 1000,
    parameter integer OUT_MAX   = 100,
    parameter integer D_MAX     = 100,
    parameter integer INT_LIM   = 2000,
    parameter integer DEAD_ZONE = 0
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick,
    input  wire               clr_int,
    input  wire signed [31:0] target,
    input  wire signed [31:0] actual,
    output reg  signed [31:0] out,
    output reg  signed [31:0] out_p,
    output reg  signed [31:0] out_d,
    output reg                done
);

    localparam integer QB      = $clog2(OUT_MAX + 1);
    localparam [63:0]  SAT_ABS = OUT_MAX * DIV;
    localparam [63:0]  D_SAT   = D_MAX * DIV;

    localparam S_IDLE = 3'd0,
               S_SUM  = 3'd1,
               S_ABS  = 3'd2,
               S_DIV  = 3'd3,
               S_D2   = 3'd4,
               S_OUT  = 3'd5;

    reg signed [31:0] e0, e1, ei;
    reg signed [31:0] de;
    reg signed [63:0] sum_r;
    reg signed [63:0] d_r;
    reg        [2:0]  st;
    reg               neg_pi, neg_d;
    reg               pi_sat, d_sat;
    reg        [63:0] rem,  drem;
    reg        [31:0] quo,  dquo;
    reg        [31:0] quo_pi, quo_d;
    reg        [5:0]  bitn, dbitn;

    wire signed [31:0] e_raw  = target - actual;
    wire signed [31:0] e_abs  = e_raw[31] ? -e_raw : e_raw;
    wire signed [31:0] e_new  = (e_abs < $signed(DEAD_ZONE)) ? 32'sd0 : e_raw;

    wire signed [31:0] ei_base = clr_int ? 32'sd0 : ei;
    wire signed [32:0] ei_raw  = $signed({ei_base[31], ei_base})
                               + $signed({e_new[31], e_new});
    wire signed [31:0] ei_new  = (KI == 0)           ? 32'sd0 :
                                 (ei_raw >  INT_LIM) ? INT_LIM :
                                 (ei_raw < -INT_LIM) ? -INT_LIM :
                                                       ei_raw[31:0];

    wire signed [63:0] sum_pi = $signed(KP) * e0 + $signed(KI) * ei;
    wire signed [63:0] d_term = $signed(KD) * de;
    wire        [63:0] sum_abs = sum_r[63] ? -sum_r : sum_r;
    wire        [63:0] d_abs   = d_r[63]  ? -d_r   : d_r;
    wire        [63:0] dsh     = (64'd0 + DIV) << bitn;
    wire        [63:0] dsh_d   = (64'd0 + DIV) << dbitn;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            e0 <= 0; e1 <= 0; ei <= 0; de <= 0;
            sum_r <= 0; d_r <= 0;
            st <= S_IDLE;
            neg_pi <= 1'b0; neg_d <= 1'b0;
            pi_sat <= 1'b0; d_sat <= 1'b0;
            rem <= 0; drem <= 0;
            quo <= 0; dquo <= 0;
            quo_pi <= 0; quo_d <= 0;
            bitn <= 0; dbitn <= 0;
            out <= 32'sd0; out_p <= 32'sd0; out_d <= 32'sd0; done <= 1'b0;
        end else begin
            done <= 1'b0;
            case (st)

                S_IDLE: if (tick) begin
                    e1   <= e0;
                    e0   <= e_new;
                    de   <= e_new - e0;
                    ei   <= ei_new;
                    st   <= S_SUM;
                end

                S_SUM: begin
                    sum_r <= sum_pi;
                    d_r   <= d_term;
                    st    <= S_ABS;
                end

                S_ABS: begin

                    neg_pi <= sum_r[63];
                    rem    <= sum_abs;
                    bitn   <= QB - 1;
                    quo    <= 32'sd0;
                    pi_sat <= (sum_abs >= SAT_ABS);
                    if (sum_abs >= SAT_ABS) quo_pi <= OUT_MAX;
                    else                     quo_pi <= 32'sd0;

                    neg_d  <= d_r[63];
                    drem   <= d_abs;
                    dbitn  <= QB - 1;
                    dquo   <= 32'sd0;
                    d_sat  <= (d_abs >= D_SAT);
                    if (d_abs >= D_SAT) quo_d <= D_MAX;
                    else                quo_d <= 32'sd0;
                    st     <= S_DIV;
                end

                S_DIV: begin
                    if (pi_sat) begin
                        st <= (d_sat) ? S_OUT : S_D2;
                    end else begin
                        if (rem >= dsh) begin
                            rem <= rem - dsh;
                            quo <= quo | (32'd1 << bitn);
                        end
                        if (bitn == 0) begin

                            quo_pi <= quo | ((rem >= dsh) ? 32'd1 : 32'd0);
                            st     <= (d_sat) ? S_OUT : S_D2;
                        end else begin
                            bitn <= bitn - 1'b1;
                        end
                    end
                end

                S_D2: begin
                    if (d_sat) begin
                        st <= S_OUT;
                    end else begin
                        if (drem >= dsh_d) begin
                            drem <= drem - dsh_d;
                            dquo <= dquo | (32'd1 << dbitn);
                        end
                        if (dbitn == 0) begin

                            quo_d <= dquo | ((drem >= dsh_d) ? 32'd1 : 32'd0);
                            st    <= S_OUT;
                        end else begin
                            dbitn <= dbitn - 1'b1;
                        end
                    end
                end

                S_OUT: begin
                    out_p <= (neg_pi ? -$signed(quo_pi) : $signed(quo_pi));
                    out_d <= (neg_d  ? -$signed(quo_d)  : $signed(quo_d));
                    out   <= (neg_pi ? -$signed(quo_pi) : $signed(quo_pi))
                           + (neg_d  ? -$signed(quo_d)  : $signed(quo_d));
                    done <= 1'b1;
                    st   <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase

            if (clr_int && !(st == S_IDLE && tick)) ei <= 32'sd0;
        end
    end

endmodule
