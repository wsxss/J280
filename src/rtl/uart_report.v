module uart_report #(
    parameter integer CLK_FREQ  = 50_000_000,
    parameter integer UART_BPS  = 9600,
    parameter integer REPORT_MS = 100
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] v1,
    input  wire [15:0] v2,
    input  wire [15:0] v3,
    output wire        tx
);

    reg  [7:0] tx_byte;
    reg        tx_start;
    wire       tx_done;

    uart_tx #(
        .UART_BPS (UART_BPS),
        .CLK_FREQ (CLK_FREQ)
    ) u_byte_tx (
        .sys_clk   (clk     ),
        .sys_rst_n (rst_n   ),
        .pi_data   (tx_byte ),
        .pi_flag   (tx_start),
        .tx        (tx      ),
        .pi_done   (tx_done )
    );

    localparam integer CYC_PER_MS = CLK_FREQ / 1000;
    localparam integer REPORT_CYC = REPORT_MS * CYC_PER_MS;

    reg [26:0] period_cnt;
    wire       period_hit = (period_cnt >= REPORT_CYC - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            period_cnt <= 27'd0;
        else if (period_hit)
            period_cnt <= 27'd0;
        else
            period_cnt <= period_cnt + 27'd1;
    end

    localparam [2:0] S_IDLE = 3'd0,
                     S_LOAD = 3'd1,
                     S_FLAG = 3'd2,
                     S_WAIT = 3'd3,
                     S_GAP  = 3'd4;

    localparam integer GAP_CYC = CLK_FREQ / UART_BPS;

    reg [15:0] gap_cnt;

    reg [2:0]  st;
    reg [3:0]  byte_idx;
    reg [15:0] r1, r2, r3;

    reg [7:0]  byte_mux;

    always @(*) begin
        case (byte_idx)
            4'd0:    byte_mux = 8'hAA;
            4'd1:    byte_mux = 8'h55;
            4'd2:    byte_mux = r1[15:8];
            4'd3:    byte_mux = r1[7:0];
            4'd4:    byte_mux = r2[15:8];
            4'd5:    byte_mux = r2[7:0];
            4'd6:    byte_mux = r3[15:8];
            4'd7:    byte_mux = r3[7:0];
            4'd8:    byte_mux = 8'h0D;
            4'd9:    byte_mux = 8'h0A;
            default: byte_mux = 8'h00;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st       <= S_IDLE;
            byte_idx <= 4'd0;
            tx_byte  <= 8'd0;
            tx_start <= 1'b0;
            r1 <= 16'd0; r2 <= 16'd0; r3 <= 16'd0;
        end else begin
            tx_start <= 1'b0;

            case (st)
                S_IDLE: begin
                    if (period_hit) begin
                        r1 <= v1; r2 <= v2; r3 <= v3;
                        byte_idx <= 4'd0;
                        st       <= S_LOAD;
                    end
                end

                S_LOAD: begin
                    tx_byte <= byte_mux;
                    st      <= S_FLAG;
                end

                S_FLAG: begin
                    tx_start <= 1'b1;
                    st       <= S_WAIT;
                end

                S_WAIT: begin
                    if (tx_done) begin
                        if (byte_idx == 4'd9) begin
                            st <= S_IDLE;
                        end else begin
                            byte_idx <= byte_idx + 4'd1;
                            gap_cnt  <= 16'd0;
                            st       <= S_GAP;
                        end
                    end
                end

                S_GAP: begin
                    if (gap_cnt >= GAP_CYC - 1)
                        st <= S_LOAD;
                    else
                        gap_cnt <= gap_cnt + 16'd1;
                end

                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
