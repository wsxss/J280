module key_filter #(
    parameter integer CNT_MAX = 1_000_000
)(
    input  wire clk,
    input  wire rst_n,
    input  wire key_in,

    output reg  key_press
);

    reg [31:0] cnt;
    reg        key_sync;
    reg        key_level;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt       <= 32'd0;
            key_sync  <= 1'b1;
            key_level <= 1'b1;
            key_press <= 1'b0;
        end else begin
            key_sync  <= key_in;
            key_press <= 1'b0;

            if (key_sync != key_level) begin
                if (cnt >= CNT_MAX - 1) begin
                    cnt       <= 32'd0;
                    key_level <= key_sync;
                    if (key_sync == 1'b0)
                        key_press <= 1'b1;
                end else begin
                    cnt <= cnt + 32'd1;
                end
            end else begin
                cnt <= 32'd0;
            end
        end
    end

endmodule
