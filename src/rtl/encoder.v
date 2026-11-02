
module encoder (

	input			clk_50m			,
    input			rst_n			,

    input			enc_a, enc_b	,

    output	[15:0]	pos

);

reg 	[15:0]	pos_reg;

assign pos = pos_reg;

reg [1:0] enc_sync_a, enc_sync_b;
reg [1:0] state_now, state_last;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
		enc_sync_a <= 'd0;
		enc_sync_b <= 'd0;
		state_now <= 'd0;
	end else begin
		enc_sync_a <= {enc_sync_a[0], enc_a};
		enc_sync_b <= {enc_sync_b[0], enc_b};
		state_now <= {enc_sync_a[1], enc_sync_b[1]};
	end
end

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        pos_reg    <= 'd10000;
        state_last <= 2'b00;
    end else begin
        state_last <= state_now;
        case({state_last, state_now})

            4'b00_01: pos_reg <= pos_reg - 1;
            4'b01_11: pos_reg <= pos_reg - 1;
            4'b11_10: pos_reg <= pos_reg - 1;
            4'b10_00: pos_reg <= pos_reg - 1;

            4'b00_10: pos_reg <= pos_reg + 1;
            4'b10_11: pos_reg <= pos_reg + 1;
            4'b11_01: pos_reg <= pos_reg + 1;
            4'b01_00: pos_reg <= pos_reg + 1;
            default: pos_reg <= pos_reg;
        endcase
    end
end

endmodule
