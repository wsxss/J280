module  adc(

    input      		clk_50m		,
    input      		rst_n		,

    output	    	adc_clk		,
    output	    	adc_oe_n	,
    input	[9:0]	adc_data_in	,
    output	[11:0]	theta_out

    );

reg				clk_div			;

wire	[11:0]	theta_12bit		;
reg		[11:0]	theta_out_reg	;
reg		[15:0]	theta_sum		;
reg		[12:0]	cnt_sample		;
reg		[15:0]	cnt_1ms			;

assign adc_oe_n     = 1'b0;
assign theta_12bit  = {adc_data_in, 2'b0};
assign theta_out    = theta_out_reg;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n)
        clk_div <= 1'b0;
    else
        clk_div <= ~clk_div;
end

assign adc_clk = clk_div;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n)
        cnt_sample <= 'd0;
    else if(cnt_sample == 13'd6249)
        cnt_sample <= 'd0;
    else
        cnt_sample <= cnt_sample + 1'b1;
end

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n)
        cnt_1ms <= 'd0;
    else if(cnt_1ms == 16'd49999)
        cnt_1ms <= 'd0;
    else
        cnt_1ms <= cnt_1ms + 1'b1;
end

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        theta_sum     <= 'd0;
        theta_out_reg <= 'd0;
    end else if(cnt_sample == 13'd6248) begin

        theta_sum <= theta_sum + theta_12bit;
    end else if(cnt_1ms == 16'd49999) begin

        theta_out_reg <= theta_sum[14:3];
        theta_sum     <= 'd0;
    end
end

endmodule
