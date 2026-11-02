module  motor_ctrl(

    input				clk_50m 	,
    input				rst_n   	,

    input       [1:0]   motor_a_sel ,
    input       [15:0]   duty_out	,

    output	reg			ain1        ,
    output	reg			ain2        ,
    output              pwm

    );

reg	[11:0]	pwm_cnt;
localparam	PWM_PERIOD = 13'd2500;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n)
        pwm_cnt <= 'd0;
    else if(pwm_cnt >= PWM_PERIOD - 1'b1)
        pwm_cnt <= 'd0;
    else
        pwm_cnt <= pwm_cnt + 'd1;
end

wire [15:0] cmp_a = duty_out * 25;

always @(posedge clk_50m or negedge rst_n) begin
    if(!rst_n) begin
        ain1 <= 1'b0;
        ain2 <= 1'b0;
    end else begin
        case(motor_a_sel)
            2'b00 : begin ain1 <= 1'b0; ain2 <= 1'b0; end
            2'b01 : begin ain1 <= 1'b1; ain2 <= 1'b0; end
            2'b10 : begin ain1 <= 1'b0; ain2 <= 1'b1; end
            2'b11 : begin ain1 <= 1'b1; ain2 <= 1'b1; end
        endcase
    end
end

assign pwm = (pwm_cnt < cmp_a) ? 1'b1 : 1'b0;

endmodule
