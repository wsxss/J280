
module  uart_tx #(

    parameter   UART_BPS    =   'd9600,
    parameter   CLK_FREQ    =   'd50_000_000

) (

     input   	            sys_clk     ,
     input   	            sys_rst_n   ,

     input			[7:0]	pi_data     ,
     input					pi_flag     ,

     output		reg			tx			,
     output		            pi_done

);

localparam  BAUD_CNT_MAX    =   CLK_FREQ/UART_BPS   ;

reg [12:0]  baud_cnt;
reg         bit_flag;
reg [3:0]   bit_cnt ;
reg         work_en ;

always@(posedge sys_clk or negedge sys_rst_n)
        if(sys_rst_n == 1'b0)
            work_en <= 1'b0;
        else    if(pi_flag == 1'b1)
            work_en <= 1'b1;
        else    if((bit_flag == 1'b1) && (bit_cnt == 4'd9))
            work_en <= 1'b0;

always@(posedge sys_clk or negedge sys_rst_n)
        if(sys_rst_n == 1'b0)
            baud_cnt <= 13'b0;
        else    if((baud_cnt == BAUD_CNT_MAX - 1) || (work_en == 1'b0))
            baud_cnt <= 13'b0;
        else    if(work_en == 1'b1)
            baud_cnt <= baud_cnt + 1'b1;

always@(posedge sys_clk or negedge sys_rst_n)
        if(sys_rst_n == 1'b0)
            bit_flag <= 1'b0;
        else    if(baud_cnt == 13'd1)
            bit_flag <= 1'b1;
        else
            bit_flag <= 1'b0;

always@(posedge sys_clk or negedge sys_rst_n)
    if(sys_rst_n == 1'b0)
        bit_cnt <= 4'b0;
    else    if((bit_flag == 1'b1) && (bit_cnt == 4'd9))
        bit_cnt <= 4'b0;
    else    if((bit_flag == 1'b1) && (work_en == 1'b1))
        bit_cnt <= bit_cnt + 1'b1;

always@(posedge sys_clk or negedge sys_rst_n)
        if(sys_rst_n == 1'b0)
            tx <= 1'b1;
        else    if(bit_flag == 1'b1)
            case(bit_cnt)
                0       : tx <= 1'b0;
                1       : tx <= pi_data[0];
                2       : tx <= pi_data[1];
                3       : tx <= pi_data[2];
                4       : tx <= pi_data[3];
                5       : tx <= pi_data[4];
                6       : tx <= pi_data[5];
                7       : tx <= pi_data[6];
                8       : tx <= pi_data[7];
                9       : tx <= 1'b1;
                default : tx <= 1'b1;
            endcase

reg         work_en_r1 ;
reg         work_en_r2 ;
always@(posedge sys_clk or negedge sys_rst_n)
        if(sys_rst_n == 1'b0)begin
            work_en_r1 <= 1'b0;
            work_en_r2 <= 1'b0;
        end
        else begin
            work_en_r1 <= work_en;
            work_en_r2 <= work_en_r1;
        end

assign pi_done = ~work_en_r1 & work_en_r2;
endmodule
