module uart_rx (
    input              clk,
    input              rst,
    input              rx_i,         //串行输入信号
    input  wire [1:0]  parity_i,     //校验选择
    input  wire [15:0] baud_i,       //波特率计数值
    input  wire        rready_i,     //数据读取应答信号
    output reg         rvalid_o,     //数据有效信号
    output reg  [7:0]  rdata_o,      //接收数据输出
    output reg         err_o         //校验错误信号
);

//波特率计数器
reg [15:0] bps_max;
reg [15:0] bps_cnt; 

reg [3:0] recv_bit_cnt;
reg [8:0] recv_reg;
reg       recv_start;

wire e_check;
wire o_check;
wire check;

wire [3:0] bit_max;
assign bit_max = parity_i ? 10 : 9;    //通过校验选择接收位数

//奇偶校验
assign e_check = ^recv_reg[7:0];  //偶校验 数据0个数为偶时，校验为0，反之为1
assign o_check = ~e_check;  //奇校验 数据1个数为奇时，校验为0，反之为1

//parity ：0 无校验 ：1 奇校验 2：偶校验 
assign check =(parity_i==1)? o_check : (parity_i==2) ? e_check : 0;

always @(posedge clk or negedge rst) begin
    if( !rst ) begin
        rvalid_o <= 0;
        rdata_o  <= 0;
    end else if(rready_i) begin
        rvalid_o  <= 0;
    end else if((recv_bit_cnt == bit_max) && (bps_cnt == baud_i/2)) begin
        if(rx_i) begin                          //检测停止位是否为高
            err_o     <= 0;
            rvalid_o  <= 1;
            if(parity_i) begin
                rdata_o   <= recv_reg[7:0];
                if(check != recv_reg[8]) begin
                    err_o <= 1;
                    rvalid_o <= 0;
                end
            end else begin
                rdata_o   <= recv_reg[8:1];
            end
        end
    end
end


always @(posedge clk or negedge rst) begin
    if( !rst ) begin
        recv_start <= 0;
    end else if(!rx_i && !recv_start) begin
        recv_start <= 1;
    end else if((recv_bit_cnt == bit_max) && (bps_cnt == baud_i/2)) begin
        recv_start <= 0;
    end
end

//波特率计数，类似分频
always @(posedge clk or negedge rst) begin
    if( !rst ) begin
        bps_cnt <= 0;
        bps_max <= baud_i;
    end else if(!recv_start)
        bps_cnt <= 0;
    else if(bps_cnt >= baud_i)
        bps_cnt <= 0;
    else
        bps_cnt <= bps_cnt + 1;
end

always @(posedge clk or negedge rst) begin
    if( !rst ) begin
        recv_reg <= 0;
    end else if((recv_bit_cnt != bit_max) && (bps_cnt == baud_i/2)) begin//在每一位数据中间采样
        recv_reg <= {rx_i, recv_reg[8:1]};//逻辑右移接收数据
    end
end

//接收位计数
always @(posedge clk or negedge rst) begin
    if( !rst ) begin
        recv_bit_cnt <= 0;
    end else if(!recv_start)
        recv_bit_cnt <= 0;
    else if(bps_cnt == baud_i)
        recv_bit_cnt <= recv_bit_cnt + 1;
end





endmodule