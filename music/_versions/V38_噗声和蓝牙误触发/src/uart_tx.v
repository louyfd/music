module uart_tx (
    input wire clk,
    input wire rst,
    input wire [1: 0] parity_i,     //校验选择
    input wire [7: 0] wdata_i,      //需要发送的数据输入
    input wire [15:0] baud_i,       //波特率计数值
    input wire wvalid_i,            //发送数据输入有效信号
    output reg wready_o,            //发送数据应答信号
    output reg tx_o                 //串口tx输出信号
);

//波特率计数器
reg [15:0] bps_max;
reg [15:0] bps_cnt;
//输入数据缓存
reg [10:0] send_reg;

reg [3:0] send_bit_cnt;
wire [3:0] bit_max;

wire e_check;
wire o_check;
wire check;
reg send_start;

assign bit_max = parity_i ? 11 : 10;    //通过校验选择最大发送位数

//奇偶校验
assign e_check = ^wdata_i; //偶校验 数据0个数为偶时，校验为0，反之为1
assign o_check = ~e_check;  //奇校验 数据1个数为奇时，校验为0，反之为1

//parity ：0 无校验 ：1 奇校验 2：偶校验 
assign check =(parity_i==1)? o_check : (parity_i==2) ? e_check : 0;


//对输入时钟的计数，计算到相应波特率的数值发送一个位，类似分频器的功能
always @(posedge clk or negedge rst) begin
    if( !rst ) begin
        bps_cnt <= 0;
        bps_max <= baud_i;
    end else if( !send_start )
        bps_cnt <= 0;
    else if(bps_cnt >= bps_max)
        bps_cnt <= 0;
    else
        bps_cnt <= bps_cnt + 1;
end

always @(posedge clk or negedge rst) begin
    if( !rst ) begin
        tx_o <= 1;
        wready_o <= 0;
        send_start <= 0;
    end else if( wvalid_i && !send_start ) begin
        wready_o <= 1;      //应答信号拉高，确认数据已经收到
        send_start  <= 1;   //开始标志位置一，开始一次发送
        if(parity_i)        //将停止位 校验位 数据位 起始位拼起来，循环发送出去
            send_reg <= {1'b1, check, wdata_i, 1'b0};
        else 
            send_reg <= {1'b1, 1'b1, wdata_i, 1'b0};
    end else if(send_bit_cnt == bit_max) begin     //发送结束，将开始标志位清0
        send_start <= 0;
    end else if(send_start && bps_cnt == 0) begin
        wready_o <= 0;      //维持了一个时钟的应答信号后，拉低
        tx_o <= send_reg[0];//开始发送数据，串口是从低位开始发送的
        send_reg <= {send_reg[0], send_reg[10:1]}; //将数据循环右移，为下一个位的发送做准备
    end 
end

//发送位计数
always @(posedge clk or negedge rst) begin
    if( !rst ) begin
        send_bit_cnt <= 0;
    end else if(!send_start)
        send_bit_cnt <= 0;
    else if(bps_cnt == bps_max)
        send_bit_cnt <= send_bit_cnt + 1;
end




endmodule
