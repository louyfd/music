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
    end
    // ★ V38：起始位中点复检 —— 到了中点还是高电平，说明刚才那个低电平只是
    //   一个毛刺、不是真的起始位，立刻作废这一轮（否则后面所有位的对齐全错，
    //   解出来的字节就是垃圾，可能撞上 0xC9 之类的命令字）。
    else if(recv_start && (recv_bit_cnt == 4'd0) && (bps_cnt == baud_i/2) && rx_i) begin
        recv_start <= 0;
    end
    else if((recv_bit_cnt == bit_max) && (bps_cnt == baud_i/2)) begin
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

//------------------------------------------------------------------
// ★★ V38：每一位采 3 次、取多数 ★★
//
//   原来每一位只在 bps_cnt == baud_i/2 采【一次】。那一次恰好撞上毛刺，
//   这一位就错了；错位之后整帧的字节边界全乱，数据会被当成命令字读出
//   任意值（包括 0xC9 —— 而 0xC9 是"翻转一个效果开关"）——
//   这正是"播放时滑音/延音自己开了"的来源之一。
//
//   现在在一个位的【中点附近】采 3 次，间隔 baud_i/8：
//       baud_i/2 - baud_i/8     baud_i/2     baud_i/2 + baud_i/8
//   baud_i=5208 时是 1953 / 2604 / 3255，离位边界（0 与 5208）都很远，安全。
//   3 取 2 表决 → 单个毛刺打不掉这一位。
//
//   ★ 正常数据下三次采到的值相同，表决结果 = 原来那一次的值
//     → 【正常通信逐位不变】，只有噪声情况下才不同（那正是要修的）。
//   ★ baud_i 很小时 baud_i/8 取到 0，三个采样点重合，自动退化成原来的单点采样。
//------------------------------------------------------------------
reg [2:0] samp_v;                  // 本位的三次采样
wire samp_maj = (samp_v[0] & samp_v[1]) | (samp_v[1] & samp_v[2]) | (samp_v[0] & samp_v[2]);
wire [15:0] samp_a = baud_i/2 - baud_i/8;
wire [15:0] samp_c = baud_i/2 + baud_i/8;
// ★★ 移入必须比第三次采样【晚一拍】★★
//   samp_v[2] <= rx_i 是非阻塞赋值 —— 在 bps_cnt == samp_c 那一拍它还被写，
//   值要到下一拍才成立。如果同一拍就用 samp_maj，第三次采样其实是【上一位】
//   的旧值，表决结果全错（这会比原来的单点采样还糟）。
//   所以移入条件用 samp_c + 1：三次采样都已站住，再移。
//   samp_c+1 = 3256，离位边界 5208 还有很远，安全。
wire [15:0] samp_d = samp_c + 16'd1;

always @(posedge clk or negedge rst) begin
    if( !rst ) begin
        samp_v <= 3'b000;
    end else if(recv_start && (recv_bit_cnt != bit_max)) begin
        if(bps_cnt == samp_a)        samp_v[0] <= rx_i;
        else if(bps_cnt == baud_i/2) samp_v[1] <= rx_i;
        else if(bps_cnt == samp_c)   samp_v[2] <= rx_i;
    end
end

always @(posedge clk or negedge rst) begin
    if( !rst ) begin
        recv_reg <= 0;
    end else if((recv_bit_cnt != bit_max) && (bps_cnt == samp_d)) begin
        recv_reg <= {samp_maj, recv_reg[8:1]};//★ V38：三次采样多数表决后再移入
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