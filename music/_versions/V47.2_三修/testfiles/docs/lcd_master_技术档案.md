# lcd_master.v 技术档案（Gowin GW5AT-60B 电子琴 LCD 显示驱动）

分析对象：`D:\CODE\FPGA_Gowin\Gowin\Gowin_V1.9.11.03_Education_x64\IDE\bin\Documents\60k\music(2)\music\src\lcd_master.v`
文件总长 1174 行；`timescale 1ns / 1ps`（lcd_master.v:13）。
纯可综合 RTL：无 `initial`、无延时语句、无仿真专用结构。

---

## 1. 模块与端口

模块名：`lcd_master`（lcd_master.v:15）。SPI 主机（Mode 0 类比：空闲 SCK=0、上升沿采样、MSB first），驱动 ST7789 320×240 屏。

| # | 端口 | 方向/类型 | 位宽 | 含义与用途 | 行号 |
|---|------|-----------|------|-----------|------|
| 1 | `sys_clk` | input wire | 1 | 50 MHz 系统时钟（顶层接 `clk_50m_ext`）。整个模块唯一时钟，所有时序基准 | lcd_master.v:16 |
| 2 | `rst_n` | input wire | 1 | 异步低有效复位，所有时序块（除两处例外，见 §4）的复位端 | lcd_master.v:17 |
| 3 | `play_start` | input wire | 1 | 演奏启动请求（电平保持型，顶层 BT 包接收器拉高）。在 `S_IDLE_WAIT` 内取上升沿 | lcd_master.v:19 |
| 4 | `play_len` | input wire | 8 | 演奏长度（音符数）。用于判断 `play_idx >= play_len - 1` 时停止 | lcd_master.v:20 |
| 5 | `score_addr` | output reg | 8 | 乐谱读地址，输出到顶层 `score_mem[0:255]`（lcd_master.v:21；top.v:3135-3139） | lcd_master.v:21 |
| 6 | `score_data` | input wire | 8 | 乐谱读数据（顶层为**寄存输出**，有 1 拍延迟）。**当前版本内部未真正使用**（见 §12 死代码） | lcd_master.v:22 |
| 7 | `key_levels` | input wire | 36 | 36 个琴键的当前按下电平（顶层 `sounding_s2`）。频谱柱的唯一内容来源 | lcd_master.v:24 |
| 8 | `timbre_idx` | input wire | 2 | 音色索引 0..3，决定柱高基准（`BAR_H0..BAR_H3`） | lcd_master.v:25 |
| 9 | `wave_mode` | input wire | 1 | 显示模式：0=频谱柱，1=实时波形 | lcd_master.v:28 |
| 10 | `wave_send` | input wire signed | 16 | 音频采样值（12.5 MHz 音频域），翻转沿握手 | lcd_master.v:30 |
| 11 | `wave_tgl` | input wire | 1 | 采样有效标志（每来一个新采样翻转一次） | lcd_master.v:31 |
| 12 | `state_out` | output wire | 4 | 主状态机状态导出，仅调试用（顶层 `lcd_state_unused`） | lcd_master.v:33, 1172 |
| 13 | `lcd_sck` | output reg | 1 | SPI 时钟，空闲 0，6.25 MHz | lcd_master.v:34 |
| 14 | `lcd_mosi` | output reg | 1 | SPI 数据（MSB first） | lcd_master.v:35 |
| 15 | `lcd_dc` | output reg | 1 | 0=命令，1=数据 | lcd_master.v:36 |
| 16 | `lcd_cs` | output reg | 1 | 片选，低有效。全程几乎恒为 0（仅复位/上电/`S_INIT` 结束那一拍为 1） | lcd_master.v:37 |
| 17 | `lcd_blk` | output reg | 1 | 背光控制，`S_POWER_ON` 内拉 1 后不再变化（复位值为 0） | lcd_master.v:38 |

引脚约束（在 `music.cst`，不在本文件内，但与本文件强绑定）：
`lcd_blk F19`、`lcd_cs W20`、`lcd_dc W19`、`lcd_mosi F20`、`lcd_sck D22`（全部 `LVCMOS33 PULL_MODE=NONE DRIVE=8 BANK_VCCIO=3.3`）；`clk_50m V22`（music.cst:10-19, 56）。

---

## 2. 全部 localparam（本文件无 `parameter`）

### 2.1 时序/计数类

| 名称 | 值 | 位宽 | 作用 | 行号 |
|------|-----|------|------|------|
| `INIT_COUNT` | `16'd30` | 16 | `S_INIT` 要发的字节数，必须等于初始化表条目数（0..29） | lcd_master.v:44 |
| `CLEAR_COUNT` | `32'd153600` | 32 | 全屏清黑字节数 = 320×240×2 | lcd_master.v:45 |
| `WIN_COUNT` | `16'd11` | 16 | 一条窗口命令的字节数（0x2A+4 + 0x2B+4 + 0x2C） | lcd_master.v:46 |
| `SCK_DIV` | `8'd1` | 8 | SCK 分频：每 `(SCK_DIV+1)=2` 个时钟推进一个 phase | lcd_master.v:48 |
| `PWR_DELAY` | `24'd10_000_000` | 24 | 上电等待 10,000,000×20ns = **200 ms** | lcd_master.v:49 |
| `ONE_SEC_CNT` | `26'd50_000_000` | 26 | 1 秒计数值；实际只用 `ONE_SEC_CNT/4` = 12,500,000（=0.25 s/音符步进） | lcd_master.v:50, 1061 |

### 2.2 频谱柱几何/颜色

| 名称 | 值 | 作用 | 行号 |
|------|-----|------|------|
| `BAR_NUM` | `6'd60` | 柱/数字个数（`dig_idx`、`play_idx` 上限） | lcd_master.v:53 |
| `BAR_W` | `16'd3` | 每根柱宽 3 像素 | lcd_master.v:54 |
| `BAR_STEP` | `16'd5` | 相邻柱起点间距 5 像素（3 宽 + 2 空） | lcd_master.v:55 |
| `BAR_MARGIN` | `16'd16` | 第 0 根柱的起始 x = 16 | lcd_master.v:56 |
| `BAR_H` | `16'd180` | 柱区满高 180 像素 | lcd_master.v:57 |
| `BAR_Y_TOP` | `16'd44` | 主区顶 y=44 | lcd_master.v:58 |
| `BAR_Y_BOT` | `16'd223` | 主区底 y=223（**未被引用**） | lcd_master.v:59 |
| `BAR_ROW_MAX` | `8'd179` | 柱区行号上限（0..179） | lcd_master.v:60 |
| `BAR_H0/H1/H2/H3` | `16'd60/100/140/180` | `timbre_idx`=0/1/2/3 对应的满高 | lcd_master.v:62-65 |
| `BAR_Y_TOP` 派生的 `bar_ye` | 44+180-1 = 223 | 柱窗口 y 范围 | lcd_master.v:222-223 |

### 2.3 波形参数

| 名称 | 值 | 作用 | 行号 |
|------|-----|------|------|
| `WAVE_Y_MID` | `16'd133` | 中线（**未被引用**，代码里用字面量 `17'sd133`） | lcd_master.v:68, 611-612 |
| `WAVE_Y_TOP` | `16'd44` | 波形区顶 | lcd_master.v:69, 425, 632 |
| `WAVE_Y_BOT` | `16'd223` | 波形区底 | lcd_master.v:70, 426 |
| `WAVE_N` | `10'd320` | 一屏列数 | lcd_master.v:71 |
| `WAVE_COL_H` | 180 | 列高（**未被引用**） | lcd_master.v:72 |
| `WAVE_CLAMP` | `16'sd90` | 纵向钳位（**未被引用**，实际用 44/223 硬钳） | lcd_master.v:74 |
| `WAVE_DECIM` | `3'd7` | 8:1 抽取（每 8 个采样入缓冲 1 个） | lcd_master.v:471 |
| `WAVE_COL_PIX` | `17'd180` | 单列像素数 | lcd_master.v:639 |
| `WAVE_COL_LIM` | `WAVE_COL_PIX*2` = 360 | 单列字节数 | lcd_master.v:640 |

### 2.4 颜色（RGB565，高字节先发）

| 名称 | 值 | 颜色 | 行号 |
|------|-----|------|------|
| `COLOR_WAVE` | `16'h07FF` | 青（R=0,G=63,B=31） | lcd_master.v:76, 626-627 |
| `COLOR_AXIS` | `16'h4208` | 暗灰（**未引用**） | lcd_master.v:77 |
| `COLOR_BG_W` | `16'h0000` | 黑（**未引用**） | lcd_master.v:78 |
| `COLOR_BAR` | `16'hF800` | 白键柱：亮红 | lcd_master.v:85, 210 |
| `COLOR_KEY_BK` | `16'h7800` | 黑键柱：暗红 | lcd_master.v:86, 210 |
| `COLOR_BG` | `16'h0000` | 黑（仅用于复位 `bar_color`） | lcd_master.v:87, 665 |
| `COLOR_H2` | `16'hFFE0` | 2 次谐波：黄 | lcd_master.v:89, 216 |
| `COLOR_H3` | `16'h07E0` | 3 次：绿（**当前不可达**） | lcd_master.v:90, 217 |
| `COLOR_H4` | `16'h07FF` | 4 次：青（**当前不可达**） | lcd_master.v:91, 217 |

### 2.5 数字（音符名）点阵

| 名称 | 值 | 行号 |
|------|-----|------|
| `DIGIT_W` | `16'd3` | lcd_master.v:81 |
| `DIGIT_H` | `16'd5` | lcd_master.v:82 |
| `DIGIT_Y` | `16'd232` | lcd_master.v:83 |

### 2.6 状态编码

`S_POWER_ON=4'd0`、`S_INIT=1`、`S_IDLE=2`、`S_CLEAR_WIN=3`、`S_CLEAR=4`、`S_BAR_WIN=5`、`S_BAR_DATA=6`、`S_DIGIT_WIN=7`、`S_DIGIT_DATA=8`、`S_IDLE_WAIT=9`、`S_CLEAR_BAR=10`、`S_WAVE_WIN=11`、`S_WAVE_DATA=12`、`S_CLEAR_BAR_DATA=13`（lcd_master.v:94-108）。共 14 个状态，`state` 为 4 位（lcd_master.v:110）。

---

## 3. 内部结构

### 3.1 全部 always 块（共 10 个）

| # | 行号 | 类型 | 内容 |
|---|------|------|------|
| 1 | lcd_master.v:235-251 | 组合 `always @(*)` | `dig_code`：`pos_in_group = dig_idx % 12` → 5 位字形编号 |
| 2 | lcd_master.v:254-270 | 组合 | `glyph`：5×3 点阵（15 位）查表 |
| 3 | lcd_master.v:280-355 | 组合 | `tbl_out[8:0]`：`{dc, data}`，`S_INIT` / `S_CLEAR_WIN` / `S_CLEAR_BAR` 三张表 |
| 4 | lcd_master.v:367-383 | 组合 | `bar_cmd_out`：柱窗口 11 字节 |
| 5 | lcd_master.v:396-412 | 组合 | `dig_cmd_out`：数字窗口 11 字节 |
| 6 | lcd_master.v:428-444 | 组合 | `wave_cmd_out`：波形列窗口 11 字节（1 像素宽） |
| 7 | lcd_master.v:476-506 | 时序（异步复位） | 波形采样接收 + 8:1 抽取 + 环形缓冲写 |
| 8 | lcd_master.v:517-520 | 时序（**无复位**） | `wave_buf` 同步读（`wave_v0/wave_v1`）→ 推断 BSRAM |
| 9 | lcd_master.v:576-587 | 时序（异步复位） | 峰值跟踪 `wave_peak`（快攻慢放） |
| 10 | lcd_master.v:645-1170 | 时序（异步复位） | 主状态机 + 全部 SPI 位推进 + 像素/字节生成 |

### 3.2 寄存器与存储资源

| 名称 | 位宽 | 作用 | 行号 |
|------|------|------|------|
| `state` | [3:0] | 主状态 | lcd_master.v:110 |
| `delay_cnt` | [23:0] | 上电延时计数 | lcd_master.v:111 |
| `send_idx` | [31:0] | 当前第几个字节（表中当索引用 `send_idx[7:0]` / `[3:0]`） | lcd_master.v:112 |
| `phase` | [1:0] | SPI 位内四相 | lcd_master.v:113 |
| `sck_div_cnt` | [7:0] | phase 内分频计数 | lcd_master.v:114 |
| `bit_idx` | [2:0] | 位序 0..7（MSB first） | lcd_master.v:115 |
| `cur_data` | [7:0] | 当前待发字节 | lcd_master.v:116 |
| `cur_dc` | 1 | **只写不读**（死寄存器） | lcd_master.v:117, 652 |
| `bar_prev` | [5:0] | **只写不读** | lcd_master.v:120, 661, 931 |
| `sec_cnt` | [25:0] | 演奏步进计时 | lcd_master.v:121 |
| `bar_x` | [15:0] | 当前柱窗口左 x | lcd_master.v:122 |
| `bar_pix_cnt` | **[16:0]** | 通用像素字节计数（**必须 17 位**，`S_CLEAR_BAR_DATA` 要数到 115199） | lcd_master.v:123, 829 |
| `bar_color` | [15:0] | **只写不读** | lcd_master.v:124, 665 |
| `bar_row` | [7:0] | 柱内当前行 0..179 | lcd_master.v:125 |
| `bar_colc` | [3:0] | 柱内当前列 0..2 | lcd_master.v:126 |
| `bar_state[0:59]` | 60×6 bit | 每列"上次画成什么样"的脏标记值（= 上次的 `bar_target`） | lcd_master.v:128, 668 |
| `wave_col` | [9:0] | 当前波形列 0..319 | lcd_master.v:132 |
| `wave_mode_d` | 1 | 模式上一次值，用于检测切换 | lcd_master.v:133 |
| `wave_ys/wave_ye` | [15:0]×2 | 保留未用 | lcd_master.v:134 |
| `wave_pix` | [1:0] | 保留未用（仅复位赋值） | lcd_master.v:135, 682 |
| **`wave_buf[0:4095]`** | 4096×16 bit signed | 唯一的片上"显存"：音频采样环形缓冲（64 Kbit → 约 4 块 18Kb BSRAM） | lcd_master.v:138 |
| `wave_wr` | [11:0] | 环形写指针 | lcd_master.v:139 |
| `wave_wr_sync` | [11:0] | **只写不读**（注释明确说不要用它，见 lcd_master.v:1046-1047） | lcd_master.v:140, 491 |
| `frame_wr` | [11:0] | 一帧开始时锁存的写指针（帧快照） | lcd_master.v:141, 1048 |
| `wave_t1/t2/t3` | 1×3 | 翻转沿同步链 | lcd_master.v:142 |
| `wave_in` | [15:0] signed | 采样数据寄存 | lcd_master.v:143 |
| `wave_dcnt` | [2:0] | 8:1 抽取计数 | lcd_master.v:473 |
| `wave_we` | 1 | **只写不读** | lcd_master.v:474, 483/493/498 |
| `wave_peak` | [15:0] | 峰值包络 | lcd_master.v:574 |
| `peak_div` | [15:0] | 峰值慢放分频 | lcd_master.v:575 |
| `dig_idx` | [5:0] | 当前数字 0..59 | lcd_master.v:176 |
| `dig_x/dig_y` | [15:0] | 数字窗口起点 | lcd_master.v:177-178 |
| `dig_pix_cnt` | [15:0] | 数字像素字节计数 0..29 | lcd_master.v:179 |
| `play_idx` | [7:0] | 当前列/音符索引 0..59 | lcd_master.v:182 |
| `playing` | 1 | 演奏中标志 | lcd_master.v:183 |
| `play_start_d` | 1 | `play_start` 打拍（边沿检测） | lcd_master.v:184 |
| `tbl_out` | [8:0] | `{dc,data}` | lcd_master.v:187 |

### 3.3 键位 → 列号映射（`col_fund`）

`col_fund[59:0]` 由 `key_levels[35:0]` 按"每 12 键一组"重排（lcd_master.v:148-169），每组的偏移顺序固定为：
索引 0→+5、1→+0、2→+6、3→+1、4→+7、5→+8、6→+2、7→+9、8→+3、9→+10、10→+4、11→+11（三组基址 0/12/24）。
`col_fund[59:36] = 24'd0`（lcd_master.v:169）→ **第 36..59 列永远不亮**（只有数字没有柱）。

- `col_h2 = col_fund << 12`（lcd_master.v:171）→ 判断"柱 i 的 2 次谐波"= `key_levels[i+12]`。
- `col_h3 = col_fund << 19`、`col_h4 = col_fund << 24`（lcd_master.v:172-173）被定义但**未被使用**，因为 `col_on3`/`col_on4` 被硬接成 `1'b0`（lcd_master.v:198-199）。

### 3.4 每列的目标值（脏标记机制）

- `col_on1 = col_fund[play_idx]`（lcd_master.v:196）
- `col_on2 = timbre_idx[0] && col_h2[play_idx]`（lcd_master.v:197）
- `col_any = col_on1 | col_on2`（lcd_master.v:200）
- `harm_cls = col_on1 ? 2'd0 : 2'd1`（lcd_master.v:201）→ 只可能是 0 或 1，所以 `bar_h_cls`/`bar_colr` 里 harm_cls=2、3 的分支（`COLOR_H3`/`COLOR_H4`）**不可达**。
- `bar_target = col_any ? {1'b0, timbre_idx, 1'b1, harm_cls} : 6'd0`（lcd_master.v:203）：bit[5]=0、bit[4:3]=音色、bit[2]=1（"亮"标志）、bit[1:0]=谐波类别；灭的时候是 0。
- `bar_dirty = (bar_state[play_idx] != bar_target)`（lcd_master.v:204）→ **只有变化的列才重画**，这是全屏刷新的核心省时机制。
- 黑键判定 `col_in_oct = play_idx % 12`，黑键 = 1/3/6/8/10（lcd_master.v:206-209）。
- 柱高与颜色：`bar_h_cls` = 音色满高（谐波类 0）或满高>>1（谐波类 1）（lcd_master.v:212-214）；`bar_colr` = 红/暗红（类 0）或黄（类 1）（lcd_master.v:215-217）。
- 柱的"从下往上长"：`bar_dist = 179 - bar_row`，`bar_row_on = col_any && (bar_dist < bar_h_cls)`（lcd_master.v:225-226）→ 只有最底下 `bar_h_cls` 行点亮，其余写黑。

### 3.5 数字点阵取模

- `pos_in_group = dig_idx % 12` → `dig_code`（lcd_master.v:233-251）：0→1、1→10、2→2、3→11、4→3、5→4、6→12、7→5、8→13、9→6、10→14、11→7（1..7 是音名数字字形，10..14 是黑键/带升号字形）。
- `glyph`（lcd_master.v:254-270）为 15 位，按"行优先、左到右、MSB 在前"排列。例：`5'd1 = 15'b010_110_010_010_111`。
- 取模：`pix_n = dig_pix_cnt[4:1]`（0..14）、`pix_row = pix_n / 3`、`pix_col = pix_n % 3`、`dig_on = glyph[(4 - pix_row)*3 + (2 - pix_col)]`（lcd_master.v:272-275）。像素颜色 `8'hFF`/`8'h00` → 0xFFFF 白 / 0x0000 黑（lcd_master.v:985）。

### 3.6 屏幕版图（320×240，横屏）

```
y=0..43     : 开机清黑后不再使用（保持黑）
y=44..223   : 主区（180 行）—— 频谱柱 或 波形，二选一，切换时整区清黑
y=224..231  : 未使用（保持黑）
y=232..236  : 60 个音符数字（DIGIT_Y=232，高 5）
y=237..239  : 未使用（保持黑）
x: 第 i 列（i=0..59）→ x = 16 + 5i .. 18 + 5i  （最右列 i=59 → x=311..313）
```

---

## 4. 状态机

主 `case (state)` 位于 lcd_master.v:686-1168，`default: state <= S_IDLE_WAIT`（lcd_master.v:1167）。

| 状态 | 行号 | 做什么 | 跳转条件 → 目标 |
|------|------|--------|-----------------|
| `S_POWER_ON` | lcd_master.v:688-703 | `lcd_cs<=1`、`lcd_sck<=0`、`lcd_blk<=1`；`delay_cnt` 自增 | `delay_cnt >= PWR_DELAY`（200 ms）→ `S_INIT`，并复位 `send_idx/bit_idx/phase/sck_div_cnt`、`lcd_cs<=0` |
| `S_INIT` | lcd_master.v:705-727 | 按 `tbl_out` 发 30 字节初始化序列 | `send_idx == INIT_COUNT-1` 且第 8 位发完 → `S_IDLE`，`lcd_cs<=1` |
| `S_IDLE` | lcd_master.v:729-737 | 1 拍：`lcd_sck<=0`、`lcd_cs<=0`、`send_idx/bit_idx/phase/sck_div_cnt` 清零 | 无条件 → `S_CLEAR_WIN` |
| `S_CLEAR_WIN` | lcd_master.v:739-761 | 发 11 字节窗口：x=0..319、**y=0..239（整屏）**、0x2C | `send_idx == WIN_COUNT-1` → `S_CLEAR` |
| `S_CLEAR` | lcd_master.v:764-790 | 写 153600 字节 `8'h00`（全屏 320×240 黑） | `send_idx == CLEAR_COUNT-1` → 初始化 `dig_idx=0`、`dig_x=BAR_MARGIN`、`dig_y=DIGIT_Y`、`dig_pix_cnt=0` → `S_DIGIT_WIN` |
| `S_CLEAR_BAR` | lcd_master.v:792-815 | 发 11 字节窗口：x=0..319、**y=44(0x2C)..223(0xDF)** | `send_idx == WIN_COUNT-1` → `bar_pix_cnt=0` → `S_CLEAR_BAR_DATA` |
| `S_CLEAR_BAR_DATA` | lcd_master.v:818-842 | 写 `57600*2 = 115200` 字节 0x00（320×180 主区黑） | `bar_pix_cnt >= 115199` → 清零 → `S_IDLE_WAIT` |
| `S_BAR_WIN` | lcd_master.v:845-894 | ①先查模式切换 ②置 `score_addr<=play_idx`、`bar_x<=16+5*play_idx` ③`bar_dirty` 则发 11 字节柱窗口后进 `S_BAR_DATA`，否则 `play_idx` 自增（0..59 环回）并停留 | 模式变化 → `S_CLEAR_BAR`；窗口发完 → `bar_pix_cnt=0`、`bar_row=0`、`bar_colc=0` → `S_BAR_DATA` |
| `S_BAR_DATA` | lcd_master.v:896-943 | 写 `bar_pix_lim = 180*3*2 = 1080` 字节（3×180 像素 RGB565） | `bar_pix_cnt >= 1079` → `bar_state[play_idx]<=bar_target`、`bar_prev<=play_idx`、`play_idx` 自增（或环回 0）→ `S_BAR_WIN` |
| `S_DIGIT_WIN` | lcd_master.v:946-977 | 置 `dig_x<=16+5*dig_idx`、`dig_y<=232`，发 11 字节数字窗口 | 窗口发完 → `dig_pix_cnt=0` → `S_DIGIT_DATA` |
| `S_DIGIT_DATA` | lcd_master.v:979-1026 | 写 `3*5*2 = 30` 字节（15 像素） | 若 `dig_idx == BAR_NUM-1(59)` → `dig_idx=0`、`play_idx=0`、`sec_cnt=0` → `S_IDLE_WAIT`；否则 `dig_idx++` → `S_DIGIT_WIN` |
| `S_IDLE_WAIT` | lcd_master.v:1031-1082 | 模式分派 + 演奏节拍等待（见下） | 见下 |
| `S_WAVE_WIN` | lcd_master.v:1084-1113 | 发 11 字节窗口：x=`wave_col`（宽 1）、y=44..223 | 窗口发完 → `bar_pix_cnt=0` → `S_WAVE_DATA` |
| `S_WAVE_DATA` | lcd_master.v:1118-1165 | 写 `WAVE_COL_LIM = 360` 字节 = 180 像素：落在 `[y0,y1]` 区间的写青，其余写黑 | `bar_pix_cnt >= 359` → 若 `wave_col >= 319` → `wave_col=0`、`S_IDLE_WAIT`；否则 `wave_col++` → `S_WAVE_WIN` |

`S_IDLE_WAIT` 内部逻辑（lcd_master.v:1031-1082）：
1. `lcd_sck<=0`、`lcd_cs<=0`；`play_start_d <= play_start`（边沿检测基准只在此时更新，lcd_master.v:1035）。
2. 若 `wave_mode != wave_mode_d` → 更新 `wave_mode_d`、`bar_pix_cnt=0`、→ `S_CLEAR_BAR`（切模式先清主区）。
3. 否则若 `wave_mode==1` → `frame_wr <= wave_wr`（帧快照）、`wave_col=0` → `S_WAVE_WIN`。
4. 否则（频谱模式）：`play_start_pulse = play_start & ~play_start_d`（lcd_master.v:185）为真 → `play_idx=0`、`playing=1`、`sec_cnt=0` → `S_BAR_WIN`；否则若 `playing` → 等 `sec_cnt >= ONE_SEC_CNT/4`（12,500,000 拍 = **0.25 s**）后 `play_idx++`（或到头 `playing=0`）→ `S_BAR_WIN`/`S_IDLE_WAIT`；否则（空闲）→ `sec_cnt=0` → `S_BAR_WIN`（自由频谱循环）。

要点：`S_IDLE_WAIT` 同时是"分派态"和"节拍等待态"——演奏时它会**停在原地**用 `sec_cnt` 数满 0.25 s 才走（lcd_master.v:1072-1074），期间不刷新任何内容。

### 4.1 计数器一览

| 计数器 | 位宽 | 上限 | 对应时间/含义 | 行号 |
|--------|------|------|--------------|------|
| `delay_cnt` | 24 | 10,000,000 | 200 ms 上电延时 | lcd_master.v:49, 692 |
| `sec_cnt` | 26 | 12,500,000 | 0.25 s 音符步进 | lcd_master.v:1061 |
| `sck_div_cnt` | 8 | `SCK_DIV`=1（即 0..1） | 每个 SPI phase 2 个时钟 | lcd_master.v:706 等 |
| `bit_idx` | 3 | 0..7 | 一个字节 8 位，MSB first | lcd_master.v:710, 714 |
| `send_idx` | 32 | 0..29 / 0..10 / 0..115199 等 | 字节序号 | lcd_master.v:112 |
| `bar_pix_cnt` | **17** | 1079 / 359 / 115199 | 通用字节计数 | lcd_master.v:123, 829, 925, 1141 |
| `bar_row` | 8 | 179 | 柱内行 | lcd_master.v:125, 919 |
| `bar_colc` | 4 | 2 | 柱内列 | lcd_master.v:126, 917 |
| `dig_pix_cnt` | 16 | 29 | 数字像素字节 | lcd_master.v:1003 |
| `wave_dcnt` | 3 | 7 | 8:1 抽取 | lcd_master.v:471, 496 |
| `peak_div` | 16 | 0xFFFF（65,535） | 1.31 ms 一次峰值衰减（>>6）→ 时间常数 ≈ 83.9 ms | lcd_master.v:581-585 |
| `wave_wr` | 12 | 模 4096 | 环形写指针 | lcd_master.v:500 |

---

## 5. SPI 写入时序实现

### 5.1 位内四相（每个字节重复 4 相，每相 `SCK_DIV+1 = 2` 个 `sys_clk`）

```
phase0: lcd_sck<=0; cur_data<=表数据; lcd_dc<=表DC     ← DC 在 SCK 低电平期间改变
phase1: lcd_sck<=0; lcd_mosi<=cur_data[7-bit_idx]     ← MOSI 提前 2 拍建立
phase2: lcd_sck<=1                                     ← 上升沿，屏采样
phase3: lcd_sck<=1; 处理 bit_idx/send_idx/状态跳转
```

- 一个位 = 4×2 = 8 个时钟 = **160 ns**；SCK 高/低各 80 ns → **SCK = 50 MHz / (4×(SCK_DIV+1)) = 6.25 MHz**。
- 一个字节 = 8 位 = **1.28 µs**。
- MSB first：`cur_data[3'd7 - bit_idx]`（lcd_master.v:710, 744, 769, 797, 823, 874, 908, 959, 990, 1095, 1130）。
- 像素数据状态（`S_BAR_DATA`/`S_WAVE_DATA`/`S_DIGIT_DATA`/`S_CLEAR*_DATA`）里 `lcd_dc` 恒为 1（lcd_master.v:904, 1126, 986, 822）。
- `lcd_cs` 只在复位、`S_POWER_ON`、`S_INIT` 结束那一拍为 1；其余全程为 0（**不按字节/命令翻转 CS**）。
- 每条命令的 DC 位来自查表结果的最高位：`{dc, data}` 拼成 9 位 `tbl_out` / `bar_cmd_out` / `dig_cmd_out` / `wave_cmd_out`（lcd_master.v:357-358, 384-385, 413-414, 445-446）。

### 5.2 初始化命令序列（`S_INIT` 表，lcd_master.v:283-317，共 30 字节）

| idx | DC | 字节 | ST7789 命令/含义 |
|-----|----|------|-----------------|
| 0 | 0 | `0x36` | MADCTL |
| 1 | 1 | `0x70` | 0b0111_0000：MY=0, MX=1, MV=1, **ML=1**, BGR=0, MH=0 → 行列交换得到 320×240 横屏、RGB 顺序 |
| 2 | 0 | `0x3A` | COLMOD |
| 3 | 1 | `0x05` | 16 bit/pixel（RGB565） |
| 4 | 0 | `0xB2` | PORCTRL |
| 5 | 1 | `0x0C` | |
| 6 | 1 | `0x0C` | |
| 7 | 1 | `0x00` | |
| 8 | 1 | `0x33` | |
| 9 | 1 | `0x33` | |
| 10 | 0 | `0xB7` | GATECTRL |
| 11 | 1 | `0x35` | |
| 12 | 0 | `0xBB` | VCOMS |
| 13 | 1 | `0x19` | |
| 14 | 0 | `0xC0` | LCMCTRL |
| 15 | 1 | `0x2C` | |
| 16 | 0 | `0xC2` | VDVVRHEN |
| 17 | 1 | `0x01` | |
| 18 | 0 | `0xC3` | VRHS |
| 19 | 1 | `0x12` | |
| 20 | 0 | `0xC4` | VDVS |
| 21 | 1 | `0x20` | |
| 22 | 0 | `0xC6` | FRCTRL2 |
| 23 | 1 | `0x0F` | |
| 24 | 0 | `0xD0` | PWCTRL1 |
| 25 | 1 | `0xA4` | |
| 26 | 1 | `0xA1` | |
| 27 | 0 | `0x21` | INVON（反显开） |
| 28 | 0 | `0x11` | SLPOUT |
| 29 | 0 | `0x29` | DISPON |

注意：**没有**软复位 `0x01`、**没有** `0x13` NORON、**没有** `0x20` INVOFF。颜色正确性依赖 `0x70`(BGR=0) 与 `0x21`(INVON) 这一对组合，二者必须同时保持。
`INIT_COUNT = 30` 与表条目数严格绑定：`send_idx` 是 8 位索引，表默认返回 `9'h000`（DC=0, 数据 0x00）。

### 5.3 窗口命令（`0x2A` / `0x2B` / `0x2C`，固定 11 字节）

三张窗口表结构一致（lcd_master.v:336-351、367-383、396-412、428-444）：

```
idx0: DC=0, 0x2A（CASET）  idx1: x 高字节  idx2: x 低字节  idx3: xe 高字节  idx4: xe 低字节
idx5: DC=0, 0x2B（RASET）  idx6: y 高字节  idx7: y 低字节  idx8: ye 高字节  idx9: ye 低字节
idx10: DC=0, 0x2C（RAMWR）
```

- 整屏清屏窗口：x `0x0000..0x013F`(0..319)、y `0x0000..0x00EF`(0..239)（lcd_master.v:319-334）。
- 主区清黑窗口：x `0x0000..0x013F`、y `0x002C..0x00DF`(44..223)（lcd_master.v:336-351）。
- 柱窗口：x = `bar_x .. bar_x+2`（`bar_x = 16 + play_idx*5`）、y = 44..223（lcd_master.v:364-365, 222-223）。
- 数字窗口：x = `dig_x .. dig_x+2`、y = 232..236（lcd_master.v:391-394）。
- 波形列窗口：x = `wave_xs = wave_xe = wave_col`（1 像素宽）、y = 44..223（lcd_master.v:422-426）。

`WIN_COUNT=11` 与"表内 0..10 共 11 条"绑定；若把 `WIN_COUNT` 调大，超出的字节会取 `default: 9'h000`（DC=0 且数据 0x00），等于往命令流里塞非法命令，会破坏后续解析。

### 5.4 像素数据发送顺序

RGB565 每像素 2 字节、**高字节先发**：
- `S_BAR_DATA`：phase0（偶数字节）装 `bar_pix_hi`；phase3（偶数字节）预装 `bar_pix_lo` 并在奇数字节结束时推进 `bar_colc`/`bar_row`（lcd_master.v:900-936）。
- `S_WAVE_DATA`：同上，偶数字节装 `wave_pix_hi`，奇数结束时 `bar_pix_cnt` 自增/换列（lcd_master.v:1122-1157）。
- `S_DIGIT_DATA`：偶数字节装 `dig_on ? 8'hFF : 8'h00`，`dig_pix_cnt` 每字节自增，`dig_on` 由 `dig_pix_cnt[4:1]` 决定（lcd_master.v:983-1001）。

---

## 6. 显存组织（关键结论）

**本模块内部没有帧显存（framebuffer），也没有开放给外部的写显存接口。**"显存"就是 ST7789 芯片内部的 GRAM，由本模块通过"设窗口 + 0x2C 连续写"的方式直接流过 SPI 写入。

- 地址映射：帧内坐标直接就是 ST7789 的列/行地址。由于 `MADCTL=0x70` 做了行列交换，`0x2A` 对应 x（0..319）、`0x2B` 对应 y（0..239）。
- 一个窗口内的字节流按"列先增、行后增"的顺序填充（ST7789 RAMWR 语义）：柱窗口 3×180 = 540 像素 = 1080 字节；数字窗口 3×5 = 15 像素 = 30 字节；波形窗口 1×180 = 180 像素 = 360 字节。
- 唯一的片上存储是 `wave_buf[0:4095]`（4096×16 bit 有符号音频采样环，约 64 Kbit，需约 4 块 18Kb BSRAM）。它的"地址映射"是：写指针 `wave_wr` 每 8 个采样 +1（8:1 抽取）；读地址 `wave_rd_addr0 = frame_wr - 320 + wave_col`、`wave_rd_addr1 = addr0 + 1`（lcd_master.v:511-512），12 位无符号回绕等价于"快照点往前数 320 个采样"，`+1` 用于把相邻两列连成折线。
- `bar_state[0:59]`（60×6 bit）是"脏跟踪"位图，不是显存，只记录上次每列画成了什么。
- 每次刷新都是"边算边写屏"，无双缓冲 → 一帧内可见逐列推进的撕裂（波形模式一帧 152 ms，肉眼可见从上到下/从左到右的刷新）。

---

## 7. 内容更新接口（外部如何影响显示）

| 输入 | 协议/握手 | 时序细节 | 行号 |
|------|----------|---------|------|
| `key_levels[35:0]` | 电平直连（无握手） | 逐列检查 `col_fund[play_idx]`；变化通过 `bar_dirty` 触发该列重画 | lcd_master.v:196-204, 862 |
| `timbre_idx[1:0]` | 电平直连 | 仅在写柱时采样，决定满高与颜色 | lcd_master.v:191-193, 203, 212-217 |
| `wave_mode` | 电平直连 | `wave_mode_d` 打拍比较，切换时先整区清黑再重画 | lcd_master.v:853-858, 1038-1043 |
| `wave_send`/`wave_tgl` | **翻转沿握手**（数据稳定，每新采样翻转一次） | 3 级同步链 `wave_t1/t2/t3`，`wave_t2 ^ wave_t3` 检出一次事件；`wave_in <= wave_send` 先打 1 拍；每 8 个事件才写一次 `wave_buf`（`WAVE_DECIM=7`） | lcd_master.v:476-506 |
| `play_start` | 电平保持型，`S_IDLE_WAIT` 内取上升沿 | `play_start_d` **只在 `S_IDLE_WAIT` 更新**（lcd_master.v:1035），所以 `play_start` 必须是电平保持的脉冲（顶层注释 top.v:3172 明确要求）；1 拍的窄脉冲可能被漏掉 | lcd_master.v:185, 1035, 1054 |
| `play_len` | 电平 | `play_idx >= play_len - 1` 时停止演奏 | lcd_master.v:1063 |
| `score_addr`/`score_data` | 8 位地址 → 8 位数据，顶层为**寄存读**（1 拍延迟） | `score_addr <= play_idx` 在 `S_BAR_WIN` phase0 设置；但 `score_data` 只参与 `bar_score_val`，而 `bar_score_val` **未被任何逻辑引用** | lcd_master.v:21-22, 189, 860 |

**结论（很重要）**：当前版本里乐谱数据（`score_addr`/`score_data`）对画面**没有任何影响**——柱的内容只由 `key_levels` + `timbre_idx` 决定，柱高也只有 4 档（60/100/140/180，谐波时减半），不是按每个音符的真实音高/音量绘制。想让屏幕显示乐谱内容，必须真正把 `score_data` 接进 `bar_target` / `bar_h_cls`。

波形数据链路（供核对注释与现实是否一致）：
- 抽取后有效采样率 = 48.83 kHz / 8 ≈ **6.10 kHz**，Nyquist 3.05 kHz；一屏 320 点 ≈ **52.4 ms** 时间窗。
- 环形缓冲 4096 点写满需 4096/6.10 kHz ≈ **671 ms**；一帧 152 ms，写指针一帧只走 ≈ 928 点，**不会被覆盖**（注释里写的 2048 点/335 ms 与实际 4096 深度不符，代码是 4096）。
- 绘图纵向刻度是**自动的**：`wave_peak` 峰值跟踪（快攻 + 每 1.31 ms 衰减 1/64，时间常数 ≈ 84 ms，lcd_master.v:576-587），`wave_sh = msb(peak) - 5`（lcd_master.v:590-598）使 `peak >> sh` 落在 [32,64)，再对采样取绝对值 → 逻辑右移 → 补回符号（17 位有符号，lcd_master.v:570-608），`y = 133 + v` 再用 **17 位有符号常量**钳到 44..223（lcd_master.v:611-618；注释 lcd_master.v:523-543 解释了历史上用无符号比较导致负半周贴底线的 bug）。
- 一列画 `[seg_lo, seg_hi] = [min(y0,y1), max(y0,y1)]` 之间的竖线，其余全写黑（lcd_master.v:620-636, 1118-1165）→ 每列都整列重写 180 像素，旧点自然被覆盖。

---

## 8. 时间预算（sys_clk=50 MHz，1 字节 = 1.28 µs）

| 动作 | 字节数 | 时间 |
|------|--------|------|
| 上电等待 | — | 200 ms |
| 初始化 30 字节 | 30 | 38.4 µs |
| 清屏窗口 11 字节 | 11 | 14.08 µs |
| 全屏清黑 | 153,600 | **196.6 ms** |
| 只清主区 | 11 + 115,200 | **147.5 ms** |
| 一根柱（脏） | 11 + 1080 | 1.397 ms |
| 60 根柱全脏 | 65,460 | **83.8 ms** |
| 一个数字 | 11 + 30 | 52.5 µs |
| 60 个数字 | 2,460 | 3.15 ms |
| 波形一列 | 11 + 360 | 474.9 µs |
| 波形一整帧（320 列） | 118,720 | **152.0 ms**（≈6.6 fps） |
| 频谱空转一圈（都不脏） | 0 | 60 个时钟 = 1.2 µs |
| 演奏节拍 | — | 0.25 s/步（`ONE_SEC_CNT/4`） |

开机到首帧可见总耗时 ≈ 200 ms + 0.038 ms + 0.014 ms + 196.6 ms + 3.15 ms ≈ **400 ms**。开机流程不经过 `S_CLEAR_BAR`（`S_CLEAR` 已整屏清掉），`S_CLEAR_BAR` 只在切模式时走。

---

## 9. 绝对不能动 / 可以安全修改

### 9.1 绝对不能动（时序/协议/坐标耦合的硬约束）

1. **端口名、方向、位宽**（lcd_master.v:15-39）：顶层按名例化（top.v:3433-3454），引脚由 `music.cst` 按名约束（`lcd_sck D22`、`lcd_mosi F20`、`lcd_dc W19`、`lcd_cs W20`、`lcd_blk F19`）。
2. **`SCK_DIV = 8'd1` 与四相结构**（lcd_master.v:48, 706-726）：决定 SCK=6.25 MHz 与 DC/MOSI 的建立窗口。改 SCK_DIV=0 会让 SCK 变 12.5 MHz。
3. **`PWR_DELAY = 10_000_000`（200 ms）**（lcd_master.v:49）：面板上电稳定时间。
4. **30 字节初始化序列的每一条数值与顺序**，尤其 `0x36=0x70`（横屏+BGR=0）与 `0x3A=0x05`（RGB565）、`0x21`(INVON)、`0x11`、`0x29`（lcd_master.v:283-317）。改 MADCTL 会让所有坐标/颜色全错；改 COLMOD 会让每像素字节数错。
5. **`INIT_COUNT=30`、`WIN_COUNT=11` 与各自查表条目数的绑定**（lcd_master.v:44/46 与 283-317/319-351）。
6. **窗口坐标常量**：`0x013F`(319)、`0x00EF`(239)、主区 `0x2C..0xDF`(44..223)、`DIGIT_Y=232`、`BAR_Y_TOP/BOT`、`WAVE_Y_TOP/BOT`。
7. **`bar_pix_cnt` 必须保持 17 位**（lcd_master.v:123）：`S_CLEAR_BAR_DATA` 要数到 115199（>65535）。缩成 16 位会提前结束清屏。
8. **每状态的字节数上限必须等于窗口像素数×2**：`CLEAR_COUNT=153600`、115200、`bar_pix_lim=1080`、`WAVE_COL_LIM=360`、`DIGIT_W*DIGIT_H*2-1=29`。任何一个对不上，ST7789 会停在半个窗口里，后续命令流错位。
9. **RGB565 高字节先发**（lcd_master.v:902-915, 1124-1137）：先低字节会变成蓝红互换的错色。
10. **`wave_buf` 必须是同步读**（lcd_master.v:517-520）：改成组合读会被展开成 4096:1 选择器（几十万 LUT），布线器会卡死（lcd_master.v:514-515 注释）。
11. **`wave_rd_addr0` 必须用 12 位无符号回绕**（lcd_master.v:511）：用 13 位再截会在负值时算错，读成 0 → 画成直线（lcd_master.v:508-510 注释）。
12. **纵向钳位必须与有符号常量比较**（lcd_master.v:613-618 用 `17'sd223`/`17'sd44`，不是 `[15:0]` 的 `WAVE_Y_BOT/TOP`）：混入无符号操作数会把负数变成巨大无符号数，负半周会被贴到底线（lcd_master.v:523-543 记录了这段历史 bug）。
13. **`S_IDLE_WAIT` 里锁的是 `wave_wr` 而不是 `wave_wr_sync`**（lcd_master.v:1048）。
14. **CS/SCK/DC 的复位电平**（lcd_master.v:655-659：sck=0, mosi=0, dc=0, cs=1, blk=0）与 `lcd_blk` 在 `S_POWER_ON` 拉 1（lcd_master.v:691）。

### 9.2 可以安全修改

- 所有 `COLOR_*` 颜色值（16 位 RGB565），改完只影响配色（注意 `COLOR_KEY_BK` 只在黑键、`COLOR_H2` 只在"基频灭+2 次谐波亮且 timbre_idx[0]=1"时生效）。
- 字形点阵 `glyph`（lcd_master.v:254-270）与 `dig_code` 映射（lcd_master.v:233-251）：只要保持 15 位、行优先 MSB 在前的排布约定。
- 柱高 `BAR_H0..BAR_H3`（lcd_master.v:62-65）：60/100/140/180，最大不要超过 180。
- `WAVE_DECIM`（lcd_master.v:471）：改抽取率等于改时间窗与缓冲占用，合法但要重算 §7 的时间预算。
- 峰值跟踪参数：`wave_peak` 初始 2048、`peak_div == 16'hFFFF` 的衰减周期、`>>6` 的衰减比、`wave_sh = msb-5` 的刻度（lcd_master.v:578-598）。
- `BAR_MARGIN` / `BAR_STEP` / `BAR_W`：约束是 `BAR_MARGIN + 59*BAR_STEP + BAR_W - 1 <= 319`（当前 16+295+3-1 = 313，右侧余 6 像素）。
- 启动清屏是否走全屏（`CLEAR_COUNT`）与 `state_out` 调试输出、以及所有死代码（见 §12）。
- `sec_cnt` 节拍（`ONE_SEC_CNT/4`）与 `play_len` 判定逻辑。

### 9.3 潜在隐患（改动前必读）

1. **窗口发送中途可能被 `bar_dirty` 变化打断**：`S_BAR_WIN` 在每个字节的 phase0 都重新判断 `bar_dirty`（lcd_master.v:845-873）。当前 `bar_target` 只依赖 `key_levels`/`timbre_idx`（`score_data` 未参与），所以窗口发送期间它保持不变，不会中途改判；**一旦将来把 `score_data` 接进 `bar_target`，就必须把 `bar_dirty` 判断限制在 `send_idx==0` 且相位起点**，否则会在 11 字节窗口中途走 `else` 分支 → 只发出一个孤立的 `0x2A`（或后续字节错位），导致 ST7789 命令解析错位、屏幕花屏。
2. **`S_BAR_WIN` 的模式切换检查（lcd_master.v:853-858）会在窗口发送中途跳去 `S_CLEAR_BAR`**，且**不清零 `send_idx`/`bit_idx`**；而 `S_CLEAR_BAR` 的查表用 `send_idx[3:0]`，于是窗口命令会从中间条目开始发（例如从 `0x01` 开始），同样会造成命令流错位。稳妥做法：只在 `send_idx==0` 时检查模式切换，并在跳转时把 `send_idx<=0`、`bit_idx<=0`、`phase<=0`。
3. **`score_addr` 与顶层寄存读有 1 拍延迟**（top.v:3135-3139）：`score_addr <= play_idx` 与被判定的 `score_data` 不同拍。目前在 `S_BAR_WIN` 的"不脏"快路径上每拍循环一次，第一拍用的是上一列的 `score_data`。因为 `score_data` 未被使用，现在没有后果；接入使用后需要先取数据再判定（多停一拍或加 `valid` 标志）。
4. **波形模式下不会重绘数字行**：`S_DIGIT_WIN` 只从 `S_CLEAR` 和 `S_DIGIT_DATA` 进入，切到波形后 y=232..236 仍保留上一次频谱模式画的内容；切模式只清 y=44..223。
5. **第 36..59 列永远不亮柱**（`col_fund[59:36]=0`，lcd_master.v:169），但数字照画（`dig_idx` 仍到 59）。
6. **`S_IDLE_WAIT` 的 `play_start` 采样窗口很稀疏**：自由频谱循环时每圈只在那里停留 1 拍（lcd_master.v:1076-1079）。因此 `play_start` 必须电平保持（顶层已保证，top.v:3172），否则 1 拍窄脉冲会被漏掉。
7. **改 `wave_mode` 的切换要花 147.5 ms 清区**（`S_CLEAR_BAR`），期间屏幕上一片黑。

---

## 10. 死代码 / 未使用清单（清理或复用时的参考）

| 名称 | 位置 | 状态 |
|------|------|------|
| `bar_score_val` | lcd_master.v:189 | 计算后**无任何引用** → `score_data` 实际是死输入 |
| `cur_dc` | lcd_master.v:117 | 只写不读（真正驱动引脚的是 `lcd_dc`） |
| `bar_prev` | lcd_master.v:120 | 只写不读 |
| `bar_color` | lcd_master.v:124 | 只在复位赋值 |
| `wave_wr_sync` | lcd_master.v:140 | 只写不读（注释明确说不要用，lcd_master.v:1046-1047） |
| `wave_we` | lcd_master.v:474 | 只写不读 |
| `wave_ys` / `wave_ye` / `wave_pix` | lcd_master.v:134-135 | 声明为"保留（现在不用了）" |
| `col_h3` / `col_h4` | lcd_master.v:172-173 | 因为 `col_on3`/`col_on4` 恒 0（lcd_master.v:198-199）而不可达 |
| `bar_h` | lcd_master.v:219 | `bar_h_cls` 的别名，无引用 |
| `WAVE_CLAMP` / `WAVE_Y_MID` / `WAVE_COL_H` / `COLOR_AXIS` / `COLOR_BG_W` / `BAR_Y_BOT` | lcd_master.v:74, 68, 72, 77, 78, 59 | 定义后未引用 |
| `COLOR_H3` / `COLOR_H4` 分支 | lcd_master.v:217 | `harm_cls` 只取 0/1，分支不可达 |
| 注释与代码不一致处 | lcd_master.v:456-468 说"环形缓冲 2048 点"、top.v:2945 说"512 深" | 实际 `wave_buf` 是 **4096** 深（lcd_master.v:138）；lcd_master.v:456-459 描述的"每列平均 45 像素"优化**并未实施**，`S_WAVE_DATA` 仍每列写 180 像素（lcd_master.v:1141） |

---

## 11. 关键行号索引

- 端口定义：lcd_master.v:15-39
- 参数区：lcd_master.v:44-108
- 寄存器声明：lcd_master.v:110-143
- `col_fund` 键位重排：lcd_master.v:148-169
- 谐波/脏标记/颜色选择：lcd_master.v:189-228
- 字形表：lcd_master.v:233-275
- 初始化表：lcd_master.v:283-317
- 整屏清屏窗口表：lcd_master.v:319-334
- 主区清黑窗口表：lcd_master.v:336-351
- 柱/数字/波形窗口表：lcd_master.v:363-446
- 波形采样接收 + 抽取：lcd_master.v:476-506
- 环形缓冲同步读：lcd_master.v:511-520
- 自动刻度（峰值跟踪 + 有符号处理 + 钳位）：lcd_master.v:570-640
- 主状态机：lcd_master.v:645-1170
  - `S_POWER_ON` 688、`S_INIT` 705、`S_IDLE` 729、`S_CLEAR_WIN` 739、`S_CLEAR` 764、`S_CLEAR_BAR` 792、`S_CLEAR_BAR_DATA` 818、`S_BAR_WIN` 845、`S_BAR_DATA` 896、`S_DIGIT_WIN` 946、`S_DIGIT_DATA` 979、`S_IDLE_WAIT` 1031、`S_WAVE_WIN` 1084、`S_WAVE_DATA` 1118
- `state_out` 输出：lcd_master.v:1172
