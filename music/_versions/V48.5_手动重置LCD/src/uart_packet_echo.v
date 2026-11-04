`timescale 1ns / 1ps

//=============================================================================
// uart_packet_echo.v   ——  蓝牙乐谱帧接收器 + 播放器（协议 v2：和弦 / 速度 / 时值）
//
//   ★ 和弦为什么这么容易 ★
//     音频引擎（top.v）本来就是 36 路独立发声，`midi_keys` 就是 36 位掩码，
//     同时置多个位 = 同时响多个音。所以"和弦"这件事在板子上【本来就支持】，
//     缺的只是"协议怎么把同一时刻的多个音送进来"。
//
//   ★ 本版的核心简化 ★
//     旧版把【字节流】存进 BRAM，播放时再逐字节解析 → 想支持和弦就得写一个
//     "向后扫描到 0xC1"的状态机 + 多周期 BRAM 读，很啰嗦。
//     现在改成：【接收时就把和弦拼好】，每个时间格只存一条记录：
//         slot_mask[i] = 这一格要点亮哪些键（36 位掩码）
//         slot_dur[i]  = 这一格要占几个时间格（1..255）
//     于是播放器退化成一句 `midi_keys <= slot_mask[当前格]`，
//     原来的演奏结构（换格、按住、停播）几乎一行没动。
//
//   ---- 帧结构 ----
//     [0xFF × 16]  前导冲刷（可选；关窗状态下只被忽略，真正"停播+清空"靠 0xAA）
//     [0xAA]       开窗 + 写指针归零 + 停播 + 把设置复位成默认
//     [0xC4 + 3B]  设置时间格（µs，小端）      可选
//     [0xC5 + 3B]  设置释音提前量（µs，小端）  可选
//     [事件...]    单音 / 休止 / 和弦，各自可跟一个 0xC2
//     [0xFF]       帧尾：关窗 + 开播 + 回传一个 0xFF
//
//   ---- 事件字节 ----
//     单音   键号 0x00..0x23        键号 就是 top.v 里 delta[键号] 的下标
//     休止   0xEE                   这一格不发声
//     和弦   0xC0 音号{1..36} 0xC1  这一格里所有音【同时】发声
//     时值   0xC2 + 1 字节 N        紧跟在上一个事件之后：该事件共占 1+N 格
//
//   ---- 操作数不受控制字节解释 ----
//     0xC2 的 N、0xC4/0xC5 的 3 个参数，即使值恰好是 0xAA/0xFF/0xEE，
//     也一律当【原始数据】，由"待收操作数"计数器保证。
//
//   ★ 与 v1 的兼容 ★
//     不带任何新字节的 v1 帧 → 时间格取默认 500ms、释音提前 300ms、每事件 1 格
//     → 每个音响 0.2s、留白 0.3s，**与 v1 逐位等价**。
//
//   ★★★ 键号到底是什么（以 top.v 的 delta[] 表为准，别再按"MIDI−48"算）★★★
//
//     键号【不是】按音高排的，它是 top.v 里 delta[键号] 的下标。按下表：
//
//         键号 0 = A#2  (117Hz)   键号 12 = A#3  (233Hz)   键号 24 = A#4  (466Hz)
//         键号 1 = C3   (131Hz)   键号 13 = C4   (262Hz)   键号 25 = C5   (523Hz)
//         键号 2 = D#3  (156Hz)   键号 14 = D#4  (311Hz)   键号 26 = D#5  (622Hz)
//         键号 3 = F3   (175Hz)   键号 15 = F4   (349Hz)   键号 27 = F5   (698Hz)
//         键号 4 = G3   (196Hz)   键号 16 = G4   (392Hz)   键号 28 = G5   (784Hz)
//         键号 5 = A2   (110Hz)   键号 17 = A3   (220Hz)   键号 29 = A4   (440Hz)
//         键号 6 = B2   (123Hz)   键号 18 = B3   (247Hz)   键号 30 = B4   (494Hz)
//         键号 7 = C#3  (139Hz)   键号 19 = C#4  (277Hz)   键号 31 = C#5  (554Hz)
//         键号 8 = D3   (147Hz)   键号 20 = D4   (294Hz)   键号 32 = D5   (587Hz)
//         键号 9 = E3   (165Hz)   键号 21 = E4   (330Hz)   键号 33 = E5   (659Hz)
//         键号10 = F#3  (185Hz)   键号 22 = F#4  (370Hz)   键号 34 = F#5  (740Hz)
//         键号11 = G#3  (208Hz)   键号 23 = G#4  (415Hz)   键号 35 = G#5  (831Hz)
//
//     换算公式（上位机用）。一个八度里的接线顺序是：
//         先 5 个黑键 A# C D# F G，再 7 个白键 A B C# D E F# G#
//         —— 不是按音高从低到高，所以键号 13 才是 C4、键号 29 才是 A4。
//         PERM = [5,0,6,1,7,8,2,9,3,10,4,11]
//         键号 = PERM[(MIDI − 45) mod 12] + 12 × ⌊(MIDI − 45) / 12⌋
//     板子的音域 = MIDI 45(A2) .. 80(G#5)，共 36 键。
//     ★ 这条不是"约定"，是被 top.v 的 delta[] 表钉死的：例如 MIDI 69(A4,440Hz)
//       → 键号 29；这同时也是判断"发出去到底响没响对"的唯一标准。
//     ★ 上位机侧由 check_rtl_contract.py 拿 delta[] 自动核对，改错了会红。
//
//   ---- 演奏录音回传（v2.2 新增，方向与上面相反：板子 → 上位机）----
//     板子把"手弹了什么"回传上去，上位机合成 MIDI 并在电脑上放出来：
//       [0x5A] [mask 36 位·小端 5B] [dt 2B·小端，单位 10ms]   一个事件
//       [0x5B] [0/1]                                          开关状态
//       [0x5C]                                                上位机 → 板子：翻转开关
//     录音开关两端都能控制：板子【按钮板第 10 键】或上位机发 0x5C。
//     ★ mask 是 key_mask 的快照，好几位就是同时按着好几个键 —— 和弦天然支持。
//     ★ 录音期间【整体抑制回显】：回显字节是任意值，撞上 0x5A 会被上位机
//       当成帧头，凭空多出一个事件还把时间轴带偏。详见第 4b/4c 节的注释。
//=============================================================================
module uart_packet_echo (
    input  wire        clk,            // 50MHz（clk_50m_ext）
    input  wire        rst_n,
    input  wire        uart_rxd,       // 蓝牙模块 TXD -> FPGA
    output reg         uart_txd,       // FPGA -> 蓝牙模块 RXD

    //==================================================================
    // ★ 录音回传（新增）
    //
    //   录的是"板子打算让哪些键响"：top.v 的 sounding_mask_aud（门控）
    //   = key_mask | sustain_latch_aud。每 10ms 采一次，一变就回传一个事件帧：
    //       [0x5A] [mask 36 位，小端 5 字节] [dt 2 字节，单位 10ms]
    //   另外还把【效果开关】回传上去（变了才发，开始录音时强制发一次）：
    //       [0x5D] [8 位：滑音/延音/音色/颤音/延迟/笛子/混响/合唱]
    //   上位机靠它把效果在 MIDI 里近似还原。
    //   开关两端都能控制：
    //       · 板子上按【按钮板的第 10 键】
    //       · 上位机发一个 0x5C
    //   开关状态变化时回传 [0x5B] [0/1]，让上位机同步界面。
    //
    //   ★★ 为什么送"门控"，而不是"包络输出" sounding_mask = (env != 0) ★★
    //     包络输出带着 1.46 秒释音尾巴，看着更"实际"，但有两个致命问题：
    //       ① 尾巴没走完就重击同一个键，掩码连着不变 —— 两下会并成一个长音
    //          （实测：音高 62 弹两次 1000ms / 2600ms，合成了一个 3730ms 长音）；
    //       ② 掩码里一直为 1 的键会在【每个事件】上重复触发 ——
    //          8 个音弹出来是 32 次起音。
    //     门控信号的起音/放音/重击边沿才是真的，延音锁存也天然在里面
    //     （sounding_mask_aud 本身就 = key_mask | sustain_latch_aud）。
    //     释音尾巴改由上位机按 RTL 常数补（见 top.v 那段注释与
    //     tools\协议说明_v2_和弦扩展.md §9.2）。
    //
    //   ★ 录音期间【不回显】—— 回显的字节是任意值，会和 0x5A 帧头撞车。
    //     反正录音时上位机也不在发谱子，不回显不影响任何事。
    //==================================================================
    input  wire [35:0] gate_mask_in,    // 门控掩码 = 按键 | 延音锁存（已同步到本时钟域）
    // ★★★ V43.7：跟谱模式判"按对了"的结果，从 clk_audio 域送进来（1 位）★★★
    //   ★ 为什么不是直接送 36 位的手弹掩码（V43.6 那么做过）：
    //     V43.6 加了一条 36 位两级同步链（72 个触发器）+ 一个 36 位比较，
    //     实测 CLKOUT1 从 59.197 掉到 47.157MHz（TNS -6.566 / 8 条），
    //     而且【重跑一次结果一模一样】—— 是真实退化不是布局撞运气。
    //   ★ 现在改成：在 clk_audio 域（那边本来就有 hand_mask 和当前格掩码、
    //     而且余量 2.3 倍）算好"按对了没有"，只把【1 位结果】同步过来。
    //     同步链从 72 个触发器降到 2 个。
    input  wire        keys_ok_in,
    // ★ V43：TTP 按键13 的【单拍脉冲】（已同步到本时钟域）—— 翻转跟谱模式
    //   ★ 为什么用脉冲而不是电平：模式和 0xCE 共用同一个 wait_mode 寄存器，
    //     只能有一个 always 块驱动它。按键那边在 top.v 里做成脉冲送进来，
    //     由本模块统一翻转 —— 不会 EX2000（交接文档坑 1）。
    input  wire        btn_mode_tgl,
    input  wire [7:0]  fx_state_in,      // 影响声音的 8 个效果开关（已同步到本时钟域）
    input  wire        btn_rec,          // 录音按钮：单拍脉冲（已同步到本时钟域）

    // ---- 按键自定义映射的写口（给 top.v 的 key_map[] 用）----
    //   ★ 跨时钟域：top.v 那边用"翻转信号 + 边沿检测"接收（和 btn_rec 同一套做法），
    //     所以这里只保证 key/note 两个字节在翻转沿到来之前已经稳定。
    output reg         keymap_we_tgl,    // 每写一条完整映射翻转一次
    output reg  [5:0]  keymap_key,       // 物理键号 0..35
    output reg  [5:0]  keymap_note,      // 目标音符 0..35；>=36 表示禁用该键

    // ---- 效果开关 / 音色的写口（给 top.v 的效果寄存器用）----
    //   ★ 和上面 keymap 同一套跨时钟域做法：翻转信号 + 边沿检测，
    //     数据位在翻转沿到来之前保持稳定，接收侧在沿上采。
    output reg         fx_tgl,           // 每收到一条"翻转效果"命令翻转一次
    output reg  [2:0]  fx_bit,           // 翻转哪一位（0..7；5 表示音色循环）
    output reg         tone_we_tgl,      // 每收到一条"设定音色"命令翻转一次
    output reg  [1:0]  tone_val,         // 目标音色 0 普通 / 1 笛子 / 2 吉他 / 3 小号
    //   ★ 当前音色（top.v 送来，用于回报）；本域里两级同步后再用
    input  wire [1:0]  tone_sel_in,

    // ---- ★ V32：当前正在播放的【事件八度偏移】（送给 top.v 的声部）----
    //   有符号 3 位：0 = 原调，+n = 升 n 个八度，-n = 降 n 个八度（-4..+3）。
    //   它跟着事件走：装载下一个事件时一起换，事件放完自动回 0。
    output reg  [2:0]  oct_cur,

    // ---- LCD 相关输出：top.v 里那条自动演奏链已停用，这里保持端口不变 ----
    output reg         score_we,
    output reg [7:0]   score_waddr,
    output reg [7:0]   score_wdata,
    output reg         play_start,
    output reg [7:0]   play_len,

    // ---- 发声：36 键掩码，直接 OR 进 top.v 的 key_mask ----
    output reg  [35:0] midi_keys,      // 当前时间格要点亮的键（和弦 = 多个位）
    // ---- ★ V37：逐键八度（36 键 × 3 位，扁平 108 位）----
    //   与 midi_keys 【同一拍、同一个 always 块】更新并共用同一个 gate，
    //   所以 top.v 那边同步过去之后，掩码和八度天生是同一采样点的一对。
    //   第 g 个键的八度 = midi_octs[g*3 +: 3]，有符号 3 位，语义同 0xCC。
    //   ★ 整格八度（0xCC）在【落盘时】就被烘进了每个键里（见下面 (0) 段），
    //     所以这里读到的永远是"本键最终该用的八度"，top.v 不需要再做选择。
    output reg  [107:0] midi_octs,
    // ★ V43：LED 指引的【当前格】掩码（不参与发声；跟谱模式等待期间也保持显示）
    output reg  [35:0] midi_guide,
    // ---- ★ V42：给 LED 用的【下一格】掩码（不参与发声，纯显示）----
    //   和 midi_keys 【同一个 gate、同一个 always 块、同一拍更新】，
    //   所以 top.v 同步过去之后这两条是天生对齐的一对。
    //   来源是影子数组 slot_nxt（slot_nxt[j] = slot_mask[j+1]），见下面那段。
    //   ★ 它【不碰 slot_mask 的读写】—— 发声路径一个字都没改。
    output reg  [35:0] midi_keys_next,
    // ★★ V48.5：软件发来的"手动重置 LCD"请求（0xD1）—— 翻转沿
    //   ★★ 为什么用翻转沿而不是电平/脉冲：lcd_master 和本模块**同在
    //      clk_50m_ext 域**，本来一个电平就够；但这是"用户按一下必须做到一次"
    //      的请求，用翻转沿 + 接收侧边沿检测最稳：
    //        · 电平方案：万一接收侧正忙、漏采了那一拍，这一下就白按了
    //          （用户会以为按钮坏了）；
    //        · 翻转沿方案：沿一直摆在那儿，接收侧总能看见一次跳变，
    //          而且天然去重（一次请求只产生一个沿）。
    output reg         lcd_rst_tgl,
    // ★ V43：跟谱演奏模式（1 = 跟谱：当前格的键全部按住才前进到下一格）
    //   给 top.v 用来驱动 LED 的 30% 预告 / 点亮 TTP13 的指示。
    output reg         wait_mode,
    // ★★ V45：节拍器 ★★
    //   · metro_bpm / metro_num / metro_den = 生效值（已夹紧）
    //   · metro_tgl = "翻转沿 + 数据握手"的信号沿，给 top.v 的 clk_50m 域
    //     （参数是准静态的，看到沿时数据早稳定了，直接锁存即可）
    //   · metro_beat_in = clk_50m 域回传的"当前第几拍"，只用于状态回报
    output reg  [7:0]  metro_bpm,
    output reg  [7:0]  metro_num,
    output reg  [7:0]  metro_den,
    output reg         metro_tgl,
    input  wire [2:0]  metro_beat_in,
    // ★★ V48：LCD 显示模式的【上位机写口】+【状态回报】★★
    //   用户要求："所有 TTP 上的可调按键全部在上位机上也可调，并且状态保持同步"。
    //   TTP 键3 = 显示画面循环（`disp_mode`），原来【完全没有接上位机】——
    //   这里补齐，做成真正的双向：
    //     · 下行：[0xD0] [DISP_MODE]（1 字节）—— 直接【设定】，不是循环切换
    //     · 上行：[0x5F] [DISP_MODE] [REC_ON]（3 字节，★ 全新独立帧，不动 0x5D）
    //             两个值【任意一个变了就发】，所以板子上按 TTP 键3 / 键10
    //             也会主动回报 —— 这才叫双向同步。
    //   ★★★ V48.2：编码是【5 档一一对应】，不再做粗粒度折叠 ★★★
    //       0 = 频谱   1 = 波形   2 = 李萨如·左×右
    //       3 = 李萨如·干声×效果后    4 = 李萨如·相位
    //     ★ V48 第一版把 2/3/4 折叠成"李萨如"一档（下行只到 0~2），
    //       用户指出"**李萨如图形有三档**"——上位机上三档也应该都能直接选，
    //       而且要能看出板子现在是哪一档。所以改成 5 档全通：
    //         · 下行 DISP_MODE 0~4（夹紧到 0~4，★ 是"夹紧"不是"取模"）
    //         · 上行照原样报 0~4，**板子在 2->3->4 之间切换时也要发 0x5F**
    //     ★ 帧长/帧头/REC_ON/发送时机 一个字节都没变，只是值域从 0~2 扩到 0~4。
    //   ★ disp_tgl / disp_val 是和 metro_tgl 一模一样的"翻转沿 + 数据握手"：
    //     数据位在翻转沿【同一拍】变，接收侧（top.v 的 clk_50m 域）
    //     过三级同步器后看到沿时，数据早就稳定了，直接锁存即可。
    output reg         disp_tgl,        // 每收到一条 0xD0 翻转一次
    output reg  [2:0]  disp_val,        // 要设成的显示模式（已夹到 0~4）
    //   ★ 板子【当前生效】的显示模式，top.v 从 clk_50m 域送回来（已同步到本域）。
    //     它由 TTP 键3 和 0xD0 两条路共同驱动，本模块【只看不写】——
    //     所以这里不存在"两个 always 块驱动同一个 reg"的问题。
    input  wire [2:0]  disp_mode_in,
    // ★★ V47：当前事件的力度（0~127，127 = 满幅 = 不带 0xC3 时的默认）★★
    //   ★ 它是【逐事件】的，不是逐音：同一个和弦里的音共用一个力度。
    output reg  [6:0]  midi_vel,
    output reg  [35:0] bt_keys         // 兼容端口，恒 0（保留是为了不动 top.v 的例化）
);

    //==================================================================
    // 参数
    //==================================================================
    localparam BAUD_DIV  = 16'd5208;      // 50MHz / 5208 = 9600
    // ★ V38：等操作数的超时阈值 = 3 个字节时间。
    //   0xC9/0xCA 插队发送时命令字和操作数是连续的（间隔 1 个字节 = 5208 拍），
    //   正常情况远早于这个期限；只有"操作数丢了"才会走到超时。
    // ★★ V39.3：等操作数的超时阈值 = 500ms ★★
    //   V39.2 一开始写的是"6 个字节时间"（6.24ms），**那是错的、而且出了实机事故**：
    //   上位机是【分批】把帧丢进发送队列的，批次之间的间隔并不等于 1 个字节时间 ——
    //   安卓版有背压（队列超过 64 批就 sleep 20ms），所以批次间隔可以到 20ms，
    //   ★ 20ms > 6.24ms → 每次跨批都可能误触发超时：
    //     如果 0xC2（时值）正好在一批的末尾、它的操作数在下一批开头，
    //     超时先把 dur_wait 清掉，随后到达的那个操作数就被当成【音符】——
    //     凭空多一个音，累积起来整个谱子全乱（用户实测："一直播放着不知道什么旋律"）。
    //   ★ 教训：这个超时是用来治「操作数真的丢了」的，它的阈值必须比
    //     【任何正常的字节间隔】都大得多。正常间隔上限是背压的 20ms，
    //     500ms 有 25 倍余量，不可能误伤；真的丢了也只是晚 0.5 秒恢复。
    //   ★ 位宽因此从 16 位扩到 25 位（16 位最大只有 1.3ms，根本不够）。
    localparam [24:0] OP_WAIT_MAX = 25'd25_000_000;   // 500ms @50MHz
    // ★ V39.2：开窗超时 = 500ms @50MHz。
    //   帧字节流是连续的（9600 波特 = 1.04ms 一字节），整整 500ms 没来字节
    //   只可能是帧断了 —— 自动关窗，别让 armed 永久停在 1。
    localparam [24:0] ARMED_TIMEOUT = 25'd25_000_000;
    localparam SLOT_MAX  = 1024;          // 最多 1024 个事件（一个事件 = 一个时间格组）
                                          //   252 事件的小步舞曲只用到 1/4，留足余量
    localparam [10:0] SLOT_MAXN = 11'd1024;

    // ---- 录音回传（全部落在"未分配字节区"，老固件收到只会静默丢弃）----
    localparam [7:0]  REC_EV_HEAD  = 8'h5A;   // FPGA -> 上位机：录音事件帧头
    localparam [7:0]  REC_ST_HEAD  = 8'h5B;   // FPGA -> 上位机：录音开关状态
    localparam [7:0]  REC_CMD_BYTE = 8'h5C;   // 上位机 -> FPGA：翻转录音开关
    localparam [7:0]  REC_FX_HEAD  = 8'h5D;   // FPGA -> 上位机：效果开关状态

    //=========================================================================
    // ★★ V43：跟谱演奏模式 ★★
    //
    //   模式含义：
    //     0 = 正常播放（谱子按时间自己往下走）—— 默认，和以前【完全一样】
    //     1 = 跟谱演奏：当前格要求的键【全部按住】才前进到下一格
    //
    //   ★ 模式是一个【电平状态】，不是"门控"：
    //     上位机按钮、TTP 按键13 都能随时翻转它，
    //     所以不存在"卡住之后用户没法自救"的死状态（V38/V39 最贵的教训）。
    //=========================================================================
    localparam [7:0]  MODE_SET_BYTE = 8'hCE;  // 上位机 -> FPGA：+1 字节（0/非0）
    // ★★ V45：节拍器（板载 READY 灯 U12）★★
    //   上位机 -> FPGA：0xCF + 【3 字节】BPM / 分子 / 分母编码
    //     BPM   ：30~240，【0 表示关掉节拍器】；1~29 夹到 30，>240 夹到 240
    //     分子  ：1~8（超出夹紧）
    //     分母编码：0 -> 2 分音符，1 -> 4 分音符，2 -> 8 分音符（超出夹到 0~2）
    //   ★ 三条一起发（原子），不会出现"BPM 换了但拍号还没换"的中间态。
    localparam [7:0]  METRO_SET_BYTE  = 8'hCF;
    //   FPGA -> 上位机：节拍器状态回报（★ 独立新帧，【不动 0x5D】）
    //     [0x5E] [BPM] [分子] [分母编码] [当前第几拍(0 起算)]   —— 共 5 字节
    //   ★ 只在【设置变了】时发（外加复位后 / 0xCB 请求时各强制一次），
    //     不做周期上报 —— 免得跟录音的 0x5A 事件流抢带宽。
    localparam [7:0]  METRO_STAT_BYTE = 8'h5E;
    // ★★ V48：LCD 显示模式（0xD0 下行设定 / 0x5F 上行状态回报）★★
    //   这是【补齐 TTP 键3】的那一条：原来显示模式只在板子上按按钮循环，
    //   上位机既设不了、也不知道板子现在是哪一档。
    //   ★★★ V48.2：编码 = 【5 档一一对应】（和板子内部的 5 档完全相同）：
    //       0 = 频谱              1 = 波形
    //       2 = 李萨如·左×右       3 = 李萨如·干声×效果后
    //       4 = 李萨如·相位
    //     ★ V48 第一版做的是"粗粒度折叠"（2/3/4 一律上报 2、下行只到 0~2），
    //       用户指出"**李萨如图形有三档**" —— 上位机上三档也要能直接选、
    //       也要能看出板子现在在哪一档。所以改成 5 档全通，
    //       **板子在 2->3->4 之间切换时也会发 0x5F**（以前因为编码相同所以不发）。
    //   ★ 上行帧只发 3 个字节、且只在值变化时发 —— 不做周期上报。
    localparam [7:0]  DISP_SET_BYTE  = 8'hD0;   // 上位机 -> FPGA：+1 字节显示模式
    localparam [7:0]  DISP_STAT_BYTE = 8'h5F;   // FPGA -> 上位机：显示模式 + 录音开关
    localparam [2:0]  DISP_MAX_MODE  = 3'd4;    // ★ 最大档号（5 档：0~4）
    // ★★★ V48.5：0xD1 = 软件【手动重置 LCD】（不带操作数）★★★
    //   用户报："**还是容易在数据量多时卡死**，虽然在软件上各个按键状态同步
    //   没问题，但是就是** LCD 卡住不动**" —— 而且实测：
    //     · 卡住时连换显示模式画面都不变（= 状态机/屏 都不响应）；
    //     · **看门狗毫无动作**（屏幕不闪）→ 说明 FPGA 的状态机是【活的】，
    //       是【屏自己失步了】（被截断的命令 / 干扰 / 掉电抖动）。
    //   ★★ 看门狗只看状态机，它看不到屏 —— 而"屏失步"和"静态画面"在
    //      没有反馈的情况下**长得一模一样**，所以自动恢复这条路走不通
    //      （V48.4 刚把"合法静态"从误触发里摘出来，不能再按时间瞎重置）。
    //      于是给出这条**人来按**的逃生通道：重跑一遍初始化表 + 全屏清 +
    //      作废谱线缓存。
    //   ★ 它【不受开窗限制、可插队】（和 0xC9/0xCA/0xCB/0xD0 同类）——
    //     恰恰是发谱子发到一半卡住时最需要按它。
    localparam [7:0]  LCD_RST_BYTE   = 8'hD1;
    // ★★ V47：MIDI 逐音力度（0xC3 + 1 字节 0~127）★★
    //   语义和 0xCC（八度）/ 0xC2（时值）一样：【只作用于紧接着的下一个事件】。
    //   ★★ 没有 0xC3 时 pend_vel 恒为 127（满幅）—— 所以不带力度的老帧
    //      行为【逐位和以前相同】，已有功能一点不受影响。
    localparam [7:0]  VEL_BYTE = 8'hC3;
    localparam [6:0]  VEL_FULL = 7'd127;      // 满幅（= 不带 0xC3 时的默认）
    //   状态回报：跟谱模式作为【0x5D 效果帧的第 4 个字节】回传
    //   （见下面 "效果状态帧" 那段）—— ★ 没有另开帧头，避免动帧启动仲裁。

    // ---- 演奏控制（上位机 -> FPGA，不受开窗限制，随时可用）----
    localparam [7:0]  KEYMAP_SET   = 8'hC6;   // 写一条按键->音符映射：+键号 +音符
    localparam [7:0]  STOP_BYTE    = 8'hC7;   // 停止演奏（★ 保留乐谱，可重播）
    localparam [7:0]  REPLAY_BYTE  = 8'hC8;   // 重播板内已存的那一帧
    // ---- ★ V31：效果开关 / 音色（上位机也能切，板上的 TTP 按钮照旧）----
    localparam [7:0]  FX_TGL_BYTE  = 8'hC9;   // 翻转一个效果开关：+位号 0..7
                                              //   位号和 fx_state 的位一一对应；
                                              //   位 5 = 音色（三档，等价于按按钮 7）
    localparam [7:0]  TONE_SET_BYTE= 8'hCA;   // 直接选音色：+0 普通 / 1 笛子 / 2 吉他 / 3 小号
    localparam [7:0]  FX_REQ_BYTE  = 8'hCB;   // 请求立刻回报一次状态（不带操作数）
    // ---- ★ V32：事件八度偏移（用来拓宽蓝牙播放的音域）----
    //   板子只有 36 个声部（A2..G#5），超出范围的音原来一律【折八度】进来。
    //   现在上位机可以给某一格事件加一个八度偏移：这一格里的音整体升/降 n 个八度，
    //   于是"高两个八度的那句旋律"能按原八度弹出来，而不是被折回来。
    //     0xCC + 1 字节：有符号 3 位（0..3 = +0..+3；4..7 = -4..-1），只作用于【下一个事件】
    localparam [7:0]  OCT_BYTE     = 8'hCC;
    // ---- ★ V37：和弦内【逐音八度】前缀 ----
    //   0xCC 只能给【一整格】一个八度，所以"同一格里跨 4 个八度"的和弦
    //   （比如左手 C2 + 右手 C6）里总有一部分音必须折八度进来 —— 旋律就断了。
    //   这一版在和弦【内部】再加一个前缀：
    //       0xC0 [0xCD <oct>] <音号> [0xCD <oct>] <音号> ... 0xC1
    //   0xCD 只作用于【紧跟在它后面的那一个音号】；没有 0xCD 的音号
    //   继续用整格八度（= 现在的行为，一个字节都不多发）。
    //   ★ oct 是有符号 3 位，语义与 0xCC 完全一致（0..3 = +0..+3，4..7 = -4..-1）。
    //   ★ 0xCD 不会和音号撞车：音号最大 0x23，0xCD 在命令区且未分配。
    localparam [7:0]  KEY_OCT_BYTE = 8'hCD;
    localparam [19:0] REC_TICK_DIV = 20'd500_000;   // 采样间隔 10ms @ 50MHz

    localparam [7:0] REST_BYTE  = 8'hEE;  // 休止符
    localparam [7:0] CLR_BYTE   = 8'hAA;  // 开窗 + 写指针归零 + 停播
    localparam [7:0] END_BYTE   = 8'hFF;  // 帧尾
    localparam [7:0] CHORD_ON   = 8'hC0;  // 和弦帧头
    localparam [7:0] CHORD_OFF  = 8'hC1;  // 和弦帧尾
    localparam [7:0] DUR_EXT    = 8'hC2;  // 时值延长：+1 字节 N
    localparam [7:0] SET_TICK   = 8'hC4;  // 时间格：+3 字节 µs
    localparam [7:0] SET_REL    = 8'hC5;  // 释音提前量：+3 字节 µs
    localparam [7:0] NOTE_MAX   = 8'h23;  // 最大键号 35

    // 默认设置：与 v1 完全一致（0.5s 一格、每事件末尾提前 0.3s 松键）
    localparam [23:0] TICK_DEF_US = 24'd500_000;
    localparam [23:0] REL_DEF_US  = 24'd300_000;

    // 50MHz：1µs = 50 个周期
    localparam [29:0] TICK_MIN_CYC = 30'd1_000_000;   // 时间格下限 20ms
    localparam [29:0] GAP_MIN_CYC  = 30'd500_000;     // 每事件末尾至少 10ms 静音
                                                      // （top.v 包络要求 gate 低满 8ms）

    //==================================================================
    // 1) 时间参数寄存器（0xC4 / 0xC5 可改；0xAA 时复位成默认）
    //==================================================================
    reg [23:0] tick_us_r, rel_us_r;

    // ×50 用移位加法，不用乘法器
    wire [29:0] tick_x50 = ({6'b0, tick_us_r} << 5)
                         + ({6'b0, tick_us_r} << 4)
                         + ({6'b0, tick_us_r} << 1);
    wire [29:0] rel_x50  = ({6'b0, rel_us_r}  << 5)
                         + ({6'b0, rel_us_r}  << 4)
                         + ({6'b0, rel_us_r}  << 1);

    wire [29:0] tick_cyc_n = (tick_x50 < TICK_MIN_CYC) ? TICK_MIN_CYC : tick_x50;

    reg  [29:0] tick_cyc, rel_cyc;
    wire [29:0] rel_max    = tick_cyc - GAP_MIN_CYC;     // 保证 gate_last >= 10ms
    wire [29:0] rel_cyc_n  = (rel_x50 > rel_max) ? rel_max : rel_x50;
    wire [29:0] gate_last  = tick_cyc - rel_cyc;         // 最后一个格里的发声时长

    // ★ 注意：tick_us_r / rel_us_r 由下面的【接收块】独占驱动（它负责复位默认值、
    //   响应 0xC4/0xC5、以及 0xAA 时复位）。这里只把 µs 换算成周期数寄存下来，
    //   绝不能再给 tick_us_r 赋值 —— 否则会报 EX2000 重复驱动。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cyc  <= 30'd25_000_000;                 // 500ms
            rel_cyc   <= 30'd15_000_000;                 // 300ms
        end else begin
            tick_cyc <= tick_cyc_n;
            rel_cyc  <= rel_cyc_n;
        end
    end

    //==================================================================
    // 2) UART 接收：起始位中点采样，8 数据位，无校验，1 停止位
    //==================================================================
    reg [1:0]  rx_sync;
    reg        rx_d;
    wire       rx_negedge = (rx_d == 1'b1) && (rx_sync[1] == 1'b0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync <= 2'b11;
            rx_d    <= 1'b1;
        end else begin
            rx_sync <= {rx_sync[0], uart_rxd};
            rx_d    <= rx_sync[1];
        end
    end

    localparam RX_IDLE = 2'd0, RX_START = 2'd1, RX_DATA = 2'd2, RX_STOP = 2'd3;

    reg [1:0]  rx_state;
    reg [15:0] rx_baud_cnt;
    reg [2:0]  rx_bit_idx;
    reg [7:0]  rx_data_tmp;
    reg [7:0]  rx_data;
    reg        rx_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state    <= RX_IDLE;
            rx_baud_cnt <= 16'd0;
            rx_bit_idx  <= 3'd0;
            rx_data_tmp <= 8'd0;
            rx_data     <= 8'd0;
            rx_valid    <= 1'b0;
        end else begin
            rx_valid <= 1'b0;
            case (rx_state)
                RX_IDLE: begin
                    rx_baud_cnt <= 16'd0;
                    rx_bit_idx  <= 3'd0;
                    if (rx_negedge) rx_state <= RX_START;
                end
                RX_START: begin
                    if (rx_baud_cnt >= (BAUD_DIV / 2) - 1) begin
                        rx_baud_cnt <= 16'd0;
                        if (rx_sync[1] == 1'b0) rx_state <= RX_DATA;
                        else                    rx_state <= RX_IDLE;
                    end else rx_baud_cnt <= rx_baud_cnt + 1'b1;
                end
                RX_DATA: begin
                    if (rx_baud_cnt >= BAUD_DIV - 1) begin
                        rx_baud_cnt <= 16'd0;
                        rx_data_tmp[rx_bit_idx] <= rx_sync[1];
                        if (rx_bit_idx >= 3'd7) rx_state <= RX_STOP;
                        else                    rx_bit_idx <= rx_bit_idx + 1'b1;
                    end else rx_baud_cnt <= rx_baud_cnt + 1'b1;
                end
                RX_STOP: begin
                    if (rx_baud_cnt >= BAUD_DIV - 1) begin
                        rx_baud_cnt <= 16'd0;
                        rx_data     <= rx_data_tmp;   // 停止位不校验，容错优先
                        rx_valid    <= 1'b1;
                        rx_state    <= RX_IDLE;
                    end else rx_baud_cnt <= rx_baud_cnt + 1'b1;
                end
                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    //==================================================================
    // 3) 乐谱存储：每个事件一条记录
    //      slot_mask[i] = 这个事件点亮哪些键（和弦 = 多个位）
    //      slot_dur[i]  = 这个事件占几个时间格（>=1）
    //
    //  ★★ 关键：数组【只能有一个写地址】★★
    //     一开始写成"先写 slot_dur[i]=1，遇到 0xC2 再回改 slot_dur[i-1]"，
    //     结果同一个数组出现两个写地址，综合器无法推断 RAM，直接退化成
    //     2048 个触发器（实测 Register 从 5474 涨到 7712）。
    //     现在改成：事件先缓存在 pend_* 寄存器里，时值（0xC2）确定后才
    //     一次性落盘，于是所有写都落在 slot_xxx[wr_slots] 这【一个】地址上。
    //==================================================================
    //★★ V33.1：数据先稳定、沿后翻（见上面 fx_set/tone_set 的说明）★★
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fx_tgl      <= 1'b0;
            tone_we_tgl <= 1'b0;
            fx_dly      <= 2'd0;
            tone_dly    <= 2'd0;
        end else begin
            fx_dly   <= {fx_dly[0],   fx_set};
            tone_dly <= {tone_dly[0], tone_set};
            if (fx_dly[1])   fx_tgl      <= ~fx_tgl;
            if (tone_dly[1]) tone_we_tgl <= ~tone_we_tgl;
        end
    end

    reg [35:0] slot_mask [0:SLOT_MAX-1];

    //==================================================================
    // ★★ V42：影子数组 slot_nxt —— 专门给 LED 的"下一格"预告用 ★★
    //
    //   slot_nxt[j] = slot_mask[j+1]      （j = 0..1022）
    //
    //   ★ 为什么另开一块数组，而不是把 slot_mask 的读地址改成 rd_next：
    //     那样要做一次地址选择，而【选择器正好串在 BSRAM 地址前面】——
    //     本文件第 1027 行那段注释就是为这件事写的（"实测会把本来只有
    //     零点几纳秒余量的 clk_50m_ext 压垮"）。而且改读地址等于动
    //     midi_keys 的数据源 = 动发声路径，是本工程最容易出回归的地方。
    //   ★ 影子数组完全不碰 slot_mask：读用同样简单的 rd_slot，
    //     写就在【唯一那一处写 slot_mask 的地方】顺手多写一格（见 (0) 段）。
    //     所以发声路径逐位不变。
    //   ★ 代价：1024×36 = 36864 位 = 2 块 BSRAM。
    //   ★ 最后一格的"下一格"是脏数据，靠读的时候用 rd_next < frame_slots
    //     判掉（返回 0），不会漏出去。
    //==================================================================
    // ★ V43.5：原来这里有一块影子数组 reg [35:0] slot_nxt [0:SLOT_MAX-1]。
    //   五次写法都进不了 BSRAM（代价 +4600 LUT），已整块删除 ——
    //   "下一格"改由读早一格（rd_next）拿到，见播放器那段。
    //==================================================================
    // ★★ V43.2：影子数组的写口【全部用寄存器】—— 这是它能不能进 BSRAM 的关键 ★★
    //
    //   ★ 实测教训（V43/V43.1 两次都没进去）：
    //     写成 `slot_nxt[wr_nxt] <= pend_mask;`（写地址是组合线）时，
    //     综合器【不肯推断 BSRAM】，整个 1024×36 退化成分分布式 LUT RAM
    //     （报告里 --SSRAM(RAM16) 808），BSRAM 仍是 88，
    //     代价是 +5400 个 LUT（65% → 74%），把 CLKOUT1 的余量吃光。
    //     而 slot_mask 的写地址是【寄存器 wr_slots】，就正常进了 BSRAM。
    //   ★ 所以这里把 使能/地址/数据 三样都先打进寄存器，形状与 slot_mask 一致。
    //   ★ 延后一拍【完全安全】：一个事件至少要等一个 UART 字节
    //     （9600 波特 @50MHz = 5208 拍），flush_ok 绝不可能连着两拍来，
    //     所以不会出现"后一笔覆盖前一笔"。
    //==================================================================
    // ★★ V43.4（G 方案）：影子数组的写地址改成【寄存器 wr_m1】★★
    //
    //   前四次失败回顾：
    //     V43   `if (wr_slots != 0) slot_nxt[wr_slots - 1] <= ...`  地址是组合线
    //     V43.1 去掉条件门、地址仍是组合线 wr_nxt
    //     V43.2 使能/地址/数据全打进寄存器，但写在【独立的 always 块】里
    //     V43.3 读口改成寄存器直读（还让 Tech-Mapping 卡住了）
    //   ★ 而 slot_oct0/1/2 那三块【都进了 BSRAM】的写法是：
    //         slot_oct0[wr_slots] <= oct_blob[35:0];   ← 地址是寄存器、在同一个块里
    //   所以这里补齐唯一没试过的组合：地址用【寄存器 wr_m1】、
    //   写在【同一个 always 块】里，形状与 slot_oct0 完全一致。
    //   ★ wr_m1 每拍无条件更新；flush 那一拍 wr_slots 还没变，
    //     所以 wr_m1 正好等于 wr_slots - 1（写第 i 格时补第 i-1 格）。
    reg [10:0] wr_m1;
    // ★ V32：这一格除了"点亮哪些键"，还带 3 位【八度偏移】
    //   {oct[2:0], dur[7:0]} = 11 位。1024×11 = 11kbit，还是占【同一块】BSRAM
    //   （原来 1024×8 也只用了半块），所以一个块都没多花。
    //   ★ 读地址仍然只有 rd_next 这一个（见下面播放器），不能为它再开一个读口，
    //     否则综合器不肯推断 RAM，会退化成触发器。
    reg [10:0] slot_dur  [0:SLOT_MAX-1];
    // ★★ V47：逐事件力度（7 位，0~127）★★
    //   ★ 和 slot_mask / slot_dur 一样的【单读单写】纪律：
    //       写地址只有 wr_slots（就在(0)那唯一一处落盘里），
    //       读地址只有 rd_slot（和 slot_mask 同地址）。
    //     ★ 破例就会掉出 BSRAM（见坑 27 那一串教训）。
    reg [6:0]  slot_vel [0:SLOT_MAX-1];
    // ★ V37：逐键八度（36 键 × 3 位 = 108 位），每格一份。
    //   1024×108 = 110592 位 → 6 块 BSRAM（V36 的 BSRAM 是 82/118）。
    //   ★★ 与 slot_mask / slot_dur 一样，【单读单写】这条硬约束不能破：
    //      读地址只有 rd_slot 一个（和 slot_mask 同地址，见播放器那段），
    //      写地址只有 wr_slots 一个（和它们同拍写在 (0) 段里）。
    //      多开一个读口综合器就不肯推断 BSRAM，会退回十万个触发器。
    //   ★ 老谱子（不带 0xCD）落盘时每个键都被填成【整格八度 pend_oct】，
    //     于是 midi_octs ≡ oct_cur 的逐键展开 —— top.v 那边取到的值与改之前
    //     逐位相同（见 top.v 的 gen_voct 段）。
    //   ★★ 为什么拆成 3 个 36 位数组，而不是一个 108 位的数组 ★★
    //     芯片的 BSRAM 标准位宽是 36（512×36）。写成 reg [107:0] slot_oct[...]
    //     时综合器在 Tech-Mapping 阶段要做超宽的位切分 + 等价化简，
    //     ★ 实测直接卡在 [75%] Tech-Mapping Phase 2 十几分钟不动。
    //     拆成 3 个 36 位数组后每个都是标准配置（1024 深 = 2 块 → 共 6 块），
    //     代价和容量完全一样，但映射一下就过了。
    reg [35:0] slot_oct0 [0:SLOT_MAX-1];
    reg [35:0] slot_oct1 [0:SLOT_MAX-1];
    reg [35:0] slot_oct2 [0:SLOT_MAX-1];

    reg [10:0] wr_slots;       // 已落盘的事件数（0..1024）
    reg        armed;           // 接收窗口：0xAA 开、0xFF 关
    reg        in_chord;        // 正在 0xC0..0xC1 之间
    reg [35:0] acc_mask;        // 和弦累积掩码
    reg        clr_pulse;       // 收到 0xAA（通知播放器停播）
    reg        stop_pulse;      // 收到 0xC7：停播但【保留乐谱】
    reg        replay_pulse;    // 收到 0xC8：重播板内已存的那一帧

    // 按键->音符 映射的写入（0xC6 + 2 字节）
    //   ★ 写口在 clk_50m_ext，key_map 本体在 top.v 的 clk_audio 域，
    //     所以这里只出"翻转信号 + 两个稳定字节"，由 top.v 那边同步 + 边沿检测。
    reg [1:0]  map_left;        // 0=空闲；2=等键号；1=等音符
    //   （keymap_key / keymap_note / keymap_we_tgl 本身就是 output reg，直接当寄存器用）

    // ★★ V33.1：翻转沿必须【晚于数据】两拍 ★★
    //   原来是"写数据"和"翻沿"同一个时钟沿发出的。接收侧(clk_50m)把
    //   随变信号各打两拍同步，两条链的布线延迟不一样 —— 采到哪一组全看运气。
    //   表现就是：上一次按的是「滑音」（位号 0），下一次按「谐波」时，
    //   接收侧还采到上一次的 0，于是翻的是滑音。
    //   现在数据在 T 拍写、沿在 T+2 拍翻，接收侧一定先看到新数据、再看到沿。
    //   （和 V29 的按键映射写口同一个纪律：key/note 先稳定，we_tgl 后翻。）
    reg        fx_set, tone_set;      // 操作数到位（单拍）
    reg [1:0]  fx_dly, tone_dly;      // 延两拍再翻沿

    // ★ V31：效果开关 / 音色的命令接收
    //   fx_left   : 1 = 下一个字节是"翻转哪个效果位"的操作数（原始数据！）
    //   tone_left : 1 = 下一个字节是"目标音色"的操作数（原始数据！）
    //   ★ 两个操作数同样要计数器挡着：它们可以是 0xAA/0xFF/0xC0 任意值，
    //     没有计数器就会被当成命令字解释（和 0xC2/0xC6 同一个道理）。
    //   fx_req    : 收到 0xCB 后置位，等效果帧真正发出去了才清 —— 用【电平】
    //               而不是脉冲，免得正好赶上 UART 忙时把请求丢了。
    //   ★ 为什么 fx_req 和 fx_req_set 是两个寄存器：
    //     置位发生在"收字节"那个 always 块里，清位发生在"发效果帧"那个
    //     always 块里 —— 同一个 reg 被两个 always 块驱动，综合器直接报
    //     EX2000（重复驱动）。所以收字节那边只出一个单拍脉冲 fx_req_set，
    //     由发帧那边自己置位/清位，各管各的。
    reg        fx_left;
    reg        tone_left;
    // ★ V43：下一个字节是 0xCE 的操作数（跟谱模式）
    //   ★ 必须像 fx_left/tone_left 一样用计数器挡在前面 ——
    //     操作数可以是 0xAA / 0xFF / 0xC0 任意值，不挡住就会被当成音符或命令。
    reg        mode_left;
    // ★★ V45：节拍器设置（0xCF + 3 字节）★★
    reg [1:0]  metro_left;      // 0=空闲；3/2/1 = 还要收几个参数字节
    reg [7:0]  metro_bpm_r;     // 收集中的原始值（还没夹紧）
    reg [7:0]  metro_num_r;
    reg [7:0]  metro_den_r;
    // ★ metro_bpm / metro_num / metro_den / metro_tgl 是【输出端口】，
    //   在上面的端口列表里已经声明成 output reg，这里【不能重复声明】。
    // ★ V43：跟谱演奏模式电平（0 = 正常，1 = 跟谱）；见上面 MODE_SET_BYTE 那段
    reg        wait_mode;
    reg        fx_req_set;       // 收到 0xCB 的单拍脉冲（本块内部用）
    // ★ V43：模式变了的单拍脉冲（0xCE 收到 / TTP 按键13 按下都算）
    //   ★ 和 fx_req_set 同一个道理：wait_mode 只在【收字节那个块】里写，
    //     发状态帧那个块只读它 + 收这个脉冲 —— 免得两个 always 块驱动同一个 reg。
    reg        mode_set;
    // ★ V38：等操作数的超时计数（★ V39.3：位宽 16→25，阈值改成 500ms）
    reg [24:0] op_wait_cnt;
    // ★ V39.2：开窗超时计数（500ms 没收到字节就自动关窗）
    reg [24:0] armed_cnt;
    // ★ V32：八度偏移的接收
    //   oct_left : 1 = 下一个字节是 0xCC 的操作数（原始数据，同样要计数器挡着）
    //   oct_hold : 最近一条 0xCC 给的偏移；挂到下一个事件上，之后就清 0
    //              （所以"只作用于下一个事件"，不会粘住后面的事件）
    //   pend_oct : 跟着待落盘事件一起走，落盘时写进 slot 里
    reg        oct_left;
    reg [2:0]  oct_hold;
    reg [2:0]  pend_oct;

    // ---- ★ V37：和弦内【逐音八度】（0xCD 前缀）----
    //   key_oct_left : 1 = 下一个字节是 0xCD 的操作数（原始数据，要计数器挡着）
    //   key_oct_seen : 刚收到过一个 0xCD，它的八度留给【下一个音号】用；
    //                  音号一到就清掉，所以只作用一个音、不会粘住后面的音
    //   key_oct_pend : 那个待用的八度
    //   acc_oct_set  : 哪些键被 0xCD 显式指定过。落盘时按它二选一：
    //                    显式指定过的 → acc_oct_b* 里那个值
    //                    没指定的     → 整格八度 pend_oct（= 现在的行为）
    //                  ★ 老谱子一个 0xCD 都不发 → set 全 0 → 全部取 pend_oct，
    //                    于是 midi_octs 恒等于 oct_cur 的逐键展开，与 V36 逐位相同。
    //   ★★ 八度拆成 3 个【36 位向量】存（b0/b1/b2 各是一位），千万别写成
    //      一个 108 位向量再按下标切片：acc_oct[idx*3 +: 3] <= ... 这种
    //      "向量的动态位选赋值"会让综合器在 Tech-Mapping 阶段卡死（实测十几分钟）。
    //      拆成位选之后，写就是 acc_oct_b0[idx] <= ...，和 acc_mask[idx] <= 1'b1
    //      完全同构 —— 一个 6→36 译码器，综合器一眼就懂。
    reg        key_oct_left;
    reg        key_oct_seen;
    reg [2:0]  key_oct_pend;
    reg [35:0] acc_oct_b0;
    reg [35:0] acc_oct_b1;
    reg [35:0] acc_oct_b2;
    reg [35:0]  acc_oct_set;
    //   tone_sel 在本域里两级同步后再用（top.v 那侧是 clk_50m 域）
    reg [1:0]  tone_s1, tone_s2;
    wire [1:0] tone_sel_sync = tone_s2;

    // 待落盘事件（时值要等后面的 0xC2 才知道，所以先缓存）
    reg [35:0] pend_mask;
    reg [7:0]  pend_dur;
    // ★★ V47：力度用【两个寄存器】，和 oct_hold/pend_oct 完全同一个套路 ★★
    //   vel_hold  = 刚收到的 0xC3 值，等着挂到下一个事件上
    //   pend_vel  = 已经挂到"待落盘事件"上的值
    //   ★ 为什么不能只有一个：0xC3 的操作数【先】到，事件字节【后】到。
    //     如果只有一个寄存器，事件字节那一拍 set_pend 会把它清成 127，
    //     刚设的力度当场丢掉。必须像八度那样"先存 hold、落盘时搬过去"。
    reg [6:0]  vel_hold;
    reg [6:0]  pend_vel;
    reg        vel_wait;         // 0xC3 的操作数还没到
    // ★★ V48：0xD0 的操作数（显示模式）还没到 ★★
    //   ★ 和 vel_wait / dur_wait 一样，是【计数器挡在命令解析前面】的那个标志：
    //     操作数可以是 0xAA / 0xFF / 0xC0 等任意值，不挡住的话会被当成音符/命令。
    reg        disp_wait;
    reg        pend_valid;

    // 设置参数接收
    reg [1:0]  cfg_left;        // 0=空闲；3/2/1 = 还要收几个参数字节
    reg        cfg_which;       // 0=SET_TICK, 1=SET_REL
    reg [15:0] cfg_acc;         // 已收的低两字节
    reg        dur_wait;        // 1 = 下一个字节是 0xC2 的操作数 N

    // 时值：总格数 = 1 + N（饱和到 255）
    wire [8:0] dur_patch   = {1'b0, rx_data} + 9'd1;
    wire [7:0] dur_patch_c = (dur_patch > 9'd255) ? 8'd255 : dur_patch[7:0];

    // 回显请求
    reg        tx_req;
    reg [7:0]  tx_byte;

    // 帧起播
    reg        frame_start;
    reg [10:0] frame_slots;    // 本帧的事件数（在收到 0xFF 那一拍锁存）

    //------------------------------------------------------------------
    // 组合：本拍要不要落盘、要不要挂新的待落盘事件
    //------------------------------------------------------------------
    // ★★ V33.1：命令的其余操作数也【不算音符】★★
    //   踩过的坑：上位机现在会把控制命令【插队】夹在谱子中间发（见电脑版的
    //   _send_front），而这一段 in_window 是【组合逻辑】，原来只挡了
    //   cfg_left / dur_wait —— 于是 0xC9/0xCA/0xCC/0xC6 的操作数（比如 0x02）
    //   一旦落在开窗期内，就同时触发了 plain_note -> 往谱子里凭空多塞一个音。
    //   所以所有"正在等操作数"的状态都要挡掉。
    wire op_wait     = (map_left != 2'd0) || oct_left || fx_left || tone_left
                       || key_oct_left || mode_left || (metro_left != 2'd0)
                       || vel_wait || disp_wait;
    // ★ V39.2：和 op_wait 同样的集合，外加 dur_wait / cfg_left ——
    //   "所有正在等操作数的状态"，用来做统一超时。（op_wait 只管 in_window，
    //   这个管超时，两者用途不同所以分开写。）
    // ★ V43：新加了 mode_left（0xCE 的操作数）—— ★ 两个集合都要加，
    //   漏一个就会出现"操作数被当成音符"（坑 3）或"卡住不超时"（坑 22）。
    // ★ V48：新加了 disp_wait（0xD0 的操作数）—— 同上，两个集合都要加。
    wire any_op_wait = (map_left != 2'd0) || oct_left || fx_left || tone_left
                       || key_oct_left || dur_wait || (cfg_left != 2'd0)
                       || mode_left || (metro_left != 2'd0) || vel_wait
                       || disp_wait;
    wire in_window   = rx_valid && armed && (cfg_left == 2'd0) && !dur_wait && !op_wait;
    wire chord_close = in_window && (rx_data == CHORD_OFF) && in_chord;
    wire plain_note  = in_window && (rx_data <= NOTE_MAX) && !in_chord;
    wire plain_rest  = in_window && (rx_data == REST_BYTE) && !in_chord;
    wire frame_end   = in_window && (rx_data == END_BYTE);

    // ★★ 0xC0（和弦帧头）绝对不能当"落盘触发" ★★
    //   落盘是"延后一拍"的：这一拍的 flush 写下去的是【上一次】set_pend 挂上的
    //   pend_*，而本拍新挂的事件要等下一次 flush 才落盘。
    //   一个和弦本来只该产生【一次】set_pend（在 0xC1 那一拍），
    //   如果 0xC0 也算触发，就等于一个和弦吃了两次 flush：
    //       0xC0 拍：把上一个事件又写了一遍   <- 前一个音被重复一次
    //       0xC1 拍：才写上这个和弦
    //   结果每个和弦前面那个音都会被写成两格（听感：那个音拖长一倍）。
    //   去掉 chord_open 之后，"事件 -> 下一次触发时落盘"这条流水线才是对的。
    wire        flush_now  = chord_close || plain_note || plain_rest || frame_end;
    wire        set_pend   = chord_close || plain_note || plain_rest;
    wire [35:0] new_p_mask = chord_close ? acc_mask
                           : plain_note  ? (36'd1 << rx_data[5:0])
                                         : 36'd0;      // 休止 = 空掩码
    wire        flush_ok   = flush_now && pend_valid && (wr_slots < SLOT_MAXN);
    wire [10:0] slots_next = wr_slots + (flush_ok ? 11'd1 : 11'd0);

    //------------------------------------------------------------------
    // ★ V37：本拍要落盘的那一份【逐键八度】
    //   对一个 memory 做部分位选赋值（mem[addr][x +: 3] <= ...）有些综合器
    //   不肯直接推断，所以这里先组合成整字 oct_blob，落盘时【整体写】。
    //   36 路都是常数下标，综合成 108 个 2:1 选择器，很便宜。
    //------------------------------------------------------------------
    wire [107:0] oct_blob;
    genvar go;
    generate
        for (go = 0; go < 36; go = go + 1) begin : gen_octblob
            assign oct_blob[go*3 +: 3] = acc_oct_set[go]
                                       ? {acc_oct_b2[go], acc_oct_b1[go], acc_oct_b0[go]}
                                       : pend_oct;
        end
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_slots    <= 11'd0;
            armed       <= 1'b0;
            in_chord    <= 1'b0;
            acc_mask    <= 36'd0;
            clr_pulse   <= 1'b0;
            stop_pulse  <= 1'b0;
            replay_pulse<= 1'b0;
            pend_mask   <= 36'd0;
            pend_dur    <= 8'd1;
            // ★ V47：力度复位 = 满幅 + 空闲（不带 0xC3 的老帧等价于满幅）
            vel_hold    <= VEL_FULL;
            pend_vel    <= VEL_FULL;
            vel_wait    <= 1'b0;
            // ★ V48：显示模式的写口 / 等待标志
            disp_wait   <= 1'b0;
            disp_tgl    <= 1'b0;
            disp_val    <= 3'd0;
            pend_valid  <= 1'b0;
            cfg_left    <= 2'd0;
            cfg_which   <= 1'b0;
            cfg_acc     <= 16'd0;
            dur_wait    <= 1'b0;
            map_left       <= 2'd0;
            keymap_key     <= 6'd0;
            keymap_note    <= 6'd0;
            keymap_we_tgl  <= 1'b0;
            fx_left        <= 1'b0;      // ★ V31
            tone_left      <= 1'b0;
            mode_left      <= 1'b0;      // ★ V43
            // ★ V45：节拍器 —— 复位成【默认 100 BPM、4/4、开着】
            //   ★ 分母编码 1 = 4 分音符；所以默认是 4/4 而不是 4/1。
            metro_left     <= 2'd0;
            metro_bpm_r    <= 8'd0;
            metro_num_r    <= 8'd0;
            metro_den_r    <= 8'd0;
            metro_bpm      <= 8'd100;
            metro_num      <= 8'd4;
            metro_den      <= 8'd1;
            metro_tgl      <= 1'b0;
            wait_mode      <= 1'b0;      // ★ V43：复位 = 正常播放（跟以前完全一样）
            mode_set       <= 1'b0;      // ★ V43
            fx_req_set     <= 1'b0;
            lcd_rst_tgl    <= 1'b0;      // ★ V48.5：手动重置 LCD 的请求沿
            op_wait_cnt    <= 25'd0;     // ★ V39.3
            armed_cnt      <= 25'd0;     // ★ V39.2
            oct_left       <= 1'b0;      // ★ V32
            oct_hold       <= 3'd0;
            pend_oct       <= 3'd0;
            key_oct_left   <= 1'b0;      // ★ V37
            key_oct_seen   <= 1'b0;
            key_oct_pend   <= 3'd0;
            acc_oct_b0     <= 36'd0;
            acc_oct_b1     <= 36'd0;
            acc_oct_b2     <= 36'd0;
            acc_oct_set    <= 36'd0;
            // ★ oct_cur 由【播放器那个 always 块】独占驱动，这里不能再赋值
            //   （两个块驱动同一个 reg 会报 EX2000 重复驱动）
            // ★ V33.1：fx_tgl / tone_we_tgl 由下面"数据先稳定、沿后翻"那个
            //   独立 always 块驱动，这里【不能】再赋值（两个块驱动同一个 reg
            //   会报 EX2000 重复驱动）。只留数据位和计数器。
            fx_bit         <= 3'd0;
            tone_val       <= 2'd0;
            frame_start <= 1'b0;
            frame_slots <= 11'd0;
            tx_req      <= 1'b0;
            tx_byte     <= 8'd0;
            bt_keys     <= 36'd0;        // ★ 只有这个块驱动 bt_keys
            tick_us_r   <= TICK_DEF_US;  // ★ 时间参数也只由这个块驱动
            rel_us_r    <= REL_DEF_US;
            // ★ score_we/score_waddr/score_wdata/play_start/play_len
            //   由下面独立的"死端口"块统一驱动，这里不能再赋值（EX2000 重复驱动）
        end else begin
            frame_start <= 1'b0;
            clr_pulse   <= 1'b0;
            stop_pulse  <= 1'b0;
            replay_pulse<= 1'b0;
            tx_req      <= 1'b0;
            rec_toggle  <= 1'b0;
            fx_req_set  <= 1'b0;     // ★ V31：单拍脉冲，每拍自动清
            fx_set      <= 1'b0;     // ★ V33.1：单拍
            tone_set    <= 1'b0;

            //------------------------------------------------------------
            // ★★ V39.2：所有"等操作数"状态统一超时 ★★
            //   V38 只给 fx_left / tone_left 加了超时，其余的
            //   （map_left / oct_left / key_oct_left / cfg_left / dur_wait）
            //   一个都没有 —— 一旦某个操作数在传输中丢了，那个状态就
            //   【永久卡住】，之后每一个字节都被当成操作数吃掉。
            //   ★ 按键映射（0xC6）是 36 对共 73 字节：map_left 卡住的话
            //     每 2 个字节被吃掉一对，比 armed 卡住更狠。
            //     （而上位机的"按键映射下发 / 停止演奏 / 重播"都会清发送队列，
            //       正好能把映射发到一半就截断 —— 这两件事是配套的。）
            //   这里把超时扩到全部等待状态：到点统统放弃、回到干净状态。
            //
            //   宽限 = 500ms（见 OP_WAIT_MAX 上面的说明）。
            //   ★ 正常情况操作数紧跟命令字 —— 最坏的间隔也就是"上位机分批之间的
            //     停顿"（安卓背压下约 20ms），离 500ms 有 25 倍余量，不会误伤；
            //   ★ 只有"真的丢了"才会走到超时。
            //   ★ 这段必须放在 if (rx_valid) 之前：真的收到操作数时，
            //     下面命令解析里的赋值会覆盖它（同一个 always 块，后写生效）。
            //------------------------------------------------------------
            if (any_op_wait) begin
                if (op_wait_cnt >= OP_WAIT_MAX) begin
                    fx_left      <= 1'b0;      // 全部放弃，回到"什么都没在等"
                    tone_left    <= 1'b0;
                    map_left     <= 2'd0;
                    oct_left     <= 1'b0;
                    key_oct_left <= 1'b0;
                    dur_wait     <= 1'b0;
                    cfg_left     <= 2'd0;
                    mode_left    <= 1'b0;      // ★ V43：0xCE 的操作数也一起放弃
                    metro_left   <= 2'd0;      // ★ V45：0xCF 的操作数也一起放弃
                    vel_wait     <= 1'b0;      // ★ V47：0xC3 的操作数也一起放弃
                    disp_wait    <= 1'b0;      // ★ V48：0xD0 的操作数也一起放弃
                    op_wait_cnt  <= 25'd0;
                end else begin
                    op_wait_cnt <= op_wait_cnt + 25'd1;
                end
            end else begin
                op_wait_cnt <= 25'd0;
            end

            //------------------------------------------------------------
            // ★★ V39.2：开窗（armed）超时 —— 500ms 收不到字节就自动关窗 ★★
            //   armed 只在 0xAA 置 1、0xFF 清 0。帧一旦被截断
            //   （上位机"停止演奏 / 重播 / 按键映射下发"会清发送队列、
            //     串口断开、蓝牙掉线），armed 就永久停在 1 ——
            //   之后收到的任何字节都被当成"帧数据"。
            //   真机上帧字节流是连续的（9600 波特 = 1.04ms 一字节），
            //   所以"整整 500ms 一个字节都没来"只可能是帧断了。
            //   ★ 这是"卡住能自己恢复"，不是"卡住就失灵"（V38 的教训）。
            //   ★ 放在 if (rx_valid) 之前：0xAA / 0xFF 的赋值会覆盖它。
            //------------------------------------------------------------
            if (rx_valid)                                  armed_cnt <= 25'd0;
            else if (armed && (armed_cnt >= ARMED_TIMEOUT)) begin
                armed     <= 1'b0;                         // 帧断了 -> 关窗
                armed_cnt <= 25'd0;
            end
            else if (armed)                                armed_cnt <= armed_cnt + 25'd1;
            else                                           armed_cnt <= 25'd0;

            //------------------------------------------------------------
            // (0) 统一落盘：★ 全文件只有这一处写 slot_mask / slot_dur ★
            //------------------------------------------------------------
            if (flush_ok) begin
                slot_mask[wr_slots] <= pend_mask;
                // ★ V43.5：影子数组整块【已删除】—— 它五次都没能进 BSRAM，
                //   现在"下一格"靠读早一格（rd_next）拿到，不需要第二块存储。
                slot_dur [wr_slots] <= {pend_oct, pend_dur};   // ★ V32：带上八度偏移
                // ★ V37：逐键八度整体落盘。没被 0xCD 指定过的键 = pend_oct，
                //   所以不带 0xCD 的谱子写进去的就是"整格八度的逐键展开"。
                slot_oct0[wr_slots] <= oct_blob[35:0];
                slot_oct1[wr_slots] <= oct_blob[71:36];
                slot_oct2[wr_slots] <= oct_blob[107:72];
                slot_vel [wr_slots] <= pend_vel;               // ★ V47：带上力度
                wr_slots            <= slots_next;
            end

            //------------------------------------------------------------
            // ★ V43：TTP 按键13 翻转跟谱模式
            //   ★ 放在这里（不在 if (rx_valid) 里面）：按键和串口是两条独立的路。
            //   ★ 和 0xCE 共用 wait_mode 这一个寄存器，本块是它唯一的驱动者。
            //------------------------------------------------------------
            if (btn_mode_tgl) begin
                wait_mode <= ~wait_mode;
                mode_set  <= 1'b1;             // 让状态帧回报一次
            end

            //------------------------------------------------------------
            // (0b) 挂起新的待落盘事件（时值在 0xC2 到来时再改 pend_dur）
            //------------------------------------------------------------
            if (set_pend) begin
                pend_mask  <= new_p_mask;
                pend_dur   <= 8'd1;
                pend_valid <= 1'b1;
                // ★ V32：把"上一条 0xCC 给的八度偏移"挂到这个事件上，然后清掉 ——
                //   于是它只作用一个事件；下一个事件默认回原调（0）。
                pend_oct   <= oct_hold;
                oct_hold   <= 3'd0;
                // ★ V47：把"上一条 0xC3 给的力度"挂到这个事件上，然后清回满幅 ——
                //   于是它只作用一个事件；下一个事件默认满幅。
                pend_vel   <= vel_hold;
                vel_hold   <= VEL_FULL;
            end

            if (rx_valid) begin
                //--------------------------------------------------------
                // (a) 设置参数的操作数（3 字节小端）—— 原始数据，不作解释
                //--------------------------------------------------------
                if (cfg_left != 2'd0) begin
                    tx_req  <= 1'b1;
                    tx_byte <= rx_data;
                    if (cfg_left == 2'd1) begin
                        // 第 3 字节到位：整字提交
                        if (cfg_which == 1'b0) tick_us_r <= {rx_data, cfg_acc};
                        else                   rel_us_r  <= {rx_data, cfg_acc};
                        cfg_left <= 2'd0;
                    end else begin
                        if (cfg_left == 2'd3) cfg_acc[7:0]  <= rx_data;
                        else                  cfg_acc[15:8] <= rx_data;
                        cfg_left <= cfg_left - 1'b1;
                    end
                end
                //--------------------------------------------------------
                // (a2) ★ V45：0xCF 的 3 个操作数 —— BPM / 分子 / 分母编码
                //      ★ 第 3 字节（metro_left == 1）到位时【整组提交】，
                //        并且在这一拍就把三个值【夹紧】—— 送进 top.v 的
                //        永远是合法值，top.v 那边不用再防。
                //      ★ 提交的同时翻转 metro_tgl，把新值交给 clk_50m 域。
                //--------------------------------------------------------
                else if (metro_left != 2'd0) begin
                    tx_req  <= 1'b1;
                    tx_byte <= rx_data;
                    if (metro_left == 2'd3)      metro_bpm_r <= rx_data;
                    else if (metro_left == 2'd2) metro_num_r <= rx_data;
                    else                         metro_den_r <= rx_data;
                    if (metro_left == 2'd1) begin
                        // BPM：0 = 关；1~29 夹到 30；>240 夹到 240
                        metro_bpm <= (metro_bpm_r == 8'd0)      ? 8'd0   :
                                     (metro_bpm_r <  8'd30)     ? 8'd30  :
                                     (metro_bpm_r >  8'd240)    ? 8'd240 : metro_bpm_r;
                        // 分子：1~8
                        metro_num <= (metro_num_r < 8'd1) ? 8'd1 :
                                     (metro_num_r > 8'd8) ? 8'd8 : metro_num_r;
                        // 分母编码：0~2
                        metro_den <= (metro_den_r > 8'd2) ? 8'd2 : metro_den_r;
                        metro_tgl <= ~metro_tgl;
                        metro_left <= 2'd0;
                    end else begin
                        metro_left <= metro_left - 2'd1;
                    end
                end
                //--------------------------------------------------------
                // (b0) ★ V47：0xC3 的操作数 = 力度（0~127）—— 原始数据
                //      ★ 只改 vel_hold 寄存器，★ 不碰数组 ★
                //      ★ 先存进 vel_hold，等下一个事件字节到来时由 set_pend
                //        搬进 pend_vel（和 oct_hold -> pend_oct 一模一样）。
                //      ★ >127 的直接夹到满幅 —— 不能简单截低位
                //        （200 截成 72 会让"很响"变成"很小声"）。
                //--------------------------------------------------------
                else if (vel_wait) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= rx_data;
                    vel_hold <= (rx_data > 8'd127) ? VEL_FULL : rx_data[6:0];
                    vel_wait <= 1'b0;
                end
                //--------------------------------------------------------
                // (b0c) ★ V48：0xD0 的操作数 = 显示模式（★ V48.2：0~4，5 档）
                //   ★ 夹紧【不是取模】：> 4 一律当作 4，8'hFF 夹到 4 而不是绕回 0 ——
                //     和上位机 fpga_proto.clamp_disp 的规则【逐字一致】，
                //     两边不会互相打架。
                //   ★ 数据位和翻转沿【同一拍】变：接收侧（top.v 的 clk_50m 域）
                //     过三级同步器后才看到沿，那时数据已稳定 2~3 拍，
                //     直接锁存即可 —— 和 0xCF（节拍器）完全同一套握手。
                //--------------------------------------------------------
                else if (disp_wait) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= rx_data;
                    disp_val <= (rx_data > {5'd0, DISP_MAX_MODE}) ? DISP_MAX_MODE
                                                                  : rx_data[2:0];
                    disp_tgl <= ~disp_tgl;
                    disp_wait <= 1'b0;
                end
                //--------------------------------------------------------
                // (b) 0xC2 的操作数 N —— 原始数据，不作解释
                //     只改 pend_dur 寄存器，★ 不碰数组 ★
                //--------------------------------------------------------
                else if (dur_wait) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= rx_data;
                    dur_wait <= 1'b0;
                    if (!in_chord && pend_valid) pend_dur <= dur_patch_c;
                end
                //--------------------------------------------------------
                // (b3) 0xC6 写一条【按键 -> 音符】映射：后跟 2 字节
                //        第 1 字节 = 物理键号 0..35
                //        第 2 字节 = 目标音符 0..35；给 >=36 的值（比如 0x3F）
                //                   表示【禁用这个键】
                //      ★ 两个操作数是【原始数据】：哪怕它们是 0xAA/0xFF/0xC0，
                //        也一律当数据，所以要用 map_left 计数器挡一下 ——
                //        和 0xC2/0xC4 的操作数一个道理。
                //      ★ 不受开窗限制：配置随时可以做。
                //--------------------------------------------------------
                else if (map_left != 2'd0) begin
                    tx_req  <= 1'b1;
                    tx_byte <= rx_data;
                    if (map_left == 2'd2) begin
                        keymap_key <= rx_data[5:0];
                        map_left   <= 2'd1;
                    end else begin
                        keymap_note   <= rx_data[5:0];
                        keymap_we_tgl <= ~keymap_we_tgl;   // 两字节齐了，通知音频域
                        map_left      <= 2'd0;
                    end
                end
                //--------------------------------------------------------
                // (b4) 0xC7 停止演奏（★ 保留乐谱，可以重播）
                //      只停播 + 放键，wr_slots / frame_slots 一个字不动。
                //      和 0xAA 的区别：0xAA 顺手把乐谱清了，这个不清。
                //--------------------------------------------------------
                else if (rx_data == STOP_BYTE) begin
                    tx_req     <= 1'b1;
                    tx_byte    <= STOP_BYTE;
                    stop_pulse <= 1'b1;
                end
                //--------------------------------------------------------
                // (b5) 0xC8 重播板上已经存着的那一帧
                //      用的是 frame_slots（上次 0xFF 时锁存的），不用重发谱子。
                //--------------------------------------------------------
                else if (rx_data == REPLAY_BYTE) begin
                    tx_req       <= 1'b1;
                    tx_byte      <= REPLAY_BYTE;
                    replay_pulse <= 1'b1;
                end
                //--------------------------------------------------------
                // (b2) 0x5C 上位机翻转录音开关（不受开窗限制，随时可切）
                //      真正的翻转在 rec_on 那个 always 块里做，这里只发 1 拍脉冲
                //--------------------------------------------------------
                else if (rx_data == REC_CMD_BYTE) begin
                    rec_toggle <= 1'b1;
                    tx_req     <= 1'b1;                // 回显一个确认
                    tx_byte    <= REC_CMD_BYTE;
                end
                //--------------------------------------------------------
                // (b6) 0xC6 命令字本身：等两个操作数
                //--------------------------------------------------------
                else if (rx_data == KEYMAP_SET) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= KEYMAP_SET;
                    map_left <= 2'd2;
                end
                //--------------------------------------------------------
                // (b7) ★ V31：0xC9 / 0xCA 的操作数 —— 原始数据，不作解释
                //        fx_left  : 下一个字节 = 翻转哪个效果位（0..7）
                //        tone_left: 下一个字节 = 目标音色（0 普通 / 1 笛子 / 2 吉他）
                //      ★ 操作数可以是 0xAA / 0xFF / 0xC0 任意值，所以必须由
                //        计数器挡在前面（和 0xC2/0xC6 完全同理）。
                //        位号只取低 3 位、音色只取低 2 位，越界值一律当 0，
                //        这样即使上位机发错也只会退化成"最保守的那一档"。
                //--------------------------------------------------------
                else if (fx_left) begin
                    tx_req  <= 1'b1;
                    tx_byte <= rx_data;
                    fx_bit  <= rx_data[2:0];
                    fx_left <= 1'b0;
                    // ★ 只在这里写【数据】，翻沿交给下面延两拍的 fx_dly ——
                    //   数据与沿同拍变化的话，接收侧两条同步链谁先到看布线，
                    //   会采到【上一次的位号】（按谐波结果翻了滑音，踩过）。
                    fx_set  <= 1'b1;
                end
                else if (tone_left) begin
                    tx_req     <= 1'b1;
                    tx_byte    <= rx_data;
                    // ★ V48.2：音色从 3 档扩到 4 档，越界值退化成最保守的"普通"
                    tone_val   <= (rx_data > 8'd3) ? 2'd0 : rx_data[1:0];
                    tone_left  <= 1'b0;
                    tone_set   <= 1'b1;
                end
                //--------------------------------------------------------
                // (b7c) ★ V43：0xCE 的操作数 = 跟谱模式（0 = 正常，非 0 = 跟谱）
                //   ★ 取"非 0 即开"，上位机发 0x01/0xFF 都是开、0x00 是关 ——
                //     越界值退化成最保守的那一档（和 tone_val 同样的思路）。
                //--------------------------------------------------------
                else if (mode_left) begin
                    tx_req    <= 1'b1;
                    tx_byte   <= rx_data;
                    wait_mode <= (rx_data != 8'd0);
                    mode_left <= 1'b0;
                    mode_set  <= 1'b1;         // 让下面的状态帧去回报一次
                end
                //--------------------------------------------------------
                // (b8) 0xC9 命令字本身：等"翻转哪一位"
                //   ★★ V39：把 V38 加的 `&& !armed` 那道门【撤掉了】★★
                //     V38 想用它挡住"命令插进谱子帧、操作数吃掉一个谱子字节"，
                //     但那有个致命副作用：armed 只在 0xAA 置 1、0xFF 清 0，
                //     ★ 而 0xC7（停止演奏）【不清 armed】——
                //     所以只要发帧中途出过状况（发了 0xAA 没等到 0xFF），
                //     armed 就永久卡在 1，效果开关从此【再也点不动】。
                //     用户实测到的"延迟关不掉了"就是这个。
                //     权衡之后：宁可偶尔被插队打扰，也不能让开关永久失灵。
                //     防"插队打断帧"改由上位机负责（见 spp_assistant.py
                //     的 _send_front 说明）。
                //--------------------------------------------------------
                else if (rx_data == FX_TGL_BYTE) begin
                    tx_req  <= 1'b1;
                    tx_byte <= FX_TGL_BYTE;
                    fx_left <= 1'b1;
                end
                //--------------------------------------------------------
                // (b9) 0xCA 命令字本身：等"目标音色"（★ V39：同样撤掉 !armed 门）
                //--------------------------------------------------------
                else if (rx_data == TONE_SET_BYTE) begin
                    tx_req    <= 1'b1;
                    tx_byte   <= TONE_SET_BYTE;
                    tone_left <= 1'b1;
                end
                //--------------------------------------------------------
                // (b9c) ★ V43：0xCE 命令字本身 —— 等"跟谱模式"那一个操作数
                //   ★ 和 0xC9/0xCA 一样【不受开窗限制】，随时可用。
                //     （不让它受 armed 门控 —— V38 那道门造成过"永久失灵"，
                //       已经回滚，这里也不再引入任何门控。）
                //--------------------------------------------------------
                else if (rx_data == MODE_SET_BYTE) begin
                    tx_req    <= 1'b1;
                    tx_byte   <= MODE_SET_BYTE;
                    mode_left <= 1'b1;
                end
                //--------------------------------------------------------
                // (b9d) ★ V45：0xCF 命令字本身 —— 等 3 个节拍器参数字节
                //   ★ 和 0xCE 一样【不受开窗限制】，随时可用（不引入任何门控）。
                //--------------------------------------------------------
                else if (rx_data == METRO_SET_BYTE) begin
                    tx_req     <= 1'b1;
                    tx_byte    <= METRO_SET_BYTE;
                    metro_left <= 2'd3;
                end
                //--------------------------------------------------------
                // (b9e) ★ V48：0xD0 命令字本身 —— 等"显示模式"那 1 个操作数
                //   ★ 和 0xCE/0xCF 一样【不受开窗限制】，随时可用。
                //     （这里刻意【不】加任何 armed 门控 —— V38 那道门造成过
                //       "效果开关永久失灵"，是最贵的一次教训，见交接文档坑 20。）
                //--------------------------------------------------------
                else if (rx_data == DISP_SET_BYTE) begin
                    tx_req    <= 1'b1;
                    tx_byte   <= DISP_SET_BYTE;
                    disp_wait <= 1'b1;
                end
                //--------------------------------------------------------
                // (b10) 0xCB 请求回报一次状态（不带操作数）
                //       只置 fx_req 这个【电平】，真正的 3 字节状态帧由下面
                //       那一路发；发出去了才清 —— 免得赶上忙的时候把请求丢了。
                //--------------------------------------------------------
                else if (rx_data == FX_REQ_BYTE) begin
                    tx_req     <= 1'b1;
                    tx_byte    <= FX_REQ_BYTE;
                    fx_req_set <= 1'b1;
                end
                //--------------------------------------------------------
                // (b10b) ★ V48.5：0xD1 手动重置 LCD（不带操作数）
                //        只翻一个沿，真正的"重跑初始化表 + 全屏清"由 lcd_master
                //        在它自己的边沿检测里做（两边同在 clk_50m_ext 域）。
                //        ★ 操作数就是它自己（0xD1 不是帧头），所以不需要
                //          _left 计数器挡着。
                //--------------------------------------------------------
                else if (rx_data == LCD_RST_BYTE) begin
                    tx_req      <= 1'b1;
                    tx_byte     <= LCD_RST_BYTE;
                    lcd_rst_tgl <= ~lcd_rst_tgl;
                end
                //--------------------------------------------------------
                // (b11) ★ V32：0xCC 的操作数 = 事件八度偏移（原始数据）
                //        有符号 3 位，只作用于【下一个事件】。
                //--------------------------------------------------------
                else if (oct_left) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= rx_data;
                    oct_hold <= rx_data[2:0];
                    oct_left <= 1'b0;
                end
                //--------------------------------------------------------
                // (b12) 0xCC 命令字本身：等那个偏移字节
                //--------------------------------------------------------
                else if (rx_data == OCT_BYTE) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= OCT_BYTE;
                    oct_left <= 1'b1;
                end
                //--------------------------------------------------------
                // (b13) ★ V37：0xCD 的操作数 = 【下一个音号】自己的八度
                //        有符号 3 位，语义和 0xCC 完全一样，但只作用一个音号。
                //        ★ 它和 0xCC 一样是【原始数据】，所以必须由计数器
                //          key_oct_left 挡在前面 —— 否则一个 0x01 之类的
                //          八度值会被下面的音号分支当成"第 1 号键"点亮。
                //--------------------------------------------------------
                else if (key_oct_left) begin
                    tx_req       <= 1'b1;
                    tx_byte      <= rx_data;
                    key_oct_pend <= rx_data[2:0];
                    key_oct_seen <= 1'b1;      // 留给紧接着的那个音号
                    key_oct_left <= 1'b0;
                end
                //--------------------------------------------------------
                // (b14) ★ V37：0xCD 命令字本身（和弦内逐音八度前缀）
                //--------------------------------------------------------
                else if (rx_data == KEY_OCT_BYTE) begin
                    tx_req       <= 1'b1;
                    tx_byte      <= KEY_OCT_BYTE;
                    key_oct_left <= 1'b1;
                end
                //--------------------------------------------------------
                // (c) 0xAA 开窗（同时停播、复位设置、清空缓冲）
                //--------------------------------------------------------
                else if (rx_data == CLR_BYTE) begin
                    armed     <= 1'b1;
                    wr_slots  <= 11'd0;
                    in_chord  <= 1'b0;
                    acc_mask  <= 36'd0;
                    acc_oct_b0 <= 36'd0;      // ★ V37：清逐键八度累加
                    acc_oct_b1 <= 36'd0;
                    acc_oct_b2 <= 36'd0;
                    acc_oct_set <= 36'd0;
                    key_oct_seen <= 1'b0;
                    key_oct_left <= 1'b0;
                    dur_wait  <= 1'b0;
                    cfg_left  <= 2'd0;
                    pend_valid<= 1'b0;
                    clr_pulse <= 1'b1;                 // 通知播放器停
                    tick_us_r <= TICK_DEF_US;
                    rel_us_r  <= REL_DEF_US;
                    // ★ 0xAA 不回显（与 v1 一致）
                end
                //--------------------------------------------------------
                // (d) 0xFF 帧尾
                //     slots_next 已经把"还没落盘的最后一个事件"算进去了
                //--------------------------------------------------------
                else if (rx_data == END_BYTE) begin
                    tx_req <= 1'b1;
                    tx_byte <= END_BYTE;
                    if (armed) begin
                        frame_slots <= slots_next;
                        frame_start <= 1'b1;
                    end
                    armed      <= 1'b0;                // 关窗，后面的字节全丢
                    in_chord   <= 1'b0;
                    pend_valid <= 1'b0;
                end
                //--------------------------------------------------------
                // (e) 设置命令
                //--------------------------------------------------------
                else if (rx_data == SET_TICK || rx_data == SET_REL) begin
                    tx_req    <= 1'b1;
                    tx_byte   <= rx_data;
                    cfg_left  <= 2'd3;
                    cfg_which <= (rx_data == SET_REL);
                end
                //--------------------------------------------------------
                // (f) 时值延长命令
                //--------------------------------------------------------
                else if (rx_data == DUR_EXT) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= rx_data;
                    dur_wait <= 1'b1;
                end
                //--------------------------------------------------------
                // (f2) ★ V47：0xC3 命令字本身 —— 等力度那 1 个字节
                //   ★ 和 0xC2 一样【不受开窗限制】，随时可用。
                //--------------------------------------------------------
                else if (rx_data == VEL_BYTE) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= VEL_BYTE;
                    vel_wait <= 1'b1;
                end
                //--------------------------------------------------------
                // (g) 和弦帧头 / 帧尾
                //--------------------------------------------------------
                else if (rx_data == CHORD_ON) begin
                    tx_req   <= 1'b1;
                    tx_byte  <= rx_data;
                    in_chord <= 1'b1;
                    acc_mask <= 36'd0;
                    acc_oct_b0 <= 36'd0;      // ★ V37：新和弦从头累积
                    acc_oct_b1 <= 36'd0;
                    acc_oct_b2 <= 36'd0;
                    acc_oct_set  <= 36'd0;
                    key_oct_seen <= 1'b0;
                end
                else if (rx_data == CHORD_OFF) begin
                    tx_req  <= 1'b1;
                    tx_byte <= rx_data;
                    if (in_chord) in_chord <= 1'b0;    // 落盘由 (0) 统一做
                    key_oct_seen <= 1'b0;              // ★ V37：前缀不带出和弦
                end
                //--------------------------------------------------------
                // (h) 音号 / 休止符
                //--------------------------------------------------------
                else if (rx_data <= NOTE_MAX) begin
                    tx_req <= 1'b1;
                    tx_byte <= rx_data;
                    if (in_chord) begin
                        acc_mask[rx_data[5:0]] <= 1'b1;             // 累进和弦
                        // ★ V37：这个音号前面若带了 0xCD，就地记下它自己的八度。
                        //   记完立刻清 key_oct_seen —— 所以一个 0xCD 只作用一个音。
                        if (key_oct_seen) begin
                            acc_oct_b0[rx_data[5:0]] <= key_oct_pend[0];
                            acc_oct_b1[rx_data[5:0]] <= key_oct_pend[1];
                            acc_oct_b2[rx_data[5:0]] <= key_oct_pend[2];
                            acc_oct_set[rx_data[5:0]] <= 1'b1;
                            key_oct_seen                 <= 1'b0;
                        end
                    end
                end
                else if (rx_data == REST_BYTE) begin
                    tx_req <= 1'b1;
                    tx_byte <= rx_data;
                    // 和弦里出现休止符：忽略（上位机不应产生）
                end
                // 其它字节：丢弃，不回显（与 v1 一致）
            end
        end
    end

    //==================================================================
    // 4) 播放器
    //    每个事件占 slot_dur 个时间格；gate 在整个事件里保持高，
    //    只在【最后一个格】的末尾留出释音提前量。
    //
    //    发声 = dur × tick_cyc - rel_cyc
    //       "前 dur-1 个格全程发声" + "最后一个格发声 (tick_cyc - rel_cyc)"
    //     = (dur-1)×tick_cyc + tick_cyc - rel_cyc
    //     = dur × tick_cyc - rel_cyc          ∎
    //    所以这里只需要两个计数器，不需要任何宽乘法。
    //
    //  ★ 存储推断的硬约束：数组必须【单读单写】
    //    slot_dur 原来被读两个地址（当前 rd_slot 与前瞻 rd_slot+1），
    //    单口 RAM 装不下两个读地址，综合器就拒绝推断 RAM、退回 1024 个触发器。
    //    现在改成：只读 rd_next = rd_slot+1 这一个地址，
    //    "ev_left == 0" 表示"该装载下一个事件了"，装载时顺带把时长读进来。
    //    代价是事件之间多 1 个时钟（20ns）的过渡，相对最短 20ms 的时间格
    //    完全可以忽略，且此时 gate=0、不发键，听不出来。
    //
    //  ★★ 板子上"一声不响"的真凶（V21/V22 都栽在这）★★
    //    rd_slot 是 11 位；原写法 rd_next = rd_slot + 11'd1 在 rd_slot=1023 时
    //    得到的是 **1024**，不会自己绕回 0（要 11 位表示 1024 绰绰有余）。
    //    而结束时判据是 `rd_next >= frame_slots` —— 1024 对任何帧都成立，
    //    于是"装载事件 0"这一拍直接被判成"本帧放完"，play_run 一拍就掉回 0，
    //    midi_keys 从头到尾都是 0：**一个音都不响**。
    //    （V21 之所以"有声音"，是因为 play_start 被误接成 frame_start，
    //      点亮了 top.v 那条自动演奏链，循环弹 0/1/2/3 —— 那不是乐谱。
    //      把那条链改回恒 0 之后，就只剩这个 bug 的"完全静音"了。）
    //
    //    修法：
    //      ① rd_next 显式做【模 1024 绕回】；
    //      ② 结束条件改用 ev_remain（还剩几个事件没装载）——
    //         不能拿 rd_next 当结束条件，因为它会绕回。
    //==================================================================
    reg [9:0]  rd_slot;        // 当前正在放的事件索引（0..1023）
    reg [11:0] ev_remain;      // 还剩几个事件没装载（0..1024）
    reg [7:0]  ev_left;         // 本事件还剩几个格；0 = 需要装载下一个事件
    reg [29:0] tick_cnt;        // 当前格内的周期计数
    reg        play_run;

    // ★ slot_dur 的唯一读地址：★ 用 10 位存 0..1023，"+1" 自然会截断成 0 ★
    //   10 位最大就是 1023，1023+1 = 1024 -> 截成 10 位 = 0，绕回是【免费的】。
    //   写成 11 位再做三目判断（(rd_slot==1023)?0:rd_slot+1）会多出一个
    //   11 位比较器 + 选择器，正好串在 BSRAM 地址前面 —— 实测会把本来只有
    //   零点几纳秒余量的 clk_50m_ext 压垮（u_lcd 那边直接翻负）。
    wire [9:0]  rd_next  = rd_slot + 10'd1;
    // ★ V32：slot_dur 现在是 {八度偏移[2:0], 时值[7:0]}，拆开用。
    //   八度偏移在【装载事件的那一拍】一起取走（读的还是 rd_next 这一个地址，
    //   没有多开读口），之后整格事件期间保持不变。
    wire [7:0]  dur_next = (slot_dur[rd_next][7:0] == 8'd0) ? 8'd1 : slot_dur[rd_next][7:0];
    wire [2:0]  oct_next = slot_dur[rd_next][10:8];
    //=========================================================================
    // ★★★ V43.8：读地址改回 rd_slot —— 让 slot_mask 重新进 BSRAM ★★★
    //
    //   为什么退回来（有实测数据，不是猜）：
    //     V43.5 为了做 LED 的"下一格 30% 预告"，把读地址从 rd_slot 改成
    //     rd_next（= rd_slot + 1，一个【加法器输出】）—— 地址前面串了加法器，
    //     综合器就不肯推断 BSRAM 了：
    //         --SSRAM(RAM16)  214 块（V41） ->  817 块
    //         BSRAM           88      ->  86
    //         Logic           38687   ->  44054（65% -> 74%）
    //     ★ 这正是本文件第 1027 行注释警告过的事（"选择器串在 BSRAM 地址前面"）。
    //   ★ 代价：没有"下一格"可读 → LED 的 30% 预告那一档只能关掉
    //     （90/75/70 三档 + 跟谱演奏模式全部保留）。
    //   ★ 收益：拿回 ≈4600 个 LUT，Logic 有望回到 66% 左右，
    //     CLKOUT1 的余量跟着回来（74% 时它在 47~59MHz 之间乱跳）。
    //=========================================================================
    wire [35:0] slot_cur = ({1'b0, rd_slot} < frame_slots) ? slot_mask[rd_slot] : 36'd0;
    // ★ V47：当前事件的力度。越界时给满幅（= 不带力度的老行为）。
    wire [6:0]  slot_vel_cur = ({1'b0, rd_slot} < frame_slots) ? slot_vel[rd_slot] : VEL_FULL;
    wire [107:0] slot_oct_cur = ({1'b0, rd_slot} < frame_slots)
                              ? {slot_oct2[rd_slot], slot_oct1[rd_slot], slot_oct0[rd_slot]}
                              : 108'd0;
    wire        gate     = play_run && (ev_left != 8'd0)
                           && ((ev_left > 8'd1) || (tick_cnt < gate_last));

    //==================================================================
    // ★★★ V43：跟谱演奏 —— "按对了才前进" ★★★
    //
    //   判据：当前格要求的键【全部同时按住】。
    //         gate_mask_in 就是"按键 | 延音锁存"（top.v 已同步到本域）。
    //   ★ 注意：上面这行【故意不写成等号】—— check_rtl_contract.py 用正则
    //     扫文本（不剥注释）来确认"gate_mask_in 从没被赋值过"，
    //     注释里写个等号会把那条契约判红。
    //   休止格：need_mask == 0 → 自动放行（不用按任何键就能过）。
    //   防跑飞：★ 必须【松开再按】才算 —— 用 keys_ok 的【上升沿】而不是电平。
    //           否则按住不放会自己一格一格往下走（同音重复的地方尤其明显）。
    //   不卡死：wait_mode 是电平状态，随时可以被 0xCE 或 TTP 按键13 翻回 0；
    //           一旦翻回 0，下面 slot_go 立刻被置 1，播放马上继续。
    //           —— 这就是"超时/开关自动恢复"而不是"门控永久失灵"（坑 20/22）。
    //
    //   ★ 正常模式（wait_mode = 0）下 slot_go 恒为 1 → can_load 恒为 1
    //     → 播放器的行为与 V42【逐位相同】，一点变化都没有。
    //==================================================================
    wire [35:0] need_mask_c = slot_cur;
    // ★★ V43.1：比较用的掩码【打一拍】—— 时序治本，别删 ★★
    //   V43 第一次构建实测：CLKOUT1 的瓶颈变成了
    //       rd_slot -> slot_mask 读 -> need_mask -> 与 gate_mask_in 比较
    //         -> keys_ok -> slot_go 的 CE
    //   19.689ns / 20ns，slack 正好 0.000（等于把 V41 治好的余量又吃光了）。
    //   ★ 打一拍就切断了这条链，而且毫无副作用：用户按键要按住好几毫秒，
    //     晚一拍（20ns）才认出"按对了"完全无感。
    //   ★ 休止格判定 need_none 也一起用打拍后的值，保持两件事同拍。
    reg [35:0] need_mask;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) need_mask <= 36'd0;
        else        need_mask <= need_mask_c;
    end
    wire        need_none = (need_mask == 36'd0);
    // ★★★ V47.1 修：跟谱模式"第一格被替弹"的真根因 ★★★
    //
    //   症状（用户 2026-10-11 报）：跟谱模式下传完谱【响一声就停】，
    //   而且"没锁住第一个音"—— 本来该停在第 1 格等你按，它却自己弹了。
    //
    //   根因：need_mask 比 rd_slot 【慢一拍】。
    //       cycle N   : rd_slot 还是哨兵 1023 -> slot_cur = 0 -> need_mask <- 0
    //                   load_now = 1 -> rd_slot <- 0, ev_left <- dur, slot_go <- 0
    //                   （这一拍 slot_go 因为 load_now 优先级更高，是对的）
    //       cycle N+1 : rd_slot 已经是 0 了，但 need_mask 还是上一拍采的 0
    //                   -> need_none = 1  ★假报"这是休止格"★
    //                   -> slot_go <- 1（下面那个 else if 命中）
    //       cycle N+2 : need_mask 才更新成 slot_mask[0]，need_none 变 0 —— 已经晚了
    //   于是 gate_snd = gate && slot_go 被这【假的一拍】绕过去，这一格就发声了。
    //   ★ 每一格装载后都会这样，不只是第一格。
    //
    //   修法：装载后的那一拍把 need_none 屏蔽掉。
    //     load_now_d = load_now 打一拍，正好覆盖 cycle N+1 那一拍。
    //   ★ 只影响"装载后的那一拍"，别的拍行为完全不变。
    reg  load_now_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) load_now_d <= 1'b0;
        else        load_now_d <= load_now;
    end
    wire need_none_ok = need_none && !load_now_d;
    //   ★★ V43.6/V43.7：绝不能再用 gate_mask_in 判"按对了" ★★
    //     它是 sounding_mask_aud = key_mask | 延音锁存，而 key_mask 里含
    //     midi_keys_aud —— **乐谱自己弹出来的音**！用它判的话，乐谱一响
    //     条件就被满足，表现是"一开跟谱就自己往下播"（用户实测报的 bug）。
    //   ★ 现在的 keys_ok_in 是 clk_audio 域用【hand_mask】算好同步过来的，
    //     见 top.v 的 keys_ok_aud。
    wire        keys_ok   = keys_ok_in;
    reg         keys_ok_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) keys_ok_d <= 1'b0;
        else        keys_ok_d <= keys_ok;
    end
    wire keys_ok_rise = keys_ok & ~keys_ok_d;

    reg  slot_go;                                   // 本格已经"按对"过了
    wire can_load = (!wait_mode) || slot_go;        // 允许装载下一格吗
    wire load_now = play_run && (ev_left == 8'd0) && (ev_remain != 12'd0) && can_load;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          slot_go <= 1'b0;
        else if (!wait_mode) slot_go <= 1'b1;       // 正常模式：永远放行
        else if (load_now)   slot_go <= 1'b0;       // 刚换了一格 → 重新等这一格
        else if (need_none_ok || keys_ok_rise) slot_go <= 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_slot   <= 10'd1023;     // 哨兵：让第一次 rd_next 绕回 0
            ev_remain <= 12'd0;
            ev_left   <= 8'd0;
            tick_cnt  <= 30'd0;
            play_run  <= 1'b0;
            oct_cur   <= 3'd0;         // ★ V32
        end else if (clr_pulse) begin
            // 收到 0xAA：立即停播并放开所有键
            play_run  <= 1'b0;
            tick_cnt  <= 30'd0;
            rd_slot   <= 10'd1023;
            ev_remain <= 12'd0;
            ev_left   <= 8'd0;
            oct_cur   <= 3'd0;         // ★ V32：回原调
        end else if (stop_pulse) begin
            // ★ 收到 0xC7：停播 + 放键，但【乐谱一个字都不动】
            //   和上面 0xAA 的区别就在这：这里不碰 wr_slots / frame_slots，
            //   所以随后发一个 0xC8 就能拿板子里原来那份谱子重播。
            //   （rd_slot 归到哨兵，重播时第一拍 rd_next 绕回 0 就是事件 0）
            play_run  <= 1'b0;
            tick_cnt  <= 30'd0;
            rd_slot   <= 10'd1023;
            ev_remain <= 12'd0;
            ev_left   <= 8'd0;
            // ★ V32：这里【故意不把 oct_cur 清 0】—— 停止时正在响的那些音
            //   已经各自锁好了自己的八度（在 top.v 的 voice_oct[] 里），
            //   清掉它反而会让释音尾巴在下一拍被搬回原调（听起来跳音）。
            //   下一个事件装载时会从它自己的 slot 里取到正确的值。
        end else if (replay_pulse) begin
            // ★ 收到 0xC8：拿板内已经存着的那一帧重播，上位机不用重发谱子
            if (frame_slots == 11'd0) begin
                play_run <= 1'b0;              // 板子里本来就没谱子
                ev_left  <= 8'd0;
            end else begin
                play_run  <= 1'b1;
                rd_slot   <= 10'd1023;
                ev_remain <= {1'b0, frame_slots};
                ev_left   <= 8'd0;
                tick_cnt  <= 30'd0;
            end
        end else if (frame_start) begin
            if (frame_slots == 11'd0) begin
                play_run  <= 1'b0;             // 空帧（前导冲刷用的 FF）：不开播
                ev_left   <= 8'd0;
                ev_remain <= 12'd0;
            end else begin
                play_run  <= 1'b1;
                rd_slot   <= 10'd1023;         // 下一拍 rd_next 绕回 0 -> 装载事件 0
                ev_remain <= {1'b0, frame_slots};
                ev_left   <= 8'd0;
                tick_cnt  <= 30'd0;
            end
        end else if (play_run) begin
            if (ev_left == 8'd0) begin
                // ---- 装载下一个事件 ----
                if (ev_remain == 12'd0) begin
                    play_run <= 1'b0;                       // 本帧放完，放键
                end else if (can_load) begin                 // ★ V43：跟谱模式下要按对才装载
                    rd_slot   <= rd_next;
                    ev_left   <= dur_next;
                    ev_remain <= ev_remain - 12'd1;
                    tick_cnt  <= 30'd0;
                    oct_cur   <= oct_next;      // ★ V32：这一格的八度偏移，跟着事件走
                end
                // ★ V43：can_load = 0（跟谱模式还没按对）时这里【什么都不做】——
                //   停在这一格等用户。此时 ev_left 一直是 0 → gate = 0 →
                //   midi_keys = 0（不发声、不占声部），但 LED 照旧显示这一格
                //   （见下面 midi_guide 那两条，它们用的是 guide_en 而不是 gate）。
            end else if (tick_cnt >= tick_cyc - 1'b1) begin
                tick_cnt <= 30'd0;
                ev_left  <= (ev_left > 8'd1) ? (ev_left - 8'd1) : 8'd0;
            end else begin
                tick_cnt <= tick_cnt + 30'd1;
            end
        end
    end

    // 键掩码：gate 落就放键
    // ★ V37：【逐键八度和掩码在同一拍、同一个块里更新，而且共用同一个 gate】。
    //   这一点是刻意的：top.v 那边两条同步链的级数一样，所以掩码和它的八度
    //   天生是同一采样点的一对，绝不会出现"新掩码配上一个格的八度"。
    // ★ V43.5：八度也改成寄存器版了（见上面 slot_oct_cur_r）
    // ★ V42：下一格要弹的键（影子数组；读地址和 slot_cur 一样是 rd_slot，
    //   所以 BSRAM 地址前面【没有多出任何选择器】）。
    //   ★ 用 rd_next < frame_slots 判"下一格真的存在"：最后一格的下一格返回 0，
    //     而不是块里残留的脏数据。
    // ★★ V43.3：读口做成 `lcd_master.v` 里 `wave_buf` 那种【寄存器直读】的形状 ★★
    //
    //   wave_buf 是 V41 里**实测确认能进 BSRAM** 的写法：
    //       wave_v0 <= wave_buf[addr];          ← 读口纯粹，外面不套任何条件
    //   而 V43/V43.1/V43.2 三次都写成
    //       wire x = (cond) ? slot_nxt[addr] : 0;
    //   读口外面套着一个条件表达式 → 三次都没进 BSRAM（808 个 RAM16）。
    //   这一版改成"先纯读进寄存器，条件判断放到寄存器【后面】"。
    //
    //   ★ 代价：slot_nxt_cur 比原来晚一拍（20ns）。
    //     对 LED 完全无所谓 —— 它和"当前格"那一档最多差 20ns，
    //     在显示器上是绝对看不出来的（慢信号，整个事件期间都不变）。
    wire guide_en = wait_mode ? play_run : gate;
    // ★★★ V44.1：谱子发声的门控 —— 跟谱模式等待期间【不出声】★★★
    //   正常模式 gate_snd ≡ gate（逐位相同，不动原有行为）；
    //   跟谱模式只有"这一格已经按对了"（slot_go=1）才让谱子发声。
    //   ★ 为什么要这样：装载进来的那一格 ev_left=dur 本来是会自动发声的，
    //     用户看到的就是"第 1 格被替弹了"。
    // ★★★ V47.2 修：把 `|| slot_go` 去掉 ★★★
    //
    //   用户 2026-10-11 报：跟谱模式下"每次按指引按下后会同时响起这个音和下个音"，
    //   听感是【同一个音的又一次触发】；★ 只有开了逐音力度才出现。
    //
    //   根因：原来的 `gate && ((!wait_mode) || slot_go)` 里那个 slot_go：
    //     cycle X   用户按对 -> keys_ok_rise -> slot_go <= 1
    //               这一拍 gate = play_run && (ev_left != 0)   ★ 如果 ev_left 还没走完
    //     cycle X+1 slot_go = 1 且 ev_left != 0
    //               -> gate_snd = 1  ✗ 谱子把【当前这一格】弹出来了
    //               -> 而 slot_cur 还是用户刚按的那个音
    //               -> 听感就是"同一个音的又一次触发"
    //               （load_now 此时还不能触发，要 ev_left==0，所以它会一直响到本格结束）
    //   ★ 为什么"开了力度才有"：开了力度后每个事件多 2 字节（0xC3 vel），
    //     整帧变长、传输变久，用户按下去的时机更容易落进"ev_left 还没走完"的窗口，
    //     把这个一直存在的隐患暴露出来了。
    //
    //   修法：跟谱模式下【谱子永远不出声】—— gate_snd = gate && !wait_mode。
    //     ★ 这才是跟谱模式的语义：灯亮着告诉你按哪些键，声音来自【你自己按的键】
    //       （手弹那条路本来就会发声）。FPGA 不该替你把这一格弹出来。
    //     ★ 而且这样【物理上不可能】再漏：只要 wait_mode=1，gate_snd 恒为 0。
    //     ★ 正常模式（wait_mode=0）下 gate_snd ≡ gate，和 V46 逐位相同。
    wire gate_snd = gate && !wait_mode;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            midi_keys      <= 36'd0;
            midi_guide     <= 36'd0;
            midi_keys_next <= 36'd0;
            midi_octs      <= 108'd0;
        end else begin
            // ★★★ V44.1：跟谱模式下【谱子不替你发声】★★★
            //   用户实测报的现象："开始时会跑一个音，从第二个音才开始停下"。
            //   根因：装载进来的那一格 ev_left=dur → gate=1 → midi_keys=这一格的键
            //         → **谱子自己把它弹出来了**；而 slot_go 要到【下一次】装载
            //         才被清 0，所以只有第 1 格会被"替弹"，第 2 格起才停下等你。
            //   正解：跟谱模式等待期间把"谱子发声"也门控掉 ——
            //         灯亮着等你按，**你按了才响**（声音来自你自己按的键，
            //         本来就会发声，所以听感完全正常）。
            //   ★ 正常模式 gate_snd ≡ gate —— 和以前【逐位相同】。
            midi_keys      <= gate_snd ? slot_cur     : 36'd0;
            // ★ 指引掩码：跟谱模式下【直接取当前格】——
            //   不能再靠 midi_keys 保持了（等待期间它恒 0，会显示成上一格）。
            //   正常模式下等于 midi_keys，和 V41 的 LED 行为一致。
            midi_guide     <= (play_run && wait_mode) ? slot_cur : midi_keys;
            // ★ V43.8：「30% 下一格预告」已关闭 → 恒 0
            midi_keys_next <= 36'd0;
            midi_octs      <= gate     ? slot_oct_cur : 108'd0;
            // ★★ V47：当前事件的力度（给 top.v 的混音缩放用）★★
            //   ★★ gate 为 0 时给【满幅 127】，不是 0 ★★ ——
            //      给了 0 的话音与音之间缩放系数会掉下去、下个音再爬上来，
            //      ramp 每采样点走 1，大落差要好几毫秒，反而弄出新"噗"声。
            //      这个寄存器是跟谱模式/正常模式【共用】的一条，
            //      不跟谱时 gate_snd ≡ gate，行为不变。
            midi_vel       <= gate     ? slot_vel_cur : VEL_FULL;
        end
    end

    //==================================================================
    // 死端口：这五个信号恒 0 —— ★★ 必须恒 0，不是"没用随便给" ★★
    //
    //   top.v 里 play_start 接的是一条【自动演奏链】：
    //       play_start 上升沿 -> auto_en <= 1'b1
    //       auto_keys[note_idx] = 1      (note_idx 每 0.5s 走一格)
    //       key_mask = ... | auto_keys_aud | ...
    //   也就是说，只要 play_start 出现一次上升沿，top.v 就会【永久】多点亮
    //   一个键，并且每 0.5 秒换下一个 —— 听感是"乐谱之外一直有个音在拖"。
    //
    //   v1 把 play_start 恒 0，这条链就是死的（LCD 乐谱显示也一直是废的）。
    //   本版曾把 play_start 接成 frame_start、play_len 接成事件数，结果就是
    //   每帧结束后多出一个长鸣音 —— 已改回恒 0，与 v1 完全一致。
    //   和弦完全不依赖这条链：它走 midi_keys（36 位掩码）直接发声。
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            score_we    <= 1'b0;
            score_waddr <= 8'd0;
            score_wdata <= 8'd0;
            play_start  <= 1'b0;
            play_len    <= 8'd0;
        end else begin
            score_we    <= 1'b0;
            score_waddr <= 8'd0;
            score_wdata <= 8'd0;
            play_start  <= 1'b0;
            play_len    <= 8'd0;
        end
    end

    //==================================================================
    // 4b) 发送 FIFO（16 字节）
    //
    //   原来只有单字节 tx_req：tx 一忙，当拍来的字节直接丢。
    //   回显时收发同速（都是 1.04ms/字节）所以看不出来，
    //   但录音事件帧一次 8 个字节，不排队必丢。
    //
    //   ★ 指针必须是 5 位。16 深的 FIFO 用 4 位指针时 wr 会绕回 0，
    //     于是"装满了 16 个"和"空"读出来是同一个值 —— txf_full 永远为假，
    //     第 17 个字节会【静默覆盖】最早那个（有可能正好是帧头 0x5A）。
    //     多一位之后 cnt 落在 0..31，full 就是明确的 16。
    //
    //   ★ 为什么 16 够用（这条是设计前提，改采样率/波特率要重新算）：
    //     录音事件最短间隔 = 采样周期 10ms，一帧 8 字节；
    //     9600 波特下 8 字节要发 8.3ms，比 10ms 短。
    //     即"产得比发得慢"，稳态下 FIFO 里最多攒几个字节，离 16 很远。
    //==================================================================
    reg  [7:0]  txf_mem [0:15];
    reg  [4:0]  txf_wr, txf_rd;        // ★ 5 位（见上）
    wire [5:0]  txf_cnt   = {1'b0, txf_wr} - {1'b0, txf_rd};
    wire        txf_full  = (txf_cnt >= 6'd16);
    wire        txf_empty = (txf_cnt == 6'd0);

    //==================================================================
    // 4c) 录音回传
    //
    //   录的是"真正在响的音"：top.v 的 sounding_mask（= 包络非 0，含 ADSR 尾巴），
    //   每 10ms 采一次，一变就回传一个 8 字节事件帧：
    //       [0x5A] [mask 36 位 小端 5 字节] [dt 2 字节小端，单位 10ms]
    //   另外把【效果开关】也回传（变了才发，开录时强制发一次）：
    //       [0x5D] [8 位：滑音/延音/音色/颤音/延迟/笛子/混响/合唱]
    //   开关两端都能翻：
    //       · 板子上按【按钮板的第 10 键】（btn_rec，单拍脉冲）
    //       · 上位机发一个 0x5C
    //   开关一变就回传 [0x5B] [0/1]，让上位机同步界面。
    //
    //   ★ 录音期间【不回显】—— 回显字节是任意值，会和 0x5A 帧头撞车。
    //     录音时上位机本来也不发谱子，不回显不影响任何事。
    //==================================================================
    reg rec_on;
    reg rec_toggle;                    // 收到 0x5C 时置 1 拍

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rec_on <= 1'b0;
        else if (btn_rec | rec_toggle) rec_on <= ~rec_on;
    end

    // ---- 三种回传帧的状态（声明统一放在采样器【前面】，避免前向引用）----
    //   帧格式：事件 [0x5A][mask 5B][dt 2B] / 开关 [0x5B][0/1] / 效果 [0x5D][8 位]
    reg [63:0] rec_shift;              // 事件帧移位寄存器
    reg [3:0]  rec_tx_idx;             // 0 = 空闲；1..8 = 正在发第几个字节
    reg        rec_on_d;               // 上一次【已经回报过】的开关
    reg [1:0]  rec_st_idx;             // 开关帧：0 空闲 / 1 帧头 / 2 数据
    reg [7:0]  fx_last;                // 上一次已经回报过的效果状态
    reg [1:0]  tone_last;              // 上一次已经回报过的音色
    reg [7:0]  fx_byte_r;              // 本次要发的效果状态
    // ★★ V43：跟谱模式搭这趟车一起报（变成第 4 个字节）★★
    //   ★ 为什么不另开一帧（比如再定义一个 0x5E 帧头）：那就要在下面那套
    //     "帧启动仲裁"里再加一路，而那段注释专门警告过
    //     ——"两路在同一拍同时启动，另一路的字节会被悄悄吞掉"。
    //     搭现成的车 = 仲裁逻辑【一个字都不用动】，风险最小。
    reg        mode_byte_r;            // 本次要发的跟谱模式
    reg        mode_last;              // 上一次报出去的模式（变了才发）
    reg [2:0]  fx_idx;                 // 0 空闲 / 1 帧头 / 2 效果 / 3 音色 / 4 跟谱模式
    // ★★ V45：节拍器状态帧（0x5E）用的寄存器 ★★
    //   ★ 必须声明在这里：mt_want / mt_start 在下面的【总线仲裁】那一段
    //     就要用到，声明放到帧那一段就成了"先用后声明"。
    reg [7:0]  mt_bpm_l, mt_num_l, mt_den_l;   // 上一次【发出去】的值
    reg [7:0]  mt_bpm_r, mt_num_r, mt_den_r;   // 发送期间锁存，免得发到一半变了
    reg [2:0]  mt_beat_r;
    reg [2:0]  mt_idx;                         // 0 空闲 / 1..5 第几个字节
    reg        mt_force;                       // 复位后 / 0xCB 请求时强制发一次
    wire       mt_want;
    assign mt_want = mt_force || (metro_bpm != mt_bpm_l)
                              || (metro_num != mt_num_l)
                              || (metro_den != mt_den_l);
    // ★★ V48：显示模式 + 录音开关状态帧（0x5F）用的寄存器 ★★
    //   ★ 和 mt_* 一样必须声明在【总线仲裁】那一段之前（先用后声明会建出隐式线）。
    //   ★★ 为什么另开一台小状态机、而不复用 fx_idx ★★：
    //      fx_idx 是 0x5D 效果帧的索引，它的字节映射（1=帧头/2=效果/3=音色/4=跟谱）
    //      是钉死的。把 0x5F 塞进去就必须改 0x5D 的长度或映射 ——
    //      而 0x5D 是【已经上线、上位机和安卓都在解析】的帧，动它风险最大。
    //      所以这里独立一台（dp_*），0x5D 一个字节都不动（和 V45 的 0x5E 同一个做法）。
    wire [2:0] dp_disp_w = (disp_mode_in > DISP_MAX_MODE) ? DISP_MAX_MODE
                                                          : disp_mode_in[2:0];
    reg [2:0]  dp_disp_l;              // 上一次【发出去】的显示模式
    reg        dp_rec_l;               // 上一次【发出去】的录音开关
    reg [2:0]  dp_disp_r;              // 发送期间锁存，免得发到一半变了
    reg        dp_rec_r;
    reg [1:0]  dp_idx;                 // 0 空闲 / 1..3 第几个字节
    reg        dp_force;               // 复位后 / 0xCB 请求时强制发一次
    wire       dp_want;
    //   ★★ "任意一个值变了就发" —— 包括【板子上按 TTP 键3 / 键10】★★
    //     disp_mode_in 变了 = 板子上按了按钮 3（或上位机设的、板子刚接受）；
    //     rec_on 变了      = 板子上按了按钮 10（或上位机发的 0x5C）。
    //     这才是用户要的"双向同步"：上位机不是单方面下命令，板子也会主动说话。
    //   ★★ V48.2：因为现在是 5 档一一对应，"板子上在 2/3/4 三档李萨如之间切换"
    //      也会让 dp_disp_w 变化 -> 也会发一帧 —— 这正是用户要的
    //      （上位机要能看出板子现在是哪一档李萨如）。
    //   ★ 录音开关这一半和 0x5B 帧【故意重复】：0x5B 是 2 字节的老帧（上位机
    //     一直在用），0x5F 是新的 3 字节帧。上位机侧已经把"上升沿 = 新一遍开始"
    //     抽成 _set_rec_on() 给两帧共用，所以谁先到都不会把录的第一遍清掉。
    assign dp_want = dp_force || (dp_disp_w != dp_disp_l) || (rec_on != dp_rec_l);
    reg        fx_force;               // 开始录音时强制发一次初始状态
    reg        fx_req;                 // ★ V31：0xCB 请求的快照（电平，发出后清）

    // ★ V31：当前音色在本域里两级同步（top.v 那侧是 clk_50m 域）
    //   只有 2 位、而且是人手/上位机触发的慢变化，两级足够；
    //   万一同步到一个中间态，下一拍就自己纠正（顶多多发一帧状态）。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tone_s1 <= 2'd0;
            tone_s2 <= 2'd0;
        end else begin
            tone_s1 <= tone_sel_in;
            tone_s2 <= tone_s1;
        end
    end

    // 采样器用的寄存器（也必须声明在 rec_load 之前）
    reg [19:0] rec_tick;
    reg [15:0] rec_dt;                 // 距上一个事件的 10ms 数
    reg [35:0] rec_mask_r;             // 上一次采到的掩码
    reg        rec_fire;               // 有事件待发（电平，被取走才清）
    reg [35:0] rec_fire_mask;
    reg [15:0] rec_fire_dt;
    reg        rec_force;              // 开始录音时强制报一次当前掩码

    //==================================================================
    // ★★ 三种回传帧的【启动】判据：同一拍最多只有一个为真 ★★
    //
    //   这是本模块最容易写错的地方。写成"各路自己判自己的条件"，
    //   就会出现两路在同一拍同时启动：两边的移位寄存器都往前走了，
    //   但 FIFO 只收得下先写的那一个 —— 另一路的字节被悄悄吞掉
    //   （事件帧因此缺一个字节，上位机整帧错位，还查不出来）。
    //
    //   所以在这里统一排出优先级，后面三路都用这几个 wire：
    //       开关帧 > 效果帧 > 事件帧 > 回显
    //   ★ 判据里必须带 !st_start / !fx_start，不能只带"那一路当前是不是忙"
    //     —— 同一拍里那一路的 index 还是 0，"忙"是下一拍才成立的。
    //==================================================================
    wire st_want = (rec_on != rec_on_d);
    //==================================================================
    // ★★ V31：效果状态帧的启动判据改了 ★★
    //   原来：rec_on && (fx_force || 状态变了)  —— 只有录音时才回报。
    //   现在：三个来源都算 ——
    //     · fx_req            : 上位机发 0xCB 主动要一次快照
    //     · fx_force && rec_on: 开始录音时强制报一次初始状态（原逻辑不动）
    //     · 状态/音色变了      : 谁按的都算（板上的 TTP 按钮、上位机、按键映射）
    //   ★ 为什么必须"变了就报"：软件那边的开关要跟着板子走 ——
    //     用户在板子上按了按钮，上位机的开关也得动，否则两边不一致。
    //     效果开关是人手按的，很稀疏，这点带宽可以忽略。
    //==================================================================
    wire fx_want = fx_req || (fx_force && rec_on)
                   || (fx_state_in != fx_last) || (tone_sel_sync != tone_last)
                   || (wait_mode != mode_last);     // ★ V43：跟谱模式变了也报
    wire ev_want = rec_fire;

    wire st_start = st_want && (rec_tx_idx == 4'd0) && (fx_idx == 3'd0)
                    && (mt_idx == 3'd0) && (dp_idx == 2'd0);
    wire fx_start = fx_want && (rec_tx_idx == 4'd0) && (rec_st_idx == 2'd0) && !st_start
                    && (mt_idx == 3'd0) && (dp_idx == 2'd0);
    //   ★ V45：mt_* 是节拍器状态帧（0x5E），优先级最低（放在最后判）。
    //   ★★ 声明顺序有讲究：st_start / fx_start 必须【写在前面】——
    //      它们也是 wire，本行要用它们做互斥，声明放到后面就成了"先用后声明"。
    wire mt_start = mt_want && (mt_idx == 3'd0) && (rec_tx_idx == 4'd0)
                    && (rec_st_idx == 2'd0) && (fx_idx == 3'd0)
                    && !st_start && !fx_start && (dp_idx == 2'd0);
    //   ★★ V48：dp_* 是显示模式状态帧（0x5F）。排在最后判（优先级最低）——
    //      它是"人手点一下才动"的慢信息，晚一帧发毫无影响；
    //      而事件帧（0x5A）是录音的时间轴，绝不能让它等。
    wire dp_start = dp_want && (dp_idx == 2'd0) && (rec_tx_idx == 4'd0)
                    && (rec_st_idx == 2'd0) && (fx_idx == 3'd0)
                    && (mt_idx == 3'd0) && !st_start && !fx_start && !mt_start;
    wire rec_load = ev_want && (rec_tx_idx == 4'd0) && (rec_st_idx == 2'd0)
                    && (fx_idx == 3'd0) && !st_start && !fx_start && (mt_idx == 3'd0)
                    && (dp_idx == 2'd0);

    wire rec_busy = (rec_tx_idx != 4'd0) || (rec_st_idx != 2'd0) || (fx_idx != 3'd0)
                    || (mt_idx != 3'd0) || (dp_idx != 2'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rec_tick <= 20'd0; rec_dt <= 16'd0; rec_mask_r <= 36'd0;
            rec_fire <= 1'b0; rec_fire_mask <= 36'd0; rec_fire_dt <= 16'd0;
            rec_force <= 1'b1;
        end else begin
            // ★ rec_fire 是【电平】：帧真正被装载时才清，不能每拍无条件清。
            //   每拍清的话，如果这次按键变化正好赶上上一帧还在往 FIFO 里推
            //   （要 8 拍），这个脉冲就被吃掉了 —— 那个音整条丢失，而且
            //   后面的 dt 是从"丢失的那次"继续算的，整条时间轴都会偏。
            //   注意这行写在采样逻辑【前面】，所以同一拍里"新事件置位"优先。
            if (rec_load) rec_fire <= 1'b0;
            if (!rec_on) begin
                // 关着时跟着当前掩码走：这样开录那一拍不会拿"上一次的状态"
                // 当变化报出去（真正的初始状态由下面的 rec_force 负责报一次）
                rec_tick   <= 20'd0;
                rec_dt     <= 16'd0;
                rec_mask_r <= gate_mask_in;
                rec_force  <= 1'b1;
            end else if (rec_tick >= REC_TICK_DIV - 1'b1) begin
                rec_tick <= 20'd0;
                // ★ 开始录音时强制报一次当前掩码（dt=0）。
                //   否则"按着键再开录音"这种情形下，那个键从头到尾都不会
                //   作为"变化"出现 —— 它整条不在录音里。
                //   报一次之后上位机就知道 t=0 时什么在响，不用猜。
                if (rec_force || (gate_mask_in != rec_mask_r)) begin
                    rec_mask_r    <= gate_mask_in;
                    rec_fire      <= 1'b1;
                    rec_fire_mask <= gate_mask_in;
                    // ★★ dt 必须把【这一拍】算进去 ★★
                    //   原来报的是 rec_dt（= 两次变化之间隔了几个"没变的"
                    //   采样边界），每来一个事件就少 10ms。12 个事件累积
                    //   −110ms，几百个事件就是好几秒 —— 听感就是"间隔不对、
                    //   越放越赶"。采样边界本身就是过去了 10ms，当然要算。
                    rec_fire_dt   <= rec_force ? 16'd0 :
                                     ((rec_dt == 16'hFFFF) ? 16'hFFFF
                                                           : (rec_dt + 1'b1));
                    rec_dt        <= 16'd0;
                    rec_force     <= 1'b0;
                end else if (rec_dt != 16'hFFFF) begin
                    rec_dt <= rec_dt + 1'b1;
                end
            end else begin
                rec_tick <= rec_tick + 1'b1;
            end
        end
    end

    // ---- 组帧（移位寄存器 / rec_load 的声明见上面采样器之前）----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rec_shift <= 64'd0; rec_tx_idx <= 4'd0;
        end else if (rec_load) begin
            // ★ 装载不等 txf_full：数据先搬进 rec_shift 就安全了。
            //   等的话，FIFO 一满、新的按键变化又会覆盖 rec_fire_*，
            //   等于白等一场还丢一个事件。
            rec_shift  <= {rec_fire_dt, 4'd0, rec_fire_mask, REC_EV_HEAD};
            rec_tx_idx <= 4'd1;
        end else if (rec_tx_idx != 4'd0 && !txf_full) begin
            rec_shift  <= {8'd0, rec_shift[63:8]};
            rec_tx_idx <= (rec_tx_idx == 4'd8) ? 4'd0 : (rec_tx_idx + 1'b1);
        end
    end

    wire       rec_push = (rec_tx_idx != 4'd0) && !txf_full;
    wire [7:0] rec_byte = rec_shift[7:0];

    // ---- 开关状态回报：[0x5B] [0/1] ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rec_on_d <= 1'b0; rec_st_idx <= 2'd0; end
        else if (rec_st_idx == 2'd0) begin
            // ★ rec_on_d 只在【真的要发】的那一拍才更新。
            //   写成"每拍 rec_on_d <= rec_on"的话，"状态变了"这个条件只维持一拍，
            //   被挡住一下之后就再也补不上了（开关回报丢失）。
            if (st_start) begin
                rec_on_d   <= rec_on;
                rec_st_idx <= 2'd1;
            end
        end else if (!txf_full) begin
            rec_st_idx <= (rec_st_idx == 2'd1) ? 2'd2 : 2'd0;
        end
    end
    wire       rec_st_push = (rec_st_idx != 2'd0) && !txf_full;
    wire [7:0] rec_st_byte = (rec_st_idx == 2'd1) ? REC_ST_HEAD : {7'd0, rec_on};

    //==================================================================
    // 效果状态帧：[0x5D] [8 位开关]
    //
    //   ★ 只在【状态变了】的时候发。效果开关是人手按的，很稀疏，
    //     一个演奏里通常就几次 —— 带宽可以忽略（事件帧才是大头）。
    //   ★ 开始录音时【强制】发一次初始状态，保证上位机一定知道
    //     "这一遍是从什么效果配置开始的"，不用去猜。
    //
    //   位定义（和 top.v 里打的那包一致）：
    //     bit0 滑音 glide_en   bit1 延音锁存 sustain_on
    //     bit2 音色 timbre     bit3 颤音 trem_on
    //     bit4 延迟 dly_on     bit5 笛子 flute_on
    //     bit6 混响 rev_on     bit7 合唱 cho_on
    //   ★ 按钮 3 的 wave_mode【不在】这里 —— 它只切 LCD 的显示（频谱/波形），
    //     一个采样点都不影响，录它没有意义。
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fx_last <= 8'd0; tone_last <= 2'd0; fx_byte_r <= 8'd0;
            mode_byte_r <= 1'b0; mode_last <= 1'b0;          // ★ V43
            fx_idx <= 3'd0; fx_force <= 1'b1; fx_req <= 1'b0;
        end else begin
            // 关着录音时把"强制报一次"重新武装（下面那帧如果在飞，让它先飞完）
            if (!rec_on && (fx_idx == 3'd0)) fx_force <= 1'b1;
            // ★ 0xCB 的请求：置位到这帧真正开始发为止（电平，丢了也能自己补）
            if (fx_req_set)   fx_req <= 1'b1;
            else if (fx_start) fx_req <= 1'b0;

            if (fx_idx == 3'd0) begin
                if (fx_start) begin
                    fx_last     <= fx_state_in;
                    tone_last   <= tone_sel_sync;
                    fx_byte_r   <= fx_state_in;
                    mode_last   <= wait_mode;     // ★ V43
                    mode_byte_r <= wait_mode;     // ★ V43（锁存，免得发到一半变了）
                    fx_idx      <= 3'd1;
                    fx_force    <= 1'b0;
                end
            end else if (!txf_full) begin
                // 1 -> 2 -> 3 -> 4 -> 0：帧头、效果、音色、跟谱模式，四个字节
                // ★ V43：原来是 1->2->3->0（三个字节），现在多一个第 4 字节。
                //   ★ 上位机必须同步改成读 4 个字节（电脑版 + 安卓版都改了）。
                fx_idx <= (fx_idx == 3'd4) ? 3'd0 : (fx_idx + 3'd1);
            end
        end
    end
    wire       fx_push = (fx_idx != 3'd0) && !txf_full;
    wire [7:0] fx_byte = (fx_idx == 3'd1) ? REC_FX_HEAD :
                         (fx_idx == 3'd2) ? fx_byte_r   :
                         (fx_idx == 3'd3) ? {6'd0, tone_last} :
                                            {7'd0, mode_byte_r};   // ★ V43

    //==================================================================
    // ★★ V45：节拍器状态帧 [0x5E][BPM][分子][分母编码][当前拍] = 5 字节 ★★
    //
    //   ★ 这是一条【全新的独立帧】，完全没有动已有的 0x5D 效果帧 ——
    //     0x5D 还是 4 字节，上位机/安卓那边读 0x5D 的代码一行都不用改。
    //   ★ 只在【设置变了】的时候发。另外两个强制点：
    //       · 复位后发一次 —— 让上位机一定知道板子的默认值是 100 BPM / 4/4
    //       · 收到 0xCB（"回报一次状态"）时跟着 0x5D 一起发一次
    //   ★ 刻意【不】周期上报 —— 免得跟录音的 0x5A 事件流抢带宽。
    //==================================================================
    //   ★ 本帧用到的 reg / wire（mt_*）都在前面 fx_idx 那一堆旁边声明 ——
    //     因为 mt_want / mt_start 在【总线仲裁】那一段就要用到，
    //     声明放到这里会变成"先用后声明"（工具可能建出隐式 1 位线）。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mt_bpm_l <= 8'd0; mt_num_l <= 8'd0; mt_den_l <= 8'd0;
            mt_bpm_r <= 8'd0; mt_num_r <= 8'd0; mt_den_r <= 8'd0;
            mt_beat_r <= 3'd0;
            mt_idx <= 3'd0; mt_force <= 1'b1;
        end else begin
            // 0xCB 请求回报时，节拍器状态也跟着强制发一次
            if (fx_req_set) mt_force <= 1'b1;

            if (mt_idx == 3'd0) begin
                if (mt_start) begin
                    mt_bpm_l  <= metro_bpm;
                    mt_num_l  <= metro_num;
                    mt_den_l  <= metro_den;
                    mt_bpm_r  <= metro_bpm;
                    mt_num_r  <= metro_num;
                    mt_den_r  <= metro_den;
                    mt_beat_r <= metro_beat_in;   // ★ 只回报，不参与任何判断
                    mt_idx    <= 3'd1;
                    mt_force  <= 1'b0;
                end
            end else if (!txf_full) begin
                // 1 -> 2 -> 3 -> 4 -> 5 -> 0：帧头、BPM、分子、分母、当前拍
                mt_idx <= (mt_idx == 3'd5) ? 3'd0 : (mt_idx + 3'd1);
            end
        end
    end
    wire       mt_push = (mt_idx != 3'd0) && !txf_full;
    wire [7:0] mt_byte = (mt_idx == 3'd1) ? METRO_STAT_BYTE :
                         (mt_idx == 3'd2) ? mt_bpm_r :
                         (mt_idx == 3'd3) ? mt_num_r :
                         (mt_idx == 3'd4) ? mt_den_r :
                                            {5'd0, mt_beat_r};

    //==================================================================
    // ★★ V48：显示模式 + 录音开关状态帧 ★★
    //     [0x5F] [DISP_MODE] [REC_ON]   —— 固定 3 字节
    //
    //   DISP_MODE = 板子当前【实际生效】的显示模式（★ V48.2：5 档一一对应）
    //               0 = 频谱              1 = 波形
    //               2 = 李萨如·左×右       3 = 李萨如·干声×效果后
    //               4 = 李萨如·相位
    //   REC_ON    = 板子当前的录音开关（0/1）
    //
    //   ★ 只在这两个值【任意一个变了】的时候发，不做周期上报
    //     —— 免得跟录音的 0x5A 事件流抢带宽。
    //   ★ 两个强制点（和 0x5E 节拍器帧完全同一套写法）：
    //       · 复位后发一次 —— 上位机因此一定拿得到板子的出厂状态
    //       · 收到 0xCB（"回报一次状态"）时跟着 0x5D/0x5E 一起发一次
    //     ★★ 后面这一条是"上位机一连接就对得上"的关键 ★★：
    //        电脑版打开串口后 600ms 会自动发一条 0xCB（fx_auto_query），
    //        板子顺势把显示模式/录音状态报上去，界面那行小字就从"未知"
    //        变成板子的真实状态 —— 不用等用户在板子上按一下才同步。
    //   ★ 本帧用到的 reg / wire（dp_*）都在前面 fx_idx 那一堆旁边声明。
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dp_disp_l <= 3'd0; dp_rec_l <= 1'b0;
            dp_disp_r <= 3'd0; dp_rec_r <= 1'b0;
            dp_idx    <= 2'd0; dp_force <= 1'b1;
        end else begin
            // 0xCB 请求回报时，显示模式/录音状态也跟着强制发一次
            if (fx_req_set) dp_force <= 1'b1;

            if (dp_idx == 2'd0) begin
                if (dp_start) begin
                    // ★ "上一次发出去的值"只在【真的要发】的那一拍更新 ——
                    //   写成"每拍都更新"的话，"值变了"这个条件只维持一拍，
                    //   被总线让路挡一下之后就再也补不上了（和 rec_on_d 同理）。
                    dp_disp_l <= dp_disp_w;
                    dp_rec_l  <= rec_on;
                    dp_disp_r <= dp_disp_w;      // 发送期间锁存
                    dp_rec_r  <= rec_on;
                    dp_idx    <= 2'd1;
                    dp_force  <= 1'b0;
                end
            end else if (!txf_full) begin
                // 1 -> 2 -> 3 -> 0：帧头、显示模式、录音开关
                dp_idx <= (dp_idx == 2'd3) ? 2'd0 : (dp_idx + 2'd1);
            end
        end
    end
    wire       dp_push = (dp_idx != 2'd0) && !txf_full;
    wire [7:0] dp_byte = (dp_idx == 2'd1) ? DISP_STAT_BYTE :
                         (dp_idx == 2'd2) ? {5'd0, dp_disp_r} :
                                            {7'd0, dp_rec_r};

    // ---- 统一入队：开关帧 > 效果帧 > 事件帧 > 回显 ----
    //   ★ 前四者已被 rec_busy 保证不会同时发生，这里的优先级只是兜底。
    //   ★ 回显也要让路：录音刚关掉的那 2 个字节（[0x5B][0]）还在飞，
    //     这时候上位机要是发了个字节，两路又会抢。
    wire       echo_push = tx_req && !rec_on && !rec_busy;
    wire       tx_push   = rec_st_push | fx_push | mt_push | dp_push | rec_push
                           | echo_push;
    wire [7:0] tx_push_data = rec_st_push ? rec_st_byte :
                              fx_push     ? fx_byte     :
                              mt_push     ? mt_byte     :
                              dp_push     ? dp_byte     :
                              rec_push    ? rec_byte    : rx_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) txf_wr <= 5'd0;
        else if (tx_push && !txf_full) txf_wr <= txf_wr + 1'b1;
    end
    always @(posedge clk) begin
        if (tx_push && !txf_full) txf_mem[txf_wr[3:0]] <= tx_push_data;
    end

    //==================================================================
    // 5) 发送：收到什么就原样回传什么（0xAA 除外），帧尾补 0xFF
    //    即时回传比"整包缓存再回传"更好：收发共用一块状态，不会互相多驱动。
    //==================================================================
    localparam TX_IDLE = 3'd0, TX_START = 3'd1, TX_DATA = 3'd2,
               TX_STOP = 3'd3, TX_GAP   = 3'd4;

    reg [2:0]  tx_state;
    reg [15:0] tx_baud_cnt;
    reg [2:0]  tx_bit_idx;
    reg [7:0]  tx_data;
    reg        tx_have;

    wire tx_load = !txf_empty && (tx_state == TX_IDLE) && !tx_have;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state    <= TX_IDLE;
            uart_txd    <= 1'b1;
            tx_baud_cnt <= 16'd0;
            tx_bit_idx  <= 3'd0;
            tx_data     <= 8'h00;
            tx_have     <= 1'b0;
            txf_rd      <= 5'd0;
        end else begin
            if (tx_load) begin
                tx_data <= txf_mem[txf_rd[3:0]];
                tx_have <= 1'b1;
                txf_rd  <= txf_rd + 1'b1;
            end

            case (tx_state)
                TX_IDLE: begin
                    uart_txd    <= 1'b1;
                    tx_baud_cnt <= 16'd0;
                    tx_bit_idx  <= 3'd0;
                    if (tx_have) begin
                        tx_have  <= 1'b0;
                        tx_state <= TX_START;
                    end
                end
                TX_START: begin
                    uart_txd <= 1'b0;
                    if (tx_baud_cnt >= BAUD_DIV - 1) begin
                        tx_baud_cnt <= 16'd0;
                        tx_bit_idx  <= 3'd0;
                        tx_state    <= TX_DATA;
                    end else tx_baud_cnt <= tx_baud_cnt + 1'b1;
                end
                TX_DATA: begin
                    uart_txd <= tx_data[tx_bit_idx];
                    if (tx_baud_cnt >= BAUD_DIV - 1) begin
                        tx_baud_cnt <= 16'd0;
                        if (tx_bit_idx >= 3'd7) tx_state <= TX_STOP;
                        else                    tx_bit_idx <= tx_bit_idx + 1'b1;
                    end else tx_baud_cnt <= tx_baud_cnt + 1'b1;
                end
                TX_STOP: begin
                    uart_txd <= 1'b1;
                    if (tx_baud_cnt >= BAUD_DIV - 1) begin
                        tx_baud_cnt <= 16'd0;
                        tx_state    <= TX_GAP;
                    end else tx_baud_cnt <= tx_baud_cnt + 1'b1;
                end
                TX_GAP: begin
                    uart_txd <= 1'b1;
                    if (tx_baud_cnt >= BAUD_DIV - 1) begin
                        tx_baud_cnt <= 16'd0;
                        tx_state    <= TX_IDLE;
                    end else tx_baud_cnt <= tx_baud_cnt + 1'b1;
                end
                default: tx_state <= TX_IDLE;
            endcase
        end
    end

endmodule
