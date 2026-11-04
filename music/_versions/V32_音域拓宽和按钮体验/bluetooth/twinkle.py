# -*- coding: utf-8 -*-
"""twinkle.py —— 小星星，按你实测的琴键编号

你的编号（已验证）：
    白一 05 = do    白二 06 = re    白三 07 = mi
    白四 08 = fa    白五 09 = sol   白六 0A = la
    （黑键 00~02 = A2/A#2/B2）

用法：
    python twinkle.py COM16      # 发给板子
    python twinkle.py            # 只打印字节，不发
    python twinkle.py COM16 2    # 需要的话垫 2 个 EE（1.0 秒静音）
    python twinkle.py COM16 0    # 默认，不垫
"""

import sys
import time

REST  = 0xEE         # 休止符：占一拍(0.5s)，不发声
CLR   = 0xAA         # 清空：写指针归零
FLUSH = 0xFF         # 板子收到 FF 会清 BRAM + 停播（用来冲掉脏字节）
N_FLUSH = 16         # 前导冲刷几个 FF

# ============================================================
# 小星星（每小节之间插一个 EE 换气）
#   1 1 5 5 6 6 5 | R | 4 4 3 3 2 2 1 | R | 5 5 4 4 3 3 2 | R
#   5 5 4 4 3 3 2 | R | 1 1 5 5 6 6 5 | R | 4 4 3 3 2 2 1
# ============================================================
SONG = [
    0x05, 0x05, 0x09, 0x09, 0x0A, 0x0A, 0x09, REST,
    0x08, 0x08, 0x07, 0x07, 0x06, 0x06, 0x05, REST,
    0x09, 0x09, 0x08, 0x08, 0x07, 0x07, 0x06, REST,
    0x09, 0x09, 0x08, 0x08, 0x07, 0x07, 0x06, REST,
    0x05, 0x05, 0x09, 0x09, 0x0A, 0x0A, 0x09, REST,
    0x08, 0x08, 0x07, 0x07, 0x06, 0x06, 0x05,
]


def build(lead=0):
    """拼出实际要发的字节流

    [16×FF 冲刷] [AA 清指针] [EE ×lead 前导静音] [乐谱] [FF 开播]

    ★ 为什么要垫 EE：蓝牙刚连上/串口刚打开时的脏字节，板子已经收进 BRAM，
      冲刷能清掉绝大部分，但偶尔还剩一点，而它和第一个音同时响就很难听。
      垫 2 个 EE 就是 1 秒静音，让那点残留落在静音里，听感上就干净了。
    """
    lead = max(0, int(lead))
    return (bytes([FLUSH] * N_FLUSH) + bytes([CLR])
            + bytes([REST] * lead) + bytes(SONG) + bytes([FLUSH]))


def main():
    n_note = sum(1 for b in SONG if b != REST)
    n_rest = len(SONG) - n_note
    print("小星星：%d 个音 + %d 个休止符，共 %d 字节"
          % (n_note, n_rest, len(SONG)))
    print("  " + " ".join("%02X" % b for b in SONG) + " FF")

    if len(sys.argv) < 2:
        # ★ IDLE 里按 F5 走这里：直接问 COM 口，不用命令行
        try:
            port = input("\nCOM 口（直接回车 = 只预览不发）: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[OK] 只预览，没有发送。")
            return 0
        if not port:
            print("[OK] 只预览，没有发送。")
            return 0
        sys.argv = [sys.argv[0], port]
    port = sys.argv[1]
    lead = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    data = build(lead)

    print("\n实际发送 %d 字节：" % len(data))
    print("  前导冲刷 : %d×FF + 1×AA" % N_FLUSH)
    print("  前导静音 : %d×EE = %.1f 秒" % (lead, lead * 0.5))
    print("  乐谱     : %d 字节" % len(SONG))
    print("  开播     : 1×FF")
    print("  → 第一个音大约在 %.1f 秒后响" % ((lead + 1) * 0.5))

    import serial
    ser = serial.Serial(port, 9600, timeout=1, write_timeout=3)
    time.sleep(1.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.1)
    ser.reset_input_buffer()

    for i in range(0, len(data), 64):
        ser.write(data[i:i + 64])
        ser.flush()
        time.sleep(0.01)
    ser.close()
    print("\n[OK] 已发给 %s，板子开始演奏" % port)
    return 0


if __name__ == "__main__":
    sys.exit(main())
