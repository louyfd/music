# -*- coding: utf-8 -*-
"""midi2seq.py —— 把任意 .mid 转成板子的字节序列并发过去

以后换曲子只需要这一条命令，不用再问人。

用法
────
    python midi2seq.py 歌.mid                 只打印字节序列（不发）
    python midi2seq.py 歌.mid COM19           转好直接发
    python midi2seq.py 歌.mid out.bin         把二进制包存成文件（含 AA/FF）
    python midi2seq.py 歌.mid COM19 off       一个休止符都不插
    python midi2seq.py 歌.mid COM19 every:7   每 7 个音插一个休止符
                                               （给 test.mid 那种严丝合缝的用）

参数顺序：<文件.mid> [COM口 | 输出文件.bin] [休止符模式]
    休止符模式：auto（默认）| off | every:N
      auto = 小节线上插一个 EE + 音符间空拍补 EE（最多 2 个）

字节规则（详见 协议说明.md）
────────────────────────────
    0x00~0x23  一个音，字节值 = 音高序号 = MIDI音高 - 48
               0 = A2(110Hz)  12 = A3(220Hz)  24 = A4(440Hz)  35 = G#5
    0xEE       休止符，占一拍
    0xAA       清空：丢已收内容、停播（发送前先来一个，冲掉残留）
    0xFF       帧尾：板子收到就从头开始放

板子播放：每 0.5 秒走一个字节（音或休止符），一帧最多 256 字节。
"""

import os
import sys
import time

REST = 0xEE
CLR = 0xAA
END = 0xFF
KEY_MIN, KEY_MAX = 0, 35
MAX_FRAME = 256
N_FLUSH = 16          # 前导冲刷几个 FF
LEAD_REST = 0         # 乐谱前面垫几个 EE（默认 0；板子已修好空帧误发声，通常不需要）

# 板子音高序号 -> 音名（delta[] 表：bit0 = A2 = 110Hz）
_NAMES = ["A2", "A#2", "B2", "C3", "C#3", "D3", "D#3", "E3", "F3", "F#3", "G3", "G#3",
          "A3", "A#3", "B3", "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4",
          "A4", "A#4", "B4", "C5", "C#5", "D5", "D#5", "E5", "F5", "F#5", "G5", "G#5"]


# ============================================================
# 1) 读 .mid
# ============================================================
def load_midi(path):
    """返回 (events, info)
       events = [(起始秒, 时长秒, key), ...]  key 已 -48，只保留 0~35
       info   = dict
    """
    try:
        import mido
    except ImportError:
        print("[ERR] 缺 mido：pip install mido")
        sys.exit(1)

    mid = mido.MidiFile(path)

    # ★ 单位换算：直接遍历 track 时 msg.time 是 tick，不是秒。
    #   秒 = tick / ticks_per_beat × (tempo / 1e6)，tempo 默认 500000us = 120bpm。
    #   ★ delta 必须【逐轨】累加再合并：直接 `for msg in mid` 时 mido 会把多轨
    #     合并，msg.time 的语义就不是单轨内的 delta，累加出来会串味。
    tpb = mid.ticks_per_beat or 480
    us_per_beat = 500000          # 默认 120 bpm
    tick2sec = lambda t: t / tpb * us_per_beat / 1e6

    msgs = []
    for ti, track in enumerate(mid.tracks):
        t = 0.0
        for msg in track:
            t += msg.time
            msgs.append((t, ti, msg))
    msgs.sort(key=lambda x: (x[0], x[1]))

    events = []
    pending = {}          # (track, channel, note) -> start_sec
    skipped = 0
    beats_per_bar = 4     # 默认 4/4

    for t_abs, ti, msg in msgs:
        if msg.type == "set_tempo":
            us_per_beat = msg.tempo
            tick2sec = lambda t, _u=us_per_beat: t / tpb * _u / 1e6
        if msg.type == "time_signature":
            # 拍数 = 分子 × (4/分母)：6/8 -> 3 拍，3/4 -> 3 拍
            try:
                beats_per_bar = max(1, int(round(msg.numerator * 4.0 / msg.denominator)))
            except Exception:
                pass
        if msg.type == "note_on" and msg.velocity > 0:
            pending[(ti, msg.channel, msg.note)] = tick2sec(t_abs)
        elif msg.type == "note_off" or (msg.type == "note_on" and msg.velocity == 0):
            k = (ti, msg.channel, msg.note)
            if k in pending:
                t0 = pending.pop(k)
                key = msg.note - 48
                if KEY_MIN <= key <= KEY_MAX:
                    events.append((t0, tick2sec(t_abs) - t0, key))
                else:
                    skipped += 1

    # 没收尾的（文件截断）按 0.3s 算
    for (ti, ch, note), t0 in pending.items():
        key = note - 48
        if KEY_MIN <= key <= KEY_MAX:
            events.append((t0, 0.3, key))
        else:
            skipped += 1

    events.sort(key=lambda e: e[0])
    info = dict(length=mid.length, skipped=skipped, bar_beats=beats_per_bar)
    return events, info


# ============================================================
# 2) 转字节序列
# ============================================================
def to_bytes(events, rest_mode="auto", info=None):
    """把音符事件转成板子的字节序列（不含 AA / FF）

    一个字节 = 一拍 = 0.5s（板子的节拍），音符起始拍位 = round(t0 / 0.5)。

    rest_mode 决定 EE 休止符怎么插：
      "auto"   默认。① 小节线上插一个 EE（靠 mid 的时间签名/拍号判断）
                      ② 音符之间空出来的拍位也补 EE（最多补 2 个）
      "off"    一个休止符都不插
      "every:N"  每 N 个音后面插一个 EE（对"完全没有空拍"的 midi 有用，
                 比如 test.mid 那种严丝合缝的，想要停顿就得用这个）

    ★ 连着的同音（12 12）必须保留 —— 板子靠"松手再按"重新起音，
      去重会让小星星的 05 05 变成一个音。
    ★ 落在同一拍位的多个音（和弦/多声部）只留第一个，板子是单音。
    """
    if rest_mode == "off":
        every_n, use_auto = 0, False
    elif isinstance(rest_mode, str) and rest_mode.startswith("every:"):
        every_n = max(1, int(rest_mode.split(":", 1)[1]))
        use_auto = False
    else:
        every_n, use_auto = 0, True

    beats_per_bar = 4
    if info:
        beats_per_bar = max(1, int(round(info.get("bar_beats", 4))))

    seq, last_slot, last_bar, last_beats = [], None, None, 1

    for t0, dur, key in events:
        slot = int(round(t0 / 0.5))
        bar = slot // beats_per_bar          # 4/4 拍：4 拍一小节
        # ★ 这个音实际占了几个拍位。1 拍 = 0.5s。
        #   "12 12" 这种连音如果每下都只占 1 拍，就说明 2 拍被占满，没有空拍。
        beats = max(1, int(round(dur / 0.5)))

        if use_auto:
            if last_slot is not None and slot != last_slot:
                gap = max(0, slot - last_slot - 1)
                if gap > 0 or bar != last_bar:
                    # ★ 只有"上一拍确实空着"才补 EE：
                    #   下一拍与上一拍紧邻，就不可能是空拍。
                    if slot > last_slot + last_beats:
                        seq.append(REST)
                        if slot > last_slot + last_beats + 1:
                            seq.append(REST)     # 还很空的话最多再补 1 个
        elif every_n:
            n_note_done = sum(1 for b in seq if b != REST)
            if last_slot is not None and n_note_done % every_n == 0:
                seq.append(REST)

        if slot == last_slot:
            continue          # 同一拍位的第二个音（和弦）跳过
        seq.append(key)
        last_slot, last_bar, last_beats = slot, bar, beats

    return seq


# ============================================================
# 3) 发送
# ============================================================
def send(port, seq, quiet=False):
    import serial

    # ★ 前导冲刷（顺序不能反）+ 前导静音：
    #   1) 16 个 FF —— 板子收到 FF 会 frame_len<=0（停播）+ 清 BRAM。
    #      蓝牙刚连上/串口刚打开时线上会有被截断的半个字节，被接收状态机
    #      当成假字节收进 BRAM，这就是"开头有杂音"的根源。
    #   2) 一个 AA —— 清写指针，从 0 开始装新乐谱。
    #   3) 2 个 EE —— 乐谱前面垫 1 秒静音，万一还剩一点残留，让它落在
    #      静音里，不会和第一个音同时响。
    #   4) 乐谱 + 结尾 FF —— 开播。
    flush = bytes([0xFF] * N_FLUSH)
    lead = bytes([REST] * LEAD_REST)
    payload = flush + bytes([CLR]) + lead + bytes(seq) + bytes([END])

    ser = serial.Serial(port, 9600, timeout=1, write_timeout=3)
    time.sleep(1.5)                     # 等蓝牙链路起来
    ser.reset_input_buffer()            # 扔掉上一次会话的残留 / +CONNECTED
    ser.reset_output_buffer()
    time.sleep(0.1)
    ser.reset_input_buffer()            # 再冲一次（有的口要两次）

    if not quiet:
        print(f"[OK] {port} 已打开 @ 9600 8N1，缓冲已清空")
        print(f"     前导: {N_FLUSH}×FF 冲刷 + 1×AA 清指针 "
              f"+ {LEAD_REST}×EE 静音({LEAD_REST*0.5:.1f}s)")
    for i in range(0, len(payload), 64):
        ser.write(payload[i:i + 64])
        ser.flush()
        time.sleep(0.01)
    ser.close()
    if not quiet:
        print(f"[OK] 已发送 乐谱 {len(seq)} 字节 + 帧尾 FF")
        print(f"     第一个音大约 {(LEAD_REST + 1) * 0.5:.1f} 秒后响")


# ============================================================
def main_idle():
    """不带命令行参数时走这里 —— IDLE 里按 F5 就是这个路径"""
    print("=" * 60)
    print(" .mid -> 板子（IDLE 模式）")
    print("=" * 60)
    try:
        midfile = input("MIDI 文件路径: ").strip().strip('"')
    except (EOFError, KeyboardInterrupt):
        print()
        return 1
    if not midfile or not os.path.isfile(midfile):
        print(f"[ERR] 找不到文件: {midfile}")
        return 1
    try:
        port = input("COM 口（直接回车 = 只预览不发）: ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        port = ""
    sys.argv = [sys.argv[0], midfile] + ([port] if port else [])
    return main()


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        if len(sys.argv) < 2:
            return main_idle()          # ★ IDLE 按 F5 走这里
        print(__doc__)
        return 0
    midfile = sys.argv[1]
    target = sys.argv[2] if len(sys.argv) > 2 else None
    rest_mode = sys.argv[3] if len(sys.argv) > 3 else "auto"

    if not os.path.isfile(midfile):
        print(f"[ERR] 找不到文件: {midfile}")
        return 1

    events, info = load_midi(midfile)
    if not events:
        print(f"[ERR] {midfile} 里没有落在 MIDI 48~83（键号 0~35）的音符。")
        print("      换个八度，或在 mido 里先把音符整体 ±12 挪进来。")
        return 1

    seq = to_bytes(events, rest_mode, info)

    n_note = sum(1 for b in seq if b != REST)
    n_rest = len(seq) - n_note
    print(f"[OK] {os.path.basename(midfile)}: 原长 {info['length']:.2f}s")
    print(f"     {n_note} 个音 + {n_rest} 个休止符 = {len(seq)} 字节"
          f"（超出音域被丢 {info['skipped']} 个）")
    if len(seq) > MAX_FRAME:
        print(f"[WARN] 超过板子 1 帧上限 {MAX_FRAME} 字节，会截到 {MAX_FRAME}")
        seq = seq[:MAX_FRAME]
        n_note = sum(1 for b in seq if b != REST)
        n_rest = len(seq) - n_note
        print(f"       截断后：{n_note} 音 + {n_rest} 休止符")

    print("HEX : " + " ".join("%02X" % b for b in seq))
    print(f"总时长 ≈ {len(seq) * 0.5:.1f} 秒")

    if target is None:
        print("\n（没给 COM 口 / 输出文件，只打印。要发就："
              f"python midi2seq.py {os.path.basename(midfile)} COM19）")
        return 0

    if target.upper().endswith(".BIN"):
        with open(target, "wb") as f:
            f.write(bytes([0xFF] * N_FLUSH) + bytes([CLR])
                    + bytes([REST] * LEAD_REST) + bytes(seq) + bytes([END]))
        print(f"\n[OK] 二进制包已写入 {target}")
        print(f"     （{N_FLUSH}×FF 冲刷 + AA + {LEAD_REST}×EE 静音"
              f"({LEAD_REST*0.5:.1f}s) + {len(seq)} 字节乐谱 + FF）")
    else:
        send(target, seq)
    return 0


if __name__ == "__main__":
    sys.exit(main())
