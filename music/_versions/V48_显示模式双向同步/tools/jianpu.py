# -*- coding: utf-8 -*-
"""jianpu.py —— 简谱 -> 板子字节 / MIDI

简谱写法和 word 里一样，直接敲：

    1 2 3 4 5 6 7      中音 do re mi fa sol la si
    1. 2. 3.           高音（数字后面跟一个点）
    1, 2, 3,           低音（数字后面跟一个逗号）
    0                  休止符
    1_ 1__             加一条下划线 = 延长 1 拍，再加一条再多 1 拍
                       例：1_ = 2 拍，1__ = 3 拍
    1- 1--             也可以直接用减号（1- = 2 拍，1-- = 3 拍）
    |                  小节线（只是好看，不发声；换行、空格同样分隔）

板子每拍 0.5 秒：一个数字 = 1 拍 = 0.5s。延长写在音符上，
发出的是"一个音 + 后面补休止"，和板子的固定节拍正好对上。

用法
────
    python jianpu.py "1 1 5 5 6 6 5 | 4 4 3 3 2 2 1"          只试听/预览，不发
    python jianpu.py "..." COM16                              转好直接发
    python jianpu.py 曲子.txt COM16                            从文件读简谱
    python jianpu.py 曲子.txt out.mid                          存成标准 MIDI
    echo 1 1 5 5 ^| 4 4 3 3 | python jianpu.py - COM16          从管道读

音域：低音区~高音区共 3 个八度，正好覆盖板子 36 键（A2 ~ G#5）。
超出会被夹到最近的一端，并在末尾提示。
"""

import os
import sys

# ============================================================
# 板子音域：键号 0~35，0 = 最低 = A2(110Hz)，24 = A4(440Hz)
# 音名按"中央 C = C4"的常规记法（板子实际发声以 delta[] 表为准）
# ============================================================
KEY_NAMES = ["A2", "A#2", "B2", "C3", "C#3", "D3", "D#3", "E3", "F3", "F#3", "G3", "G#3",
             "A3", "A#3", "B3", "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4",
             "A4", "A#4", "B4", "C5", "C#5", "D5", "D#5", "E5", "F5", "F#5", "G5", "G#5"]

# 板子键号 -> 标准 MIDI 音高（只用于导出 .mid）
#   板子的 0 号键实际发 110Hz；按"中央 C=60"的正规记法是 A2 = MIDI 45，
#   但本工具沿用你上位机一直用的 -48 约定，导出 .mid 时给 48，这样
#   midi2seq.py 再把 48 减回来正好是 0 号键。
KEY_TO_MIDI = [k + 48 for k in range(36)]

REST_BYTE = 0xEE
CLR_BYTE = 0xAA
END_BYTE = 0xFF
N_FLUSH = 16

# 简谱数字 1~7 -> 中音区在板子上的键号
#   ★★ 默认 1 = 板子键号 5 = D3 = 147Hz ★★
#   这个默认值和 bluetooth\twinkle.py 里那首小星星（05 05 09 09 0A 0A 09…）
#   完全一致 —— 两个入口发出来的音高一样，不会一边对一个不对。
#      中音 1 = 5  (D3)    2=6  3=7  4=8  5=9  6=10  7=11
#      高音 1. = 17       低音 1, = -7（超出板子，会被夹到 0 号键）
#   ★ 想要标准音高（中音 1 = 中央 C = 262Hz）就在谱子开头写一行 #base=15，
#     那样中音 1=15、高音 1.=27、低音 1,=3，三个八度都够用。
DIGIT_BASE = {1: 5, 2: 6, 3: 7, 4: 8, 5: 9, 6: 10, 7: 11}


def parse(text, verbose=True, base=None):
    """简谱文本 -> [(键号, 拍数), ...]，键号 None 表示休止符

    base = 中音"1"对应的板子键号。默认 5（=D3），所以
      1=D3  7=G#3   1.=D4（高八度）  1,=A2（低八度）
    想让"1"变成别的音，可以在谱子开头写一行 #base=15（=C4）。

    小节线 | 会额外插一个休止符（换气），想连奏就别写 |。
    行首带 # 的行是设置行，不发声。
    """
    if base is None:
        base = DIGIT_BASE[1]

    # 记号：数字 + 可选八度点/逗号 + 可选延长符
    #   | 单独处理：它要产生一个"换气"休止符
    tokens = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("#"):
            for item in line[1:].replace(",", " ").split():
                if "=" in item:
                    k, v = item.split("=", 1)
                    if k.strip().lower() in ("base", "基准") and v.strip().lstrip("-").isdigit():
                        base = int(v.strip())
                        if verbose:
                            print(f"  [设置] 中音 1 = 板子键号 {base}"
                                  f"（={KEY_NAMES[max(0,min(35,base))]}）")
            continue
        for raw in line.replace("|", " | ").replace("\n", " ").split():
            raw = raw.replace("．", ".").replace("，", ",").replace("－", "-").replace("＿", "_")
            if raw:
                tokens.append(raw)

    out = []
    for tok in tokens:
        # ---- 小节线：插一个换气休止符 ----
        if tok == "|":
            if out and out[-1][0] is not None:      # 连着两个 | 只算一个
                out.append((None, 1))
            continue

        if not tok[0].isdigit():
            if verbose:
                print(f"  [跳过] 看不懂的记号: {tok!r}")
            continue
        digit = int(tok[0])
        if digit > 7:
            if verbose:
                print(f"  [跳过] 简谱只有 0~7: {tok!r}")
            continue

        rest = tok[1:]
        octave = 0
        for ch in rest:
            if ch == ".":
                octave += 1
            elif ch == ",":
                octave -= 1
            elif ch in "_-":
                pass
            else:
                if verbose:
                    print(f"  [跳过] 记号 {tok!r} 里的 {ch!r} 看不懂")
        dur = 1 + rest.count("_") + rest.count("-")

        if digit == 0:
            out.append((None, dur))
            continue

        # 键号 = 基准 + 八度偏移 + 音级偏移（基准默认 5 = 中音 1 = D3）
        key = base + (DIGIT_BASE[digit] - DIGIT_BASE[1]) + octave * 12
        if key < 0 or key > 35:
            clamped = max(0, min(35, key))
            if verbose:
                print(f"  [音域] {tok!r} -> 键号 {key} 超出 0~35，夹到 {clamped}"
                      f"({KEY_NAMES[clamped]})")
            key = clamped
        out.append((key, dur))
    return out


def to_bytes(seq, lead=0):
    """[(键号, 拍数)] -> 板子字节序列（不含前导/帧尾）

    每个音只发一次（占 1 拍），多出来的拍数补 EE 休止符。
    """
    body = []
    for key, dur in seq:
        body.append(REST_BYTE if key is None else key)
        body.extend([REST_BYTE] * (dur - 1))
    return bytes([REST_BYTE] * lead) + bytes(body)


def preview(seq):
    print("\n简谱预览（每拍 0.5 秒）：")
    line1, line2 = [], []
    n = 0
    for key, dur in seq:
        name = "休止" if key is None else KEY_NAMES[key]
        line1.append(f"{name:<5}")
        line2.append(f"{dur:<5}")
        n += dur
        if len(line1) == 12:
            print("   " + " ".join(line1))
            print("   " + " ".join(line2))
            line1, line2 = [], []
    if line1:
        print("   " + " ".join(line1))
        print("   " + " ".join(line2))
    n_note = sum(1 for k, _ in seq if k is not None)
    print(f"\n  共 {len(seq)} 个记号：{n_note} 个音 + {len(seq)-n_note} 个休止符")
    print(f"  总拍数 {n} = {n*0.5:.1f} 秒")
    print(f"  字节数 {sum(d for _, d in seq)}（板子一帧上限 256）")


def write_midi(seq, path, tempo_us=500000):
    """导出标准 MIDI：每拍 0.5 秒

    ★ 音长必须写满整拍，不能留缝。板子上位机 midi2seq.py 是靠"音符之间的
      空拍"来插休止符的 —— 音长只写 0.9 拍的话，每个音后面都会多出一拍休止，
      听到的节奏就慢了一倍。
    """
    try:
        import mido
    except ImportError:
        print("[ERR] 导出 MIDI 需要 mido：pip install mido")
        return False

    mid = mido.MidiFile(ticks_per_beat=480)
    tr = mido.MidiTrack()
    mid.tracks.append(tr)
    tr.append(mido.MetaMessage("set_tempo", tempo=tempo_us, time=0))
    tr.append(mido.MetaMessage("time_signature", numerator=4, denominator=4, time=0))

    TPB = 480
    def ticks(beats):
        return max(1, int(round(beats * TPB)))

    # 先攒绝对时间，再统一转 delta，避免 note_off/note_on 落在同一 tick 上
    events = []          # (绝对 tick, 类型, 参数)
    t = 0
    for key, dur in seq:
        if key is None:
            t += ticks(dur)
            continue
        midi_note = KEY_TO_MIDI[key]
        events.append((t, "on", midi_note))
        events.append((t, "off", midi_note))
        t += ticks(dur)                      # ★ 整拍，不留缝
    events.sort(key=lambda e: (e[0], 0 if e[1] == "off" else 1))

    prev = 0
    for at, kind, note in events:
        d = at - prev
        prev = at
        if kind == "on":
            tr.append(mido.Message("note_on", note=note, velocity=80, time=d))
        else:
            tr.append(mido.Message("note_off", note=note, velocity=0, time=d))

    mid.save(path)
    print(f"\n[OK] 已写入 MIDI: {path}")
    print("     键号 0 导出为 MIDI 48，midi2seq.py 减 48 正好还原成 0 号键；")
    print("     每音写满整拍，所以 midi2seq.py 读回来不会多插休止符。")
    return True


def find_tool(name):
    r"""在 tools、music、music\bluetooth 里找一个脚本

    jianpu.py 在 tools 下，midi2seq.py 在 bluetooth 下，所以得找一下。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    up = os.path.dirname(here)
    for c in (os.path.join(here, name),
              os.path.join(up, name),
              os.path.join(up, "bluetooth", name)):
        if os.path.isfile(c):
            return os.path.abspath(c)
    return None


def preview_via_midi2seq(midfile):
    """导出的 .mid 用 midi2seq.py 的同一套逻辑读回来，打印板子字节

    直接 import（不绕 subprocess），这样在管道/批处理里输出不会被吞掉。
    """
    tool = find_tool("midi2seq.py")
    if not tool:
        print("     （没找到 midi2seq.py，跳过字节预览）")
        return
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location("_m2s", tool)
        m2s = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m2s)
        events, info = m2s.load_midi(midfile)
        seq = m2s.to_bytes(events, "auto", info)
    except Exception as e:
        print(f"     [跳过] 读回 .mid 失败: {e}")
        return
    print(f"\n用 midi2seq.py 的逻辑读回来（校验往返一致）：")
    print("   " + " ".join("%02X" % b for b in seq))


def open_serial(port, tries=4):
    """打开串口，失败自动重试

    IDLE 里反复按 F5 时，上一次的串口对象可能还没被回收，Windows 会报
    "拒绝访问"。等半秒重试几次就好。
    """
    import serial
    import time as _t
    last = None
    for k in range(tries):
        try:
            return serial.Serial(port, 9600, timeout=1, write_timeout=3)
        except Exception as e:
            last = e
            print(f"   [重试 {k+1}/{tries}] 打开 {port} 失败: {str(e)[:60]}")
            _t.sleep(0.6)
    raise last


def send(port, body, quiet=False):
    import time
    ser = open_serial(port)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.1)
    ser.reset_input_buffer()
    payload = bytes([0xFF] * N_FLUSH) + bytes([CLR_BYTE]) + body + bytes([END_BYTE])
    for i in range(0, len(payload), 64):
        ser.write(payload[i:i + 64])
        ser.flush()
        time.sleep(0.01)
    ser.close()
    if not quiet:
        print(f"\n[OK] 已发给 {port}（{N_FLUSH}×FF 冲刷 + AA + {len(body)} 字节 + FF）")


def ask(prompt, default=""):
    """IDLE 里能正常用的输入（F5 运行时 print 完提示再等输入）"""
    try:
        s = input(f"{prompt}{' [' + default + ']' if default else ''}: ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return None
    return s if s else default


def main_idle():
    """不带命令行参数时走这里 —— IDLE 里按 F5 就是这个路径"""
    print("=" * 60)
    print(" 简谱 -> 板子（IDLE 模式）")
    print("=" * 60)
    print("""
写法：
   1 2 3 4 5 6 7   中音 do~si        1. 2.  高音
   1, 2,            低音             0      休止
   1_  1__          延长 1 拍 / 2 拍  |      小节线（只为好看）
""")

    mode = ask("怎么给谱子？(1=直接敲  2=从文件读)", "1")
    if mode is None:
        return 1
    if mode.strip() == "2":
        path = ask("简谱文件路径")
        if not path or not os.path.isfile(path):
            print(f"[ERR] 找不到文件: {path}")
            return 1
        with open(path, encoding="utf-8") as f:
            text = f.read()
    else:
        print("（可以直接粘贴多行；空行结束）")
        lines = []
        while True:
            try:
                ln = input("简谱> ")
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not ln.strip():
                break
            lines.append(ln)
        text = "\n".join(lines)

    if not text.strip():
        print("[ERR] 没输入简谱。例子：1 1 5 5 6 6 5")
        return 1

    seq = parse(text)
    if not seq:
        print("[ERR] 没解析出任何音符。例子：1 1 5 5 6 6 5 | 4 4 3 3 2 2 1")
        return 1
    preview(seq)
    body = to_bytes(seq)
    print("\n板子字节：" + " ".join("%02X" % b for b in body))

    port = ask("\n要发给板子吗？填 COM 口（直接回车 = 不发送，只预览）", "")
    if not port:
        print("[OK] 只预览，没有发送。")
        return 0
    try:
        send(port, body)
    except Exception as e:
        print(f"\n[ERR] 发送失败: {e}")
        print("     检查：口对不对（跑 scan_ports.py）、蓝牙是否已连接、"
              "串口助手是不是还占着这个口。")
        return 1
    return 0


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        if len(sys.argv) < 2:
            return main_idle()          # ★ IDLE 按 F5 走这里
        print(__doc__)
        return 0

    src = sys.argv[1]
    if src == "-":
        text = sys.stdin.read()
    elif os.path.isfile(src):
        with open(src, encoding="utf-8") as f:
            text = f.read()
    else:
        text = src          # 直接把简谱写在命令行上

    target = sys.argv[2] if len(sys.argv) > 2 else None

    print("输入简谱：" if not os.path.isfile(src) else f"读入 {src}：")
    for ln in text.strip().splitlines()[:6]:
        print("   " + ln.strip())

    seq = parse(text)
    if not seq:
        print("[ERR] 没解析出任何音符。例子：1 1 5 5 6 6 5 | 4 4 3 3 2 2 1")
        return 1

    preview(seq)
    body = to_bytes(seq)
    print("\n板子字节：" + " ".join("%02X" % b for b in body))

    if target is None:
        print("\n（没给 COM 口 / 输出文件，只预览不发。"
              '要发就：python jianpu.py "1 1 5 5" COM19）')
        return 0

    if target.lower().endswith(".mid"):
        ok = write_midi(seq, target)
        if ok:
            preview_via_midi2seq(target)
        return 0 if ok else 1
    send(target, body)
    return 0


if __name__ == "__main__":
    sys.exit(main())
