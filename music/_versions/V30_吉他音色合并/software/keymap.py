# -*- coding: utf-8 -*-
"""keymap.py —— 演奏按键自定义映射（按键 -> 音符/声部）

板子那边（top.v + uart_packet_echo.v）：
  · 物理键 p 按下后驱动哪个【声部】由一张表 key_map[0..35] 决定，默认恒等
  · 声部 n 的频率永远是常数 delta[n] —— 所以滑音/移调那一整套一个字没改
  · 写一条映射 = 发 [0xC6][键号][音符]，不受开窗限制，随时可写
  · 音符给 >=36 的值（用 0x3F）表示【禁用这个键】

★ 这张表只影响【手弹】。乐谱播放走的是 midi_keys，本来就是声部号，
  所以同一份谱子不管映射怎么改，弹出来还是谱子上写的音；
  录音回传采的也是声部掩码，电脑合成 MIDI 时也不用套表。
  换句话说：电脑这边【不需要】按映射反查，这个模块只负责编辑和下发。
"""

from __future__ import annotations

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fpga_proto as P                                    # noqa: E402

# ---- 协议字节（要和 uart_packet_echo.v 里的 localparam 一致）----
KEYMAP_SET = 0xC6      # 写一条映射：+键号 +音符
STOP_BYTE = 0xC7       # 停止演奏（保留乐谱，可重播）
REPLAY_BYTE = 0xC8     # 重播板内已存的那一帧

N_KEYS = P.N_KEYS
DISABLE = 0x3F         # "禁用这个键"用的哨兵值（>=36 即可）


def default_map():
    """默认恒等：键号 n 就发声符 n（= 加这个功能之前的行为）"""
    return list(range(N_KEYS))


def is_default(m):
    return list(m) == default_map()


def norm(m):
    """把任意输入规整成 36 个 0..35 或 DISABLE 的整数。

    ★ 取值规则（要有唯一答案，不能"越界当禁用、非数字当默认"这样两套）：
        · 0..35        -> 就是它
        · DISABLE(63)  -> 禁用（这是【显式】表达的"禁用"）
        · 其它任何值    -> 退回【恒等默认】
      为什么非法值退回默认而不是禁用：配置文件坏掉时，宁可那个键还能弹，
      也不要悄无声息地把一排键禁掉 —— 后者排查起来毫无线索。
    """
    out = default_map()
    if not m:
        return out
    for i in range(min(N_KEYS, len(m))):
        try:
            v = int(m[i])
        except (TypeError, ValueError):
            continue                      # 非法 -> 保持恒等默认
        if v == DISABLE:
            out[i] = DISABLE
        elif 0 <= v < N_KEYS:
            out[i] = v
        # 其它越界值 -> 也保持恒等默认
    return out


def name_of(v):
    """目标值 -> 给界面看的名字"""
    if v == DISABLE or not (0 <= v < N_KEYS):
        return "禁用"
    return "%d %s" % (v, P.key_name(v))


def choices():
    """下拉框的候选（下标就是值；最后一项是禁用）"""
    out = ["%d %s" % (i, P.key_name(i)) for i in range(N_KEYS)]
    out.append("禁用")
    return out


def choice_to_value(idx):
    return DISABLE if idx >= N_KEYS else idx


def value_to_choice(v):
    return N_KEYS if (v == DISABLE or not (0 <= v < N_KEYS)) else v


# ----------------------------------------------------------------------
# 编码
# ----------------------------------------------------------------------
def encode_set(key, note):
    """一条映射 -> 3 个字节"""
    if not (0 <= key < N_KEYS):
        raise ValueError("键号 %r 超出 0..%d" % (key, N_KEYS - 1))
    n = DISABLE if (note == DISABLE or not (0 <= note < N_KEYS)) else note
    return bytes([KEYMAP_SET, key & 0xFF, n & 0xFF])


def encode_all(m):
    """整张表 -> 字节（36 x 3 = 108 字节）"""
    out = bytearray()
    m = norm(m)
    for k in range(N_KEYS):
        out += encode_set(k, m[k])
    return bytes(out)


def encode_reset():
    """恢复默认：把恒等表整张发一遍"""
    return encode_all(default_map())


def describe(m):
    """人看的一行摘要"""
    m = norm(m)
    if is_default(m):
        return "按键映射：默认（键号 n 就发音符 n）"
    # ★ 禁用单独算一类，不要既数进"被改过"又数进"被禁用"（那样提示会重复计数）
    disabled = [k for k in range(N_KEYS) if m[k] == DISABLE]
    changed = [(k, m[k]) for k in range(N_KEYS) if m[k] != k and m[k] != DISABLE]
    parts = []
    if changed:
        parts.append("%d 个键被改过" % len(changed))
    if disabled:
        parts.append("%d 个键被禁用" % len(disabled))
    return "按键映射：" + ("，".join(parts) if parts else "默认")


def detail(m, per_line=4):
    """逐键列出（只列非恒等的，界面和日志用）"""
    m = norm(m)
    out = []
    for k in range(N_KEYS):
        if m[k] != k:
            out.append("键%-2d(%s) -> %s" % (k, P.key_name(k), name_of(m[k])))
    if not out:
        return "  （全默认）"
    lines = []
    for i in range(0, len(out), per_line):
        lines.append("  " + " | ".join(out[i:i + per_line]))
    return "\n".join(lines)


# ----------------------------------------------------------------------
def _selftest():
    ok = 0
    fail = []

    def check(name, cond, extra=None):
        nonlocal ok
        if cond:
            ok += 1
            print("  ok   %s" % name)
        else:
            fail.append(name)
            print("  FAIL %s%s" % (name, ("  -> " + repr(extra)) if extra is not None else ""))

    print("[1] 默认表是恒等（加这功能之前的行为）")
    d = default_map()
    check("长度 36", len(d) == 36)
    check("恒等", all(d[i] == i for i in range(36)))
    check("is_default 认得出来", is_default(d))
    check("describe 说是默认", "默认" in describe(d))

    print("[2] 编码：一条 = 3 字节，且顺序是 [0xC6][键][音]")
    b = encode_set(0, 7)
    check("长度 3", len(b) == 3, b)
    check("命令字 0xC6", b[0] == 0xC6, b[0])
    check("键号在前、音符在后", (b[1], b[2]) == (0, 7), b)

    print("[3] 禁用：用 >=36 的哨兵值")
    b = encode_set(5, DISABLE)
    check("禁用编成 0x3F", b[2] == 0x3F, b)
    check("name_of 显示「禁用」", name_of(DISABLE) == "禁用")
    check("越界的音符也当禁用", encode_set(5, 99)[2] == 0x3F)

    print("[4] 整张表 108 字节，36 条按序")
    m = default_map()
    m[0] = 7
    m[1] = DISABLE
    blob = encode_all(m)
    check("36 x 3 = 108 字节", len(blob) == 108, len(blob))
    check("第 0 条是 键0->7", tuple(blob[0:3]) == (0xC6, 0, 7), blob[0:3])
    check("第 1 条是 键1->禁用", tuple(blob[3:6]) == (0xC6, 1, 0x3F), blob[3:6])
    check("第 2 条是 键2->2（恒等）", tuple(blob[6:9]) == (0xC6, 2, 2), blob[6:9])
    check("最后一条是 键35->35", tuple(blob[-3:]) == (0xC6, 35, 35), blob[-3:])
    # 每一帧的命令字都在
    check("每条都带 0xC6", all(blob[i * 3] == 0xC6 for i in range(36)))

    print("[5] ★ 两个操作数不可能撞上控制字节（这是设计性质，不是巧合）")
    #   键号与音符都被限在 0..35 / 0x3F，最大 0x3F；
    #   而所有控制字节都 >= 0x5A。所以这条命令天然不会和帧头/帧尾/和弦那些撞车。
    seen = set()
    for k in range(N_KEYS):
        for n in list(range(N_KEYS)) + [DISABLE]:
            b = encode_set(k, n)
            seen.add(b[1])
            seen.add(b[2])
    check("操作数最大只到 0x3F", max(seen) == 0x3F, hex(max(seen)))
    check("最小的控制字节是 0x5A，中间有 0x40~0x59 的余量",
          max(seen) < 0x5A, (hex(max(seen)), hex(0x5A)))
    # 板子那边仍然用 map_left 计数器挡着（防御性），但即使没有计数器也不会误判
    check("命令字本身固定是 0xC6，不会被操作数带偏", encode_set(0, 0)[0] == 0xC6)
    blob = encode_all([DISABLE] * 36)
    check("全禁用表里每 3 字节一组、不夹别的字节",
          all(blob[i * 3] == 0xC6 for i in range(36)) and len(blob) == 108)

    print("[6] 规整 norm()：越界/垃圾值都变禁用，长度不足补恒等")
    check("长度不足补恒等", norm([7])[1] == 1, norm([7])[1])
    check("超范围退回恒等（不是悄悄禁键）", norm([99])[0] == 0, norm([99])[0])
    check("负数也退回恒等", norm([-1])[0] == 0, norm([-1])[0])
    check("垃圾值也退回恒等（配置坏掉不该把键禁掉）", norm(["x"])[0] == 0, norm(["x"])[0])
    check("显式 DISABLE 才算禁用", norm([DISABLE])[0] == DISABLE, norm([DISABLE])[0])
    check("None 得到默认表", norm(None) == default_map())
    check("多给的部分忽略", len(norm(list(range(100)))) == 36)

    print("[7] 界面取值来回一致")
    for v in list(range(36)) + [DISABLE]:
        check("值 %s 经下拉框来回不变" % name_of(v),
              choice_to_value(value_to_choice(v)) == v, v)
    check("禁用对应最后一个选项", value_to_choice(DISABLE) == N_KEYS)
    check("候选表 37 项（36 个音 + 禁用）", len(choices()) == 37, len(choices()))

    print("[8] 摘要与明细")
    m = default_map()
    m[5] = 29        # 键5(A2) 改成 A4
    m[6] = DISABLE
    check("describe 报出「1 个键被改过」", "1 个键被改过" in describe(m), describe(m))
    check("describe 报出「1 个键被禁用」", "1 个键被禁用" in describe(m))
    det = detail(m)
    check("明细里有 键5 -> 29 A4", "键5" in det and "A4" in det, det)
    check("明细里有 键6 -> 禁用", "键6" in det and "禁用" in det, det)
    check("全默认时明细写「全默认」", "全默认" in detail(default_map()))

    print("[9] 停止/重播两个字节（要和 RTL 一致）")
    check("停止 = 0xC7", STOP_BYTE == 0xC7)
    check("重播 = 0xC8", REPLAY_BYTE == 0xC8)
    check("三个新字节互不相同、也不和已有的撞",
          len({KEYMAP_SET, STOP_BYTE, REPLAY_BYTE, 0x5A, 0x5B, 0x5C, 0x5D,
               0xAA, 0xEE, 0xFF, 0xC0, 0xC1, 0xC2, 0xC4, 0xC5}) == 15)

    print()
    print("=" * 62)
    if fail:
        print("失败 %d 项 / 通过 %d 项" % (len(fail), ok))
        for n in fail:
            print("   - %s" % n)
        return 1
    print("ALL CHECKS PASSED  (%d 项)" % ok)
    return 0


if __name__ == "__main__":
    import io
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8",
                                      errors="replace", line_buffering=True)
    except Exception:                                       # noqa: BLE001
        pass
    sys.exit(_selftest())
