# -*- coding: utf-8 -*-
"""test_echo_collision.py —— 【回显撞帧头】这一类 bug 的回归测试

## 这个文件守的是什么

板子会把收到的字节【原样回显】给上位机（协议 v1 就有的行为，0xAA 除外）。
而上位机要在这条混合流里找【板子主动发的状态帧】。于是有一个天生的危险：

    只要上位机【发出去】的字节里出现了一个帧头，
    它的回显就会被上位机当成一帧真状态。

帧头是 0x5A/0x5B/0x5D/0x5E/0x5F。而谱子里有几类字节是**任意值**：
    · 0xC3 的【力度】操作数：0~127      ← 最常见，每事件一个
    · 0xC2 的【时值】操作数：0~255
    · 0xC4/0xC5 的【时间参数】3 字节：任意
所以这不是"罕见数据"，是**必然会撞上**的。

## 真实事故（用户 2026-10-11 实测报的）

> "每次发谱，软件自己就把**混响和合唱**显示成打开，音色有时还变成**笛子**；
>  点一下「刷新状态」就正常了。"

根因：力度 0x5D(=93) 正好撞上帧头，紧接着是和弦头 0xC0：

    C3 5D C0 01 ...   ->  老代码：fx=0xC0（混响+合唱）、tone=0x01（笛子）

本机 MIDI 目录 12 首里有 **10 首**会命中（93 是很常见的力度值）。
修复见 `fpga_rec.py` 里 `_fx_cand`（候选帧）那一段。

## 怎么跑

    python test_echo_collision.py                 # 跑 MIDI 目录里所有谱子
    python test_echo_collision.py <目录或.mid>     # 指定文件/目录
"""

from __future__ import annotations

import glob
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import fpga_rec as R          # noqa: E402
import fpga_midi as M         # noqa: E402
import fpga_proto as P        # noqa: E402

# 板子的回显规则：收到什么回什么，只有 0xAA（开窗）不回显
def board_echo(sent: bytes) -> bytes:
    return bytes(b for b in sent if b != 0xAA)


def parse(sent: bytes):
    """把"上位机发出去的字节"经过板子回显后，喂给解析器。"""
    rx = R.RecParser()
    rx.feed(board_echo(sent))
    return rx


def _selftest(paths):
    ok = 0
    fail = []

    def check(name, cond, extra=None):
        nonlocal ok
        if cond:
            ok += 1
            print("  ok   " + name)
        else:
            fail.append(name)
            print("  FAIL " + name + ("   " + str(extra) if extra is not None else ""))

    print("=" * 78)
    print("[1] ★ 真实谱子：整帧发出去 -> 回显 -> 解析，状态不许被弄脏")
    print("=" * 78)
    n_scores = 0
    for p in paths:
        try:
            _fr, blob, _rep = M.midi_to_frame(p)
        except Exception as exc:                        # noqa: BLE001
            print("  (跳过) %-44s %s" % (os.path.basename(p), exc))
            continue
        n_scores += 1
        rx = parse(blob)
        dirty = (rx.fx_state != 0) or (rx.tone_sel not in (None, 0)) \
            or (rx.rec_on is not None)
        check("%-44s fx=%02X tone=%-4s" % (os.path.basename(p)[:44],
                                           rx.fx_state, rx.tone_sel),
              not dirty,
              "被弄脏了: fx=%02X tone=%s rec_on=%s"
              % (rx.fx_state, rx.tone_sel, rx.rec_on))
    check("至少跑了一首谱子（跑不到就说明路径不对）", n_scores > 0, n_scores)

    print()
    print("=" * 78)
    print("[2] ★★ 穷举【力度】0~255：经过真实编码器发出去，不许弄脏状态")
    #   ★ 必须走 P.pack_vel —— 那才是真正写进串口的东西。
    #     直接手搓 bytes([0xC3, v]) 等于绕过了发送侧的消毒，测的就不是线上行为。
    print("=" * 78)
    check("★ 编码器本身不会发出撞帧头的力度字节（0~127 全扫）",
          all(P.pack_vel(v)[1] not in P.FRAME_HEADS for v in range(0, 128)),
          [v for v in range(0, 128) if P.pack_vel(v)[1] in P.FRAME_HEADS])
    bad = []
    for v in range(256):
        for tail in (b"\xc0\x00\xc1", b"\xc0\x01\x02\xc1", b"\x00", b"\xee"):
            sent = b"\xaa" + P.pack_vel(v) + tail + b"\xff"
            rx = parse(sent)
            if rx.fx_state != 0 or rx.tone_sel not in (None, 0) \
                    or rx.rec_on is not None:
                bad.append((v, tail.hex(), hex(rx.fx_state), rx.tone_sel))
    check("256 个力度 x 4 种后继 = 1024 组，全部不弄脏状态", not bad, bad[:6])

    print()
    print("=" * 78)
    print("[3] ★★ 穷举【时值 / 时间参数 / 节拍器 BPM】")
    print("=" * 78)
    bad2 = []
    # 时值 N：走真实的 _encode_event（发 [0xC2][N]）
    for n in range(1, P.MAX_TICKS):
        sent = b"\xaa" + P.encode_body([P.Event((5,), n + 1)]) + b"\xff"
        rx = parse(sent)
        if rx.fx_state != 0 or rx.tone_sel not in (None, 0):
            bad2.append(("dur", n, hex(rx.fx_state)))
    # 时间参数：走真实的 encode_settings（[0xC4]/[0xC5] + 3 字节）
    for us in list(range(0, 3000)) + [83333, 100000, 200000, 300000, 500000, 1000000]:
        sent = b"\xaa" + P.encode_settings(us, us) + b"\x00\xee\xff"
        rx = parse(sent)
        if rx.fx_state != 0 or rx.tone_sel not in (None, 0):
            bad2.append(("time", us, hex(rx.fx_state)))
    check("时值 255 组 + 时间参数 3006 组，全部不弄脏状态", not bad2, bad2[:6])

    # ★ 节拍器：这一格最阴 —— 板子收到 0xCF 后【紧接着就回一帧 0x5E】，
    #   假帧正好被那一帧"确认"下来（实测过 BPM=93 会中）。
    bad3 = []
    for bpm in range(0, 256):
        sent = (b"\xaa" + P.pack_metro_set(bpm, 4, 1)
                + b"\x5e\x64\x04\x01\x00" + b"\xff")
        rx = parse(sent)
        if rx.fx_state != 0 or rx.tone_sel not in (None, 0):
            bad3.append((bpm, hex(rx.fx_state), rx.tone_sel))
    check("★ 256 个 BPM（含板子紧接着回的 0x5E），全部不弄脏状态",
          not bad3, bad3[:6])
    check("★ BPM=93(0x5D) 会被改成 92 发出去（差 1 BPM，听不出来）",
          P.pack_metro_set(93, 4, 1)[1] == 92, P.pack_metro_set(93, 4, 1).hex(" "))
    check("★ BPM=0（关）不受消毒影响，还是 0",
          P.pack_metro_set(0, 4, 1)[1] == 0, P.pack_metro_set(0, 4, 1).hex(" "))

    print()
    print("=" * 78)
    print("[4] ★ 真帧必须照旧（不能因为加校验就不认板子的状态帧了）")
    print("=" * 78)
    rx = R.RecParser()
    rx.feed(R.pack_disp_frame(2, 1))
    rx.feed(b"\x5d\xc0\x01\x01")
    check("真 0x5D 帧 5D C0 01 01 -> fx=0xC0 / tone=1 / wait_mode=1",
          (rx.fx_state, rx.tone_sel, rx.wait_mode) == (0xC0, 1, 1),
          (hex(rx.fx_state), rx.tone_sel, rx.wait_mode))
    rx2 = R.RecParser()
    rx2.feed(b"\x5b\x01")
    check("真 0x5B 帧照旧认", rx2.rec_on is True, rx2.rec_on)

    print()
    print("=" * 78)
    if fail:
        print("失败 %d 项 / 通过 %d 项" % (len(fail), ok))
        for n in fail:
            print("   - %s" % n)
        return 1
    print("ALL CHECKS PASSED  (%d 项) —— 回显撞帧头这一类已封住" % ok)
    return 0


def _collect(argv):
    if len(argv) > 1:
        out = []
        for a in argv[1:]:
            if os.path.isdir(a):
                out += sorted(glob.glob(os.path.join(a, "*.mid")))
            else:
                out.append(a)
        return out
    here = os.path.dirname(os.path.abspath(__file__))
    out = sorted(glob.glob(os.path.join(here, "MIDI", "*.mid")))
    # 工程里的测试谱子也带上（它们是随版本归档的，最稳）
    sc = os.path.join(here, "..", "..", "FPGA_Gowin", "Gowin",
                      "Gowin_V1.9.11.03_Education_x64", "IDE", "bin",
                      "Documents", "60k", "music_player", "music",
                      "testfiles", "scores")
    out += sorted(glob.glob(os.path.normpath(sc) + os.sep + "*.mid"))
    return out


if __name__ == "__main__":
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8",
                                      errors="replace")
    except Exception:                                   # noqa: BLE001
        pass
    raise SystemExit(_selftest(_collect(sys.argv)))
