# -*- coding: utf-8 -*-
"""spp_assistant 自检脚本 —— 用假串口替身验证收发逻辑, 不碰真实端口。

运行: python selftest.py
退出码 0 表示全部通过。
"""

import os
import sys
import time
import tkinter as tk

# Windows 控制台默认 cp1252, 中文直接抛 UnicodeEncodeError —— 先把输出流改成 UTF-8
try:
    sys.stdout = __import__("io").TextIOWrapper(
        sys.stdout.buffer, encoding="utf-8", errors="replace", line_buffering=True)
    sys.stderr = __import__("io").TextIOWrapper(
        sys.stderr.buffer, encoding="utf-8", errors="replace", line_buffering=True)
except Exception:                                       # noqa: BLE001
    pass

import spp_assistant as app_mod

FAILS = []


def check(name, cond, extra=""):
    print(("  [PASS] " if cond else "  [FAIL] ") + name + ("  " + str(extra) if extra else ""))
    if not cond:
        FAILS.append(name)


# 自检里不该弹出任何对话框(模态框会把测试卡死), 全部换成记录器
POPUPS = []


def _recorder(kind):
    def fn(title, message, **kw):
        POPUPS.append((kind, message))
        return "ok"
    return fn


app_mod.messagebox.showerror = _recorder("error")
app_mod.messagebox.showwarning = _recorder("warning")
app_mod.messagebox.showinfo = _recorder("info")

# 从干净配置开始, 免得上一次残留影响默认值
try:
    os.remove(os.path.join(app_mod.app_dir(), app_mod.CONFIG_NAME))
except OSError:
    pass


# ---------------------------------------------------------------- 纯函数
print("[1] 纯函数")
check("parse_hex '01 02 AB'", app_mod.parse_hex("01 02 AB") == b"\x01\x02\xab")
check("parse_hex '0102ab' 连写", app_mod.parse_hex("0102ab") == b"\x01\x02\xab")
check("parse_hex '0x01,0x02'", app_mod.parse_hex("0x01,0x02") == b"\x01\x02")
check("parse_hex 空串", app_mod.parse_hex("   ") == b"")
for bad, why in (("01 0G", "非法字符"), ("ABC", "奇数长度")):
    try:
        app_mod.parse_hex(bad)
        check("parse_hex {} 应报错".format(why), False)
    except ValueError:
        check("parse_hex {} 应报错".format(why), True)
check("decode_bytes 归一化 CRLF", app_mod.decode_bytes(b"a\r\nb\rc", "UTF-8") == "a\nb\nc")

# ---------------------------------------------------------------- 蓝牙识别
print("[2] 蓝牙端口识别 (SPP.py 的关键字逻辑 + 精确判定)")
from serial.tools import list_ports  # noqa: E402
ports = list(list_ports.comports())
bt = app_mod.find_bluetooth_ports()
print("      全部端口: " + " | ".join(app_mod.port_label(p) for p in ports))
check("find_bluetooth_ports 找到蓝牙口", len(bt) >= 1, "{} 个".format(len(bt)))
check("find_bluetooth_ports 与 comports() 顺序一致(第一个即 SPP.py 会连的)",
      [p.device for p in bt] == [p.device for p in ports if app_mod.is_bluetooth_port(p)])

outgoing = [p for p in bt if app_mod.bluetooth_mac(p)]
check("认出了出站口(带对端 MAC)", len(outgoing) >= 1,
      [p.device for p in outgoing])
if outgoing:
    mac = app_mod.bluetooth_mac(outgoing[0])
    label = app_mod.port_label(outgoing[0])
    check("出站口标签含 [蓝牙/出站]", "[蓝牙/出站]" in label, label)
    check("出站口标签含 MAC", mac in label, mac)
    name = app_mod.bt_device_name(mac)
    check("能从注册表读出对端设备名", bool(name), "{} -> {}".format(mac, name or "(空)"))
    print("      出站口: {}".format(label))
incoming = [p for p in bt if not app_mod.bluetooth_mac(p)]
if incoming:
    check("入站口标签含 [蓝牙/入站]",
          "[蓝牙/入站]" in app_mod.port_label(incoming[0]),
          app_mod.port_label(incoming[0]))


# ---------------------------------------------------------------- 假串口
class FakeSerial(object):
    """假串口替身: 两种读取模式都支持。"""

    def __init__(self, port=None, **kw):
        self.port = port
        self.kw = kw
        self.written = bytearray()
        self.is_open = True
        self.in_waiting = 0
        self._rx = bytearray()

    def write(self, data):
        self.written += data
        return len(data)

    def feed(self, data):
        self._rx += data
        self.in_waiting = len(self._rx)

    def read(self, n=1):
        time.sleep(0.02)
        out = bytes(self._rx[:n])
        del self._rx[:n]
        self.in_waiting = len(self._rx)
        return out

    def readline(self, size=-1):
        time.sleep(0.05)
        if not self._rx:
            return b""
        idx = self._rx.find(b"\n")
        if idx >= 0:
            out = bytes(self._rx[:idx + 1])
            del self._rx[:idx + 1]
        else:
            out = bytes(self._rx)
            del self._rx[:]
        self.in_waiting = len(self._rx)
        return out

    def close(self):
        self.is_open = False


real_serial = app_mod.serial.Serial
app_mod.serial.Serial = FakeSerial


def pump_until(cond, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        root.update()
        if cond():
            return True
        time.sleep(0.02)
    return False


def rx_now():
    return gui.rx_text.get("1.0", "end-1c")


print("[3] 界面与收发 (按行读取模式, 默认)")
root = tk.Tk()
root.withdraw()
gui = app_mod.SppAssistant(root)
gui.autoconn_var.set(False)          # 关掉启动自动连, 免得干扰后续断言
# ★ V31：连上之后程序会自动发一条 0xCB 查询效果/音色状态（界面上的开关要跟板子对齐）。
#   但下面好几节是【逐字节完全一致】的比对（比如"整帧发出去正好 N 个字节"），
#   混进这条查询就会挂 —— 所以自检里统一先关掉它，
#   到 [12] 节再单独打开、单独验它真的会发。
#   （这不是"为了让测试过而改程序"：自动查询本身在 [12] 节有专门的断言。）
gui.fx_auto_query = False
check("端口下拉框有内容", len(gui.port_box.cget("values")) >= 1)
check("默认选中的是蓝牙出站口",
      bool(app_mod.bluetooth_mac(gui.port_info.get(gui.port_var.get()))),
      gui.port_var.get())

gui.bt_only_var.set(True)
gui.refresh_ports()
labels = list(gui.port_box.cget("values"))
check("仅蓝牙过滤后只剩蓝牙口", all("蓝牙" in x for x in labels), labels)
gui.bt_only_var.set(False)
gui.refresh_ports()

check("默认是「按行读取」模式", gui.line_mode_var.get() is True)
gui.open_port()
opened = pump_until(lambda: gui.is_open())
check("串口已打开 (后台线程打开, 不冻结界面)", opened)
check("打开时 timeout=1 (与 SPP.py 一致)",
      opened and gui.ser.kw.get("timeout") == 1.0,
      gui.ser.kw.get("timeout") if opened else None)
fake = gui.ser

# --- 文本发送 + CRLF
gui.hex_send_var.set(False)
gui.lineend_var.set(app_mod.LINEEND_CHOICES[3])
gui.tx_text.delete("1.0", "end")
gui.tx_text.insert("1.0", "AT")
gui.send_once()
check("文本发送到达串口 (AT + CRLF)", pump_until(lambda: bytes(fake.written) == b"AT\r\n"),
      bytes(fake.written))
check("TX 计数正确", pump_until(lambda: gui.tx_bytes == 4), gui.tx_bytes)

# --- HEX 发送
fake.written = bytearray()
gui.hex_send_var.set(True)
gui.tx_text.delete("1.0", "end")
gui.tx_text.insert("1.0", "01 02 AB")
gui.send_once()
check("HEX 发送到达串口", pump_until(lambda: bytes(fake.written) == b"\x01\x02\xab"),
      bytes(fake.written))
gui.hex_send_var.set(False)

# --- 按行读取: 两行应产生两个时间戳(两条一行)
gui.clear_rx()
fake.feed(b"LINE1\r\nLINE2\r\n")
check("按行读取收到两行", pump_until(lambda: "LINE1" in rx_now() and "LINE2" in rx_now()),
      repr(rx_now()))
check("每行各一个时间戳", rx_now().count("]") == 2, repr(rx_now()))
check("行尾被归一成一条一行", "\r" not in rx_now(), repr(rx_now()))

# --- HEX 显示
gui.hex_show_var.set(True)
gui.clear_rx()
fake.feed(b"\x01\x02\r\n")
check("HEX 显示模式", pump_until(lambda: "01 02" in rx_now()), repr(rx_now()))
gui.hex_show_var.set(False)

# --- 暂停显示时仍计数
gui.clear_rx()
gui.pause_var.set(True)
gui.on_pause_toggle()
before = gui.rx_bytes
fake.feed(b"XXXX\r\n")
check("暂停时不显示", not pump_until(lambda: "XXXX" in rx_now(), 0.6))
check("暂停时仍计数", gui.rx_bytes == before + 6, "{} -> {}".format(before, gui.rx_bytes))
gui.pause_var.set(False)
gui.on_pause_toggle()

# --- 定时发送
fake.written = bytearray()
gui.tx_text.delete("1.0", "end")
gui.tx_text.insert("1.0", "T")
gui.timer_ms_var.set("100")
gui.timer_on_var.set(True)
gui.on_timer_toggle()
check("定时发送重复发出", pump_until(lambda: len(fake.written) >= 3, 2.0), len(fake.written))
gui.on_timer_toggle(force_off=True)
check("定时发送已停止", gui.timer_job is None and not gui.timer_on_var.get())

gui.close_port("测试切换模式")

print("[4] 空闲合并模式 (关掉按行读取)")
gui.line_mode_var.set(False)
gui.open_port()
opened2 = pump_until(lambda: gui.is_open())
check("重新打开后 timeout=0.02", opened2 and gui.ser.kw.get("timeout") == 0.02,
      gui.ser.kw.get("timeout") if opened2 else None)
fake2 = gui.ser
gui.clear_rx()
fake2.feed(b"HELLO\r\n")
check("收到数据", pump_until(lambda: "HELLO" in rx_now()), repr(rx_now()))
check("一批只有一个时间戳 (不再被劈开)", rx_now().count("]") == 1, repr(rx_now()))
gui.close_port("测试自动连接")

print("[5] 自动连接蓝牙 (SPP.py 的思路, 但优先出站口)")
ok = gui.auto_connect_bluetooth(silent=True)
check("auto_connect_bluetooth 被受理", ok is True)
check("已连上", pump_until(lambda: gui.is_open()))
if gui.is_open():
    dev = gui.current_device()
    info = gui.port_info.get(gui.port_var.get())
    check("自动连接选的是出站口(带 MAC)", bool(app_mod.bluetooth_mac(info)),
          "{} {}".format(dev, gui.port_var.get()))
gui.close_port("测试结束")

print("[5b] 打开失败的恢复 (模块不在时的 121 路径)")


class FailSerial(object):
    def __init__(self, port=None, **kw):
        raise OSError(22, "信号灯超时时间已到", None, 121)


app_mod.serial.Serial = FailSerial
gui.open_port(silent=True)
check("失败时不会标记成已连接", not pump_until(lambda: gui.is_open(), 1.5))
check("失败后按钮恢复可用",
      pump_until(lambda: str(gui.open_btn.cget("state")) == "normal" and
                 gui.open_btn.cget("text") == "打开串口"),
      "{} / {}".format(gui.open_btn.cget("state"), gui.open_btn.cget("text")))
check("失败后能再次发起打开 (序号机制没卡住)", gui.open_port(silent=True) is True)
pump_until(lambda: str(gui.open_btn.cget("state")) == "normal", 2.0)
app_mod.serial.Serial = FakeSerial

print("[7] MIDI 组装与发送")
check("note_to_number C4 = 60", app_mod.note_to_number("C4") == 60)
check("note_to_number C#4 = Db4 = 61",
      app_mod.note_to_number("C#4") == 61 and app_mod.note_to_number("Db4") == 61)
check("note_to_number 接受纯数字", app_mod.note_to_number("60") == 60)
check("number_to_note 60 = C4", app_mod.number_to_note(60) == "C4")
check("边界 C-1=0 / G9=127",
      app_mod.note_to_number("C-1") == 0 and app_mod.note_to_number("G9") == 127)

check("Note On ch1 -> 90 3C 64",
      app_mod.build_midi("Note On (90)", 1, "C4", "100")[0] == b"\x90\x3c\x64")
check("Note On ch10 -> 99 3C 64",
      app_mod.build_midi("Note On (90)", 10, "C4", "100")[0] == b"\x99\x3c\x64")
check("Control Change -> B0 07 7F",
      app_mod.build_midi("Control Change (B0)", 1, "7", "127")[0] == b"\xb0\x07\x7f")
check("Program Change -> C0 05",
      app_mod.build_midi("Program Change (C0)", 1, "5", "")[0] == b"\xc0\x05")
check("Pitch Bend -> E0 00 40",
      app_mod.build_midi("Pitch Bend (E0)", 1, "0", "64")[0] == b"\xe0\x00\x40")
check("SysEx 自动补 F0/F7",
      app_mod.build_midi("SysEx (F0..F7)", 1, "7D 01 02", "")[0] == b"\xf0\x7d\x01\x02\xf7")
check("实时消息是单字节",
      app_mod.build_midi("系统实时 时钟 (F8)", 1, "", "")[0] == b"\xf8")

for _bad, _why in ((("Note On (90)", 1, "H9", "100"), "非法音名"),
                   (("Note On (90)", 1, "C4", "200"), "力度越界"),
                   (("Note On (90)", 17, "C4", "1"), "通道越界"),
                   (("SysEx (F0..F7)", 1, "80", ""), "SysEx 载荷超过 7 位")):
    try:
        app_mod.build_midi(*_bad)
        check("build_midi {} 应报错".format(_why), False)
    except ValueError:
        check("build_midi {} 应报错".format(_why), True)

check("describe_midi 认得 Note On",
      "Note On" in app_mod.describe_midi(b"\x90\x3c\x64"))

gui.open_port()
check("重新打开串口(供 MIDI 测试)", pump_until(lambda: gui.is_open()))
fake3 = gui.ser
gui.midi_type_var.set("Note On (90)")
gui.midi_ch_var.set("2")
gui.midi_d1_var.set("A4")
gui.midi_d2_var.set("127")
gui.on_midi_change()
check("预览同时显示 hex 和音名",
      "91 45 7F" in gui.midi_preview_var.get() and "A4" in gui.midi_preview_var.get(),
      gui.midi_preview_var.get())

fake3.written = bytearray()
gui.midi_tail_var.set(False)          # 这一段测的是"不加帧尾"的裸 MIDI 语义
gui.send_midi()
check("MIDI 按原始字节发出(不带行尾)",
      pump_until(lambda: bytes(fake3.written) == b"\x91\x45\x7f"), bytes(fake3.written))

gui.fill_midi_hex()
check("填入发送区会打开 HEX 发送", gui.hex_send_var.get() is True)
check("填入发送区内容正确",
      gui.tx_text.get("1.0", "end-1c").strip() == "91 45 7F",
      repr(gui.tx_text.get("1.0", "end-1c")))

fake3.written = bytearray()
gui.midi_tail_var.set(True)           # 勾上帧尾: 末尾要补 FF
gui.send_midi()
check("勾上帧尾后末尾补 FF",
      pump_until(lambda: bytes(fake3.written) == b"\x91\x45\x7f\xff"), bytes(fake3.written))
check("帧头/帧尾默认是 AA / FF",
      gui.midi_head_hex_var.get() == "AA" and gui.midi_tail_hex_var.get() == "FF",
      (gui.midi_head_hex_var.get(), gui.midi_tail_hex_var.get()))
gui.close_port("MIDI 测试结束")

print("[8] 发送 MIDI 文件 (.mid / .syx)")
import mido as _mido  # noqa: E402

folder = app_mod.midi_folder()
check("MIDI 默认目录是 <程序目录>\\MIDI 且已创建",
      os.path.basename(folder) == "MIDI" and os.path.isdir(folder), folder)

# ---- 8a) 转 FPGA 帧（默认路径）----
mid_path = os.path.join(folder, "_selftest_a.mid")
mf = _mido.MidiFile()
tr = _mido.MidiTrack()
mf.tracks.append(tr)
tr.append(_mido.Message('program_change', program=12, time=0))
tr.append(_mido.Message('note_on', note=60, velocity=100, time=0))
tr.append(_mido.Message('note_off', note=60, velocity=64, time=0))
mf.save(mid_path)

gui.open_port()
check("打开串口(供 MIDI 文件测试)", pump_until(lambda: gui.is_open()))
fake4 = gui.ser
fake4.written = bytearray()
gui.midi_realtime_var.set(False)
gui.midi_to_fpga_var.set(True)
check("start_midi_play 受理", gui.start_midi_play(mid_path) is True)
check("转 FPGA 帧时按钮会变成「停止发送」或已发完",
      gui.midi_file_btn.cget("text") in ("停止发送", "发送MIDI文件"))
check("发完按钮恢复成「发送MIDI文件」",
      pump_until(lambda: gui.midi_file_btn.cget("text") == "发送MIDI文件", 6.0),
      gui.midi_file_btn.cget("text"))
_got = bytes(fake4.written)
_frame, _good, _rep = app_mod.fpga_midi.midi_to_frame(mid_path)
check("发出的是 FPGA 帧: 16×FF + AA 开头, FF 结尾",
      _got[:17] == bytes([0xFF] * 16 + [0xAA]) and _got[-1] == 0xFF, _got[:20])
check("帧字节与 fpga_midi 算出来的完全一致", _got == _good, (len(_got), len(_good)))
check("MIDI 60(C4) 被换成键号 0x0D, 不是 0x3C",
      bytes([0x0D]) in _got and bytes([0x3C]) not in _got, _got[:24])

# ---- 8b) 不勾「转FPGA帧」= 老行为：裸 MIDI 原样转发 ----
fake4.written = bytearray()
gui.midi_to_fpga_var.set(False)
check("start_midi_play 受理(裸 MIDI)", gui.start_midi_play(mid_path) is True)
_expect = bytes([0xC0, 12, 0x90, 60, 100, 0x80, 60, 64])
check("裸 MIDI 模式下文件里的消息逐条发出, 字节与顺序都对",
      pump_until(lambda: bytes(fake4.written) == _expect + b"\xff", 6.0), bytes(fake4.written))
gui.midi_tail_var.set(False)
fake4.written = bytearray()
gui.start_midi_play(mid_path)
check("不勾帧尾时就是纯裸 MIDI",
      pump_until(lambda: bytes(fake4.written) == _expect, 6.0), bytes(fake4.written))
check("发完按钮恢复成「发送MIDI文件」(裸 MIDI)",
      pump_until(lambda: gui.midi_file_btn.cget("text") == "发送MIDI文件", 3.0),
      gui.midi_file_btn.cget("text"))
gui.midi_tail_var.set(True)

mid2 = os.path.join(folder, "_selftest_b.mid")
mf2 = _mido.MidiFile()
tr2 = _mido.MidiTrack()
mf2.tracks.append(tr2)
tr2.append(_mido.Message('note_on', note=60, velocity=1, time=0))
# MIDI 文件里的时间必须是整数 tick, 不能直接写秒
_ticks = int(round(_mido.second2tick(0.3, mf2.ticks_per_beat, 500000)))
tr2.append(_mido.Message('note_on', note=62, velocity=1, time=_ticks))
mf2.save(mid2)
check("构造出的测试文件间隔约 0.3 秒 ({} tick)".format(_ticks), _ticks > 0)

fake4.written = bytearray()
gui.midi_realtime_var.set(True)
_t0 = time.time()
gui.start_midi_play(mid2)
pump_until(lambda: len(fake4.written) >= 6, 6.0)
_elapsed = time.time() - _t0
check("勾「按原速」会按文件时值等待", _elapsed >= 0.28, "{:.2f}s".format(_elapsed))

fake4.written = bytearray()
gui.midi_realtime_var.set(False)
_t0 = time.time()
gui.start_midi_play(mid2)
pump_until(lambda: len(fake4.written) >= 6, 6.0)
check("不勾则全速发出不等时值", (time.time() - _t0) < 3.0, "{:.2f}s".format(time.time() - _t0))
gui.stop_midi_file()

# 转 FPGA 帧时「按原速」用不上（时值已经写进帧里），控件要被禁用
gui.midi_to_fpga_var.set(True)
gui.on_midi_change()
check("转 FPGA 帧时「按原速」被禁用",
      str(gui.midi_realtime_chk.cget("state")) == "disabled",
      gui.midi_realtime_chk.cget("state"))
gui.midi_to_fpga_var.set(False)
gui.on_midi_change()

syx_path = os.path.join(folder, "_selftest_c.syx")
_mido.write_syx_file(syx_path, [_mido.Message('sysex', data=[0x7D, 1, 2])])
fake4.written = bytearray()
gui.start_midi_play(syx_path)
check("SysEx 文件也能发（末尾按当前帧尾设置补 FF）",
      pump_until(lambda: bytes(fake4.written) == bytes([0xF0, 0x7D, 1, 2, 0xF7, 0xFF]), 6.0),
      bytes(fake4.written))
fake4.written = bytearray()
gui.midi_tail_var.set(False)
gui.start_midi_play(syx_path)
check("不勾帧尾时 SysEx 就是原样 5 字节",
      pump_until(lambda: bytes(fake4.written) == bytes([0xF0, 0x7D, 1, 2, 0xF7]), 6.0),
      bytes(fake4.written))
gui.midi_tail_var.set(True)

POPUPS[:] = []
check(".mid 不存在时返回失败而不是崩",
      gui.start_midi_play(os.path.join(folder, "没有这个文件.mid")) is False)
check("并且给出了错误提示", any(k == "error" for k, _ in POPUPS), POPUPS)

guint_closed = gui.close_port("MIDI 文件测试结束")
_cleanup_midi = (mid_path, mid2, syx_path)   # 留到 [9] 用完再删

print("[9] 发送数据正确性 (丢字节 / 顺序 / 凭据)")


class DripSerial(object):
    """每次只接受 1 个字节的串口替身, 用来验证不会静默丢字节。"""

    def __init__(self):
        self.written = bytearray()

    def write(self, data):
        if not data:
            return 0
        self.written += data[:1]
        return 1


_drip = DripSerial()
_n = app_mod.write_all(_drip, b"\x01\x02\x03\x04\x05")
check("write_all 面对分片写也会写完整",
      bytes(_drip.written) == b"\x01\x02\x03\x04\x05" and _n == 5, bytes(_drip.written))


class ZeroSerial(object):
    def write(self, data):
        return 0


try:
    app_mod.write_all(ZeroSerial(), b"\x01")
    check("write_all 写不动时必须报错(不能装作成功)", False)
except IOError:
    check("write_all 写不动时必须报错(不能装作成功)", True)


class DripFakeSerial(FakeSerial):
    """完整界面的替身, 但写口每次只吃 1 个字节。"""

    def write(self, data):
        if not data:
            return 0
        self.written += data[:1]
        return 1


app_mod.serial.Serial = DripFakeSerial
gui.close_port("切换到分片写替身")
gui.open_port()
check("以分片写替身重新打开", pump_until(lambda: gui.is_open()))
gui.tx_log = []
fake5 = gui.ser
# 这一段刻意走【裸 MIDI】路径: 逐条字节顺序才是它能验的东西
gui.midi_to_fpga_var.set(False)
gui.midi_tail_var.set(False)
gui.midi_realtime_var.set(False)
gui.start_midi_play(mid_path)
check("分片写下, 文件字节仍完整且顺序正确",
      pump_until(lambda: bytes(fake5.written) == _expect, 8.0), bytes(fake5.written))
check("发送凭据逐批记录, 顺序与内容一致",
      pump_until(lambda: [h for _, _, h in gui.tx_log] ==
                 ["C0 0C", "90 3C 64", "80 3C 40"], 3.0),
      [h for _, _, h in gui.tx_log])
check("每条凭据都带标签", all(lb.startswith("MIDI文件") for _, lb, _ in gui.tx_log),
      [lb for _, lb, _ in gui.tx_log])

# --- FPGA 帧路径: 一整帧, 一条凭据, 内容要和 fpga_midi 算的一致
gui.tx_log = []
fake5.written = bytearray()
gui.midi_to_fpga_var.set(True)
gui.midi_tail_var.set(True)
gui.start_midi_play(mid_path)
_frame2, _good2, _rep2 = app_mod.fpga_midi.midi_to_frame(mid_path)
check("FPGA 帧整帧发出, 字节完全一致",
      pump_until(lambda: bytes(fake5.written) == _good2, 8.0),
      (len(fake5.written), len(_good2)))
check("FPGA 帧的发送凭据带「FPGA帧」标签",
      pump_until(lambda: any(lb.startswith("FPGA帧") for _, lb, _ in gui.tx_log), 3.0),
      [lb for _, lb, _ in gui.tx_log])
# ★ 凭据是 _pump 从 rxq 里搬过来的, 比"串口真的写完"晚一个 pump 周期。
#   这里必须等, 不能写完就直接断言 —— 会随调度随机挂(踩过)。
check("凭据里的 hex 就是整帧字节",
      pump_until(lambda: any(h.replace(" ", "") == _good2.hex().upper()
                             for _, _, h in gui.tx_log), 3.0),
      [h for _, _, h in gui.tx_log])
gui.midi_to_fpga_var.set(False)

# --- 回显
gui.tx_echo_var.set(True)
gui.clear_rx()
gui.midi_type_var.set("Note On (90)")
gui.midi_ch_var.set("1")
gui.midi_d1_var.set("C4")
gui.midi_d2_var.set("100")
gui.on_midi_change()
gui.send_midi()
check("「显示已发送」能把发出去的字节显示出来",
      pump_until(lambda: "<<<" in rx_now() and "90 3C 64" in rx_now(), 3.0),
      repr(rx_now()[-60:]))
gui.tx_echo_var.set(False)

# --- 导出已发送数据
_txpath = os.path.join(app_mod.app_dir(), "_selftest_tx.txt")
_real_save = app_mod.filedialog.asksaveasfilename
app_mod.filedialog.asksaveasfilename = lambda **kw: _txpath
gui.export_tx_log()
app_mod.filedialog.asksaveasfilename = _real_save
check("导出已发送数据: 文件生成", os.path.exists(_txpath))
if os.path.exists(_txpath):
    _content = open(_txpath, encoding="utf-8").read()
    check("导出内容含正确的 HEX 且标注了条数",
          "FPGA帧" in _content and "90 3C 64" in _content and "合计" in _content,
          _content[:300].replace("\n", " | "))
    check("导出内容按写出顺序排列（FPGA 帧在前, 单条 Note On 在后）",
          _content.index("FPGA帧") < _content.index("90 3C 64"))
    os.remove(_txpath)

# --- 只导出 MIDI 字节流(不发送)
_real_open = app_mod.filedialog.askopenfilename
app_mod.filedialog.askopenfilename = lambda **kw: mid_path
POPUPS[:] = []
_bin = os.path.splitext(mid_path)[0] + ".发送字节流.bin"
_txt = os.path.splitext(mid_path)[0] + ".发送字节流.txt"

# (a) 默认路径: 导出的必须是【FPGA 帧】, 且和 fpga_midi 算出来的完全一致
gui.midi_to_fpga_var.set(True)
gui.midi_tail_var.set(True)
gui.export_midi_bytes()
check("转 FPGA 帧时导出的字节流 = 乐谱帧",
      os.path.exists(_bin) and open(_bin, "rb").read() == _good2,
      len(open(_bin, "rb").read()) if os.path.exists(_bin) else "(无文件)")
check("导出的 .txt 里有逐事件记录和键号音名",
      os.path.exists(_txt) and "键号" in open(_txt, encoding="utf-8").read()
      and "键13(C4)" in open(_txt, encoding="utf-8").read())
check("导出字节流时确实没有发送",
      any("没有发送任何数据" in m for _, m in POPUPS), POPUPS)
for _f in (_bin, _txt):
    if os.path.exists(_f):
        os.remove(_f)

# (b) 裸 MIDI 路径: 逐条消息拼出来的字节流
POPUPS[:] = []
gui.midi_to_fpga_var.set(False)
gui.midi_tail_var.set(False)
gui.export_midi_bytes()
check("不转 FPGA 帧时导出的是逐条裸 MIDI",
      os.path.exists(_bin) and open(_bin, "rb").read() == _expect,
      open(_bin, "rb").read() if os.path.exists(_bin) else "(无文件)")
check("裸 MIDI 的 .txt 有逐条记录",
      os.path.exists(_txt) and "90 3C 64" in open(_txt, encoding="utf-8").read())
check("导出字节流时确实没有发送(裸 MIDI)",
      any("没有发送任何数据" in m for _, m in POPUPS), POPUPS)
for _f in (_bin, _txt):
    if os.path.exists(_f):
        os.remove(_f)
gui.midi_to_fpga_var.set(True)
gui.midi_tail_var.set(True)

# 导出必须和真正发出去的一致: 裸 MIDI 模式下勾了帧尾就要把帧尾也算进去
gui.midi_to_fpga_var.set(False)
gui.midi_tail_var.set(True)
gui.midi_tail_hex_var.set("FF")
gui.on_midi_change()
app_mod.filedialog.askopenfilename = lambda **kw: mid_path
gui.export_midi_bytes()
app_mod.filedialog.askopenfilename = _real_open
check("裸 MIDI 导出把帧尾也算进去了(和实发一致)",
      os.path.exists(_bin) and open(_bin, "rb").read() == _expect + b"\xff",
      open(_bin, "rb").read() if os.path.exists(_bin) else "(无文件)")
check("导出 .txt 里标注了帧尾",
      "帧尾" in open(_txt, encoding="utf-8").read())
gui.midi_tail_var.set(False)
gui.on_midi_change()
for _f in (_bin, _txt):
    if os.path.exists(_f):
        os.remove(_f)
gui.midi_to_fpga_var.set(True)
gui.midi_tail_var.set(True)

app_mod.serial.Serial = FakeSerial
gui.close_port("发送正确性测试结束")
for _p in _cleanup_midi:                    # 这里才删测试用的 MIDI 文件
    try:
        os.remove(_p)
    except OSError:
        pass

print("[10] 帧尾检查: MIDI 不会额外追加 FF 之类")
for _name, _ch, _d1, _d2, _want in (
        ("Note On (90)", 1, "C4", "100", b"\x90\x3c\x64"),
        ("Note Off (80)", 1, "C4", "0", b"\x80\x3c\x00"),
        ("Control Change (B0)", 1, "7", "127", b"\xb0\x07\x7f"),
        ("Program Change (C0)", 1, "5", "", b"\xc0\x05"),
        ("Channel Aftertouch (D0)", 1, "64", "", b"\xd0\x40"),
        ("系统实时 停止 (FC)", 1, "", "", b"\xfc")):
    _got = app_mod.build_midi(_name, _ch, _d1, _d2)[0]
    check("{} : 发的就是消息本身, 无追加字节".format(_name), _got == _want,
          "{} vs {}".format(app_mod.fmt_hex(_got), app_mod.fmt_hex(_want)))

_syx = app_mod.build_midi("SysEx (F0..F7)", 1, "7D 01 02", "")[0]
check("SysEx 末字节 F7 是 SysEx 自身结束符(不是额外帧尾)",
      _syx[0] == 0xF0 and _syx[-1] == 0xF7 and _syx.count(0xF7) == 1,
      app_mod.fmt_hex(_syx))

_tail_mid = os.path.join(folder, "_selftest_tail.mid")
_tm = _mido.MidiFile()
_tt = _mido.MidiTrack()
_tm.tracks.append(_tt)
_tt.append(_mido.Message('note_on', note=60, velocity=80, time=0))
_tt.append(_mido.Message('note_off', note=60, velocity=0, time=288))
_tm.save(_tail_mid)
_stream = bytearray()
for _msg in _mido.MidiFile(_tail_mid):
    if not _msg.is_meta:          # 元事件(end_of_track 等)本来就不发
        _stream += bytes(_msg.bytes())
check("MIDI 文件整条流里不含任何 0xFF",
      0xFF not in _stream and len(_stream) == 6,
      "{} max=0x{:02X}".format(app_mod.fmt_hex(_stream), max(_stream)))
check("末字节是对应的数据字节, 不是补出来的帧尾",
      _stream[-1] == 0x00, "末字节=0x{:02X}".format(_stream[-1]))
os.remove(_tail_mid)

# 对照: 只有"文本发送"才追加行尾, MIDI 不走那条路
gui.open_port()
check("重新打开串口(帧尾对照)", pump_until(lambda: gui.is_open()))
_fake6 = gui.ser
_fake6.written = bytearray()
gui.hex_send_var.set(False)
gui.lineend_var.set(app_mod.LINEEND_CHOICES[3])
gui.tx_text.delete("1.0", "end")
gui.tx_text.insert("1.0", "AB")
gui.send_once()
check("对照: 文本发送才会追加 CRLF",
      pump_until(lambda: bytes(_fake6.written) == b"AB\r\n"), bytes(_fake6.written))
gui.close_port("帧尾检查结束")

print("[11] MIDI 帧尾 (可选, 默认关闭)")
gui.midi_tail_var.set(False)
gui.midi_tail_hex_var.set("FF")
gui.midi_type_var.set("Note On (90)")
gui.midi_ch_var.set("1")
gui.midi_d1_var.set("C4")
gui.midi_d2_var.set("100")
gui.on_midi_change()
check("不勾帧尾时, 发出去的就是纯 MIDI 消息, 没有任何追加",
      gui._midi_current()[0] == b"\x90\x3c\x64", app_mod.fmt_hex(gui._midi_current()[0]))

gui.midi_tail_var.set(True)
gui.on_midi_change()
check("勾上帧尾 FF 后, 组装结果末尾就是 FF",
      gui._midi_current()[0] == b"\x90\x3c\x64\xff",
      app_mod.fmt_hex(gui._midi_current()[0]))
check("预览里标出了帧尾", "帧尾 FF" in gui.midi_preview_var.get(),
      gui.midi_preview_var.get())

gui.midi_tail_hex_var.set("0D 0A")
gui.on_midi_change()
check("帧尾支持多个字节 (0D 0A)",
      gui._midi_current()[0] == b"\x90\x3c\x64\x0d\x0a",
      app_mod.fmt_hex(gui._midi_current()[0]))

gui.midi_tail_hex_var.set("GG")
gui.on_midi_change()
check("帧尾写错时预览给提示而不是崩",
      "参数有误" in gui.midi_preview_var.get(), gui.midi_preview_var.get())

POPUPS[:] = []
gui.send_midi()
check("帧尾非法时拒绝发送并弹错误", gui.last_error == "" and len(POPUPS) > 0, POPUPS)

gui.midi_tail_hex_var.set("FF")
gui.on_midi_change()
gui.open_port()
check("打开串口(帧尾测试)", pump_until(lambda: gui.is_open()))
_fake7 = gui.ser
_fake7.written = bytearray()
gui.send_midi()
check("实发: 消息 + FF 一起出去",
      pump_until(lambda: bytes(_fake7.written) == b"\x90\x3c\x64\xff"),
      bytes(_fake7.written))

_tailf = os.path.join(folder, "_selftest_tail2.mid")
_tf = _mido.MidiFile()
_tt2 = _mido.MidiTrack()
_tf.tracks.append(_tt2)
_tt2.append(_mido.Message('note_on', note=60, velocity=80, time=0))
_tt2.append(_mido.Message('note_off', note=60, velocity=0, time=288))
_tf.save(_tailf)
_fake7.written = bytearray()
gui.midi_to_fpga_var.set(False)         # 这一段验的是【裸 MIDI 追加帧尾】的语义
gui.midi_realtime_var.set(False)
gui.start_midi_play(_tailf)
check("裸 MIDI 文件发送: 只在整段末尾追加一次 FF",
      pump_until(lambda: bytes(_fake7.written) ==
                 b"\x90\x3c\x50\x80\x3c\x00\xff", 8.0),
      bytes(_fake7.written))
check("整段里 FF 只出现一次(不是每条都加)",
      pump_until(lambda: bytes(_fake7.written).count(0xFF) == 1, 2.0),
      bytes(_fake7.written).count(0xFF))
os.remove(_tailf)

# 中途停止时不应补帧尾(帧不完整, 不该告诉对端"结束")
_stopf = os.path.join(folder, "_selftest_tail3.mid")
_sf = _mido.MidiFile()
_st = _mido.MidiTrack()
_sf.tracks.append(_st)
_st.append(_mido.Message('note_on', note=60, velocity=1, time=0))
_st.append(_mido.Message('note_on', note=62, velocity=1,
                         time=int(round(_mido.second2tick(3.0, _sf.ticks_per_beat, 500000)))))
_sf.save(_stopf)
_fake7.written = bytearray()
gui.midi_realtime_var.set(True)
gui.start_midi_play(_stopf)
pump_until(lambda: len(_fake7.written) >= 3, 4.0)
gui.stop_midi_file()
pump_until(lambda: gui.midi_file_btn.cget("text") == "发送MIDI文件", 6.0)
check("中途停止时不追加帧尾",
      len(_fake7.written) == 3 and 0xFF not in bytes(_fake7.written),
      bytes(_fake7.written))
os.remove(_stopf)
gui.midi_realtime_var.set(False)
gui.midi_to_fpga_var.set(True)

gui.midi_tail_var.set(False)
gui.on_midi_change()
gui.close_port("帧尾测试结束")

print("[6] 配置读写")
cfg = gui._collect_config()
app_mod.save_config(cfg)
reloaded = app_mod.load_config()
check("配置可写入并读回", reloaded.get("baud") == cfg.get("baud") and
      reloaded.get("history") == cfg.get("history"))
check("读取模式已存入配置", "line_mode" in reloaded and "autoconn" in reloaded)

print("[7] 演奏录音回传 (板子 -> 上位机)")
# 这一段只关心"收", 换回普通的整批写替身
app_mod.serial.Serial = FakeSerial
gui.close_port("录音测试")
gui.open_port()
check("打开串口(录音测试)", pump_until(lambda: gui.is_open()))
fake7 = gui.ser
gui.clear_rx()
gui.rec_clear()

_fr = app_mod.fpga_rec
check("fpga_rec.py 已随程序载入", _fr is not None)

# 板子那边每 10ms 采一次 key_mask, 一变就回一帧:
#   [0x5A][mask 36位 5字节][dt 2字节]      [0x5B][0/1]
_rec_stream = (_fr.pack_state(True)
               + _fr.pack_event(0, 1 << 13)                     # 键13 = C4
               + _fr.pack_event(500, 0)                         # 抬手
               + _fr.pack_event(250, (1 << 13) | (1 << 17) | (1 << 20)))
gui.rxq.put(("data", _rec_stream))
check("回传被解析成 3 个事件",
      pump_until(lambda: len(gui.rec.events) == 3, 3.0), gui.rec.events)
check("时间戳累加成 0 / 500 / 750 ms",
      [t for t, _ in gui.rec.events] == [0, 500, 750],
      [t for t, _ in gui.rec.events])
check("板子回报的开关状态同步到了勾选框", gui.rec_var.get() is True)
# 录音帧是二进制(0x5A + 掩码 + 时间), 当文本打出来全是乱码 —— 必须挡在接收区外面
check("录音帧没有被当文本灌进接收区", len(rx_now()) < 120 and "板子回报" in rx_now(),
      repr(rx_now()))

# 上位机这边也能开关: 点勾选框 -> 发 0x5C -> 板子翻转后回 0x5B
fake7.written = bytearray()
gui.rec_var.set(False)
gui.on_rec_toggle()
check("点勾选框会发出 0x5C",
      pump_until(lambda: bytes(fake7.written) == b"\x5c", 3.0), bytes(fake7.written))
check("等板子回话时状态栏提示已发出指令",
      "开关指令" in gui.rec_status_var.get(), gui.rec_status_var.get())
gui.rxq.put(("data", _fr.pack_state(False)))
# ★ 两个条件都要放进 pump_until 里等。rec_var 是刚被我们手动设成 False 的,
#   单独拿它当条件会"立刻为真", 等于没等 _pump 把 0x5B 处理掉。
check("板子回报已关闭后, 勾选框自动跟着取消(板子是权威)",
      pump_until(lambda: gui.rec_on is False and gui.rec_var.get() is False, 3.0),
      (gui.rec_var.get(), gui.rec_on))

# 板子不在线/不回话: 不能一直骗自己说开着
gui.rec_var.set(True)
gui.rec_on = False
gui.rec_pending = time.time() - 99
gui._rec_timeout_check()
check("板子不回话时勾选框退回真实状态",
      gui.rec_var.get() is False and gui.rec_pending == 0.0,
      (gui.rec_var.get(), gui.rec_pending))

# 合成: 事件 -> MIDI, 音高必须走置换表(键号是接线顺序, 不是音高顺序)
check("键13 就是 C4(60)", _fr.JP.key_to_midi(13) == 60, _fr.JP.key_to_midi(13))
# ★ 键 17 落在 A3(57) 而不是 F4(65) —— 接线顺序和音高顺序是错开的, 这正是
#   必须过置换表、不能拿键号直接当半音数用的原因。
check("键17 是 A3(57) 而不是 F4(65)", _fr.JP.key_to_midi(17) == 57,
      _fr.JP.key_to_midi(17))
_p = gui._synth_rec("自检")
check("界面上的「合成」能产出 MIDI 文件", bool(_p) and os.path.isfile(_p), _p)
if _p and os.path.isfile(_p):
    _m = _mido.MidiFile(_p)
    _notes = [m.note for m in _m if m.type == "note_on" and m.velocity > 0]
    # 第 1 个事件 1 个音, 第 2 个是空掩码(抬手), 第 3 个是三音和弦 => 4 个 note_on
    check("合成出 4 个音(1 个单音 + 1 个三音和弦)", len(_notes) == 4, _notes)
    _want_chord = [_fr.JP.key_to_midi(k) for k in (13, 17, 20)]
    check("和弦三个音一起按下, 音高逐个对上", _notes[1:] == _want_chord,
          (_notes[1:], _want_chord))
    os.remove(_p)

gui.rec_clear()
check("清空录音后事件归零", len(gui.rec.events) == 0, gui.rec.events)

# ★ 每次【重新开始录音】都必须从零开始。否则两遍会首尾相接被合成成一段，
#   时间轴还对不上（上一遍的尾巴 + 这一遍的开头）。
gui._rec_feed(_fr.pack_event(100, 1 << 13) + _fr.pack_event(100, 0))
check("先攒下 2 个事件", len(gui.rec.events) == 2, gui.rec.events)
gui.rxq.put(("data", _fr.pack_state(True)))       # 板子回报：开始录音
check("重新开始录音会清空上一遍的事件",
      pump_until(lambda: gui.rec_on is True and len(gui.rec.events) == 0, 3.0),
      (gui.rec_on, len(gui.rec.events)))
check("时间戳也一起归零（和新一遍对齐）", gui.rec.t_ms == 0, gui.rec.t_ms)

gui.close_port("录音测试结束")

print("[8] 演奏录音的效果开关（录音里带上效果，播放时近似还原）")
app_mod.serial.Serial = FakeSerial
gui.open_port()
check("打开串口(效果测试)", pump_until(lambda: gui.is_open()))
fake8 = gui.ser
gui.clear_rx()
gui.rec_clear()

_fx = app_mod.fpga_rec
check("fpga_rec 里有 8 个效果位", len(_fx.FX_BITS) == 8, len(_fx.FX_BITS))
check("wave_mode 不在录音的 8 位里（它只影响 LCD）",
      all(en != "wave" for en, _cn in _fx.FX_BITS))

# 板子回传：开录（效果全关）-> 弹 C4 -> 开混响+合唱 -> 弹 A3 -> 再开笛子 -> 弹 D4
gui.rxq.put(("data",
             _fx.pack_state(True) + _fx.pack_fx(0)
             + _fx.pack_event(0, 1 << 13) + _fx.pack_event(400, 0)
             + _fx.pack_fx((1 << 6) | (1 << 7))
             + _fx.pack_event(0, 1 << 17) + _fx.pack_event(400, 0)
             + _fx.pack_fx((1 << 6) | (1 << 7) | (1 << 5))
             + _fx.pack_event(0, 1 << 20) + _fx.pack_event(400, 0)
             + _fx.pack_state(False)))
check("鼓捣完等状态落定",
      pump_until(lambda: not gui.rec_on and len(gui.rec.events) == 6, 3.0),
      (gui.rec_on, len(gui.rec.events)))
check("效果状态记成了时间轴（3 次变化）", len(gui.rec.states) == 3, gui.rec.states)
check("状态栏显示当前效果", "笛子" in gui.rec_status_var.get()
      and "混响" in gui.rec_status_var.get(), gui.rec_status_var.get())

_p = gui._synth_rec("效果自检")
check("带效果的合成也能产出 MIDI", bool(_p) and os.path.isfile(_p), _p)
if _p and os.path.isfile(_p):
    _fm = list(_mido.MidiFile(_p))
    _pc = [m.program for m in _fm if m.type == "program_change"]
    check("第一个状态先明确设一次默认音色（Lead，持续型不衰减）",
          _pc and _pc[0] == _fx.FX_LEAD_PROG, _pc)
    check("开笛子 -> 换成 GM 长笛音色", _pc == [_fx.FX_LEAD_PROG, _fx.FX_FLUTE_PROG], _pc)
    check("默认音色不是钢琴（GM 钢琴自带衰减，板子按住是恒定满幅）",
          _fx.FX_LEAD_PROG >= 8, _fx.FX_LEAD_PROG)
    check("默认音色也不是方波/锯齿（用户反馈太尖锐，板子是纯正弦）",
          _fx.FX_LEAD_PROG not in (80, 81) and _fx.FX_TIMBRE_PROG not in (80, 81),
          (_fx.FX_LEAD_PROG, _fx.FX_TIMBRE_PROG))
    _cc = [(m.control, m.value) for m in _fm if m.type == "control_change"]
    check("开混响 -> CC91，开合唱 -> CC93", (91, 100) in _cc and (93, 100) in _cc, _cc)
    check("没开的颤音/延迟不许乱发 CC",
          (1, 100) not in _cc and (94, 100) not in _cc, _cc)
    _nn = [m.note for m in _fm if m.type == "note_on" and m.velocity > 0]
    check("效果不影响音高本身", _nn == [_fx.JP.key_to_midi(k) for k in (13, 17, 20)], _nn)
    os.remove(_p)

check("效果变化被写进日志", "效果（按时间轴还原" in rx_now(), repr(rx_now()[-160:]))

gui.rec_clear()
gui.close_port("效果测试结束")

print("[10] 演奏控制：停止 / 重播 / 停止并清空")
app_mod.serial.Serial = FakeSerial
gui.open_port()
check("打开串口(演奏控制测试)", pump_until(lambda: gui.is_open()))
fake9 = gui.ser
gui.tx_log = []
gui.midi_head_var.set(True)
gui.midi_head_hex_var.set("AA")
gui.midi_tail_var.set(True)
gui.midi_tail_hex_var.set("FF")

# ---- ① 停止演奏 = 0xC7（保留乐谱，可重播）----
fake9.written = bytearray()
gui.stop_score()
check("「停止演奏」只发一个 0xC7",
      pump_until(lambda: bytes(fake9.written) == b"\xc7", 3.0),
      bytes(fake9.written))
check("带「停止演奏」标签",
      pump_until(lambda: any(lb == "停止演奏" for _, lb, _ in gui.tx_log), 3.0),
      [lb for _, lb, _ in gui.tx_log])

# ---- ② 重播 = 0xC8 ----
fake9.written = bytearray()
gui.replay_score()
check("「重播」只发一个 0xC8",
      pump_until(lambda: bytes(fake9.written) == b"\xc8", 3.0),
      bytes(fake9.written))

# ---- ③ 停止并清空 = AA FF（不依赖新指令，老 bitstream 也能用）----
fake9.written = bytearray()
gui.stop_and_clear()
check("「停止并清空」发 AA FF",
      pump_until(lambda: bytes(fake9.written) == b"\xaa\xff", 3.0),
      bytes(fake9.written))
check("带「停止并清空」标签",
      pump_until(lambda: any(lb == "停止并清空" for _, lb, _ in gui.tx_log), 3.0),
      [lb for _, lb, _ in gui.tx_log])

# ★ AA FF 为什么能停：AA 开窗+停播+松键，紧跟的 FF 是【空帧】-> 板子不开播。
#   拿协议解码器验证"确实是空帧"，而不是靠嘴说。
_pp = app_mod.fpga_proto
_fr = _pp.decode_frame(b"\xff" * 16 + b"\xaa\xff")
check("停止指令解出来是空帧（0 个事件）", len(_fr.events) == 0, len(_fr.events))
check("空帧用的是默认时间格/释音（没夹设置字节）",
      _fr.tick_us == _pp.DEFAULT_TICK_US and _fr.rel_us == _pp.DEFAULT_REL_US,
      (_fr.tick_us, _fr.rel_us))
check("空帧不带任何音符字节（板子判为不开播）",
      all(not (0x00 <= b <= 0x23) for b in b"\xaa\xff"), True)

# ---- 三个控制指令都要先把发送队列清掉 ----
#   否则它们要排在几千字节谱子后面才发得出去，等轮到板子早弹完了 = 等于停不了
for _name, _fn, _want in (("停止演奏", gui.stop_score, b"\xc7"),
                          ("重播", gui.replay_score, b"\xc8"),
                          ("停止并清空", gui.stop_and_clear, b"\xaa\xff")):
    fake9.written = bytearray()
    for _ in range(30):
        gui.txq.put((b"\x01" * 100, "假字节"))
    _fn()
    check("%s：先把没发出去的队列清掉" % _name, gui.txq.qsize() <= 1, gui.txq.qsize())
    check("%s：清完队列后紧接着发的就是它" % _name,
          pump_until(lambda w=_want: bytes(fake9.written)[-len(w):] == w, 3.0),
          bytes(fake9.written)[-8:])

# ---- 串口没开时：只提示，不崩 ----
gui.close_port("演奏控制测试结束")
for _name, _fn in (("停止演奏", gui.stop_score), ("重播", gui.replay_score),
                   ("停止并清空", gui.stop_and_clear)):
    _fn()
    check("串口没开时点「%s」不崩" % _name, True)

print("[11] 演奏按键自定义映射")
_km = app_mod.keymap
check("keymap.py 载入", _km is not None)
check("默认是恒等", _km.is_default(gui._keymap()))

# 改一个键 -> 存进配置
_m = gui._keymap()
_m[5] = 29           # 键5(A2) 改成 A4
_m[6] = _km.DISABLE  # 键6 禁用
gui._set_keymap(_m)
check("改完存下来了", gui._keymap()[5] == 29 and gui._keymap()[6] == _km.DISABLE,
      gui._keymap()[5:7])
check("那行小字跟着变", "改过" in gui.keymap_lbl.cget("text"), gui.keymap_lbl.cget("text"))

# 下发：36 条 x 3 字节，顺序对
gui.open_port()
check("重新打开串口(映射测试)", pump_until(lambda: gui.is_open()))
fake11 = gui.ser
fake11.written = bytearray()
gui.on_keymap_send()
check("映射表整张发出去（36 x 3 = 108 字节）",
      pump_until(lambda: len(fake11.written) == 108, 3.0), len(fake11.written))
_b = bytes(fake11.written)
check("每条都是 0xC6 打头",
      all(_b[i * 3] == 0xC6 for i in range(36)))
check("第 5 条 = 键5 -> 29(A4)", tuple(_b[15:18]) == (0xC6, 5, 29), _b[15:18])
check("第 6 条 = 键6 -> 禁用(0x3F)", tuple(_b[18:21]) == (0xC6, 6, 0x3F), _b[18:21])
check("第 0 条是恒等", tuple(_b[0:3]) == (0xC6, 0, 0), _b[0:3])

# 恢复默认
gui._set_keymap(_km.default_map())
check("恢复默认后 is_default 成立", _km.is_default(gui._keymap()))
fake11.written = bytearray()
gui.on_keymap_send()
check("默认表也能整张下发", pump_until(lambda: len(fake11.written) == 108, 3.0),
      len(fake11.written))

# ★ 这条最重要：映射只影响手弹，不影响放谱子。
#   板子那边映射做在"物理键->声部"这一步，midi_keys 本来就是声部号，
#   所以电脑发谱【不需要】按映射反查 —— 这条断言把设计意图钉住。
check("下发日志里说明了「只影响手弹」",
      "只影响【手弹】" in gui.rx_text.get("1.0", "end-1c"),
      repr(gui.rx_text.get("1.0", "end-1c")[-120:]))

gui.close_port("映射测试结束")

print("[12] 效果开关 / 音色（上位机 -> 板子；板上的 TTP 按钮照旧保留）")
_fr = app_mod.fpga_rec
_fx_names = dict(_fr.FX_BITS)
app_mod.serial.Serial = FakeSerial
gui.open_port()
check("打开串口(效果开关测试)", pump_until(lambda: gui.is_open()))
fake12 = gui.ser
# ★ 连上之后程序自己会发一条 0xCB 查询（600ms 后），先把它等出去再清空缓冲，
#   否则后面那些"只该发两个字节"的断言会被它多出来的字节搞挂。
pump_until(lambda: False, 0.9)
fake12.written = bytearray()

# ---- ① 界面上到底有没有这些控件（★ 这条是防"加了但看不见"的）----
check("有 7 个效果开关按钮（bit5 是音色档，不做成开关）",
      sorted(gui.fx_btns) == [0, 1, 2, 3, 4, 6, 7], sorted(gui.fx_btns))
check("bit5 确实没被做成开关", 5 not in gui.fx_btns)
check("音色下拉框正好三档",
      list(gui.tone_box.cget("values")) == ["普通", "笛子", "吉他"],
      gui.tone_box.cget("values"))
check("那行效果小字在布局里", bool(gui.fx_lbl.winfo_manager()), gui.fx_lbl.winfo_manager())

# ---- ② 每个开关按钮发的位号都要对得上（一位一位来）----
for _bit in sorted(gui.fx_btns):
    fake12.written = bytearray()
    gui.toggle_effect(_bit)
    _want = bytes([0xC9, _bit])
    check("点「%s」发 0xC9 %02X" % (_fx_names[_fr.FX_BITS[_bit][0]], _bit),
          pump_until(lambda w=_want: bytes(fake12.written) == w, 3.0),
          bytes(fake12.written))

# ---- ③ 音色：三档各发一次 ----
for _i, _n in enumerate(_fr.TONE_NAMES):
    gui.tone_var.set(_n)
    fake12.written = bytearray()
    gui.set_tone()
    _want = bytes([0xCA, _i])
    check("选「%s」发 0xCA %02X" % (_n, _i),
          pump_until(lambda w=_want: bytes(fake12.written) == w, 3.0),
          bytes(fake12.written))

# ---- ④ 刷新状态 = 0xCB ----
fake12.written = bytearray()
gui.request_fx_state()
check("「刷新状态」发 0xCB",
      pump_until(lambda: bytes(fake12.written) == b"\xcb", 3.0), bytes(fake12.written))

# ---- ⑤ 板子回报状态 -> 界面跟着变（两端同步的关键）----
#   回报是 3 个字节 [0x5D][效果][音色]，这里【故意拆成两批】送：
#   解析器必须能等那个音色字节（它完全可能落在下一个串口批次里）。
gui.rec_clear()
gui.rxq.put(("data", bytes([0x5D, 0b01001010])))     # 延音 + 颤音 + 混响
check("回报前开关是暗的", str(gui.fx_btns[1].cget("relief")) == "raised",
      gui.fx_btns[1].cget("relief"))
check("板子的状态收到了（效果 8 位）",
      pump_until(lambda: gui.rec.fx_state == 0b01001010, 3.0), gui.rec.fx_state)
gui.rxq.put(("data", bytes([2])))                    # 音色 = 吉他（下一批才到）
check("音色字节晚一批到也能接上（吉他）",
      pump_until(lambda: gui.rec.tone_sel == 2, 3.0), gui.rec.tone_sel)
check("界面按回报把开关点亮（延音/颤音/混响）",
      pump_until(lambda: all(str(gui.fx_btns[b].cget("relief")) == "sunken"
                             for b in (1, 3, 6)), 3.0),
      {b: gui.fx_btns[b].cget("relief") for b in sorted(gui.fx_btns)})
check("没开的那些还是暗的",
      all(str(gui.fx_btns[b].cget("relief")) == "raised" for b in (0, 2, 4, 7)),
      {b: gui.fx_btns[b].cget("relief") for b in sorted(gui.fx_btns)})
check("音色下拉框跟着跳到「吉他」",
      pump_until(lambda: gui.tone_var.get() == "吉他", 3.0), gui.tone_var.get())
check("那行小字把效果和音色都写出来了",
      pump_until(lambda: "吉他" in gui.fx_lbl.cget("text"), 3.0),
      repr(gui.fx_lbl.cget("text")))
check("小字里效果名也在", "颤音" in gui.fx_lbl.cget("text"), repr(gui.fx_lbl.cget("text")))

# ---- ⑥ 老固件兼容：只有 2 个字节的状态帧，后面紧跟事件帧不能被打乱 ----
gui.rec_clear()
gui.rxq.put(("data", bytes([0x5D, 0b00000001]) + _fr.pack_event(0, 1 << 13)))
check("老固件的 2 字节状态帧也认（音色留空 = 未知）",
      pump_until(lambda: gui.rec.fx_state == 1 and gui.rec.tone_sel is None, 3.0),
      (gui.rec.fx_state, gui.rec.tone_sel))
check("紧跟其后的事件帧没有被吃掉",
      pump_until(lambda: len(gui.rec.events) == 1, 3.0), len(gui.rec.events))

# ---- ⑦ 串口没开时：只提示，不崩 ----
gui.close_port("效果开关测试结束")
gui.toggle_effect(1)
gui.set_tone()
gui.request_fx_state()
check("串口没开时点效果/音色不崩", True)

# ---- ⑧ 连上以后自动查一次状态（自检全程把它关掉了，这里单独打开验一次）----
gui.fx_auto_query = True
fake12.written = bytearray()
gui.open_port()
check("重新打开串口(自动查询测试)", pump_until(lambda: gui.is_open()))
check("连上后会自动发一条 0xCB 把开关对齐",
      pump_until(lambda: b"\xcb" in bytes(gui.ser.written), 3.0),
      bytes(gui.ser.written))
gui.fx_auto_query = False
gui.close_port("自动查询测试结束")

print("[13] 布局自检（★ 这一节是补的：之前两个 Frame 放同一个 grid 单元，")
print("     后建的把先建的整个盖住，表现就是「加了按钮但界面上看不见」）")
root.update_idletasks()


def _grid_cells(parent):
    """列出 parent 里各控件占的 grid 单元，返回 {单元: [控件, ...]}"""
    out = {}
    for w in parent.winfo_children():
        gi = w.grid_info()
        if not gi:
            continue
        key = (int(gi.get("row", 0)), int(gi.get("column", 0)))
        out.setdefault(key, []).append(str(w))
    return out


# ① 整个界面：任何两个控件都不能占同一个 grid 单元
_all_dup = []


def _scan(w):
    cells = _grid_cells(w)
    for k, ws in cells.items():
        if len(ws) > 1:
            _all_dup.append((str(w), k, ws))
    for c in w.winfo_children():
        _scan(c)


_scan(root)
check("全界面没有两个控件占同一个 grid 单元", not _all_dup,
      _all_dup[:3])

# ② 跨满整行的那些"功能行"必须各占一行
#    ★ 不能断言"每行只有一个控件" —— row=0 的 MIDI 组装行本来就是分列摆 8 个控件的，
#      那是正常的。真正的风险是【跨满整行的框架】撞到同一行，会互相盖住。
_midi = gui.midi_stop_btn.master.master
_spans = {}
for w in _midi.winfo_children():
    gi = w.grid_info()
    if gi and int(gi.get("columnspan", 1)) >= 9:
        _spans.setdefault(int(gi.get("row", 0)), []).append(str(w))
check("跨满整行的功能行各占一行（不会互相盖住）",
      all(len(v) == 1 for v in _spans.values()),
      {k: v for k, v in _spans.items() if len(v) > 1})
check("演奏控制行和演奏录音行在不同的行号上",
      gui.midi_stop_btn.master.grid_info()["row"] !=
      gui.rec_chk.master.grid_info()["row"],
      (gui.midi_stop_btn.master.grid_info()["row"],
       gui.rec_chk.master.grid_info()["row"]))

# ③ 新控件都要真的在布局里、并且窗口放得下
for _name, _w in (("停止演奏", gui.midi_stop_btn),
                  ("重播", gui.midi_replay_btn),
                  ("按键映射状态小字", gui.keymap_lbl),
                  ("效果开关那一排的第一个按钮", gui.fx_btns[0]),
                  ("音色下拉框", gui.tone_box),
                  ("效果状态小字", gui.fx_lbl)):
    check("%s 在布局里" % _name, bool(_w.winfo_manager()), _w.winfo_manager())
check("窗口需要的宽度没超过默认窗口宽（980）",
      root.winfo_reqwidth() <= 980, root.winfo_reqwidth())
check("启动时映射状态小字不是空的", gui.keymap_lbl.cget("text") != "",
      repr(gui.keymap_lbl.cget("text")))

root.destroy()
app_mod.serial.Serial = real_serial
try:
    os.remove(os.path.join(app_mod.app_dir(), app_mod.CONFIG_NAME))
except OSError:
    pass

print("")
if FAILS:
    print("FAILED ({}): {}".format(len(FAILS), ", ".join(FAILS)))
    sys.exit(1)
print("ALL CHECKS PASSED")
