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
# ★ 连上之后程序会在 400ms 后自动下发一遍映射（板子掉电就回默认）。
#   这里先把它等出去再清空缓冲，否则下面的"正好 108 字节"会被它多出来的
#   那一份搞挂（这个竞态一直在，只是以前碰巧没撞上）。
pump_until(lambda: False, 0.9)
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
check("音色是【四个】可以直接点的按钮（★ V48.2 加了「小号」）",
      len(gui.tone_btns) == 4
      and [b.cget("text") for b in gui.tone_btns] == ["普通", "笛子", "吉他", "小号"],
      [b.cget("text") for b in gui.tone_btns])
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
    fake12.written = bytearray()
    gui.set_tone(_i)
    _want = bytes([0xCA, _i])
    check("点「%s」按钮发 0xCA %02X" % (_n, _i),
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
check("音色按钮跟着跳到「吉他」（第 3 个高亮）",
      pump_until(lambda: str(gui.tone_btns[2].cget("relief")) == "sunken"
                 and gui.tone_var.get() == "吉他", 3.0),
      (gui.tone_btns[2].cget("relief"), gui.tone_var.get()))
check("其它两个音色按钮没被点亮",
      all(str(gui.tone_btns[i].cget("relief")) == "raised" for i in (0, 1)),
      [gui.tone_btns[i].cget("relief") for i in (0, 1, 2)])
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

# ---- ⑨ ★ V31.1：体验优化（插队 / 待确认 / 状态未知 / 防重复）----
#   用户报过三个毛病：① 按一个有时带得另外几个一起变 ② 有延迟不知道开没开成
#   ③ 音色连点导致指令重复。这一节就是钉住这三个的修法。
import queue as _queue_mod

# 上一节结束时把串口关了，这里要重新打开（没连上时这些按钮本来就只提示不动作）
gui.fx_auto_query = False
gui.open_port()
check("重新打开串口(体验优化测试)", pump_until(lambda: gui.is_open()))
fake12 = gui.ser
pump_until(lambda: False, 0.9)             # 等自动查询先出去（避免混进断言）
fake12.written = bytearray()

_saved_txq = gui.txq
gui.txq = _queue_mod.Queue()               # 换一个【没有发送线程消费】的队列，好断言
for _ in range(40):
    gui.txq.put((b"\x01" * 100, "假谱子字节"))
gui.toggle_effect(6)                       # 混响
# ★ V39：把 V38 改的"队列非空就不插队"【撤回】了 —— 改成排队之后，命令要排在
#   几十秒的谱子后面，而界面 1.2 秒就撤掉"待确认"，实测就是"延迟关不掉"。
#   所以断言恢复成"插到队首"（= 点一下立刻生效）。
check("★ 效果指令插到【队首】（不排在几千字节谱子后面）",
      gui.txq.queue[0] == (b"\xc9\x06", "效果开关 混响"), gui.txq.queue[0])
check("队列里原来的谱子字节一条没丢（没有清队列）",
      len(gui.txq.queue) == 41, len(gui.txq.queue))
check("按下后按钮立刻变「待确认」（橙色）",
      str(gui.fx_btns[6].cget("bg")) == "#e08a1e", gui.fx_btns[6].cget("bg"))
check("小字上写着在等板子确认",
      "待板子确认" in str(gui.fx_lbl.cget("text")), gui.fx_lbl.cget("text"))
gui.txq = _saved_txq

# 板子回报 -> 待确认撤掉、按钮按回报落定
gui.rxq.put(("data", bytes([0x5D, 0b01000000])))     # 板子说：混响 开
check("板子回报后「待确认」撤掉",
      pump_until(lambda: gui.fx_pending is None, 3.0), gui.fx_pending)
check("按钮按板子的回报落定（混响亮）",
      pump_until(lambda: str(gui.fx_btns[6].cget("relief")) == "sunken", 3.0),
      gui.fx_btns[6].cget("relief"))
check("日志里能看到「已确认」",
      "已确认" in gui.rx_text.get("1.0", "end-1c"))

# 新会话（还没收到过回报）-> 显示"未知"，而不是假装"全关"
gui.rec.resync()
gui._update_fx_ui()
check("★ 没收到过回报时显示「状态未知」（不把未知画成关）",
      "状态未知" in str(gui.fx_lbl.cget("text")), gui.fx_lbl.cget("text"))
check("未知状态下按钮是浅灰而不是绿色",
      str(gui.fx_btns[6].cget("bg")) == "#eeeeee", gui.fx_btns[6].cget("bg"))

# 音色连点：同一档在待确认期间只发一次
fake12.written = bytearray()
gui.fx_pending = None
gui.set_tone(2)
gui.set_tone(2)
gui.set_tone(2)
check("★ 同一档连点三次只发一条 0xCA",
      pump_until(lambda: bytes(fake12.written) == b"\xca\x02", 2.0)
      and pump_until(lambda: False, 0.4) is False
      and bytes(fake12.written) == b"\xca\x02", bytes(fake12.written))
check("音色按钮也显示成待确认（橙色）",
      str(gui.tone_btns[2].cget("bg")) == "#e08a1e", gui.tone_btns[2].cget("bg"))
gui.fx_pending = None

# 超时（板子不回报）-> 撤掉待确认 + 自动重问一次
fake12.written = bytearray()
gui.toggle_effect(1)
gui.fx_pending["t"] -= 2.0                  # 假装已经过了 1.2 秒
gui._fx_pending_check()
check("超时后撤掉「待确认」", gui.fx_pending is None)
check("超时后自动重问了一次状态（0xCB）",
      b"\xcb" in bytes(fake12.written), bytes(fake12.written))
check("超时后日志里说明了原因", "还没回报" in gui.rx_text.get("1.0", "end-1c"))

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
                  ("音色按钮（普通）", gui.tone_btns[0]),
                  ("效果状态小字", gui.fx_lbl)):
    check("%s 在布局里" % _name, bool(_w.winfo_manager()), _w.winfo_manager())
check("窗口需要的宽度没超过默认窗口宽（980）",
      root.winfo_reqwidth() <= 980, root.winfo_reqwidth())
check("启动时映射状态小字不是空的", gui.keymap_lbl.cget("text") != "",
      repr(gui.keymap_lbl.cget("text")))

print("[14] ★ V44：节拍器（0xCF 设置 / 0x5E 回报 / 待确认 / 不回环重发）")
_mr = app_mod.fpga_rec
# ★ 上一节刚点过音色按钮，线上可能还有一条 0xCA 在飞 —— 先让它落地再清空，
#   否则下面那些"一个字节都不该有"的断言会被它搞挂（这类竞态踩过）。
pump_until(lambda: False, 0.4)
fake12.written = bytearray()

# ---- ① 控件都在布局里、默认值 = 板子出厂值 ----
check("节拍器控件都在布局里",
      all(bool(w.winfo_manager()) for w in (gui.metro_chk, gui.metro_bpm_box,
                                            gui.metro_num_box, gui.metro_den_box,
                                            gui.metro_lbl)),
      [w.winfo_manager() for w in (gui.metro_chk, gui.metro_bpm_box,
                                   gui.metro_num_box, gui.metro_den_box, gui.metro_lbl)])
check("默认值 = 板子出厂值（100 BPM / 分子 4 / 分母 4 / 开关开）",
      (gui.metro_bpm_var.get(), gui.metro_num_var.get(), gui.metro_den_var.get(),
       bool(gui.metro_on_var.get())) == ("100", "4", "4", True),
      (gui.metro_bpm_var.get(), gui.metro_num_var.get(), gui.metro_den_var.get(),
       gui.metro_on_var.get()))
check("分母下拉只给 2 / 4 / 8（★ 界面给人看的分母，线上发的是编码 0/1/2）",
      [str(v) for v in gui.metro_den_box.cget("values")] == ["2", "4", "8"],
      gui.metro_den_box.cget("values"))
# ★ V44：用户明确说过节拍器「不要跟音色放一块」——这条把它钉在代码里
check("★ 节拍器控件和「音色」控件不在同一行",
      gui.metro_lbl.master is not gui.tone_btns[0].master,
      (str(gui.metro_lbl.master), str(gui.tone_btns[0].master)))

# ---- ② 改 BPM：先攒一下（防连发），再发一条带 3 个操作数的 0xCF ----
fake12.written = bytearray()
gui.metro_bpm_var.set("120")
gui.on_metro_change()
check("★ 改完不是立刻发（敲「1」「2」「0」不会连发三条）",
      pump_until(lambda: False, 0.2) is False and bytes(fake12.written) == b"",
      bytes(fake12.written))
check("攒够 0.35 秒后发 0xCF 78 04 01（120 BPM，4/4，分母编码 1）",
      pump_until(lambda: bytes(fake12.written) == b"\xcf\x78\x04\x01", 3.0),
      bytes(fake12.written))
check("发出去之后是「待确认」", gui.metro_pending is not None, gui.metro_pending)
check("小字上写着在等板子确认", "待板子确认" in gui.metro_lbl.cget("text"),
      gui.metro_lbl.cget("text"))

# ---- ③ 板子回报 0x5E：三个值全对上才清「待确认」，界面同步成板子的值 ----
#   ★ 故意拆成两批送：5 个字节完全可能落在两个串口批次里。
gui.rxq.put(("data", bytes([0x5E, 120, 4])))
check("0x5E 被切两半时不乱猜（一个字段都不填）",
      pump_until(lambda: gui.rec.metro_bpm is None, 1.0), gui.rec.metro_bpm)
gui.rxq.put(("data", bytes([0x01, 0x02])))
check("后半截到了就解析出来（120 / 4 / 编码1 / 第2拍）",
      pump_until(lambda: (gui.rec.metro_bpm, gui.rec.metro_num,
                          gui.rec.metro_den, gui.rec.metro_beat) == (120, 4, 1, 2), 3.0),
      (gui.rec.metro_bpm, gui.rec.metro_num, gui.rec.metro_den, gui.rec.metro_beat))
check("★ 三个值都对上，「待确认」才撤掉",
      pump_until(lambda: gui.metro_pending is None, 3.0), gui.metro_pending)
check("日志里看到「已确认」", "已确认" in rx_now())
check("小字里显示的是板子的真实值（第 2 拍 = 协议里的 BEAT 1 + 1）",
      "120 BPM 4/4" in gui.metro_lbl.cget("text")
      and "第 3 拍" in gui.metro_lbl.cget("text"),
      gui.metro_lbl.cget("text"))

# ---- ④ ★★ 回环守卫：板子回报回写控件，不能再被当成"用户改动"发一遍 ★★ ----
pump_until(lambda: False, 0.4)
fake12.written = bytearray()
gui.rxq.put(("data", bytes([0x5E, 90, 3, 1, 1])))
check("板子那边被改成 90 BPM 3/4 —— 界面控件跟着同步",
      pump_until(lambda: (gui.metro_bpm_var.get(), gui.metro_num_var.get(),
                          gui.metro_den_var.get()) == ("90", "3", "4"), 3.0),
      (gui.metro_bpm_var.get(), gui.metro_num_var.get(), gui.metro_den_var.get()))
check("★ 回写控件没有触发回环重发（板子回报后线上一个字节都没有）",
      pump_until(lambda: False, 0.6) is False and bytes(fake12.written) == b"",
      bytes(fake12.written))

# ---- ⑤ 开关：关 = 发 BPM 0（★ 0 是"关闭"，不是"最慢"）----
pump_until(lambda: False, 0.3)
fake12.written = bytearray()
gui.metro_on_var.set(False)
gui.on_metro_toggle()
check("关掉开关发 0xCF 00 03 01（BPM 0 = 关闭节拍器）",
      pump_until(lambda: bytes(fake12.written) == b"\xcf\x00\x03\x01", 3.0),
      bytes(fake12.written))
check("关掉时 BPM 框不会被改成 0（不然再打开就丢掉速度了）",
      gui.metro_bpm_var.get() == "90", gui.metro_bpm_var.get())

# ---- ⑥ 越界输入：发出去的是夹紧后的值，最后由板子的回报把控件拉齐 ----
gui.metro_on_var.set(True)
gui.on_metro_toggle()                       # 先打开（发 90 BPM 3/4）
check("重新打开开关会带上当前速度发一条", pump_until(lambda: len(fake12.written) >= 4, 3.0),
      bytes(fake12.written))
pump_until(lambda: False, 0.4)
fake12.written = bytearray()
gui.metro_bpm_var.set("999")
gui.on_metro_change()
check("★ BPM 999 发出去的是夹紧后的 240（0xF0）",
      pump_until(lambda: bytes(fake12.written) == b"\xcf\xf0\x03\x01", 3.0),
      bytes(fake12.written))
#   ★ 控件【不在打字过程中】被回写（否则想敲 150 会先被夹成 30，
#     再接一个 5 变成 305、又夹成 240 —— 根本输不进 150）。
check("★ 打字过程中不擅自回写控件（999 还留在框里，等板子回报来拉齐）",
      gui.metro_bpm_var.get() == "999", gui.metro_bpm_var.get())
gui.rxq.put(("data", bytes([0x5E, 240, 3, 1, 0])))
check("板子回报 240 之后，控件才被拉成 240",
      pump_until(lambda: gui.metro_bpm_var.get() == "240", 3.0),
      gui.metro_bpm_var.get())
pump_until(lambda: False, 0.4)
fake12.written = bytearray()
gui.metro_num_var.set("99")
gui.on_metro_change()
check("★ 分子 99 夹成 8（0x08）",
      pump_until(lambda: bytes(fake12.written) == b"\xcf\xf0\x08\x01", 3.0),
      bytes(fake12.written))

# ---- ⑦ 板子不回报：超时撤掉「待确认」并说明原因 ----
pump_until(lambda: False, 0.4)
fake12.written = bytearray()
gui.metro_bpm_var.set("100")
gui.metro_num_var.set("4")
gui.on_metro_change()
check("先发出去（0xCF 64 04 01）",
      pump_until(lambda: bytes(fake12.written) == b"\xcf\x64\x04\x01", 3.0),
      bytes(fake12.written))
gui.metro_pending["t"] -= 2.0               # 假装已经过了 1.2 秒
gui._metro_pending_check()
check("超时后撤掉「待确认」", gui.metro_pending is None, gui.metro_pending)
check("超时后日志里说明了原因（不谎称设置成功）",
      "还没回报节拍器状态" in rx_now())

# ---- ⑧ 串口没开时：只记下不崩，连上后自动补发 ----
gui.close_port("节拍器测试结束")
pump_until(lambda: False, 0.3)
fake12.written = bytearray()
gui.metro_bpm_var.set("140")
gui.on_metro_change()
check("串口没开时改节拍器不崩，并标成「还没发出去」",
      gui.metro_unsent is True and bytes(fake12.written) == b"",
      (gui.metro_unsent, bytes(fake12.written)))
check("小字上说明了「改动还没发出去」",
      "还没发出去" in gui.metro_lbl.cget("text"), gui.metro_lbl.cget("text"))
gui.open_port()
check("重新打开串口(节拍器测试)", pump_until(lambda: gui.is_open()))
fake13 = gui.ser
check("连上后自动补发那条没发出去的改动（0xCF 8C 04 01 = 140 BPM 4/4）",
      pump_until(lambda: b"\xcf\x8c\x04\x01" in bytes(fake13.written), 3.0),
      bytes(fake13.written))
check("连上后小字退回「板子状态未知」（协议里没有查询节拍器的命令）",
      "未知" in gui.metro_lbl.cget("text"), gui.metro_lbl.cget("text"))
gui.close_port("节拍器测试结束")

# ==========================================================================
print("[15] ★ V45：MIDI 逐音力度（0xC3）开关")
#   协议：上位机 -> 板子 [0xC3][VEL]（VEL 0~127，127 = 满幅 = 默认）。
#   ★ 语义是"作用于紧接着的下一个事件"（和 0xC2 时值延长同类）。
#   ★★ 没有 0xC3 前缀时板子按 127 处理 —— 所以关掉开关时产出的帧必须
#      和加这个功能之前【逐字节相同】，用户就是拿它做 A/B 对比试听的。

# ---- ① 控件在布局里、默认开、状态一眼能看出来 ----
check("力度控件都在布局里",
      all(bool(w.winfo_manager()) for w in (gui.midi_vel_chk, gui.vel_lbl)),
      [w.winfo_manager() for w in (gui.midi_vel_chk, gui.vel_lbl)])
check("★ MIDI 力度开关默认开（用户要求）", gui.midi_vel_var.get() is True,
      gui.midi_vel_var.get())
check("小字上写着「力度: 开」（开关状态看得出来）",
      "力度: 开" in gui.vel_lbl.cget("text"), gui.vel_lbl.cget("text"))
# ★ 用户明确说过：不要和「音色」控件挤一行，也不要和节拍器那行挤
check("★ 力度控件不和「音色」控件在同一行",
      gui.vel_lbl.master is not gui.tone_btns[0].master,
      (str(gui.vel_lbl.master), str(gui.tone_btns[0].master)))
check("★ 力度控件也不在节拍器那一行里",
      gui.vel_lbl.master is not gui.metro_lbl.master,
      (str(gui.vel_lbl.master), str(gui.metro_lbl.master)))
#   ★ 加了这一行之后窗口仍然放得下（硬约束：<= 980）
check("★ 加上力度这一行后，窗口需要的宽度还是没超过 980",
      root.winfo_reqwidth() <= 980, root.winfo_reqwidth())

# ---- ② 造一个"两个音力度不同"的 MIDI ----
_vel_mid = os.path.join(folder, "_selftest_vel.mid")
_vm = _mido.MidiFile(ticks_per_beat=480)
_vt = _mido.MidiTrack()
_vm.tracks.append(_vt)
_vt.append(_mido.MetaMessage('set_tempo', tempo=500000, time=0))
_vt.append(_mido.Message('note_on', note=60, velocity=100, time=0))     # C4 力度 100
_vt.append(_mido.Message('note_off', note=60, velocity=0, time=480))
_vt.append(_mido.Message('note_on', note=64, velocity=40, time=0))      # E4 力度 40
_vt.append(_mido.Message('note_off', note=64, velocity=0, time=480))
_vm.save(_vel_mid)

app_mod.serial.Serial = FakeSerial
gui.open_port()
check("打开串口(力度测试)", pump_until(lambda: gui.is_open()))
fake14 = gui.ser
# ★ 关掉自动查状态（0xCB），否则下面"逐字节一致"的比对里会混进它的字节
gui.fx_auto_query = False
gui.midi_to_fpga_var.set(True)
gui.midi_tail_var.set(True)
gui.midi_realtime_var.set(False)

_fr_v, _blob_v, _rep_v = app_mod.fpga_midi.midi_to_frame(_vel_mid)
check("★ 开关打开时生成的帧里确实有 C3 64（力度 100）和 C3 28（力度 40）",
      bytes([0xC3, 100]) in _blob_v and bytes([0xC3, 40]) in _blob_v,
      _blob_v.hex())
fake14.written = bytearray()
gui.start_midi_play(_vel_mid)
check("★ 开关打开时，发出去的帧 = 带 0xC3 的那个帧（逐字节一致）",
      pump_until(lambda: bytes(fake14.written) == _blob_v, 8.0),
      (len(fake14.written), len(_blob_v)))
check("发完按钮恢复成「发送MIDI文件」(力度测试)",
      pump_until(lambda: gui.midi_file_btn.cget("text") == "发送MIDI文件", 6.0),
      gui.midi_file_btn.cget("text"))

# ---- ③ 关掉开关：一个 0xC3 都不发，而且要和"改造前"的帧逐字节相同 ----
_blob_off_v = app_mod.fpga_midi.midi_to_frame(_vel_mid, use_velocity=False)[1]
gui.midi_vel_var.set(False)
gui.on_midi_vel_change()
check("关掉后小字写「力度: 关」",
      "力度: 关" in gui.vel_lbl.cget("text"), gui.vel_lbl.cget("text"))
check("关掉后日志里说明了「一个 0xC3 都不发」",
      "一个 0xC3 都不发" in rx_now(), repr(rx_now()[-200:]))
fake14.written = bytearray()
gui.start_midi_play(_vel_mid)
check("★★ 关掉开关时发出去的帧里一个 0xC3 都没有",
      pump_until(lambda: len(fake14.written) == len(_blob_off_v), 8.0)
      and 0xC3 not in bytes(fake14.written),
      (len(fake14.written), bytes(fake14.written).hex()[:120]))
check("★★ 关掉开关发出去的帧 = 不带力度的帧（即改造前的字节流）",
      bytes(fake14.written) == _blob_off_v,
      (bytes(fake14.written).hex(), _blob_off_v.hex()))
check("★★ 开着比关掉正好多 2 字节 × 2 个事件（就是那两组 C3 VEL）",
      len(_blob_v) - len(_blob_off_v) == 4,
      (len(_blob_v), len(_blob_off_v)))

# ---- ④ 录音回传那条路：明确不带力度（开关开着也不发 0xC3）----
#   「发回板子」用的就是 start_midi_play(path, use_velocity=False) ——
#   那份 MIDI 是上位机合成的、velocity 是写死的常数，跟着开关走会让
#   "发回板子"整体变小声（= 动到老功能），所以那条路固定不带力度。
gui.midi_vel_var.set(True)
gui.on_midi_vel_change()
fake14.written = bytearray()
gui.start_midi_play(_vel_mid, use_velocity=False)
check("★ 明确指定 use_velocity=False（录音回传那条路）：一个 0xC3 都不发",
      pump_until(lambda: len(fake14.written) == len(_blob_off_v), 8.0)
      and 0xC3 not in bytes(fake14.written),
      bytes(fake14.written).hex()[:120])
check("★ 而且和「开关关掉」编出来的是同一份字节",
      bytes(fake14.written) == _blob_off_v)

# ---- ⑤ 导出字节流跟着同一个开关走（导出 = 实发，不然核对没意义）----
_real_open_vel = app_mod.filedialog.askopenfilename
app_mod.filedialog.askopenfilename = lambda **kw: _vel_mid
_bin_v = os.path.splitext(_vel_mid)[0] + ".发送字节流.bin"
_txt_v = os.path.splitext(_vel_mid)[0] + ".发送字节流.txt"
gui.midi_vel_var.set(False)
gui.on_midi_vel_change()
gui.export_midi_bytes()
check("★★ 关掉力度时导出的字节流里也没有 0xC3",
      os.path.exists(_bin_v) and 0xC3 not in open(_bin_v, "rb").read(),
      open(_bin_v, "rb").read().hex()[:120] if os.path.exists(_bin_v) else "(无文件)")
check("★★ 关掉力度时导出的字节流 = 实发的那份字节",
      os.path.exists(_bin_v) and open(_bin_v, "rb").read() == _blob_off_v)
check("导出 .txt 的报告里写明了力度是关的",
      os.path.exists(_txt_v) and "力度" in open(_txt_v, encoding="utf-8").read())
gui.midi_vel_var.set(True)
gui.on_midi_vel_change()
gui.export_midi_bytes()
check("★ 打开力度时导出的字节流 = 带 0xC3 的那份",
      os.path.exists(_bin_v) and open(_bin_v, "rb").read() == _blob_v)
check("★ 打开力度的导出 .txt 里能看到逐事件力度那一行",
      "逐事件力度" in open(_txt_v, encoding="utf-8").read())
app_mod.filedialog.askopenfilename = _real_open_vel
for _f in (_bin_v, _txt_v):
    if os.path.exists(_f):
        os.remove(_f)
try:
    os.remove(_vel_mid)
except OSError:
    pass
gui.midi_vel_var.set(True)          # 还原成默认（开着）
gui.on_midi_vel_change()
gui.close_port("力度测试结束")

# ==========================================================================
print("[16] ★ V46/V48.2：LCD 显示模式（0xD0 设定 / 0x5F 回报 / 待确认 / 不回环重发）")
#   协议：上位机 -> 板子 [0xD0][DISP_MODE]，★ V48.2 起 DISP_MODE = 0~4 五档：
#           0=频谱 1=波形 2=李萨如·左右 3=李萨如·干湿 4=李萨如·相位
#         （★ 超出范围【夹到 0~4】，不是取模）；
#         板子 -> 上位机 [0x5F][DISP_MODE][REC_ON]，固定 3 字节，
#         板子在【这两个值变化时】主动发 —— 包括在板子上按按钮 3 / 按钮 10。
#   ★ 这一节同时守两件"不许动"的事：0x5D 效果帧还是 4 字节、没被碰过；
#     以及界面宽度硬约束（<= 980）。

# ---- ① 控件都在布局里、默认值正确、位置不和别的控件挤一行 ----
check("显示模式控件都在布局里（★ 五个单选）",
      all(bool(w.winfo_manager()) for w in gui.disp_btns)
      and len(gui.disp_btns) == 5
      and bool(gui.disp_lbl.winfo_manager()) and bool(gui.disp_rec_lbl.winfo_manager()),
      ([w.winfo_manager() for w in gui.disp_btns], len(gui.disp_btns)))
check("★ 五个档位的文字就是协议里的五个名字（★ V48.2：李萨如三档都在）",
      [b.cget("text") for b in gui.disp_btns] == list(app_mod._DISP_NAMES)
      and len(app_mod._DISP_NAMES) == 5,
      [b.cget("text") for b in gui.disp_btns])
check("★ 三个李萨如档位的按钮文字互不相同（能分出是哪一档）",
      len(set(b.cget("text") for b in gui.disp_btns[2:5])) == 3,
      [b.cget("text") for b in gui.disp_btns[2:5]])
check("★ 默认选中「频谱」（协议里的 0），但小字【不假装】板子就在频谱",
      gui.disp_var.get() == 0 and "未知" in gui.disp_lbl.cget("text"),
      (gui.disp_var.get(), gui.disp_lbl.cget("text")))
#   ★ 录音小字：板子【没报过】时必须写「未知」，不许假装是"关"。
#     注意本节之前的好几节已经喂过 0x5B 了，所以这里不能直接断言"未知"——
#     先把 rec_on 临时退回 None，验完再还原（验的就是"None -> 未知"这条映射）。
_save_rec_on = gui.rec.rec_on
gui.rec.rec_on = None
gui._update_disp_ui()
check("板子还没报过录音时，小字写「录音: 未知」（不假装是关）",
      gui.disp_rec_lbl.cget("text") == "录音: 未知", gui.disp_rec_lbl.cget("text"))
gui.rec.rec_on = _save_rec_on
gui._update_disp_ui()
check("板子报过之后，小字就按板子报的值显示",
      gui.disp_rec_lbl.cget("text") == "录音: "
      + app_mod.fpga_rec.rec_disp_name(_save_rec_on),
      (gui.disp_rec_lbl.cget("text"), _save_rec_on))
# ★ 用户明确说过：不要和「音色」挤一行，也不要和节拍器/MIDI 力度那两行挤
check("★ 显示控件不和「音色」控件在同一行",
      gui.disp_lbl.master is not gui.tone_btns[0].master,
      (str(gui.disp_lbl.master), str(gui.tone_btns[0].master)))
check("★ 显示控件不在节拍器那一行里",
      gui.disp_lbl.master is not gui.metro_lbl.master,
      (str(gui.disp_lbl.master), str(gui.metro_lbl.master)))
check("★ 显示控件也不在 MIDI 力度那一行里",
      gui.disp_lbl.master is not gui.vel_lbl.master,
      (str(gui.disp_lbl.master), str(gui.vel_lbl.master)))
#   ★ 加了这一行之后窗口仍然放得下（硬约束：<= 980）
root.update_idletasks()
check("★ 加上「显示」这一整行后，窗口需要的宽度还是没超过 980",
      root.winfo_reqwidth() <= 980, root.winfo_reqwidth())

# ---- ② 点一下「波形」：先攒 0.35 秒，再发 D0 01 ----
app_mod.serial.Serial = FakeSerial
gui.open_port()
check("打开串口(显示模式测试)", pump_until(lambda: gui.is_open()))
fake15 = gui.ser
gui.fx_auto_query = False
#   先让"连上自动下发按键映射"那批字节落地，否则下面"一个字节都不该有"的
#   断言会被它搞挂（这类竞态踩过）。
pump_until(lambda: False, 0.6)
gui.disp_unsent = False
gui.disp_dirty_at = 0.0
fake15.written = bytearray()

gui.disp_var.set(1)                      # 波形
gui.on_disp_change()
check("★ 点完不是立刻发（手滑连点三个按钮不会连发三条）",
      pump_until(lambda: False, 0.2) is False and bytes(fake15.written) == b"",
      bytes(fake15.written))
check("★★ 攒够 0.35 秒后发出 D0 01（波形）—— 逐字节一致",
      pump_until(lambda: bytes(fake15.written) == b"\xd0\x01", 3.0),
      bytes(fake15.written))
check("发出去之后是「待确认」", gui.disp_pending is not None, gui.disp_pending)
check("小字上写着在等板子确认", "待板子确认" in gui.disp_lbl.cget("text"),
      gui.disp_lbl.cget("text"))

# ---- ③ 板子回报 0x5F：模式对上才撤「待确认」，控件同步成板子的值 ----
#   ★ 故意拆成两批送：3 个字节完全可能落在两个串口批次里。
gui.clear_rx()                           # 让下面的日志断言从零开始
gui.rxq.put(("data", bytes([0x5F, 1])))
check("0x5F 被切两半时不乱猜（disp_mode 还是 None）",
      pump_until(lambda: gui.rec.disp_mode is None, 1.0), gui.rec.disp_mode)
gui.rxq.put(("data", bytes([0x01])))
check("后半截到了就解析出来（波形 + 录音开）",
      pump_until(lambda: (gui.rec.disp_mode, gui.rec.rec_on) == (1, True), 3.0),
      (gui.rec.disp_mode, gui.rec.rec_on))
check("★ 模式对上，「待确认」才撤掉",
      pump_until(lambda: gui.disp_pending is None, 3.0), gui.disp_pending)
check("日志里看到「已确认」", "已确认" in rx_now(), repr(rx_now()[-160:]))
check("★ 录音小字跟着这一帧 0x5F 变成「录音: 开」",
      gui.disp_rec_lbl.cget("text") == "录音: 开", gui.disp_rec_lbl.cget("text"))
check("★ 小字里显示的是板子的值（板子：显示 波形）",
      "波形" in gui.disp_lbl.cget("text"), gui.disp_lbl.cget("text"))

# ---- ④ ★★ 回环守卫：板子回报回写控件，不能再被当成"用户点击"发一遍 ★★ ----
pump_until(lambda: False, 0.4)
fake15.written = bytearray()
gui.rxq.put(("data", bytes([0x5F, 2, 0])))
check("板子那边切成了李萨如（还顺手关了录音）—— 单选按钮跟着同步",
      pump_until(lambda: gui.disp_var.get() == 2, 3.0), gui.disp_var.get())
check("★ 录音小字跟着变成「录音: 关」",
      gui.disp_rec_lbl.cget("text") == "录音: 关", gui.disp_rec_lbl.cget("text"))
check("★ 回写控件没有触发回环重发（板子回报之后线上一个字节都没有）",
      pump_until(lambda: False, 0.6) is False and bytes(fake15.written) == b"",
      bytes(fake15.written))
#   ★ 再直接验一次那道闸本身：disp_syncing 立着的时候改动不许发出去
#     （照抄 metro_syncing 的那个用法）
gui.disp_syncing = True
gui.disp_var.set(0)
gui.on_disp_change()
gui.disp_syncing = False
check("★ disp_syncing 立着时改动被挡住（闸本身有效，不只是靠控件不触发）",
      pump_until(lambda: False, 0.5) is False and bytes(fake15.written) == b""
      and gui.disp_dirty_at == 0.0,
      (bytes(fake15.written), gui.disp_dirty_at))
gui.disp_var.set(2)                      # 还原成板子报的那个值

# ---- ⑤ 0x5F 不影响 0x5D（★ 4 字节效果帧一个字节都没被动过）----
gui.rxq.put(("data", app_mod.fpga_rec.pack_mode_frame(1)))
check("0x5D 的 4 字节帧照旧解析（跟谱模式 = 1）",
      pump_until(lambda: gui.rec.wait_mode == 1, 3.0), gui.rec.wait_mode)
gui.rxq.put(("data", bytes([0x5F, 0, 1])))
check("0x5D 之后紧跟 0x5F：显示模式照常解析",
      pump_until(lambda: (gui.rec.disp_mode, gui.rec.rec_on) == (0, True), 3.0),
      (gui.rec.disp_mode, gui.rec.rec_on))
check("★ 0x5F 没把 0x5D 的效果/音色/跟谱模式冲掉",
      (gui.rec.fx_state, gui.rec.tone_sel, gui.rec.wait_mode) == (0, 0, 1),
      (gui.rec.fx_state, gui.rec.tone_sel, gui.rec.wait_mode))

# ---- ⑥ 板子不回报：超时如实标「未确认」 ----
pump_until(lambda: False, 0.5)
fake15.written = bytearray()
gui.disp_var.set(2)                      # 李萨如（和板子当前报的 0 不同）
gui.on_disp_change()
check("先发出去（D0 02 = 李萨如）",
      pump_until(lambda: bytes(fake15.written) == b"\xd0\x02", 3.0),
      bytes(fake15.written))
gui.disp_pending["t"] -= 2.0             # 假装已经过了 1.2 秒
gui._disp_pending_check()
check("超时后撤掉「待确认」", gui.disp_pending is None, gui.disp_pending)
check("★ 超时后界面如实写「未确认」（不谎称设置成功）",
      "未确认" in gui.disp_lbl.cget("text"), gui.disp_lbl.cget("text"))
check("超时后日志里说明了原因",
      "还没回报显示模式状态" in rx_now(), repr(rx_now()[-200:]))

# ---- ⑦ 串口没开时：只记下不崩，连上后自动补发 ----
gui.close_port("显示模式测试结束")
pump_until(lambda: False, 0.3)
fake15.written = bytearray()
gui.disp_var.set(0)                      # 频谱
gui.on_disp_change()
check("串口没开时点显示模式不崩，并标成「还没发出去」",
      gui.disp_unsent is True and bytes(fake15.written) == b"",
      (gui.disp_unsent, bytes(fake15.written)))
check("小字上说明了「改动还没发出去」",
      "还没发出去" in gui.disp_lbl.cget("text"), gui.disp_lbl.cget("text"))
gui.open_port()
check("重新打开串口(显示模式测试)", pump_until(lambda: gui.is_open()))
fake16 = gui.ser
check("连上后自动补发那条没发出去的改动（D0 00 = 频谱）",
      pump_until(lambda: b"\xd0\x00" in bytes(fake16.written), 3.0),
      bytes(fake16.written))
check("连上后小字退回「板子显示未知」（协议里没有查询显示模式的命令）",
      "未知" in gui.disp_lbl.cget("text"), gui.disp_lbl.cget("text"))
gui.close_port("显示模式测试结束")

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
