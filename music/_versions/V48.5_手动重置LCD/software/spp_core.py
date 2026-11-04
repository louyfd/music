# -*- coding: utf-8 -*-
"""蓝牙串口助手 —— 核心代码（无 GUI，可直接嵌进你自己的脚本）

这是 spp_assistant.py 里跟界面无关的那部分，单独抽出来，方便复用：

  * 蓝牙 SPP 端口发现（关键字识别 + BTHENUM/UUID 精确判定 + 从注册表读对端设备名）
  * 串口收发：后台线程 + 独立发送队列，保证「不丢字节、不乱序」
  * MIDI 消息组装（Note On/Off、CC、Program、Pitch Bend、Aftertouch、SysEx、实时消息）
  * MIDI 文件（.mid/.midi/.kar/.syx）逐条流式发送，可按原速或全速，支持帧尾
  * 可选帧尾（发送完追加一次，给按帧解析的自定义协议用）

依赖：
    pyserial   必需      python -m pip install pyserial
    mido       仅「发送 MIDI 文件」需要   python -m pip install mido

命令行自测：
    python spp_core.py list                      列出所有端口（标出蓝牙出站/入站）
    python spp_core.py listen                    自动连蓝牙口并打印收到的数据
    python spp_core.py hex "90 3C 64"            发一串 hex
    python spp_core.py note C4 100 --channel 1   发一个 Note On
    python spp_core.py file MIDI/test.mid        发一个 MIDI 文件（按原速）
    python spp_core.py file MIDI/test.mid --fast --tail FF
"""

import argparse
import os
import re
import sys
import threading
import time
import queue
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except Exception:                                   # pragma: no cover
    serial = None
    list_ports = None

try:
    import mido
except Exception:
    mido = None


# ==========================================================================
# 基础工具
# ==========================================================================
BT_SPP_UUID = "00001101-0000-1000-8000-00805F9B34FB"
BT_KEYWORDS = ['bluetooth', 'bt', '蓝牙', 'rfcomm', 'spp']
IDLE_FLUSH = 0.05        # 「空闲合并」模式：空闲超过这么多秒就把缓冲作为一批投递
MAX_CHUNK = 8192         # 单批上限


def fmt_hex(data):
    """bytes -> '90 3C 64'"""
    return " ".join("%02X" % b for b in data)


def parse_hex(text):
    """'90 3C 64' / '903C64' / '0x90,0x3C' -> bytes；非法时抛 ValueError。"""
    cleaned = re.sub(r"[\s,;]+", "", str(text))
    cleaned = re.sub(r"0[xX]", "", cleaned)
    if not cleaned:
        return b""
    if not re.fullmatch(r"[0-9A-Fa-f]+", cleaned):
        raise ValueError("包含非十六进制字符, 只能出现 0-9 A-F")
    if len(cleaned) % 2:
        raise ValueError("十六进制字符个数是奇数, 无法组成完整字节")
    return bytes.fromhex(cleaned)


def write_all(ser, data):
    """把 data 一个字节不落地写进串口，返回实际写入字节数。

    pyserial 的 write() 允许只写进去一部分（返回值 < len(data)），
    不看返回值就会**静默丢字节**。这里循环写完，写不动就抛异常。
    """
    total = 0
    size = len(data)
    while total < size:
        n = ser.write(data[total:])
        if not n:
            raise IOError("串口写入返回 0 字节 (已写 {}/{} 字节)".format(total, size))
        total += n
    return total


# ==========================================================================
# 蓝牙端口识别
# ==========================================================================
_BT_NAME_CACHE = {}


def bt_keyword_match(info):
    """按关键字识别蓝牙口（原 SPP.py 的做法）。"""
    desc = (getattr(info, "description", "") or "").lower()
    hwid = (getattr(info, "hwid", "") or "").lower()
    name = (getattr(info, "name", "") or "").lower()
    return any(k in desc or k in hwid or k in name for k in BT_KEYWORDS)


def is_bluetooth_port(info):
    """精确判定（BTHENUM / SPP UUID）与关键字判定取并集。"""
    hwid = (getattr(info, "hwid", "") or "").upper()
    if ("BTHENUM" in hwid) or (BT_SPP_UUID in hwid):
        return True
    return bt_keyword_match(info)


def bluetooth_mac(info):
    """从 BTHENUM 硬件 ID 里取出对端设备 MAC；入站口返回空串。"""
    hwid = getattr(info, "hwid", "") or ""
    for part in hwid.split("&"):
        token = part.split("_")[0].strip()
        if re.fullmatch(r"[0-9A-Fa-f]{12}", token):
            if token.upper() == "0" * 12:      # 全 0 = 本机入站口，没有对端
                return ""
            return ":".join(token[i:i + 2] for i in range(0, 12, 2)).upper()
    return ""


def bt_device_name(mac):
    """从注册表读对端设备名（例如 JDY-31-SPP）。读不到返回空串。"""
    if not mac:
        return ""
    if mac in _BT_NAME_CACHE:
        return _BT_NAME_CACHE[mac]
    name = ""
    try:
        import winreg
        path = r"SYSTEM\CurrentControlSet\Enum\BTHENUM\Dev_" + mac.replace(":", "")
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path) as key:
            for i in range(winreg.QueryInfoKey(key)[0]):
                try:
                    with winreg.OpenKey(key, winreg.EnumKey(key, i)) as sub:
                        val, _ = winreg.QueryValueEx(sub, "FriendlyName")
                        if val:
                            name = str(val)
                            break
                except OSError:
                    continue
    except Exception:
        name = ""
    _BT_NAME_CACHE[mac] = name
    return name


def port_label(info):
    """端口描述。蓝牙口按 MAC 区分——Windows 会重新分配蓝牙 SPP 的 COM 编号，
    同一个模块今天在 COM12、明天可能在 COM17，认 COM 号会认错口。"""
    mac = bluetooth_mac(info)
    if not is_bluetooth_port(info):
        desc = (getattr(info, "description", "") or "").strip()
        return "{}  [串口] {}".format(info.device, desc).strip()
    if not mac:
        return "{:<6} [蓝牙/入站] 等对方来连".format(info.device)
    return "{:<6} [蓝牙/出站] {} ({})".format(
        info.device, bt_device_name(mac) or "未知设备", mac)


def list_all_ports():
    """列出所有串口（含非蓝牙）。"""
    return sorted(list_ports.comports(), key=lambda i: i.device) if list_ports else []


def find_bluetooth_ports():
    """列出蓝牙口，保持 comports() 原生顺序。"""
    if list_ports is None:
        return []
    return [i for i in list_ports.comports() if is_bluetooth_port(i)]


def pick_bluetooth_port():
    """挑一个蓝牙口：**优先出站口**（带对端 MAC 的那个）。

    入站口打开会「成功」，但其实没连上任何模块——看着像连上了，
    实际一个字节都收不到。只有出站口才会主动去连模块。
    返回 ListPortInfo，找不到返回 None。
    """
    ports = find_bluetooth_ports()
    if not ports:
        return None
    for p in ports:
        if bluetooth_mac(p):
            return p
    return ports[0]


# ==========================================================================
# MIDI 消息组装
# ==========================================================================
NOTE_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]

# (显示名, 状态字节, 数据1 名称, 数据2 名称)
MIDI_SPECS = [
    ("Note On (90)", 0x90, "音符号", "力度"),
    ("Note Off (80)", 0x80, "音符号", "力度"),
    ("Control Change (B0)", 0xB0, "控制器号", "值"),
    ("Program Change (C0)", 0xC0, "音色号", None),
    ("Pitch Bend (E0)", 0xE0, "低7位LSB", "高7位MSB"),
    ("Channel Aftertouch (D0)", 0xD0, "压力值", None),
    ("Poly Aftertouch (A0)", 0xA0, "音符号", "压力值"),
    ("SysEx (F0..F7)", 0xF0, "载荷HEX", None),
    ("系统实时 时钟 (F8)", 0xF8, None, None),
    ("系统实时 开始 (FA)", 0xFA, None, None),
    ("系统实时 继续 (FB)", 0xFB, None, None),
    ("系统实时 停止 (FC)", 0xFC, None, None),
]
MIDI_TYPE_NAMES = [s[0] for s in MIDI_SPECS]
_MIDI_BY_NAME = dict((s[0], s) for s in MIDI_SPECS)
NOTE_FIELDS = ("音符号",)


def number_to_note(n):
    """60 -> 'C4'（中央 C = C4 = 60，科学音高记法）"""
    return "{}{}".format(NOTE_NAMES[n % 12], n // 12 - 1)


def note_to_number(text):
    """'C4' / 'C#4' / 'Db4' / '60' -> 0..127，非法时抛 ValueError。"""
    s = str(text).strip()
    if not s:
        raise ValueError("音符号为空")
    if re.fullmatch(r"[+-]?\d+", s):
        n = int(s)
        if 0 <= n <= 127:
            return n
        raise ValueError("音符号 {} 超出 0..127".format(n))
    m = re.fullmatch(r"([A-Ga-g])([#b]?)(-?\d+)", s)
    if not m:
        raise ValueError("音符号 {!r} 不合法, 例: C4 / C#4 / Db4 / 60".format(s))
    semi = NOTE_NAMES.index(m.group(1).upper())
    if m.group(2) == "#":
        semi += 1
    elif m.group(2) == "b":
        semi -= 1
    n = (int(m.group(3)) + 1) * 12 + semi
    if not 0 <= n <= 127:
        raise ValueError("音符号 {!r} 换算为 {} 超出 0..127".format(s, n))
    return n


def _midi_field_value(field_name, text):
    if field_name in NOTE_FIELDS:
        return note_to_number(text)
    s = str(text).strip()
    if not re.fullmatch(r"[+-]?\d+", s):
        raise ValueError("{} 必须是 0..127 的整数".format(field_name))
    n = int(s)
    if not 0 <= n <= 127:
        raise ValueError("{} = {} 超出 0..127".format(field_name, n))
    return n


def build_midi(name, channel=1, d1="", d2=""):
    """把一条 MIDI 消息组装成原始字节，返回 (bytes, 中文说明)。

    通道用界面上的 1..16，内部转成状态字节低 4 位 0..15。
    注意：**不会追加任何帧尾**，发出去的就是消息本身。
    """
    spec = _MIDI_BY_NAME.get(name)
    if spec is None:
        raise ValueError("未知的 MIDI 类型: {}".format(name))
    status, f1, f2 = spec[1], spec[2], spec[3]

    if f1 is None and f2 is None and not name.startswith("SysEx"):
        return bytes([status]), "{}  单字节, 无数据".format(name)

    if name.startswith("SysEx"):
        payload = bytearray(parse_hex(str(d1)))
        if payload[:1] == b"\xf0":
            del payload[:1]
        if payload[-1:] == b"\xf7":
            del payload[-1:]
        bad = [b for b in payload if b > 0x7F]
        if bad:
            raise ValueError("SysEx 载荷只能是 7 位数据 0x00..0x7F, "
                             "发现 0x{:02X}".format(bad[0]))
        data = bytes([0xF0]) + bytes(payload) + bytes([0xF7])
        return data, "SysEx 载荷 {} 字节 (自动补 F0 / F7)".format(len(payload))

    ch = int(channel)
    if not 1 <= ch <= 16:
        raise ValueError("通道必须在 1..16")

    out = bytearray([status | (ch - 1)])
    parts = []
    if f1:
        v = _midi_field_value(f1, d1)
        shown = "{} ({})".format(number_to_note(v), v) if f1 in NOTE_FIELDS else v
        out.append(v)
        parts.append("{}={}".format(f1, shown))
    if f2:
        v = _midi_field_value(f2, d2)
        out.append(v)
        parts.append("{}={}".format(f2, v))
    desc = "{} ch{}  {}".format(name.split(" (")[0], ch, "  ".join(parts))
    return bytes(out), desc


def describe_midi(data):
    """把一段字节翻译成 MIDI 说明；不是合法 MIDI 就返回空串。"""
    if not data:
        return ""
    b0 = data[0]
    if b0 == 0xF0:
        return "SysEx {} 字节".format(len(data)) if data[-1:] == b"\xf7" else ""
    if b0 >= 0xF8:
        return {0xF8: "时钟", 0xFA: "开始", 0xFB: "继续",
                0xFC: "停止", 0xFE: "Active Sensing", 0xFF: "复位"}.get(b0, "系统实时")
    if b0 < 0x80 or len(data) < 2:
        return ""
    kind = b0 & 0xF0
    ch = (b0 & 0x0F) + 1
    names = {0x80: "Note Off", 0x90: "Note On", 0xA0: "Poly Aftertouch",
             0xB0: "Control Change", 0xC0: "Program Change",
             0xD0: "Channel Aftertouch", 0xE0: "Pitch Bend"}
    label = names.get(kind)
    if not label:
        return ""
    if kind in (0xC0, 0xD0):
        return "{} ch{} 值={}".format(label, ch, data[1])
    if len(data) < 3:
        return ""
    if kind in (0x80, 0x90):
        return "{} ch{} 音={} ({}) 力度={}".format(
            label, ch, number_to_note(data[1]), data[1], data[2])
    return "{} ch{} 数据={} {}".format(label, ch, data[1], data[2])


def load_midi_file(path):
    """读 .mid/.midi/.kar/.syx，返回 (可发送的消息列表, 说明文字)。"""
    if mido is None:
        raise RuntimeError("需要 mido 库: python -m pip install mido")
    if str(path).lower().endswith(".syx"):
        msgs = list(mido.read_syx_file(path))
        note = "SysEx 文件"
    else:
        mid = mido.MidiFile(path)
        msgs = list(mid)          # 迭代会合并所有音轨, 且 time 已换算成秒
        note = "{} 音轨, 时长 {:.1f} 秒".format(len(mid.tracks), mid.length)
    return [m for m in msgs if not getattr(m, "is_meta", False)], note


# ==========================================================================
# 串口链路
# ==========================================================================
class SppLink(object):
    """蓝牙串口链路。收发都在后台线程，主线程只管调 send / 收回调。

    用法::

        link = SppLink(on_data=lambda b: print("收到", fmt_hex(b)))
        link.open()                      # 自动挑蓝牙出站口，9600
        link.send_hex("90 3C 64")
        link.send_midi_file("MIDI/test.mid", realtime=True, tail=b"\\xff")
        link.close()

    收数据回调 on_data(data: bytes) 在**后台线程**里被调用，别在里面直接操作 GUI，
    要么丢进队列，要么用 after() 之类的机制切回主线程。
    """

    def __init__(self, on_data=None, log=None):
        self.on_data = on_data
        self.log = log or (lambda msg: None)
        self.ser = None
        self.device = ""
        self.rx_bytes = 0
        self.tx_bytes = 0
        self.tx_log = []              # 真正写出端口的每一批: (时间, 标签, hex)
        self.line_mode = True
        self._stop = threading.Event()
        self._reader = None
        self._writer = None
        self._txq = queue.Queue()
        self._midi_stop = threading.Event()
        self._midi_playing = False

    # ---------------------------------------------------------------- 开关
    @property
    def is_open(self):
        return self.ser is not None

    def open(self, port=None, baudrate=9600, line_mode=True,
             bytesize=8, parity="N", stopbits=1, prefer_bluetooth=True):
        """打开串口。port 传 None 就自动挑蓝牙出站口。

        返回实际使用的端口名；失败抛异常（不吞错误）。
        """
        if serial is None:
            raise RuntimeError("需要 pyserial: python -m pip install pyserial")
        if self.is_open:
            self.close()

        if port is None:
            if not prefer_bluetooth:
                raise ValueError("没给 port 又没开蓝牙优先")
            info = pick_bluetooth_port()
            if info is None:
                raise RuntimeError("未找到蓝牙串口设备，请先在系统里配对蓝牙模块")
            port = info.device
            self.log("自动选择 {}".format(port_label(info)))

        self.line_mode = bool(line_mode)
        # 按行读取用 timeout=1；空闲合并模式用小超时以便及时冲刷缓冲
        read_timeout = 1.0 if self.line_mode else 0.02
        ser = serial.Serial(port=port, baudrate=baudrate, bytesize=bytesize,
                            parity=parity, stopbits=stopbits,
                            timeout=read_timeout, write_timeout=1.0)

        # 每次打开都用全新的 stop 事件和发送队列：
        # 旧会话残留的线程只可能碰它自己的旧队列，不会抢走新会话的数据。
        self._stop = threading.Event()
        self._txq = queue.Queue()
        self.ser = ser
        self.device = port
        self.rx_bytes = 0
        self.tx_bytes = 0

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._writer = threading.Thread(target=self._write_loop, daemon=True)
        self._writer.start()
        self.log("已打开 {} @ {}".format(port, baudrate))
        return port

    def close(self):
        if self.ser is None:
            return
        self._midi_stop.set()
        self._stop.set()
        ser, self.ser = self.ser, None
        try:
            self._txq.put_nowait(None)     # 叫醒停在 get() 上的写线程
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        self.log("已关闭 {}".format(self.device))

    # ---------------------------------------------------------------- 收发
    def _read_loop(self, ser=None):
        ser = ser or self.ser
        evt = self._stop
        if self.line_mode:
            # 按行读取：直接阻塞 readline()。
            # 不要先判断 in_waiting —— 部分蓝牙驱动上它常年返回 0，那样永远读不到数据。
            while not evt.is_set():
                try:
                    data = ser.readline()
                except Exception as exc:
                    if not evt.is_set():
                        self.log("[串口异常] {}".format(exc))
                        self._stop.set()
                    return
                if data:
                    self._deliver(data)
        else:
            # 空闲合并：不依赖换行符，适合二进制流。
            # 不能读一个字节就投递一批 —— 那会把 "HELLO\r\n" 切成两段。
            buf = bytearray()
            last = time.time()
            while not evt.is_set():
                try:
                    waiting = ser.in_waiting
                    data = ser.read(waiting if waiting else 1)
                except Exception as exc:
                    if buf:
                        self._deliver(bytes(buf))
                    if not evt.is_set():
                        self.log("[串口异常] {}".format(exc))
                        self._stop.set()
                    return
                now = time.time()
                if data:
                    buf += data
                    last = now
                if buf and ((now - last) >= IDLE_FLUSH or len(buf) >= MAX_CHUNK):
                    self._deliver(bytes(buf))
                    del buf[:]
                    last = now

    def _deliver(self, data):
        self.rx_bytes += len(data)
        if self.on_data:
            try:
                self.on_data(data)
            except Exception as exc:
                self.log("on_data 回调出错: {}".format(exc))

    def _write_loop(self, ser=None, txq=None):
        ser = ser or self.ser
        txq = txq or self._txq
        evt = self._stop
        while not evt.is_set():
            try:
                item = txq.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:               # close() 放的哨兵
                return
            data, label = item if isinstance(item, tuple) else (item, "")
            try:
                if self.ser is not ser:
                    return                 # 过期会话：这一批随旧队列作废，绝不乱序重排
                written = write_all(ser, data)
            except Exception as exc:
                self.log("发送失败: {}".format(exc))
                return
            # 到这一步才敢记：这批字节是真的出去了
            self.tx_bytes += written
            stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self.tx_log.append((stamp, label, fmt_hex(data)))
            del self.tx_log[:-5000]

    def send(self, data, label=""):
        """把一批字节排进发送队列（后台线程按顺序写出）。"""
        if not self.is_open:
            raise RuntimeError("串口未打开")
        if isinstance(data, str):
            data = data.encode("utf-8")
        self._txq.put((bytes(data), label))

    def send_hex(self, text, label="HEX"):
        """发送 hex 文本，例如 '90 3C 64'。"""
        self.send(parse_hex(text), label)

    def send_midi(self, name, channel=1, d1="", d2="", tail=b""):
        """组装并发送一条 MIDI 消息。tail 非空则在末尾追加（发送完加一次）。"""
        data, desc = build_midi(name, channel, d1, d2)
        if tail:
            data = data + bytes(tail)
            desc += "  + 帧尾 " + fmt_hex(tail)
        self.send(data, "MIDI " + name)
        return data, desc

    # ------------------------------------------------------------ MIDI 文件
    def send_midi_file(self, path, realtime=True, tail=b"", done=None):
        """把一个 .mid/.midi/.kar/.syx 逐条发出去。

        realtime=True 按文件里的时间间隔播放
        tail         整段发完后**追加一次**（不是每条都加）；中途停止则不加
        done(sent, elapsed, stopped, tailed) 在发完后（后台线程）回调
        """
        if not self.is_open:
            raise RuntimeError("串口未打开")
        if self._midi_playing:
            raise RuntimeError("上一次 MIDI 文件还在发送中")
        msgs, note = load_midi_file(path)
        if not msgs:
            raise ValueError("这个文件里没有可发送的消息")

        self._midi_stop.clear()
        self._midi_playing = True
        self.log("载入 {} : {}, {} 条".format(os.path.basename(path), note, len(msgs)))
        threading.Thread(
            target=self._midi_worker,
            args=(path, msgs, bool(realtime), bytes(tail), done), daemon=True).start()

    def stop_midi_file(self):
        self._midi_stop.set()

    def _midi_worker(self, path, msgs, realtime, tail, done):
        sent = 0
        t0 = time.time()
        name = os.path.basename(path)
        for msg in msgs:
            if self._midi_stop.is_set() or not self.is_open:
                break
            if realtime:
                delay = float(getattr(msg, "time", 0) or 0)
                if delay > 0 and self._midi_stop.wait(delay):
                    break
            try:
                # 注意：mido 的 Message.bytes() 返回的是 list[int]，不是 bytes
                # （取 bytes 要用 .bin()）。直接交给 serial.write() 会抛异常。
                data = bytes(msg.bytes())
            except Exception:
                continue
            self._txq.put((data, "MIDI文件 {} #{}".format(name, sent + 1)))
            sent += 1

        stopped = self._midi_stop.is_set() or not self.is_open
        tailed = False
        if tail and not stopped:
            self._txq.put((tail, "MIDI文件 {} 帧尾".format(name)))
            tailed = True
        elapsed = time.time() - t0
        self._midi_playing = False
        self.log("{} {}: 共 {} 条, 用时 {:.1f} 秒{}".format(
            name, "已停止" if stopped else "发送完成", sent, elapsed,
            ", 已追加帧尾" if tailed else (", 中途停止, 未追加帧尾" if stopped else "")))
        if done:
            try:
                done(sent, elapsed, stopped, tailed)
            except Exception as exc:
                self.log("done 回调出错: {}".format(exc))


# ==========================================================================
# 命令行自测
# ==========================================================================
def _cli():
    ap = argparse.ArgumentParser(description="蓝牙串口助手核心代码 —— 命令行自测")
    ap.add_argument("--port", help="串口号，不填则自动挑蓝牙出站口")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--tail", default="", help="帧尾 hex，例如 FF；默认不加")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="列出所有端口")
    sub.add_parser("listen", help="连上并打印收到的数据（Ctrl+C 结束）")
    p_hex = sub.add_parser("hex", help="发 hex")
    p_hex.add_argument("data")
    p_note = sub.add_parser("note", help="发 Note On")
    p_note.add_argument("note")
    p_note.add_argument("velocity", nargs="?", default="100")
    p_note.add_argument("--channel", default="1")
    p_file = sub.add_parser("file", help="发 MIDI 文件")
    p_file.add_argument("path")
    p_file.add_argument("--fast", action="store_true", help="全速发送，不等时值")
    args = ap.parse_args()

    if args.cmd == "list":
        ports = list_all_ports()
        if not ports:
            print("没找到任何串口")
            return 0
        for p in ports:
            print("  {}".format(port_label(p)))
        return 0

    def on_data(data):
        text = data.decode("utf-8", errors="replace").rstrip("\r\n")
        print("[{}] {}    |  {}".format(
            datetime.now().strftime("%H:%M:%S"), fmt_hex(data), text))

    link = SppLink(on_data=on_data, log=lambda m: print("* " + m))
    tail = parse_hex(args.tail) if args.tail else b""
    try:
        link.open(port=args.port, baudrate=args.baud)
    except Exception as exc:
        print("打开失败: {}".format(exc))
        return 1

    try:
        if args.cmd == "listen":
            print("正在接收，Ctrl+C 结束 ...")
            while True:
                time.sleep(0.5)
        elif args.cmd == "hex":
            data = parse_hex(args.data)
            link.send(data + tail, "HEX")
            print("已发: {}{}".format(fmt_hex(data),
                                    "  + 帧尾 " + fmt_hex(tail) if tail else ""))
        elif args.cmd == "note":
            data, desc = link.send_midi("Note On (90)", args.channel,
                                        args.note, args.velocity, tail=tail)
            print("已发: {}   ({})".format(fmt_hex(data), desc))
        elif args.cmd == "file":
            done_evt = threading.Event()
            link.send_midi_file(args.path, realtime=not args.fast, tail=tail,
                                done=lambda *a: done_evt.set())
            done_evt.wait()
        time.sleep(0.5)          # 给写线程一点时间把队列清干净
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(_cli())
