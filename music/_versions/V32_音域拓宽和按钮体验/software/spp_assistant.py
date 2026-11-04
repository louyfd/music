# -*- coding: utf-8 -*-
"""SPP 蓝牙串口助手 —— 收发整合页面

功能概览:
  * 端口: 自动扫描串口/蓝牙 SPP 口, 支持刷新、仅显示蓝牙口、解析并显示蓝牙 MAC
  * 参数: 波特率 / 数据位 / 校验 / 停止位 / 流控 / 编码
  * 接收: 文本或 HEX, 时间戳, 自动滚动, 暂停, 清空, 保存日志, RX 字节计数
  * 发送: 文本或 HEX, 行尾(无/LF/CR/CRLF), 定时重复发送, 发送历史
  * MIDI: 组装 MIDI 消息 / 发送 .mid 文件(默认转成 FPGA 乐谱帧整帧发)
  * 演奏录音回传: 板子手弹的内容回传上来 -> 合成 MIDI -> 在这台电脑上播放
  * 蓝牙: 一键打开 Windows 蓝牙设置
  * 配置自动记忆到程序目录 spp_config.json

依赖: pyserial (tkinter 随 Python 自带)
"""

import json
import os
import queue
import re
import subprocess
import sys
import threading
import time
from datetime import datetime

import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
from tkinter import ttk

try:
    import serial
    from serial.tools import list_ports
except Exception:  # pragma: no cover - 仅在缺依赖时走到
    serial = None
    list_ports = None

try:
    import mido
except Exception:  # 没装 mido 也能用, 只是「发送MIDI文件」不可用
    mido = None

# ★ 乐谱 -> FPGA 帧的转换器（同目录 fpga_midi.py / fpga_proto.py）
#   串口助手的「发送MIDI文件」不再是"把裸 MIDI 原样转发" ——
#   板子只认 uart_packet_echo.v 那套帧（帧头 AA / 帧尾 FF / 键号 0x00~0x23）。
try:
    import fpga_midi
    import fpga_proto
except Exception:  # pragma: no cover
    fpga_midi = None
    fpga_proto = None

# ★ 板子演奏录音的回传解析 + 合成（同目录 fpga_rec.py）
#   板子（uart_packet_echo.v）每 10ms 采一次 key_mask，一变就回传
#   [0x5A][mask 36位 5字节][dt 2字节]，开关状态回 [0x5B][0/1]，
#   上位机发 [0x5C] 就能翻转它的录音开关 —— 两端都能开关。
try:
    import fpga_rec
except Exception:  # pragma: no cover
    fpga_rec = None

# ★ 演奏按键自定义映射（同目录 keymap.py）—— 按键 -> 音符，纯软件编辑 + 下发
try:
    import keymap
except Exception:  # pragma: no cover
    keymap = None


def key_no_name(k):
    """键号 -> 音名（拿不到映射表就只显示数字）。键号是接线顺序, 不是音高顺序。"""
    try:
        return fpga_proto.key_name(k)
    except Exception:                                       # noqa: BLE001
        return "?"


APP_NAME = "SPP 蓝牙串口助手"
APP_VERSION = "1.2"
CONFIG_NAME = "spp_config.json"
LOG_PREFIX = "spp_log"
MIDI_SUBDIR = "MIDI"          # MIDI 文件的默认目录, 在程序目录下
BT_SPP_UUID = "00001101-0000-1000-8000-00805F9B34FB"

BAUD_CHOICES = ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]
BYTESIZE_CHOICES = ["8", "7", "6", "5"]
PARITY_CHOICES = ["无", "偶校验", "奇校验", "标记", "空格"]
STOPBITS_CHOICES = ["1", "1.5", "2"]
FLOW_CHOICES = ["无", "RTS/CTS", "XON/XOFF", "DSR/DTR"]
ENCODING_CHOICES = ["UTF-8", "GBK", "GB2312", "ASCII", "Latin-1"]
LINEEND_CHOICES = ["无", "LF(\\n)", "CR(\\r)", "CRLF(\\r\\n)"]

MAX_RX_CHARS = 400000  # 接收区文本上限, 超出后裁掉前半部分
IDLE_FLUSH = 0.05      # 接收合并: 空闲超过该秒数才把缓冲作为一批投递
TX_ECHO_MAX = 64       # 发送回显单行最多显示多少字节(超出只显头尾)
TX_SEND_CHUNK = 256    # 整帧发送的块大小
MAX_CHUNK = 8192       # 单批上限, 防止连续高速流把缓冲撑爆

# 演奏录音回传
REC_ACK_TIMEOUT = 0.8  # 发 0x5C 后等板子回 0x5B 的秒数, 超时就把勾选框退回去
REC_BPM = 100          # 合成 MIDI 时写的速度记号

# ---- ★ V31：效果开关按钮上的短标签（统一 2 个汉字，宽度才排得齐）----
#   ★ 为什么不直接用 fpga_rec.FX_BITS 里的正式名字：
#     · bit1 的正式名字是"延音锁存"（4 个字）——实测文字宽 48px、连边距要 54px，
#       塞进等宽按钮里会被裁掉一截（用户报过"排版不太好"就是这个）；
#     · bit2 的正式名字是"音色"，和右边的音色三选一（普通/笛子/吉他）撞名。
#   所以按钮上用短名，完整名字留给状态小字和日志。
FX_BTN_LABELS = {0: "滑音", 1: "延音", 2: "谐波", 3: "颤音",
                 4: "延迟", 6: "混响", 7: "合唱"}
#   ★ 它只影响谱面记号, 不影响播放快慢 —— 事件的时间戳本来就是真实毫秒,
#     换算成 tick 时用的也是这个 BPM, 一进一出正好抵消, 放出来还是原速。


# --------------------------------------------------------------------------
# 工具函数
# --------------------------------------------------------------------------
def app_dir():
    """返回程序所在目录 (打包成 exe 后为 exe 所在目录)。"""
    if getattr(sys, "frozen", False):
        return os.path.dirname(os.path.abspath(sys.executable))
    return os.path.dirname(os.path.abspath(__file__))


def midi_folder():
    """MIDI 文件默认目录。

    打包后程序在 <项目>\\SPP_Assistant\\ 里, 而 MIDI 文件按约定放在
    <项目>\\MIDI, 所以往上一级找; 直接跑源码时程序就在 <项目>\\, 用自身目录。
    其它摆放方式则退回 exe 同级的 MIDI 目录。
    """
    candidates = []
    if getattr(sys, "frozen", False):
        if os.path.basename(app_dir()).lower() == "spp_assistant":
            candidates.append(os.path.dirname(app_dir()))
    candidates.append(app_dir())
    for base in candidates:
        if not base:
            continue
        folder = os.path.join(base, MIDI_SUBDIR)
        try:
            os.makedirs(folder, exist_ok=True)
            return folder
        except OSError:
            continue
    return os.path.join(app_dir(), MIDI_SUBDIR)


def load_config():
    path = os.path.join(app_dir(), CONFIG_NAME)
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
        if isinstance(data, dict):
            return data
    except Exception:
        pass
    return {}


def save_config(cfg):
    path = os.path.join(app_dir(), CONFIG_NAME)
    try:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(cfg, fh, ensure_ascii=False, indent=2)
    except Exception:
        pass


BT_KEYWORDS = ['bluetooth', 'bt', '蓝牙', 'rfcomm', 'spp']


def bt_keyword_match(info):
    """SPP.py 的蓝牙识别逻辑(实测可用), 原样保留。"""
    desc = (getattr(info, "description", "") or "").lower()
    hwid = (getattr(info, "hwid", "") or "").lower()
    name = (getattr(info, "name", "") or "").lower()
    return any(k in desc or k in hwid or k in name for k in BT_KEYWORDS)


def is_bluetooth_port(info):
    """精确判定(BTHENUM / SPP UUID) 与 SPP.py 的关键字判定取并集。"""
    hwid = (getattr(info, "hwid", "") or "").upper()
    if ("BTHENUM" in hwid) or (BT_SPP_UUID in hwid):
        return True
    return bt_keyword_match(info)


def find_bluetooth_ports():
    """按 SPP.py 的方式列出蓝牙口, 保持 comports() 原生顺序。

    第一个元素就是 SPP.py 里 bt_ports[0] 会去连的那个口, 顺序不能改,
    否则自动连接会连到另一个口。
    """
    if list_ports is None:
        return []
    return [i for i in list_ports.comports() if is_bluetooth_port(i)]


def bluetooth_mac(info):
    """从 BTHENUM 硬件 ID 中提取蓝牙 MAC, 取不到返回空串。"""
    hwid = getattr(info, "hwid", "") or ""
    for part in hwid.split("&"):
        token = part.split("_")[0].strip()
        if re.fullmatch(r"[0-9A-Fa-f]{12}", token):
            if token.upper() == "0" * 12:  # 入站端口没有真实 MAC
                return ""
            return ":".join(token[i:i + 2] for i in range(0, 12, 2)).upper()
    return ""


_BT_NAME_CACHE = {}


def bt_device_name(mac):
    """从注册表读对端蓝牙设备的友好名(例如 JDY-31-SPP)。读不到返回空串。"""
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
    """生成下拉框里显示的端口描述。

    蓝牙口一律按 MAC 区分: Windows 会重新分配蓝牙 SPP 的 COM 编号,
    同一个模块今天在 COM12、明天可能在 COM17, 认 COM 号会认错口。
    出站口(带对端 MAC)打开时会主动去连那个模块;
    入站口(MAC 全 0)只是等对方来连, 等不到就报 121 信号灯超时。
    """
    mac = bluetooth_mac(info)
    if not is_bluetooth_port(info):
        desc = (getattr(info, "description", "") or "").strip()
        return "{}  [串口] {}".format(info.device, desc).strip()
    if not mac:
        return "{:<6} [蓝牙/入站] 等对方来连".format(info.device)
    name = bt_device_name(mac)
    return "{:<6} [蓝牙/出站] {} ({})".format(info.device, name or "未知设备", mac)


def fmt_hex(data):
    return " ".join("%02X" % b for b in data)


def write_all(ser, data):
    """把 data 一个字节不落地写进串口, 返回实际写入字节数。

    pyserial 的 write() 允许只写进去一部分(返回值 < len(data)), 不管返回值
    就会静默丢字节 —— 这是"发出去的数据不对"最隐蔽的来源。这里循环写完,
    真的写不动就抛异常, 由上层报错, 绝不装作成功。
    """
    total = 0
    size = len(data)
    while total < size:
        n = ser.write(data[total:])
        if not n:
            raise IOError("串口写入返回 0 字节 (已写 {}/{} 字节)".format(total, size))
        total += n
    return total


def parse_hex(text):
    """把用户输入的十六进制串转成 bytes, 非法时抛 ValueError。"""
    cleaned = re.sub(r"[\s,;]+", "", text)
    cleaned = re.sub(r"0[xX]", "", cleaned)
    if not cleaned:
        return b""
    if not re.fullmatch(r"[0-9A-Fa-f]+", cleaned):
        raise ValueError("包含非十六进制字符, 只能出现 0-9 A-F")
    if len(cleaned) % 2:
        raise ValueError("十六进制字符个数是奇数, 无法组成完整字节")
    return bytes.fromhex(cleaned)


def decode_bytes(data, encoding):
    """按指定编码解码, 失败字符用替代符显示, 并统一换行。"""
    try:
        text = data.decode(encoding, errors="replace")
    except LookupError:
        text = data.decode("utf-8", errors="replace")
    return text.replace("\r\n", "\n").replace("\r", "\n")


# --------------------------------------------------------------------------
# MIDI: 组装成 hex 字节发出去
# --------------------------------------------------------------------------
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
MIDI_NOTE_CHOICES = None  # 延迟构造, 见 note_choices()


def note_choices():
    """0..127 对应的音名列表, 用于下拉候选。"""
    global MIDI_NOTE_CHOICES
    if MIDI_NOTE_CHOICES is None:
        MIDI_NOTE_CHOICES = [number_to_note(i) for i in range(128)]
    return MIDI_NOTE_CHOICES


def number_to_note(n):
    """60 -> 'C4' (中央 C = C4 = 60, 科学音高记法)。"""
    return "{}{}".format(NOTE_NAMES[n % 12], n // 12 - 1)


def note_to_number(text):
    """'C4' / 'C#4' / 'Db4' / '60' -> 0..127, 非法时抛 ValueError。"""
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
    """把一条 MIDI 消息组装成原始字节, 返回 (bytes, 中文说明)。

    通道用 1..16(界面上的说法), 内部转成状态字节的低 4 位 0..15。
    """
    spec = _MIDI_BY_NAME.get(name)
    if spec is None:
        raise ValueError("未知的 MIDI 类型: {}".format(name))
    status, f1, f2 = spec[1], spec[2], spec[3]

    if f1 is None and f2 is None and not name.startswith("SysEx"):
        return bytes([status]), "{}  单字节, 无数据".format(name)

    if name.startswith("SysEx"):
        payload = bytearray(parse_hex(str(d1)))
        if payload[:1] == b"\xf0":       # 用户多写了首字节就吃掉
            del payload[:1]
        if payload[-1:] == b"\xf7":      # 尾字节同理
            del payload[-1:]
        bad = [b for b in payload if b > 0x7F]
        if bad:
            raise ValueError("SysEx 载荷只能是 7 位数据 0x00..0x7F, "
                             "发现 0x{:02X}".format(bad[0]))
        data = bytes([0xF0]) + bytes(payload) + bytes([0xF7])
        return data, "SysEx 载荷 {} 字节 (自动补 F0 / F7)".format(len(payload))

    try:
        ch = int(channel)
    except (TypeError, ValueError):
        raise ValueError("通道必须是 1..16 的整数")
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
    """把一段字节翻译成 MIDI 说明; 不是合法 MIDI 就返回空字符串。"""
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


def install_excepthook(root):
    """打包成无控制台 exe 后, 异常要能看到, 否则窗口会静默消失。"""
    log_path = os.path.join(app_dir(), "spp_error.log")

    def record(txt):
        try:
            with open(log_path, "a", encoding="utf-8") as fh:
                fh.write("\n[{}]\n{}\n".format(datetime.now().isoformat(), txt))
        except Exception:
            pass
        try:
            messagebox.showerror(APP_NAME, txt[-1500:])
        except Exception:
            pass

    def hook(exc_type, exc, tb):
        import traceback
        record("".join(traceback.format_exception(exc_type, exc, tb)))

    sys.excepthook = hook
    root.report_callback_exception = lambda *a: hook(*a)


# --------------------------------------------------------------------------
# 主界面
# --------------------------------------------------------------------------
class SppAssistant(object):
    def __init__(self, root):
        self.root = root
        self.cfg = load_config()

        # 串口状态
        self.ser = None
        self.io_lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.reader = None
        self.writer = None
        self.rxq = queue.Queue()
        self.txq = queue.Queue()
        self.uiqueue = queue.Queue()   # 后台线程 -> UI 线程 的回调

        # 计数与状态
        self.rx_bytes = 0
        self.tx_bytes = 0
        self.paused = False
        self.last_error = ""
        self.timer_job = None
        self.history = list(self.cfg.get("history", []))[:30]
        self.port_map = {}          # 显示文本 -> device
        self.port_info = {}         # 显示文本 -> ListPortInfo
        self.line_mode = True       # 打开时的读取模式快照
        self._opening = False
        self._open_seq = 0          # 打开请求序号: 只认最新一次, 过期结果丢弃
        self._midi_stop = threading.Event()
        self._midi_playing = False
        self.tx_log = []            # 真正发出去的每一批字节: (时间, 标签, hex)
        self._closing = False

        # 演奏录音回传
        self.rec = fpga_rec.RecParser() if fpga_rec else None
        self.rec_on = False         # 板子回报的录音开关(板子是权威)
        self.rec_pending = 0.0      # 刚发过 0x5C、还没等到 0x5B 的时刻
        self._host_playing = False
        self._host_stop = threading.Event()

        self._build_ui()
        self._apply_config()
        # ★ V31：连上后自动查一次效果/音色状态（自检里会关掉）
        self.fx_auto_query = True
        # ★ V31.1：体验优化用的状态
        self.fx_pending = None       # 刚发出、还没等到板子回报的那次操作
        self._pending_retry = False  # 超时后只自动重问一次
        self.fx_query_at = 0.0
        self._keymap_info()      # 启动就把当前映射状态显示在那行小字上（否则一直是空的）
        self.refresh_ports()
        self._pump()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._log_sys("就绪。可点「自动连接蓝牙」直接连模块, 或手动选端口后「打开串口」。")
        if self.autoconn_var.get():
            root.after(500, self._maybe_autoconnect)

    # ------------------------------------------------------------------
    # 界面构建
    # ------------------------------------------------------------------
    def _build_ui(self):
        root = self.root
        root.title("{} v{}".format(APP_NAME, APP_VERSION))
        root.geometry("980x720")
        root.minsize(820, 560)

        self._build_menu()

        style = ttk.Style()
        try:
            style.theme_use("vista")
        except Exception:
            pass

        # ---- 工具栏 ----
        bar = ttk.Frame(root, padding=(8, 6, 8, 4))
        bar.grid(row=0, column=0, sticky="ew")
        bar.columnconfigure(1, weight=1)

        ttk.Label(bar, text="端口").grid(row=0, column=0, padx=(0, 4))
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, state="readonly", width=46)
        self.port_box.grid(row=0, column=1, sticky="ew", padx=(0, 6))

        ttk.Button(bar, text="刷新", width=6, command=self.refresh_ports).grid(row=0, column=2, padx=(0, 6))
        self.auto_btn = tk.Button(bar, text="自动连接蓝牙", width=12, relief="raised",
                                  bg="#1f6fb2", fg="white", activebackground="#1a5c94",
                                  activeforeground="white", command=self.auto_connect_bluetooth)
        self.auto_btn.grid(row=0, column=3, padx=(0, 6))
        self.bt_only_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="仅蓝牙", variable=self.bt_only_var,
                        command=self.refresh_ports).grid(row=0, column=4, padx=(0, 6))
        self.autoconn_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="启动自动连", variable=self.autoconn_var).grid(
            row=0, column=5, padx=(0, 6))
        ttk.Button(bar, text="蓝牙设置", width=9,
                   command=self.open_bluetooth_settings).grid(row=0, column=6)

        bar2 = ttk.Frame(root, padding=(8, 0, 8, 6))
        bar2.grid(row=1, column=0, sticky="ew")

        ttk.Label(bar2, text="波特率").grid(row=0, column=0, padx=(0, 4))
        self.baud_var = tk.StringVar(value="9600")
        ttk.Combobox(bar2, textvariable=self.baud_var, values=BAUD_CHOICES,
                     width=8).grid(row=0, column=1, padx=(0, 10))

        ttk.Label(bar2, text="数据位").grid(row=0, column=2, padx=(0, 4))
        self.bytesize_var = tk.StringVar(value="8")
        ttk.Combobox(bar2, textvariable=self.bytesize_var, values=BYTESIZE_CHOICES,
                     width=4, state="readonly").grid(row=0, column=3, padx=(0, 10))

        ttk.Label(bar2, text="校验").grid(row=0, column=4, padx=(0, 4))
        self.parity_var = tk.StringVar(value="无")
        ttk.Combobox(bar2, textvariable=self.parity_var, values=PARITY_CHOICES,
                     width=8, state="readonly").grid(row=0, column=5, padx=(0, 10))

        ttk.Label(bar2, text="停止位").grid(row=0, column=6, padx=(0, 4))
        self.stopbits_var = tk.StringVar(value="1")
        ttk.Combobox(bar2, textvariable=self.stopbits_var, values=STOPBITS_CHOICES,
                     width=5, state="readonly").grid(row=0, column=7, padx=(0, 10))

        ttk.Label(bar2, text="流控").grid(row=0, column=8, padx=(0, 4))
        self.flow_var = tk.StringVar(value="无")
        ttk.Combobox(bar2, textvariable=self.flow_var, values=FLOW_CHOICES,
                     width=9, state="readonly").grid(row=0, column=9, padx=(0, 10))

        ttk.Label(bar2, text="编码").grid(row=0, column=10, padx=(0, 4))
        self.enc_var = tk.StringVar(value="UTF-8")
        ttk.Combobox(bar2, textvariable=self.enc_var, values=ENCODING_CHOICES,
                     width=8, state="readonly").grid(row=0, column=11, padx=(0, 12))

        self.open_btn = tk.Button(bar2, text="打开串口", width=12, relief="raised",
                                  bg="#2d7d46", fg="white", activebackground="#256b3b",
                                  activeforeground="white", command=self.toggle_port)
        self.open_btn.grid(row=0, column=12)

        # ---- 收发分栏 ----
        pane = ttk.Panedwindow(root, orient="vertical")
        pane.grid(row=2, column=0, sticky="nsew", padx=8, pady=(0, 4))
        root.rowconfigure(2, weight=1)
        root.columnconfigure(0, weight=1)

        pane.add(self._build_rx(pane), weight=3)
        pane.add(self._build_tx(pane), weight=1)

        # ---- 状态栏 ----
        status = ttk.Frame(root, padding=(8, 2, 8, 6))
        status.grid(row=3, column=0, sticky="ew")
        status.columnconfigure(1, weight=1)

        self.dot = tk.Canvas(status, width=12, height=12, highlightthickness=0)
        self.dot.grid(row=0, column=0, padx=(0, 6))
        self.dot_id = self.dot.create_oval(2, 2, 11, 11, fill="#b0b0b0", outline="#808080")

        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(status, textvariable=self.status_var).grid(row=0, column=1, sticky="w")
        self.count_var = tk.StringVar(value="RX 0 字节    TX 0 字节")
        ttk.Label(status, textvariable=self.count_var).grid(row=0, column=2, sticky="e")

        self.rx_text.bind("<Control-a>", self._select_all_rx)
        self.tx_text.bind("<Control-Return>", lambda e: (self.send_once(), "break")[1])
        root.bind("<F5>", lambda e: self.refresh_ports())

    def _build_menu(self):
        menubar = tk.Menu(self.root)

        m_file = tk.Menu(menubar, tearoff=0)
        m_file.add_command(label="保存接收日志...", command=self.save_log)
        m_file.add_command(label="导出已发送数据...", command=self.export_tx_log)
        m_file.add_command(label="清空收发计数与发送记录", command=self.reset_counters)
        m_file.add_separator()
        m_file.add_command(label="退出", command=self.on_close)
        menubar.add_cascade(label="文件", menu=m_file)

        m_tool = tk.Menu(menubar, tearoff=0)
        m_tool.add_command(label="刷新端口列表  (F5)", command=self.refresh_ports)
        m_tool.add_command(label="导出 MIDI 字节流 (只导出不发送)...",
                           command=self.export_midi_bytes)
        m_tool.add_command(label="打开 Windows 蓝牙设置", command=self.open_bluetooth_settings)
        m_tool.add_command(label="打开设备管理器", command=self.open_device_manager)
        menubar.add_cascade(label="工具", menu=m_tool)

        m_help = tk.Menu(menubar, tearoff=0)
        m_help.add_command(label="使用说明", command=self.show_help)
        m_help.add_command(label="关于", command=self.show_about)
        menubar.add_cascade(label="帮助", menu=m_help)

        self.root.config(menu=menubar)

    def _build_rx(self, parent):
        frame = ttk.LabelFrame(parent, text="接收区", padding=6)

        head = ttk.Frame(frame)
        head.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 4))

        self.hex_show_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(head, text="HEX 显示", variable=self.hex_show_var).pack(side="left")
        self.ts_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(head, text="时间戳", variable=self.ts_var).pack(side="left", padx=(8, 0))
        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(head, text="自动滚动", variable=self.autoscroll_var).pack(side="left", padx=(8, 0))
        self.line_mode_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(head, text="按行读取", variable=self.line_mode_var,
                        command=self.on_line_mode_toggle).pack(side="left", padx=(8, 0))
        self.pause_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(head, text="暂停显示", variable=self.pause_var,
                        command=self.on_pause_toggle).pack(side="left", padx=(8, 0))
        self.tx_echo_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(head, text="显示已发送", variable=self.tx_echo_var).pack(
            side="left", padx=(8, 0))

        ttk.Button(head, text="清空", width=6, command=self.clear_rx).pack(side="right")
        ttk.Button(head, text="保存日志", width=9, command=self.save_log).pack(side="right", padx=(0, 6))

        self.rx_text = tk.Text(frame, wrap="word", height=18, undo=False,
                               font=("Consolas", 10), bg="#fbfbfb")
        self.rx_text.grid(row=1, column=0, sticky="nsew")
        sb = ttk.Scrollbar(frame, orient="vertical", command=self.rx_text.yview)
        sb.grid(row=1, column=1, sticky="ns")
        self.rx_text.configure(yscrollcommand=sb.set, state="disabled")

        frame.rowconfigure(1, weight=1)
        frame.columnconfigure(0, weight=1)
        return frame

    def _build_tx(self, parent):
        frame = ttk.LabelFrame(parent, text="发送区", padding=6)

        # ---- MIDI 组装行: 把 MIDI 消息拼成 hex 字节发出去 ----
        midi = ttk.Frame(frame)
        midi.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 5))
        midi.columnconfigure(6, weight=1)

        ttk.Label(midi, text="MIDI").grid(row=0, column=0, padx=(0, 4))

        self.midi_type_var = tk.StringVar(value=MIDI_TYPE_NAMES[0])
        self.midi_type_box = ttk.Combobox(midi, textvariable=self.midi_type_var,
                                          values=MIDI_TYPE_NAMES, width=17,
                                          state="readonly")
        self.midi_type_box.grid(row=0, column=1, padx=(0, 8))
        self.midi_type_box.bind("<<ComboboxSelected>>", lambda e: self.on_midi_change())

        ttk.Label(midi, text="通道").grid(row=0, column=2, padx=(0, 3))
        self.midi_ch_var = tk.StringVar(value="1")
        self.midi_ch_box = ttk.Combobox(midi, textvariable=self.midi_ch_var, width=4,
                                        state="readonly",
                                        values=[str(i) for i in range(1, 17)])
        self.midi_ch_box.grid(row=0, column=3, padx=(0, 8))
        self.midi_ch_box.bind("<<ComboboxSelected>>", lambda e: self.on_midi_change())

        self.midi_d1_var = tk.StringVar(value="C4")
        self.midi_d1_box = ttk.Combobox(midi, textvariable=self.midi_d1_var, width=9,
                                        values=note_choices())
        self.midi_d1_box.grid(row=0, column=4, padx=(0, 5))
        self.midi_d1_box.bind("<KeyRelease>", lambda e: self.on_midi_change())
        self.midi_d1_box.bind("<<ComboboxSelected>>", lambda e: self.on_midi_change())

        self.midi_d2_var = tk.StringVar(value="100")
        self.midi_d2_box = ttk.Combobox(midi, textvariable=self.midi_d2_var, width=6,
                                        values=[str(i) for i in range(128)])
        self.midi_d2_box.grid(row=0, column=5, padx=(0, 8))
        self.midi_d2_box.bind("<KeyRelease>", lambda e: self.on_midi_change())
        self.midi_d2_box.bind("<<ComboboxSelected>>", lambda e: self.on_midi_change())

        # 布局要点: 预览独占一行占满宽度; 右侧那组控件(帧尾/按原速/发送文件)
        # 也独占一行并跨越所有列。绝不能让它们只占第 7、8 列 —— 那样会把
        # 这两列撑宽, 使第 0 行的按钮总宽超出窗口而被裁掉(踩过两次)。
        self.midi_preview_var = tk.StringVar(value="")
        ttk.Label(midi, textvariable=self.midi_preview_var, anchor="w",
                  font=("Consolas", 9)).grid(row=1, column=0, columnspan=9,
                                             sticky="ew", pady=(2, 0))

        side = ttk.Frame(midi)
        side.grid(row=2, column=0, columnspan=9, sticky="e", pady=(3, 0))

        # ---- 帧头 / 帧尾 ----
        # ★ 板子的 uart_packet_echo.v 是【帧结构】协议：
        #     帧头 0xAA  = 开窗（从这里开始才收乐谱；也是它负责停播+清空）
        #     帧尾 0xFF  = 关窗 + 开始演奏
        #   只有帧尾没有帧头 = 窗口从没打开 = 一个字节都进不去 = 一声不响。
        #   所以两个都要有，默认就是 AA ... FF。
        self.midi_head_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(side, text="帧头", variable=self.midi_head_var,
                        command=self.on_midi_tail_change).pack(side="left")
        self.midi_head_hex_var = tk.StringVar(value="AA")
        self.midi_head_entry = ttk.Entry(side, textvariable=self.midi_head_hex_var, width=7)
        self.midi_head_entry.pack(side="left", padx=(3, 10))
        self.midi_head_entry.bind("<KeyRelease>", lambda e: self.on_midi_tail_change())

        self.midi_tail_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(side, text="帧尾", variable=self.midi_tail_var,
                        command=self.on_midi_tail_change).pack(side="left")
        self.midi_tail_hex_var = tk.StringVar(value="FF")
        self.midi_tail_entry = ttk.Entry(side, textvariable=self.midi_tail_hex_var, width=7)
        self.midi_tail_entry.pack(side="left", padx=(3, 10))
        self.midi_tail_entry.bind("<KeyRelease>", lambda e: self.on_midi_tail_change())

        # ★ 转 FPGA 帧（默认开）：
        #   开 = 选中的 .mid 先转成板子认的乐谱帧再发（键号 + 和弦 + AA/FF）
        #   关 = 老行为：把裸 MIDI 消息原样发出去（给别的设备/纯转发用）
        self.midi_to_fpga_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(side, text="转FPGA帧", variable=self.midi_to_fpga_var,
                        command=self.on_midi_change).pack(side="left", padx=(0, 10))

        # ---- 移调（单位：半音，可自己敲）----
        # ★ 为什么要这个：板子只有 36 个键 = 3 个八度（A2..G#5），曲子经常落不进去。
        #   移调是【在生成帧的时候把每个键号整体加上这个数】，超出 0..35 的音会被
        #   丢掉并在日志里报数 —— 所以换几档、看"丢几个音"，就能找到最合适的音区。
        #
        #   注意「通道」那一栏是 MIDI 协议自己的通道号（1~16，就是 90 3C 64 里
        #   90 的低 4 位），给标准 MIDI 设备分轨用的，板子根本不看它，和移调无关。
        ttk.Label(side, text="移调").pack(side="left")
        self.midi_trans_var = tk.StringVar(value="自动")
        self.midi_trans_box = ttk.Combobox(
            side, textvariable=self.midi_trans_var, width=6,
            values=["自动", "-24", "-12", "-7", "-5", "-2", "-1",
                    "0", "+1", "+2", "+5", "+7", "+12", "+24"])
        self.midi_trans_box.pack(side="left", padx=(3, 10))
        self.midi_trans_box.bind("<KeyRelease>", lambda e: self.on_midi_change())
        self.midi_trans_box.bind("<<ComboboxSelected>>", lambda e: self.on_midi_change())

        self.midi_realtime_var = tk.BooleanVar(value=True)
        self.midi_realtime_chk = ttk.Checkbutton(
            side, text="按原速", variable=self.midi_realtime_var)
        self.midi_realtime_chk.pack(side="left", padx=(0, 12))
        self.midi_file_btn = ttk.Button(side, text="发送MIDI文件", width=11,
                                        command=self.send_midi_file)
        self.midi_file_btn.pack(side="left")

        # ---- 演奏控制行（单独一行，不挤在 side 那一排里）----
        #   ★ 为什么单独一行：side 那排已经有 7 个控件，再加按钮总宽会超过窗口，
        #     最后几个会被挤到右边缘外面看不见（这个坑踩过两次）。
        #
        #   三个按钮的分工：
        #     「停止演奏」  发 0xC7 —— 停播 + 放键，但【乐谱留在板子里】，可以重播
        #     「重播」      发 0xC8 —— 拿板内那份谱子从头再放一遍，不用重发谱子
        #     「停止并清空」发 AA FF —— 停播 + 放键 + 清空乐谱 + 关窗
        #                   （纯用现有协议，板子没收到 0xC7/0xC8 时也能用）
        play = ttk.Frame(midi)
        play.grid(row=3, column=0, columnspan=9, sticky="ew", pady=(4, 0))

        self.midi_stop_btn = tk.Button(play, text="停止演奏", width=10,
                                       bg="#b03a2e", fg="white",
                                       activebackground="#8e2f25", activeforeground="white",
                                       command=self.stop_score)
        self.midi_stop_btn.pack(side="left")

        self.midi_replay_btn = ttk.Button(play, text="重播", width=8,
                                          command=self.replay_score)
        self.midi_replay_btn.pack(side="left", padx=(6, 0))

        ttk.Button(play, text="停止并清空", width=11,
                   command=self.stop_and_clear).pack(side="left", padx=(6, 0))

        ttk.Button(play, text="按键映射…", width=11,
                   command=self.open_keymap).pack(side="left", padx=(18, 0))

        self.keymap_lbl = ttk.Label(play, text="", anchor="w",
                                    font=("Consolas", 9))
        self.keymap_lbl.pack(side="left", padx=(8, 0))

        # ---- ★ V31：效果开关 / 音色（上位机也能切，板上的 TTP 按钮照旧）----
        #   和录音开关同一个思路：上位机只发"翻转第几位"，板子收到后回报
        #   完整状态（0x5D），界面按回报刷新 —— 板子永远是权威，
        #   所以"在板子上按按钮"和"在软件上点按钮"不会打架。
        #   ★ 音色是三档（普通/笛子/吉他），一位翻转表达不了，所以单独用下拉框，
        #     发的是 0xCA（直接选档）。
        fxrow = ttk.Frame(midi)
        fxrow.grid(row=5, column=0, columnspan=9, sticky="ew", pady=(4, 0))

        ttk.Label(fxrow, text="效果开关").pack(side="left", padx=(0, 6))
        self.fx_btns = {}
        for bit, (_key, _cn) in enumerate(fpga_rec.FX_BITS):
            if _key == "flute":
                continue            # bit5 是音色档，用右边的下拉框，不做成开关
            # ★ 按钮文字统一两个汉字、宽度按字数配（这里踩过一次）：
            #   原来直接拿 FX_BITS 里的中文名当按钮文字，而"延音锁存"是 4 个字
            #   （实测文字宽 48px、连边距要 54px），却被塞进 width=5 的按钮
            #   （45px）里 —— 表现就是那个按钮的字被裁掉一截，看着像排版坏了。
            #   现在统一用 FX_BTN_LABELS 的短名（都是 2 个字），宽度一致、不再裁。
            #   完整名字在状态小字里给（例如"延音锁存 颤音 混响 音色:吉他"）。
            b = tk.Button(fxrow, text=FX_BTN_LABELS[bit], width=4,
                          command=lambda bb=bit: self.toggle_effect(bb))
            b.pack(side="left", padx=1)
            self.fx_btns[bit] = b

        ttk.Label(fxrow, text="音色").pack(side="left", padx=(12, 4))
        # ★ 音色改成三个【可以直接点】的按钮（普通 / 笛子 / 吉他），不用下拉框：
        #   板子上是"按钮 7 循环三档"，软件里更希望"想听哪个点哪个"。
        #   点一下发 0xCA + 档位；当前是哪一档由板子的 0x5D 回报决定
        #   （所以按下去先不变色，等回报到了才亮 —— 和上面那排效果开关一致）。
        #   tone_var 只用来记"板子现在报的是哪一档"，界面高亮和自检都看它。
        self.tone_var = tk.StringVar(value="未知")
        self.tone_btns = []
        for _i, _n in enumerate(fpga_rec.TONE_NAMES):
            _tb = tk.Button(fxrow, text=_n, width=4,
                            command=lambda ii=_i: self.set_tone(ii))
            _tb.pack(side="left", padx=1)
            self.tone_btns.append(_tb)

        ttk.Button(fxrow, text="刷新状态", width=9,
                   command=self.request_fx_state).pack(side="left", padx=(8, 0))
        self.fx_lbl = ttk.Label(fxrow, text="", anchor="w",
                                font=("Consolas", 9))
        self.fx_lbl.pack(side="left", padx=(8, 0))

        # ---- 演奏录音回传 ----
        # ★ 板子那边按键 10（TTP229 16 键板上的第 10 个键）是录音开关。
        #   这里这个勾选框和它是【同一个开关】：勾上 = 通知板子开始录，
        #   板子随手弹随手把 key_mask 回传上来；在板子上按键 10 也能开关，
        #   板子会用 [0x5B][0/1] 回报状态，这个勾选框会跟着变 —— 两端同步。
        #   录完点「合成并播放」：回传的事件 -> MIDI -> 就在这台电脑上放出来。
        rec = ttk.Frame(midi)
        # ★ 这里必须是 row=4：row=3 已经被上面那行「演奏控制」占了。
        #   两个 Frame 放同一个 grid 单元的话，后创建的那个会把先创建的整个盖住
        #   —— 表现就是"加了按钮但界面上看不见"（踩过）。
        rec.grid(row=4, column=0, columnspan=9, sticky="ew", pady=(4, 0))

        ttk.Label(rec, text="演奏录音").pack(side="left", padx=(0, 6))

        self.rec_var = tk.BooleanVar(value=False)
        self.rec_chk = ttk.Checkbutton(rec, text="录音 (板子按键10)", variable=self.rec_var,
                                       command=self.on_rec_toggle)
        self.rec_chk.pack(side="left", padx=(0, 10))

        self.rec_status_var = tk.StringVar(value="未录音")
        ttk.Label(rec, textvariable=self.rec_status_var, anchor="w",
                  font=("Consolas", 9)).pack(side="left", padx=(0, 10))

        self.rec_play_btn = ttk.Button(rec, text="合成并播放", width=11,
                                       command=self.rec_play)
        self.rec_play_btn.pack(side="left", padx=(0, 4))
        ttk.Button(rec, text="导出MIDI", width=10,
                   command=self.rec_export).pack(side="left", padx=(0, 4))
        ttk.Button(rec, text="发回板子", width=10,
                   command=self.rec_send_back).pack(side="left", padx=(0, 4))
        ttk.Button(rec, text="清空录音", width=10,
                   command=self.rec_clear).pack(side="left")

        ttk.Button(midi, text="发送MIDI", width=11,
                   command=self.send_midi).grid(row=0, column=7, padx=(0, 4))
        ttk.Button(midi, text="填入发送区", width=11,
                   command=self.fill_midi_hex).grid(row=0, column=8)

        head = ttk.Frame(frame)
        head.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 4))
        # 用 grid 而不是 pack: 固定宽度的控件先占位, 让「历史」下拉吸收剩余空间,
        # 否则控件总宽超过窗口时, 最后 pack 的发送按钮会被挤出右边缘看不见。
        head.columnconfigure(7, weight=1)

        self.hex_send_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(head, text="HEX 发送", variable=self.hex_send_var).grid(
            row=0, column=0, sticky="w")

        ttk.Label(head, text="行尾").grid(row=0, column=1, padx=(10, 4))
        self.lineend_var = tk.StringVar(value=LINEEND_CHOICES[3])
        ttk.Combobox(head, textvariable=self.lineend_var, values=LINEEND_CHOICES,
                     width=10, state="readonly").grid(row=0, column=2)

        self.timer_on_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(head, text="定时发送", variable=self.timer_on_var,
                        command=self.on_timer_toggle).grid(row=0, column=3, padx=(12, 0))
        self.timer_ms_var = tk.StringVar(value="1000")
        ttk.Entry(head, textvariable=self.timer_ms_var, width=6).grid(
            row=0, column=4, padx=(4, 2))
        ttk.Label(head, text="ms").grid(row=0, column=5, padx=(0, 12))

        ttk.Label(head, text="历史").grid(row=0, column=6, padx=(0, 4))
        self.hist_box = ttk.Combobox(head, values=self.history, width=14, state="readonly")
        self.hist_box.grid(row=0, column=7, sticky="ew")
        self.hist_box.bind("<<ComboboxSelected>>", self.on_history_pick)

        self.send_btn = tk.Button(head, text="发送 (Ctrl+Enter)", width=17,
                                  bg="#1f6fb2", fg="white", activebackground="#1a5c94",
                                  activeforeground="white", command=self.send_once)
        self.send_btn.grid(row=0, column=8, padx=(10, 0))

        self.tx_text = tk.Text(frame, wrap="word", height=5, font=("Consolas", 10), bg="#fbfbfb")
        self.tx_text.grid(row=2, column=0, sticky="nsew")
        sb = ttk.Scrollbar(frame, orient="vertical", command=self.tx_text.yview)
        sb.grid(row=2, column=1, sticky="ns")
        self.tx_text.configure(yscrollcommand=sb.set)

        frame.rowconfigure(2, weight=1)
        frame.columnconfigure(0, weight=1)
        return frame

    # ------------------------------------------------------------------
    # 配置记忆
    # ------------------------------------------------------------------
    def _apply_config(self):
        c = self.cfg
        self.baud_var.set(c.get("baud", "9600"))
        self.bytesize_var.set(c.get("bytesize", "8"))
        self.parity_var.set(c.get("parity", "无"))
        self.stopbits_var.set(c.get("stopbits", "1"))
        self.flow_var.set(c.get("flow", "无"))
        self.enc_var.set(c.get("encoding", "UTF-8"))
        self.hex_show_var.set(bool(c.get("hex_show", False)))
        self.hex_send_var.set(bool(c.get("hex_send", False)))
        self.ts_var.set(bool(c.get("timestamp", True)))
        self.autoscroll_var.set(bool(c.get("autoscroll", True)))
        self.bt_only_var.set(bool(c.get("bt_only", False)))
        self.autoconn_var.set(bool(c.get("autoconn", True)))
        self.line_mode_var.set(bool(c.get("line_mode", True)))
        self.lineend_var.set(c.get("lineend", LINEEND_CHOICES[3]))
        self.timer_ms_var.set(c.get("timer_ms", "1000"))
        mtype = c.get("midi_type", MIDI_TYPE_NAMES[0])
        if mtype not in _MIDI_BY_NAME:
            mtype = MIDI_TYPE_NAMES[0]
        self.midi_type_var.set(mtype)
        self.midi_ch_var.set(c.get("midi_ch", "1"))
        self.midi_d1_var.set(c.get("midi_d1", "C4"))
        self.midi_d2_var.set(c.get("midi_d2", "100"))
        self.midi_realtime_var.set(bool(c.get("midi_realtime", True)))
        self.midi_to_fpga_var.set(bool(c.get("midi_to_fpga", True)))
        self.midi_head_var.set(bool(c.get("midi_head_on", True)))
        self.midi_head_hex_var.set(c.get("midi_head_hex", "AA"))
        self.midi_trans_var.set(c.get("midi_trans", "自动"))
        self.midi_tail_var.set(bool(c.get("midi_tail_on", True)))
        self.midi_tail_hex_var.set(c.get("midi_tail_hex", "FF"))
        self.on_midi_change()
        geo = c.get("geometry")
        if geo:
            try:
                # ★★ 恢复的窗口大小【不能小于 minsize】★★
                #   踩过的坑：配置里存下来的是 668x537，比程序自己声明的
                #   minsize(820x560) 还小。窗口那么矮，PanedWindow 把接收区分走
                #   大半，【发送区下半截被直接裁掉】—— 新加的功能行根本露不出来，
                #   用户报"界面上看不到修改的地方"就是这么来的。
                #   Tk 的 minsize 管得住用户拖边框，但管不住 wm geometry 直接设一个
                #   更小的值，所以这里必须自己判。
                m = re.match(r"\s*(\d+)x(\d+)", str(geo))
                if m and int(m.group(1)) >= 820 and int(m.group(2)) >= 560:
                    self.root.geometry(geo)
                else:
                    self.root.geometry("980x720")
                    self._log_sys("配置里存的窗口大小 %s 比最小尺寸(820x560)还小，"
                                  "已改用默认的 980x720 —— 否则发送区下半截会被裁掉。"
                                  % geo)
            except Exception:
                pass
        if self.history:
            self.hist_box.configure(values=self.history)
        self.refresh_hist_box()

    def _collect_config(self):
        return {
            "port": self.port_var.get(),
            "baud": self.baud_var.get(),
            "bytesize": self.bytesize_var.get(),
            "parity": self.parity_var.get(),
            "stopbits": self.stopbits_var.get(),
            "flow": self.flow_var.get(),
            "encoding": self.enc_var.get(),
            "hex_show": self.hex_show_var.get(),
            "hex_send": self.hex_send_var.get(),
            "timestamp": self.ts_var.get(),
            "autoscroll": self.autoscroll_var.get(),
            "bt_only": self.bt_only_var.get(),
            "autoconn": self.autoconn_var.get(),
            "line_mode": self.line_mode_var.get(),
            "lineend": self.lineend_var.get(),
            "timer_ms": self.timer_ms_var.get(),
            "midi_type": self.midi_type_var.get(),
            "midi_ch": self.midi_ch_var.get(),
            "midi_d1": self.midi_d1_var.get(),
            "midi_d2": self.midi_d2_var.get(),
            "midi_realtime": self.midi_realtime_var.get(),
            "midi_to_fpga": self.midi_to_fpga_var.get(),
            "midi_head_on": self.midi_head_var.get(),
            "midi_head_hex": self.midi_head_hex_var.get(),
            "midi_trans": self.midi_trans_var.get(),
            "midi_tail_on": self.midi_tail_var.get(),
            "midi_tail_hex": self.midi_tail_hex_var.get(),
            "history": self.history[:30],
            "geometry": self.root.geometry(),
        }

    # ------------------------------------------------------------------
    # 端口扫描
    # ------------------------------------------------------------------
    def refresh_ports(self):
        if list_ports is None:
            self._log_sys("未安装 pyserial, 无法扫描串口。")
            return
        prev = self.port_var.get()
        infos = list(list_ports.comports())
        infos.sort(key=lambda i: i.device)

        self.port_map = {}
        self.port_info = {}
        labels = []
        for info in infos:
            if self.bt_only_var.get() and not is_bluetooth_port(info):
                continue
            label = port_label(info)
            self.port_map[label] = info.device
            self.port_info[label] = info
            labels.append(label)

        self.port_box.configure(values=labels)
        if labels:
            target = None
            if prev in self.port_map:          # 保留用户上次的选择
                target = prev
            if target is None:                 # 否则优先蓝牙出站口: 只有它能真正连上模块
                for lb in labels:
                    if bluetooth_mac(self.port_info[lb]):
                        target = lb
                        break
            if target is None:
                target = labels[0]
            self.port_var.set(target)
        else:
            self.port_var.set("")

        bt_n = sum(1 for i in infos if is_bluetooth_port(i))
        if not self.is_open():
            self.status_var.set("找到 {} 个端口 (其中蓝牙 SPP {} 个), 未连接".format(len(infos), bt_n))
        self._set_dot("#b0b0b0" if not self.is_open() else "#2d7d46")

    def _device_of(self, label):
        return self.port_map.get(label, label if str(label).startswith("COM") else "")

    def current_device(self):
        return self._device_of(self.port_var.get())

    def _label_for_device(self, device):
        for label, dev in self.port_map.items():
            if dev == device:
                return label
        return None

    def _maybe_autoconnect(self):
        if self.autoconn_var.get() and not self.is_open():
            self.auto_connect_bluetooth(silent=True)

    def auto_connect_bluetooth(self, silent=False):
        """自动连蓝牙模块 —— 用 SPP.py 那套识别+连接思路。

        与 SPP.py 的唯一区别: 优先挑「出站口」(带对端 MAC 的那个)。
        SPP.py 直接取 bt_ports[0], 而 Windows 会重新分配蓝牙 SPP 的
        COM 编号: 取到入站口时打开会「成功」, 其实没连上任何模块 ——
        看着像连上了, 实际一个字节都收不到。出站口才是真会去连模块的口。
        """
        if self.is_open():
            self.close_port("切换端口")
        bt = find_bluetooth_ports()
        if not bt:
            self._log_sys("未找到蓝牙串口设备: 请先配对蓝牙模块(点「蓝牙设置」)")
            if not silent:
                messagebox.showwarning(APP_NAME, "未找到蓝牙串口设备。\n请先在此电脑上配对蓝牙模块。")
            return False
        outgoing = [i for i in bt if bluetooth_mac(i)]
        target = outgoing[0] if outgoing else bt[0]
        if not outgoing:
            self._log_sys("未找到出站口(带对端 MAC), 退回连接 {}".format(target.device))

        self.bt_only_var.set(True)
        self.refresh_ports()
        label = self._label_for_device(target.device)
        if label is None:
            self._log_sys("端口 {} 不在当前列表中".format(target.device))
            if not silent:
                messagebox.showwarning(APP_NAME, "端口 {} 不在当前列表中。".format(target.device))
            return False
        self.port_var.set(label)
        self._log_sys("自动连接 -> {}".format(label))
        return self.open_port(silent=silent)

    # ------------------------------------------------------------------
    # 串口开关
    # ------------------------------------------------------------------
    def _serial_kwargs(self):
        parity = {"无": "N", "偶校验": "E", "奇校验": "O", "标记": "M", "空格": "S"}
        stopbits = {"1": 1, "1.5": 1.5, "2": 2}
        bytesize = {"5": 5, "6": 6, "7": 7, "8": 8}
        flow = {"无": {}, "RTS/CTS": {"rtscts": True},
                "XON/XOFF": {"xonxoff": True}, "DSR/DTR": {"dsrdtr": True}}
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            raise ValueError("波特率必须是整数")
        # 按行读取用 timeout=1, 与实测可用的 SPP.py 一致;
        # 空闲合并模式用小超时, 才能及时冲刷缓冲。
        read_timeout = 1.0 if self.line_mode_var.get() else 0.02
        kw = dict(baudrate=baud,
                  bytesize=bytesize.get(self.bytesize_var.get(), 8),
                  parity=parity.get(self.parity_var.get(), "N"),
                  stopbits=stopbits.get(self.stopbits_var.get(), 1),
                  timeout=read_timeout, write_timeout=1.0)
        kw.update(flow.get(self.flow_var.get(), {}))
        return kw

    def toggle_port(self):
        if self._opening:
            return
        if self.is_open():
            self.close_port("手动关闭")
        else:
            self.open_port()

    def open_port(self, silent=False):
        """打开串口 —— 真正打开放到后台线程。

        蓝牙出站口打开时要等对端应答, 实测能阻塞 5 秒以上(模块不在时会
        报 121)。放在 UI 线程上做, 启动自动连就会把窗口冻住 5 秒, 所以
        这里只做校验, serial.Serial() 交给 _open_worker。
        """
        if self._opening:
            return False
        if serial is None:
            if not silent:
                messagebox.showerror(APP_NAME, "缺少 pyserial 依赖, 无法打开串口。")
            return False
        device = self.current_device()
        if not device:
            if not silent:
                messagebox.showwarning(APP_NAME, "请先选择一个端口。")
            return False
        try:
            kwargs = self._serial_kwargs()
        except ValueError as exc:
            if not silent:
                messagebox.showerror(APP_NAME, str(exc))
            return False

        self.line_mode = bool(self.line_mode_var.get())
        self._opening = True
        # 每次打开都用全新的 stop 事件。用 clear() 复用同一个事件会出问题:
        # 上次关闭时还没退出的旧读写线程会被"复活", 它们从共享的 txq 里
        # 取走数据、发现自己的句柄已过期再静默丢弃 —— 表现就是偶发丢包。
        self.stop_evt = threading.Event()
        evt = self.stop_evt
        # 每个会话一个全新的发送队列: 旧会话残留的写线程只可能碰到它自己的
        # 旧队列, 绝不会抢走新会话的数据(否则顺序会被打乱)。
        self.txq = queue.Queue()
        txq = self.txq
        self._open_seq += 1
        seq = self._open_seq
        self.open_btn.configure(state="disabled", text="正在打开...")
        self.status_var.set("正在打开 {} ...".format(device))
        self._log_sys("正在打开 {} ({} {}) ...".format(
            device, kwargs["baudrate"], "按行读取" if self.line_mode else "空闲合并读取"))
        threading.Thread(target=self._open_worker,
                         args=(device, kwargs, silent, seq, evt, txq), daemon=True).start()
        return True

    def _open_worker(self, device, kwargs, silent, seq, evt, txq):
        try:
            ser = serial.Serial(port=device, **kwargs)
        except Exception as exc:
            # 注意: except 块结束时 Python 会 `del exc`, 而回调是延迟执行的
            # lambda。直接把 exc 捕获进去, 等回调真正跑的时候变量已经没了,
            # 会抛 NameError 并被 _pump 的兜底 except 静默吞掉 —— 表现就是
            # "打开失败后按钮一直卡在'正在打开...'"。必须先绑到别的名字。
            err = exc
            self.uiqueue.put(lambda: self._open_failed(device, err, silent, seq))
            return
        self.uiqueue.put(lambda: self._open_succeeded(ser, device, kwargs, seq, evt, txq))

    def _reset_open_button(self):
        self._opening = False
        self.open_btn.configure(state="normal", text="打开串口",
                                bg="#2d7d46", activebackground="#256b3b")

    def _open_failed(self, device, exc, silent, seq):
        if seq != self._open_seq:
            return                      # 过期结果: 用户已经改主意了
        text = str(exc)
        hint = ""
        if ("121" in text) or ("信号灯" in text) or ("semaphore" in text.lower()):
            hint = ("\n\n这是蓝牙出站口在等模块应答, 常见原因:\n"
                    "  - 模块没上电 / 不在范围内\n"
                    "  - 模块已被别的程序或手机占用\n"
                    "出站口(带 MAC 的那个)打开时会主动去连模块; 入站口只是等对方来连。")
        self._reset_open_button()
        self._log_sys("打开 {} 失败: {}".format(device, exc))
        self.status_var.set("打开失败: {}".format(exc))
        self._set_dot("#c0392b")
        if not silent:
            messagebox.showerror(APP_NAME, "打开 {} 失败:\n{}{}".format(device, exc, hint))

    def _open_succeeded(self, ser, device, kwargs, seq, evt, txq):
        if seq != self._open_seq or self._closing:
            # 过期结果(用户关掉或又发起了新的打开): 丢弃句柄, 不动界面
            try:
                ser.close()
            except Exception:
                pass
            return
        self._opening = False
        self.ser = ser
        self.rx_bytes = 0
        self.tx_bytes = 0
        self.last_error = ""
        self._update_counts()

        # 新会话从字节流中间接上：丢掉可能残留的半截录音帧，已录事件留着
        # 按键映射：板子一断电就回默认，所以每次连上自动下发一遍
        self.root.after(400, self._keymap_autosend)
        # ★ V31：顺便问一次效果/音色的当前状态，把界面上那几个开关对齐
        #   （自检里可以把它关掉：好几节要做"逐字节完全一致"的比对）
        if getattr(self, "fx_auto_query", True):
            self.root.after(600, self.request_fx_state)

        if self.rec is not None:
            self.rec.resync()
            self.rec_on = False       # 板子上电默认不录；真实状态等它自己回报
            self.rec_pending = 0.0
            self.rec_var.set(False)
            self._rec_info()

        self.reader = threading.Thread(target=self._read_loop, args=(ser, evt), daemon=True)
        self.reader.start()
        self.writer = threading.Thread(target=self._write_loop,
                                       args=(ser, evt, txq), daemon=True)
        self.writer.start()

        self.open_btn.configure(state="normal", text="关闭串口",
                                bg="#b03a2e", activebackground="#8e2f25")
        self.port_box.configure(state="disabled")
        self.status_var.set("已连接 {} @ {} 已打开".format(device, kwargs["baudrate"]))
        self._set_dot("#2d7d46")
        self._log_sys("已打开 {} ({} {} {}), 编码 {}, {}".format(
            device, kwargs["baudrate"], kwargs["bytesize"],
            self.parity_var.get(), self.enc_var.get(),
            "按行读取" if self.line_mode else "空闲合并读取"))

    def close_port(self, reason="关闭", silent=False):
        if self._opening:
            # 还在打开中: 让它作废, 按钮复原
            self._open_seq += 1
            self.stop_evt.set()
            self._reset_open_button()
        if not self.is_open():
            return
        self.stop_evt.set()
        with self.io_lock:
            ser, self.ser = self.ser, None
        # 放一个哨兵唤醒还停在 txq.get() 上的写线程, 让它立刻退出。
        # 不能去"排空队列": 没用, 而且旧线程若把抢到的数据放回队尾,
        # 会把发送顺序打乱(MIDI 对顺序极敏感)。
        try:
            self.txq.put_nowait(None)
        except Exception:
            pass
        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass
        self.on_timer_toggle(force_off=True)

        self.open_btn.configure(text="打开串口", bg="#2d7d46", activebackground="#256b3b")
        self.port_box.configure(state="readonly")
        self._set_dot("#b0b0b0")
        if not silent:
            self.status_var.set(reason)
        self._log_sys("[{}] {}".format(reason, self.last_error) if self.last_error else "[{}]".format(reason))

    def is_open(self):
        return self.ser is not None

    # ------------------------------------------------------------------
    # 收发线程
    # ------------------------------------------------------------------
    def _read_loop(self, ser, evt):
        if self.line_mode:
            self._read_loop_lines(ser, evt)
        else:
            self._read_loop_idle(ser, evt)

    def _read_loop_lines(self, ser, evt):
        """按行读取 —— 与实测可用的 SPP.py 同一套思路(readline + timeout=1)。

        比 SPP.py 多做一步: 不先判断 in_waiting 再读。部分蓝牙驱动上
        in_waiting 常年返回 0, 那样会永远读不到数据; 直接阻塞 readline 更稳。
        """
        while not evt.is_set():
            try:
                data = ser.readline()
            except Exception as exc:
                if not evt.is_set():
                    self.rxq.put(("error", str(exc)))
                return
            if data:
                self.rxq.put(("data", data))

    def _read_loop_idle(self, ser, evt):
        """按空闲间隔把数据合并成一批再投递(不依赖换行符, 适合二进制流)。

        不能读到一个字节就投递一批 —— 底层 1 字节阻塞读会让 "HELLO\\r\\n"
        被切成 "H" + "ELLO\\r\\n", 界面上就会插进两个时间戳把内容劈开。
        """
        buf = bytearray()
        last = time.time()
        while not evt.is_set():
            try:
                waiting = ser.in_waiting
                data = ser.read(waiting if waiting else 1)
            except Exception as exc:
                if buf:
                    self.rxq.put(("data", bytes(buf)))
                if not evt.is_set():
                    self.rxq.put(("error", str(exc)))
                return
            now = time.time()
            if data:
                buf += data
                last = now
            if buf and ((now - last) >= IDLE_FLUSH or len(buf) >= MAX_CHUNK):
                self.rxq.put(("data", bytes(buf)))
                del buf[:]
                last = now

    def _write_loop(self, ser, evt, txq):
        while not evt.is_set():
            try:
                item = txq.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:          # close_port 放的哨兵: 立刻收工
                return
            if isinstance(item, tuple):
                data, label = item
            else:
                data, label = item, ""
            try:
                with self.io_lock:
                    if self.ser is not ser:
                        # 本线程属于已经过期的会话, 这一批随旧队列一起作废。
                        # 绝不能放回队尾 —— 那会把新会话的发送顺序打乱。
                        return
                    written = write_all(ser, data)
                # 到这一步才敢记: 这批字节是真的出去了
                self.rxq.put(("tx", (written, data, label)))
            except Exception as exc:
                self.rxq.put(("error", "发送失败: {}".format(exc)))
                return

    def _pump(self):
        """定时把后台线程的数据搬到界面上。"""
        try:
            # 先执行后台线程排队过来的 UI 回调(例如"打开完成/打开失败")
            while True:
                try:
                    callback = self.uiqueue.get_nowait()
                except queue.Empty:
                    break
                try:
                    callback()
                except Exception:
                    pass
            processed = 0
            while processed < 80:
                try:
                    kind, payload = self.rxq.get_nowait()
                except queue.Empty:
                    break
                processed += 1
                if kind == "data":
                    self._on_rx(payload)
                elif kind == "tx":
                    written, sent_data, sent_label = payload
                    self.tx_bytes += written
                    self._update_counts()
                    self._record_tx(sent_data, sent_label)
                elif kind == "error":
                    self.last_error = payload
                    self._log_sys("[串口异常] {}".format(payload))
                    self.status_var.set("连接中断: {}".format(payload))
                    self.close_port("连接已断开", silent=True)
                    self.status_var.set("连接中断: {}".format(payload))
        except Exception:
            pass
        self._rec_timeout_check()
        self._fx_pending_check()          # ★ V31.1：效果开关的"待确认"超时检查
        if not self._closing:
            self.root.after(40, self._pump)

    # ------------------------------------------------------------------
    # 接收显示
    # ------------------------------------------------------------------
    def _on_rx(self, data):
        self.rx_bytes += len(data)
        self._update_counts()

        # 先过一遍录音解析器 —— 录音帧是二进制, 不能当文本打进接收区
        if self._rec_feed(data):
            return

        if self.paused:
            return

        if self.hex_show_var.get():
            body = fmt_hex(data)
        else:
            body = decode_bytes(data, self.enc_var.get())
            if self.line_mode:
                # 按行读取时 readline 已带回车换行, 统一成一条一行
                body = body.rstrip("\n") + "\n"

        stamp = ""
        if self.ts_var.get():
            stamp = "[{}] ".format(datetime.now().strftime("%H:%M:%S.%f")[:-3])

        self._insert_rx(stamp + body)

    def _insert_rx(self, text):
        widget = self.rx_text
        widget.configure(state="normal")
        widget.insert("end", text)
        if int(widget.index("end-1c").split(".")[0]) > 6000 or \
                len(widget.get("1.0", "end-1c")) > MAX_RX_CHARS:
            widget.delete("1.0", "200.0")
        if self.autoscroll_var.get():
            widget.see("end")
        widget.configure(state="disabled")

    def _log_sys(self, text):
        self._insert_rx("\n*** {} ***\n".format(text))

    def clear_rx(self):
        self.rx_text.configure(state="normal")
        self.rx_text.delete("1.0", "end")
        self.rx_text.configure(state="disabled")

    def _select_all_rx(self, _event=None):
        self.rx_text.tag_add("sel", "1.0", "end")
        return "break"

    def on_line_mode_toggle(self):
        if self.is_open():
            self._log_sys("读取模式将改为「{}」, 重新打开串口后生效".format(
                "按行" if self.line_mode_var.get() else "空闲合并"))

    def on_pause_toggle(self):
        self.paused = self.pause_var.get()
        self._log_sys("接收显示已暂停 (数据仍在后台接收并计数)" if self.paused else "接收显示已恢复")

    def _update_counts(self):
        self.count_var.set("RX {} 字节    TX {} 字节".format(self.rx_bytes, self.tx_bytes))

    def reset_counters(self):
        self.rx_bytes = 0
        self.tx_bytes = 0
        self.tx_log = []
        self._update_counts()
        self._log_sys("收发计数与已发送记录已清零")

    # ------------------------------------------------------------------
    # 发送凭据: 记录真正写出端口的字节
    # ------------------------------------------------------------------
    def _record_tx(self, data, label):
        """记下真正发出去的字节 —— 这是"发出去的数据没问题"的凭据。

        只在 write_all() 成功返回后才调用, 所以记录里的就是实际出去的字节。
        """
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.tx_log.append((stamp, label, fmt_hex(data)))
        del self.tx_log[:-5000]
        if self.tx_echo_var.get() and not self.paused:
            # 长帧(一整首乐谱)只显示头尾, 否则接收区会被一次刷屏几千个字节
            if len(data) > TX_ECHO_MAX:
                body = "{} … {}  [共 {} 字节, 中间略]".format(
                    fmt_hex(data[:TX_ECHO_MAX // 2]),
                    fmt_hex(data[-TX_ECHO_MAX // 2:]), len(data))
            else:
                body = "{}  [{} 字节]".format(fmt_hex(data), len(data))
            self._insert_rx("<<< {}{}\n".format(
                "[{}] ".format(label) if label else "", body))

    def export_tx_log(self):
        """把已发送的字节导出成文本, 便于逐字节核对。"""
        if not self.tx_log:
            messagebox.showinfo(APP_NAME, "还没有发送过数据。")
            return
        default = "spp_tx_{}.txt".format(datetime.now().strftime("%Y%m%d_%H%M%S"))
        path = filedialog.asksaveasfilename(
            title="导出已发送数据", initialdir=app_dir(), initialfile=default,
            defaultextension=".txt",
            filetypes=[("文本文件", "*.txt"), ("所有文件", "*.*")])
        if not path:
            return
        try:
            total = 0
            with open(path, "w", encoding="utf-8") as fh:
                fh.write("# SPP 蓝牙串口助手 —— 已发送数据记录\n")
                fh.write("# 按实际写出端口的顺序记录\n")
                fh.write("# 时间\t\t标签\t字节数\tHEX\n")
                for stamp, label, hx in self.tx_log:
                    n = len(hx.split()) if hx else 0
                    total += n
                    fh.write("{}\t{}\t{}\t{}\n".format(stamp, label or "-", n, hx))
                fh.write("# 合计 {} 批 / {} 字节\n".format(len(self.tx_log), total))
            self.status_var.set("已发送数据已导出: {}".format(path))
            self._log_sys("已导出 {} 批 / {} 字节到 {}".format(
                len(self.tx_log), total, path))
        except Exception as exc:
            messagebox.showerror(APP_NAME, "导出失败:\n{}".format(exc))

    def export_midi_bytes(self):
        """把 MIDI 文件将要发出的字节流导出(只导出, 不发送), 便于先核对。

        导出内容必须和「发送MIDI文件」真正发出去的一模一样, 否则核对没意义 ——
        所以这里也跟着「转FPGA帧」走: 勾了就导出乐谱帧(AA...FF), 没勾才导出裸 MIDI。
        """
        if mido is None:
            messagebox.showerror(APP_NAME, "需要 mido 库才能读取 MIDI 文件。")
            return
        path = filedialog.askopenfilename(
            title="选择要导出字节流的 MIDI 文件",
            initialdir=midi_folder(),
            filetypes=[("MIDI 文件", "*.mid *.midi *.kar *.syx"),
                       ("所有文件", "*.*")])
        if not path:
            return
        base = os.path.splitext(path)[0] + ".发送字节流"
        txt_path, bin_path = base + ".txt", base + ".bin"

        # ---- 转 FPGA 帧：导出的是乐谱帧 ----
        if self.midi_to_fpga_var.get() and fpga_midi is not None \
                and not path.lower().endswith(".syx"):
            try:
                head = self.midi_head_bytes()
                tail = self.midi_tail_bytes()
                tr = self.midi_transpose()
                frame, blob, rep = fpga_midi.midi_to_frame(path, head=head, tail=tail,
                                                           shift=tr)
            except Exception as exc:
                messagebox.showerror(APP_NAME, "转 FPGA 帧失败:\n{}".format(exc))
                return
            try:
                with open(txt_path, "w", encoding="utf-8") as fh:
                    fh.write("# {}\n".format(os.path.basename(path)))
                    fh.write("# 这就是点「发送MIDI文件」时会出去的字节（FPGA 乐谱帧）\n")
                    fh.write("# " + rep.text().replace("\n", "\n# ") + "\n")
                    fh.write("# 序号\t键号\t音名\t时值(格)\n")
                    for i, e in enumerate(frame.events, 1):
                        names = " ".join("键%02d(%s)" % (k, key_no_name(k))
                                         for k in e.keys) or "休止"
                        fh.write("{}\t{}\t{}\t{}\n".format(
                            i, " ".join("%02X" % k for k in e.keys) or "EE",
                            names, e.ticks))
                    fh.write("# --- 帧字节（原始 HEX）---\n")
                    for off in range(0, len(blob), 32):
                        fh.write(fmt_hex(blob[off:off + 32]) + "\n")
                    fh.write("# 合计 {} 字节\n".format(len(blob)))
                with open(bin_path, "wb") as fh:
                    fh.write(blob)
            except Exception as exc:
                messagebox.showerror(APP_NAME, "导出失败:\n{}".format(exc))
                return
            self._log_sys("已导出 FPGA 帧(未发送): {} 事件 / {} 字节".format(
                len(frame.events), len(blob)))
            messagebox.showinfo(
                APP_NAME,
                "已导出，没有发送任何数据。\n\n"
                "逐事件 + 帧字节：{}\n裸帧字节流：{}\n\n"
                "合计 {} 个事件 / {} 字节".format(
                    os.path.basename(txt_path), os.path.basename(bin_path),
                    len(frame.events), len(blob)))
            return

        # ---- 裸 MIDI：老行为 ----
        try:
            msgs, note = self._load_midi_file(path)
        except Exception as exc:
            messagebox.showerror(APP_NAME, "读取 MIDI 文件失败:\n{}".format(exc))
            return
        playable = [m for m in msgs if not getattr(m, "is_meta", False)]
        if not playable:
            messagebox.showwarning(APP_NAME, "这个文件里没有可发送的消息。")
            return
        # 帧尾也算进去 —— 导出的必须和真正发出去的一模一样, 否则核对就没意义
        try:
            tail = self.midi_tail_bytes()
        except ValueError as exc:
            messagebox.showerror(APP_NAME, "帧尾设置有误:\n{}".format(exc))
            return

        blob = bytearray()
        try:
            with open(txt_path, "w", encoding="utf-8") as fh:
                fh.write("# {}\n".format(os.path.basename(path)))
                fh.write("# {}\n".format(note))
                fh.write("# 共 {} 条消息 —— 这就是点「发送MIDI文件」时会出去的字节\n"
                         .format(len(playable)))
                fh.write("# 序号\t间隔(秒)\t消息\t字节数\tHEX\n")
                for i, msg in enumerate(playable, 1):
                    raw = bytes(msg.bytes())
                    blob += raw
                    fh.write("{}\t{}\t{}\t{}\t{}\n".format(
                        i, getattr(msg, "time", ""), msg, len(raw), fmt_hex(raw)))
                if tail:
                    blob += tail
                    fh.write("# --- 整段发完后追加一次帧尾 ---\n")
                    fh.write("# 帧尾\t\t\t\t{}\t{}\n".format(len(tail), fmt_hex(tail)))
                    fh.write("# 注: 中途停止时不会追加帧尾\n")
                fh.write("# 合计 {} 字节\n".format(len(blob)))
            with open(bin_path, "wb") as fh:
                fh.write(bytes(blob))
        except Exception as exc:
            messagebox.showerror(APP_NAME, "导出失败:\n{}".format(exc))
            return
        self._log_sys("已导出字节流(未发送): {} 条 / {} 字节{}".format(
            len(playable), len(blob), ", 含帧尾 " + fmt_hex(tail) if tail else ""))
        messagebox.showinfo(
            APP_NAME,
            "已导出，没有发送任何数据。\n\n"
            "逐条 hex：{}\n裸字节流：{}\n\n"
            "合计 {} 条消息 / {} 字节{}".format(
                os.path.basename(txt_path), os.path.basename(bin_path),
                len(playable), len(blob),
                "（含帧尾 {}）".format(fmt_hex(tail)) if tail else "（不含帧尾）"))

    def _set_dot(self, color):
        self.dot.itemconfigure(self.dot_id, fill=color)

    # ------------------------------------------------------------------
    # 发送
    # ------------------------------------------------------------------
    def send_once(self):
        if not self.is_open():
            messagebox.showwarning(APP_NAME, "串口未打开。")
            return
        raw = self.tx_text.get("1.0", "end-1c")
        try:
            if self.hex_send_var.get():
                payload = parse_hex(raw)
            else:
                if not raw:
                    return
                payload = raw.encode(self.enc_var.get(), errors="replace")
                payload += {"无": b"", "LF(\\n)": b"\n", "CR(\\r)": b"\r",
                            "CRLF(\\r\\n)": b"\r\n"}.get(self.lineend_var.get(), b"")
        except (ValueError, LookupError) as exc:
            messagebox.showerror(APP_NAME, "发送内容有误: {}".format(exc))
            return
        if not payload:
            return
        self.txq.put((payload, "HEX" if self.hex_send_var.get() else "文本"))
        if raw.strip():
            self._push_history(raw)

    # ------------------------------------------------------------------
    # MIDI: 组装成 hex 发送
    # ------------------------------------------------------------------
    def on_midi_change(self, _event=None):
        """MIDI 类型或参数变了: 调整字段可用状态并刷新 hex 预览。"""
        name = self.midi_type_var.get()
        spec = _MIDI_BY_NAME.get(name)
        if spec is None:
            return
        f1, f2 = spec[2], spec[3]
        realtime = name.startswith("系统实时")
        sysex = name.startswith("SysEx")

        self.midi_ch_box.configure(state="disabled" if (realtime or sysex) else "readonly")
        self.midi_d2_box.configure(state="normal" if f2 else "disabled")
        if f1 is None:
            self.midi_d1_box.configure(state="disabled")
        else:
            self.midi_d1_box.configure(state="normal")
            if f1 == "音符号":
                self.midi_d1_box.configure(values=note_choices())
            elif f1 == "载荷HEX":
                self.midi_d1_box.configure(values=[])
                try:
                    parse_hex(self.midi_d1_var.get())
                except ValueError:
                    self.midi_d1_var.set("7D 01 02 F7")
            else:
                self.midi_d1_box.configure(values=[str(i) for i in range(128)])

        try:
            data, desc = self._midi_current()
            self.midi_preview_var.set("= {}   {}".format(fmt_hex(data), desc))
        except ValueError as exc:
            self.midi_preview_var.set("参数有误: {}".format(exc))

        # 「按原速」只管裸 MIDI 转发; 转 FPGA 帧时时值在帧里, 用不上
        try:
            self.midi_realtime_chk.configure(
                state="normal" if not self.midi_to_fpga_var.get() else "disabled")
        except Exception:
            pass

    def midi_tail_bytes(self):
        """帧尾字节。未勾选或填空白则返回 b"", 内容非法时抛 ValueError。"""
        if not self.midi_tail_var.get():
            return b""
        text = self.midi_tail_hex_var.get().strip()
        if not text:
            return b""
        return parse_hex(text)

    def midi_head_bytes(self):
        """帧头字节（默认 AA）。未勾选或填空白则返回 b"", 内容非法时抛 ValueError。

        ★ 板子那边 0xAA 是"开窗"：没有它，后面所有乐谱字节都会被丢掉，
          收到帧尾 0xFF 也不会开播 —— 这是"只勾帧尾、忘了帧头"一声不响的根因。
        """
        if not self.midi_head_var.get():
            return b""
        text = self.midi_head_hex_var.get().strip()
        if not text:
            return b""
        return parse_hex(text)

    def midi_transpose(self):
        """「移调」栏 -> 半音数；"自动"或空 -> None（交给 fpga_midi 自己挑音区）。

        范围 ±24 个半音（±2 个八度）。板子只有 36 个键（A2..G#5），移调后
        超出 0..35 的音会被丢掉 —— 换几档看日志里"丢掉的音"有几个，
        找到丢得最少、又最接近原调的那档就行。
        """
        t = self.midi_trans_var.get().strip()
        if not t or t.lower() in ("自动", "auto"):
            return None
        try:
            v = int(t.replace("＋", "+").replace("－", "-"))
        except ValueError:
            raise ValueError("移调要填整数半音（例：-12 / 0 / +7），或填「自动」")
        if not -24 <= v <= 24:
            raise ValueError("移调范围是 -24..+24 个半音（±2 个八度）")
        return v

    def on_midi_tail_change(self):
        self.on_midi_change()

    def _midi_current(self):
        """返回 (要发出去的字节, 说明)。勾了帧尾就把帧尾接在消息后面。"""
        data, desc = build_midi(self.midi_type_var.get(), self.midi_ch_var.get(),
                                self.midi_d1_var.get(), self.midi_d2_var.get())
        tail = self.midi_tail_bytes()
        if tail:
            data = data + tail
            desc += "  + 帧尾 " + fmt_hex(tail)
        return data, desc

    def send_midi(self):
        """把 MIDI 消息按原始字节发出去(不走行尾, 不做文本编码)。"""
        try:
            data, desc = self._midi_current()
        except ValueError as exc:
            messagebox.showerror(APP_NAME, "MIDI 参数有误:\n{}".format(exc))
            return
        if not self.is_open():
            messagebox.showwarning(APP_NAME, "串口未打开。")
            return
        self.txq.put((data, "MIDI " + self.midi_type_var.get()))
        self._log_sys("发送 MIDI: {}   ({})".format(fmt_hex(data), desc))

    def fill_midi_hex(self):
        """把 MIDI 消息以 hex 文本填进发送区, 方便改完再手动发。"""
        try:
            data, _desc = self._midi_current()
        except ValueError as exc:
            messagebox.showerror(APP_NAME, "MIDI 参数有误:\n{}".format(exc))
            return
        self.hex_send_var.set(True)
        self.tx_text.delete("1.0", "end")
        self.tx_text.insert("1.0", fmt_hex(data))

    # ---- MIDI 文件: 选一个 .mid/.syx, 逐条转成原始字节发出去 ----
    def send_midi_file(self):
        """选文件并发送; 发送过程中按钮变成「停止发送」。"""
        if self._midi_playing:
            self.stop_midi_file()
            return
        if mido is None:
            messagebox.showerror(
                APP_NAME, "发送 MIDI 文件需要 mido 库。\n\n请先执行:\n"
                          "python -m pip install mido")
            return
        if not self.is_open():
            messagebox.showwarning(APP_NAME, "串口未打开。")
            return
        path = filedialog.askopenfilename(
            title="选择要发送的 MIDI 文件",
            initialdir=midi_folder(),
            filetypes=[("MIDI 文件", "*.mid *.midi *.kar *.syx"),
                       ("所有文件", "*.*")])
        if not path:
            return
        self.start_midi_play(path)

    def _load_midi_file(self, path):
        """读 .mid/.syx, 返回 (消息列表, 说明文字)。"""
        if path.lower().endswith(".syx"):
            msgs = list(mido.read_syx_file(path))
            note = "SysEx 文件"
        else:
            mid = mido.MidiFile(path)
            # 迭代 MidiFile 会把所有音轨按时间合并, 且 time 已换算成秒
            msgs = list(mid)
            note = "{} 音轨, 时长 {:.1f} 秒".format(len(mid.tracks), mid.length)
        return msgs, note

    def start_midi_play(self, path):
        """载入并开始发送。返回 False 表示没发起来(界面上会有说明)。

        两条路:
          「转FPGA帧」勾上(默认) —— 先把 .mid 转成板子认的乐谱帧再整帧发出去。
              这是板子唯一能响的方式: 键号(0x00~0x23) + 和弦(0xC0..0xC1)
              + 时值(0xC2) + 速度(0xC4/0xC5), 外面套帧头 AA / 帧尾 FF。
          「转FPGA帧」不勾 —— 老行为: 把裸 MIDI 消息原样转发(给别的设备用)。
              板子收到裸 MIDI 会一个字节不认, 一声不响。
        """
        if mido is None:
            messagebox.showerror(APP_NAME, "需要 mido 库才能读取 MIDI 文件。")
            return False
        if not self.is_open():
            messagebox.showwarning(APP_NAME, "串口未打开。")
            return False
        try:
            head = self.midi_head_bytes()
            tail = self.midi_tail_bytes()
        except ValueError as exc:
            messagebox.showerror(APP_NAME, "帧头/帧尾设置有误:\n{}".format(exc))
            return False
        try:
            self.midi_transpose()          # 提前校验，别等转换到一半才报错
        except ValueError as exc:
            messagebox.showerror(APP_NAME, "移调设置有误:\n{}".format(exc))
            return False

        # .syx 不是乐谱, 只能走裸字节转发
        if self.midi_to_fpga_var.get() and not path.lower().endswith(".syx"):
            return self._start_fpga_frame(path, head, tail)

        try:
            msgs, note = self._load_midi_file(path)
        except Exception as exc:
            self._log_sys("读取 {} 失败: {}".format(os.path.basename(path), exc))
            messagebox.showerror(APP_NAME, "读取 MIDI 文件失败:\n{}".format(exc))
            return False
        playable = [m for m in msgs if not getattr(m, "is_meta", False)]
        if not playable:
            messagebox.showwarning(APP_NAME, "这个文件里没有可发送的消息。")
            return False

        realtime = bool(self.midi_realtime_var.get())
        self._midi_stop.clear()
        self._midi_playing = True
        self.midi_file_btn.configure(text="停止发送")
        self._log_sys("载入 {} : {}, {} 条可发送消息, {}{}".format(
            os.path.basename(path), note, len(playable),
            "按原速播放" if realtime else "全速发送",
            ", 整段发完追加帧尾 " + fmt_hex(tail) if tail else ", 不加帧尾"))
        self._log_sys("注意: 这是【裸 MIDI】, 板子不认; 想让它响请勾上「转FPGA帧」。")
        threading.Thread(target=self._midi_play_worker,
                         args=(path, playable, realtime, tail), daemon=True).start()
        return True

    # ------------------------------------------------------------------
    # 乐谱 -> FPGA 帧 -> 整帧发送
    # ------------------------------------------------------------------
    def _start_fpga_frame(self, path, head, tail):
        if fpga_midi is None or fpga_proto is None:
            messagebox.showerror(
                APP_NAME,
                "缺 fpga_midi.py / fpga_proto.py / score_jp.py。\n\n"
                "这三个文件要和本程序放在同一目录。")
            return False
        name = os.path.basename(path)
        try:
            tr = self.midi_transpose()
            if tr is not None:
                self._log_sys("按「移调 %+d 半音」生成（不再自动挑音区）" % tr)
            frame, blob, rep = fpga_midi.midi_to_frame(path, head=head, tail=tail,
                                                       shift=tr)
        except Exception as exc:
            self._log_sys("转换 {} 失败: {}".format(name, exc))
            messagebox.showerror(APP_NAME, "转 FPGA 帧失败:\n{}".format(exc))
            return False

        for line in rep.text().splitlines():
            self._log_sys(line)
        if rep.dropped:
            self._log_sys("★ 有 %d 个音超出板子音域(0..35 号键)被丢掉了。"
                          "换个「移调」档位再试，挑「丢掉的音 = 0」的那一档。"
                          % rep.dropped)
        if not head:
            self._log_sys("★ 警告: 帧头是空的。板子靠 0xAA 开窗, 没有它一个字节都不收, "
                          "帧尾 0xFF 也不会开播。")
        if not tail:
            self._log_sys("★ 警告: 帧尾是空的。板子靠 0xFF 关窗并开始演奏, "
                          "没有它只会收谱、不会响。")

        self._midi_stop.clear()
        self._midi_playing = True
        self.midi_file_btn.configure(text="停止发送")
        self._log_sys("整帧发送 {}: {} 字节, 9600 8N1 约 {:.1f} 秒发完".format(
            name, len(blob), len(blob) * 10.0 / 9600.0))
        threading.Thread(target=self._frame_send_worker,
                         args=(name, blob), daemon=True).start()
        return True

    def _frame_send_worker(self, name, blob, chunk=TX_SEND_CHUNK):
        """整帧按块投递。块大小只影响"停止发送"的响应速度, 不影响内容。"""
        sent = 0
        t0 = time.time()
        for off in range(0, len(blob), chunk):
            if self._midi_stop.is_set() or not self.is_open():
                break
            piece = blob[off:off + chunk]
            self.txq.put((piece, "FPGA帧 {} {}/{}".format(name, off + len(piece), len(blob))))
            sent += len(piece)
        stopped = self._midi_stop.is_set() or not self.is_open()
        elapsed = time.time() - t0
        self.uiqueue.put(lambda: self._frame_send_done(name, sent, len(blob), elapsed, stopped))

    def _frame_send_done(self, name, sent, total, elapsed, stopped):
        self._midi_playing = False
        self.midi_file_btn.configure(text="发送MIDI文件")
        if stopped:
            self._log_sys("{} 已停止: 发出 {}/{} 字节, 用时 {:.1f} 秒".format(
                name, sent, total, elapsed))
        else:
            self._log_sys("{} 整帧发送完成: {}/{} 字节, 用时 {:.1f} 秒".format(
                name, sent, total, elapsed))
            self._log_sys("板子现在会按帧里的时值自己演奏, 蓝牙可以不用管了。")

    def _midi_play_worker(self, path, msgs, realtime, tail=b""):
        """逐条发送。帧尾只在**整段发完之后追加一次**, 不是每条都加。"""
        sent = 0
        t0 = time.time()
        name = os.path.basename(path)
        for msg in msgs:
            if self._midi_stop.is_set() or not self.is_open():
                break
            if realtime:
                delay = float(getattr(msg, "time", 0) or 0)
                if delay > 0 and self._midi_stop.wait(delay):
                    break
            try:
                # 注意: mido 的 Message.bytes() 返回的是 list[int], 不是 bytes
                # (取 bytes 要用 .bin())。直接丢给 serial.write() 会抛
                # TypeError 并被当成断线, 一条都发不出去。
                data = bytes(msg.bytes())
            except Exception:
                continue
            self.txq.put((data, "MIDI文件 {} #{}".format(name, sent + 1)))
            sent += 1

        stopped = self._midi_stop.is_set() or not self.is_open()
        tailed = False
        if tail and not stopped:
            # 整段正常发完, 排在队尾追加一次。写线程按顺序取, 所以它一定最后出去。
            self.txq.put((tail, "MIDI文件 {} 帧尾".format(name)))
            tailed = True
        elapsed = time.time() - t0
        self.uiqueue.put(
            lambda: self._midi_play_done(name, sent, elapsed, stopped, tailed))

    def _midi_play_done(self, name, sent, elapsed, stopped, tailed):
        self._midi_playing = False
        self.midi_file_btn.configure(text="发送MIDI文件")
        if tailed:
            extra = ", 已追加帧尾"
        elif stopped:
            extra = ", 中途停止, 未追加帧尾"
        else:
            extra = ""
        self._log_sys("{} {}: 共 {} 条, 用时 {:.1f} 秒{}".format(
            name, "已停止" if stopped else "发送完成", sent, elapsed, extra))

    def stop_midi_file(self):
        self._midi_stop.set()

    # ------------------------------------------------------------------
    # 演奏按键自定义映射（按键 -> 音符）
    # ------------------------------------------------------------------
    def _keymap(self):
        """当前映射（36 个值）"""
        return keymap.norm(self.cfg.get("keymap"))

    def _set_keymap(self, m):
        self.cfg["keymap"] = list(keymap.norm(m))
        save_config(self.cfg)
        self._keymap_info()

    def _keymap_info(self):
        """刷新那一行小字"""
        try:
            self.keymap_lbl.configure(text=keymap.describe(self._keymap()))
        except Exception:
            pass

    def _keymap_autosend(self):
        """连上之后自动下发一次映射（板子没有掉电保存，一断电就回默认）"""
        if not self.is_open():
            return
        self._keymap_info()
        if keymap is None:
            return
        if keymap.is_default(self._keymap()):
            self._log_sys("按键映射：当前是默认的（键号 n 就发音符 n），不下发。")
            return
        self.on_keymap_send()

    def on_keymap_send(self):
        """把整张映射表下发给板子（36 条，每条 3 字节）"""
        if not self.is_open():
            messagebox.showwarning(APP_NAME, "串口未打开，没法下发按键映射。")
            return False
        blob = keymap.encode_all(self._keymap())
        # ★ 清一下队列：别让映射指令排在几百字节谱子后面才轮上
        self._flush_tx_queue()
        self.txq.put((blob, "按键映射"))
        self._log_sys("按键映射：已下发 %d 字节（36 条 x 3）。" % len(blob))
        det = keymap.detail(self._keymap())
        for line in det.splitlines():
            self._log_sys(line)
        self._log_sys("★ 这张表只影响【手弹】：乐谱播放走的是声部号，"
                      "所以同一份谱子不管映射怎么改，弹出来还是谱子上写的音。")
        return True

    def open_keymap(self):
        """打开映射编辑器：36 行，每行一个下拉框"""
        if fpga_proto is None:
            messagebox.showerror(APP_NAME, "缺 fpga_proto.py（要和本程序放同一目录）。")
            return
        win = tk.Toplevel(self.root)
        win.title("演奏按键自定义映射")
        win.transient(self.root)
        win.geometry("520x640")

        tip = ("把某个按键重新指定成另一个音。改动【只影响手弹】，"
               "不影响放谱子。\n"
               "「禁用」= 这个键按下去不发声（触键不灵时可以关掉它）。\n"
               "改完点「下发到板子」—— 板子一断电就回默认，所以每次连上会自动下发一遍。")
        ttk.Label(win, text=tip, justify="left", wraplength=490).pack(
            anchor="w", padx=8, pady=(8, 6))

        head = ttk.Frame(win)
        head.pack(fill="x", padx=8)
        ttk.Label(head, text="物理键", width=10).pack(side="left")
        ttk.Label(head, text="默认音", width=8).pack(side="left")
        ttk.Label(head, text="改成的音", width=16).pack(side="left")

        # 可滚动的 36 行
        wrap = ttk.Frame(win)
        wrap.pack(fill="both", expand=True, padx=8, pady=4)
        canvas = tk.Canvas(wrap, highlightthickness=0)
        sb = ttk.Scrollbar(wrap, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas)
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        cur = self._keymap()
        vars_ = []
        choices = keymap.choices()
        for k in range(keymap.N_KEYS):
            row = ttk.Frame(inner)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text="键 %-2d" % k, width=10).pack(side="left")
            ttk.Label(row, text=fpga_proto.key_name(k), width=8).pack(side="left")
            v = tk.StringVar(value=choices[keymap.value_to_choice(cur[k])])
            cb = ttk.Combobox(row, textvariable=v, values=choices,
                              width=16, state="readonly")
            cb.pack(side="left")
            # 改一下就立刻生效到本地（下发要另外点按钮）
            cb.bind("<<ComboboxSelected>>",
                    lambda e, kk=k, vv=v: self._keymap_row_changed(kk, vv))
            vars_.append(v)

        btns = ttk.Frame(win)
        btns.pack(fill="x", padx=8, pady=8)
        ttk.Button(btns, text="下发到板子", width=12,
                   command=self.on_keymap_send).pack(side="left")
        ttk.Button(btns, text="全部恢复默认", width=13,
                   command=lambda: self._keymap_reset(vars_)).pack(side="left", padx=6)

        def _close():
            self._set_keymap(self._keymap())
            win.destroy()

        ttk.Button(btns, text="关闭", width=8, command=_close).pack(side="right")
        win.protocol("WM_DELETE_WINDOW", _close)
        self._keymap_win_vars = vars_

    def _keymap_row_changed(self, k, var):
        idx = keymap.choices().index(var.get()) if var.get() in keymap.choices() else 0
        m = self._keymap()
        m[k] = keymap.choice_to_value(idx)
        self._set_keymap(m)

    def _keymap_reset(self, vars_):
        m = keymap.default_map()
        self._set_keymap(m)
        ch = keymap.choices()
        for k, v in enumerate(vars_):
            v.set(ch[keymap.value_to_choice(m[k])])
        self._log_sys("按键映射：已恢复默认（键号 n 就发音符 n）。"
                      "记得点「下发到板子」。")

    # ------------------------------------------------------------------
    # 演奏控制：停止 / 重播 / 停止并清空
    # ------------------------------------------------------------------
    def _flush_tx_queue(self):
        """把还没发出去的队列清掉，返回丢掉了几批。

        ★ 必须先清队列再发控制指令：发送线程会把整帧几百上千字节一口气塞进队列，
          不清的话停止指令要排在这些字节后面才发得出去 —— 等它轮到，
          板子早把整首曲子收完、也弹得差不多了，等于停不了。
        """
        dropped = 0
        while True:
            try:
                item = self.txq.get_nowait()
            except queue.Empty:
                break
            if item is None:
                self.txq.put(None)      # 关闭串口用的哨兵，不能丢
                break
            dropped += 1
        return dropped

    def _stop_local(self):
        """停掉这边的发送和本地播放"""
        self._midi_stop.set()
        self._host_stop.set()

    def stop_score(self):
        """停止演奏（★ 乐谱留在板子里，可以重播）。

        发一个 0xC7。板子收到后：停播 + 把所有键放掉，
        但 wr_slots / frame_slots 一个字不动 —— 所以随后一个 0xC8 就能重播。
        """
        self._stop_local()
        if not self.is_open():
            self._log_sys("停止演奏：串口没开，板子那边停不了（本地发送/播放已停）。")
            return
        d = self._flush_tx_queue()
        if d:
            self._log_sys("停止演奏：丢掉队列里还没发出去的 %d 批字节。" % d)
        self.txq.put((bytes([keymap.STOP_BYTE]), "停止演奏"))
        self._log_sys("停止演奏：已发 0xC7 —— 板子立刻松键停播，"
                      "乐谱【保留在板子里】，点「重播」可以再来一遍。")

    def replay_score(self):
        """重播板子里已经存着的那一帧（重新发谱子）"""
        self._stop_local()
        if not self.is_open():
            self._log_sys("重播：串口没开。")
            return
        d = self._flush_tx_queue()
        if d:
            self._log_sys("重播：丢掉队列里还没发出去的 %d 批字节。" % d)
        self.txq.put((bytes([keymap.REPLAY_BYTE]), "重播"))
        self._log_sys("重播：已发 0xC8 —— 板子用内存里原来那份谱子从头再放一遍。")

    def stop_and_clear(self):
        """停止演奏并清空乐谱（纯用现有协议：帧头 + 帧尾，什么都不夹）。

            0xAA   开窗 + 写指针归零 + 【停播】+ 把所有键放掉
            0xFF   关窗；此时这一帧的事件数是 0，板子判为"空帧 → 不开播"

        ★ 和 0xC7 的区别：这个顺手把乐谱清了，想再听要重发一遍。
          好处是【不依赖新指令】—— 板子还没烧新 bitstream 时这个照样管用。
        """
        self._stop_local()
        if not self.is_open():
            self._log_sys("停止并清空：串口没开，板子那边停不了（本地发送/播放已停）。")
            return
        d = self._flush_tx_queue()
        if d:
            self._log_sys("停止并清空：丢掉队列里还没发出去的 %d 批字节。" % d)

        try:
            head = self.midi_head_bytes()
            tail = self.midi_tail_bytes()
        except ValueError as exc:
            self._log_sys("停止并清空：帧头/帧尾设置有误（%s），改用默认的 AA FF。" % exc)
            head, tail = b"", b""
        if not head:
            head = bytes([0xAA])
        if not tail:
            tail = bytes([0xFF])
        blob = head + tail
        self.txq.put((blob, "停止并清空"))
        self._log_sys("停止并清空：已发 %s —— 停播松键 + 清空乐谱 + 关窗，"
                      "想再听要重发一次谱子。" % fmt_hex(blob))

    # ------------------------------------------------------------------
    # ★ V31：效果开关 / 音色（上位机 -> 板子）
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # ★ V31.1：体验优化 —— 插队发送 / 待确认显示 / 状态未知提示
    #   用户反馈的三个问题：
    #     ① 按一个开关有时会"连带"另外几个一起变
    #        -> 根因是【界面显示的状态和板子真实状态不一致】（连上时那次查询
    #           没回来、或者在板子上按过按钮）。第一次点击后板子回报整字节，
    #           于是一排按钮一起跳。现在：没收到过回报就一律显示"未知"，
    #           不再假装是"关"。
    #     ② 传输延迟导致"不知道到底开没开"
    #        -> 命令【插队】发（见 _send_front），并且按下后按钮显示橙色
    #           "待确认"，板子回报到了才落定；1.2 秒没回报就标"未确认"并重问一次。
    #     ③ 音色连点导致重复指令
    #        -> 同一档在"待确认"期间再点不再重发；不同档允许改主意（后发覆盖）。
    # ------------------------------------------------------------------
    def _send_front(self, payload, label):
        """把一条【控制指令】插到发送队列最前面（而不是排到队尾）。

        ★ 为什么可以插队：效果/音色/查询这三个字节（0xC9/0xCA/0xCB）在板子那边
          是【不受开窗限制】的命令，夹在谱子字节中间也不会被当成音符 ——
          所以插队不破坏正在发的谱子，却能让"按下开关"立刻生效。
          不插队的话它要排在几千字节谱子后面（9600 波特率要好几秒），
          这就是"有传输延迟、不知道开没开成"的来源。

        ★ 只给这三个字节插队：
          0xCC（八度偏移）会改"待落盘事件"，插进去会改错音符；
          0xAA/0xFF 是帧头帧尾，更不能插。
        """
        try:
            with self.txq.mutex:                    # queue.Queue 内部那把锁
                self.txq.queue.appendleft((payload, label))   # 底层是 deque
                self.txq.not_empty.notify()
            return True
        except Exception:                                       # noqa: BLE001
            return False

    def _fx_pending_set(self, kind, val):
        """记下"刚发出去、还没等到板子回报"的那一次操作（用于橙色待确认）。"""
        self.fx_pending = {"kind": kind, "val": val, "t": time.time()}
        self._pending_retry = False
        self._update_fx_ui()

    def _fx_pending_check(self):
        """发出去 1.2 秒还没等到回报 -> 提示一次，并撤掉"待确认"标记。

        板子不回报的常见原因：还没烧 V31 之后的固件（0xC9/0xCA/0xCB 老固件不认）、
        或者模块掉线。不提示的话用户只会看到"点了好像没反应"。
        """
        p = getattr(self, "fx_pending", None)
        if not p or (time.time() - p["t"]) < 1.2:
            return
        self.fx_pending = None
        if not self._pending_retry and self.is_open():
            self._pending_retry = True
            self.request_fx_state(quiet=True)      # 再问一次，别让界面停在错的显示上
            self._log_sys("★ 板子 1.2 秒还没回报效果状态：刚点的那个标成"
                          "「未确认」并重新问了一次。常见原因是板子还没烧 "
                          "V31 之后的固件（0xC9/0xCA/0xCB 老固件不认）。")
        self._update_fx_ui()

    def toggle_effect(self, bit):
        """翻转第 bit 个效果开关（和板子上按对应按钮同义）。

        ★ 为什么发"翻转"而不是"设成 1/0"：
          板子才是权威。翻转的语义和按物理按钮一模一样，所以上位机
          不需要知道当前状态就能安全地切 —— 不会出现"软件以为它是关的、
          其实是开的，一设反而设反了"。
        ★ 按下之后按钮变橙 = 待确认，板子回报 0x5D 才落定 —— 传输延迟不再靠猜。
        """
        if fpga_rec is None:
            messagebox.showerror(APP_NAME, "缺 fpga_rec.py（要和本程序放同一目录）。")
            return
        if not self.is_open():
            self._log_sys("效果开关：串口没打开，板子那边切不了。")
            return
        name = dict((k, cn) for k, cn in fpga_rec.FX_BITS).get(
            fpga_rec.FX_BITS[bit][0], "位%d" % bit)
        payload = fpga_rec.pack_fx_toggle(bit)
        fronted = self._send_front(payload, "效果开关 " + name)
        if not fronted:
            self.txq.put((payload, "效果开关 " + name))
        self._fx_pending_set("fx", bit)
        self._log_sys("效果开关：已发出 0xC9 %02X「%s」%s，等板子回报…"
                      % (bit, name, "（插到队首）" if fronted else "（排队）"))

    def set_tone(self, val=None):
        """点音色按钮：直接切到那一档（发 0xCA）。

        ★ 和效果开关用"翻转"不同，音色这边是【直接选档】：三档没法用一位翻转
          表达，而且软件里本来就是"我想听吉他"这种明确意图。
        ★ 防重复：同一档在"待确认"期间再点【不再重发】（这正是"连续按下导致
          指令重复"的来源）；点了别的档就照发，后发的覆盖先发的。
        """
        if fpga_rec is None or val is None:
            return
        p = getattr(self, "fx_pending", None)
        if p and p.get("kind") == "tone" and p.get("val") == val:
            self._log_sys("音色：%s 已经发过、还在等板子回报，这次不重复发。"
                          % fpga_rec.TONE_NAMES[val])
            return
        name = fpga_rec.TONE_NAMES[val]
        if not self.is_open():
            self._log_sys("音色：串口没打开，板子那边切不了。")
            self._update_fx_ui()
            return
        payload = fpga_rec.pack_tone_set(val)
        fronted = self._send_front(payload, "音色 " + name)
        if not fronted:
            self.txq.put((payload, "音色 " + name))
        self._fx_pending_set("tone", val)
        self._log_sys("音色：已发出 0xCA %02X「%s」%s，等板子回报…"
                      % (val, name, "（插到队首）" if fronted else "（排队）"))

    def request_fx_state(self, quiet=False):
        """请板子回报一次效果/音色状态（发 0xCB）。"""
        if fpga_rec is None or not self.is_open():
            return
        payload = fpga_rec.pack_fx_req()
        if not self._send_front(payload, "查询效果状态"):
            self.txq.put((payload, "查询效果状态"))
        self.fx_query_at = time.time()
        if not quiet:
            self._log_sys("已请求板子回报一次效果/音色状态（0xCB）。")

    def _update_fx_ui(self):
        """按板子回报的状态刷新那一排开关和音色按钮。

        三种显示状态（这就是"不知道自己开没开成"的解法）：
          · 已确认（绿/蓝实心）: 板子回报过、就是它说的那个状态
          · 待确认（橙色）      : 刚点过、板子还没回报（传输延迟期间）
          · 未知（灰 + 小字）    : 还没收到过任何回报 —— 这时【不假装是"关"】
        """
        st = self.rec.fx_state if self.rec is not None else 0
        known = (self.rec is not None and self.rec.fx_seen)
        pend = getattr(self, "fx_pending", None)
        for bit, b in getattr(self, "fx_btns", {}).items():
            on = bool((st >> bit) & 1)
            is_pend = bool(pend and pend.get("kind") == "fx" and pend.get("val") == bit)
            try:
                if is_pend:
                    b.configure(relief="raised", bg="#e08a1e", fg="white",
                                activebackground="#c4761a", activeforeground="white")
                elif not known:
                    b.configure(relief="raised", bg="#eeeeee", fg="#666666",
                                activebackground="#dddddd", activeforeground="#444444")
                else:
                    b.configure(relief=("sunken" if on else "raised"),
                                bg=("#2e7d32" if on else "#e0e0e0"),
                                fg=("white" if on else "black"),
                                activebackground=("#256428" if on else "#d0d0d0"),
                                activeforeground=("white" if on else "black"))
            except Exception:                                       # noqa: BLE001
                pass
        tone = self.rec.tone_sel if self.rec is not None else None
        # 音色按钮：当前那一档高亮；刚点过还没确认的画橙色
        for i, tb in enumerate(getattr(self, "tone_btns", [])):
            is_pend = bool(pend and pend.get("kind") == "tone" and pend.get("val") == i)
            on = (tone == i)
            try:
                if is_pend:
                    tb.configure(relief="raised", bg="#e08a1e", fg="white",
                                 activebackground="#c4761a", activeforeground="white")
                elif not known:
                    tb.configure(relief="raised", bg="#eeeeee", fg="#666666",
                                 activebackground="#dddddd", activeforeground="#444444")
                else:
                    tb.configure(relief=("sunken" if on else "raised"),
                                 bg=("#1f5fa8" if on else "#e0e0e0"),
                                 fg=("white" if on else "black"),
                                 activebackground=("#1a4f8c" if on else "#d0d0d0"),
                                 activeforeground=("white" if on else "black"))
            except Exception:                                       # noqa: BLE001
                pass
        try:
            self.tone_var.set(fpga_rec.tone_name(tone))
            txt = fpga_rec.fx_summary(st, tone)
            if pend:
                txt = ("待板子确认…（%s）  " % ("效果 " + str(pend.get("val"))
                                               if pend.get("kind") == "fx"
                                               else "音色 " + str(pend.get("val")))) + txt
            elif not known:
                txt = "状态未知 —— 连上后会自动问一次，也可以点「刷新状态」"
            self.fx_lbl.configure(text=txt)
        except Exception:                                           # noqa: BLE001
            pass

    # ------------------------------------------------------------------
    # 演奏录音回传：板子 -> 上位机
    # ------------------------------------------------------------------
    #   板子(uart_packet_echo.v)每 10ms 采一次 key_mask, 一变就回传:
    #       [0x5A][mask 36位·小端 5字节][dt 2字节·小端, 单位 10ms]
    #       [0x5B][0/1]                  录音开关状态
    #   上位机发 [0x5C] 就能翻转板子的录音开关 —— 所以板子按键 10 和这个
    #   勾选框是【同一个开关】, 谁都能开、谁都能关, 状态靠 0x5B 同步。
    def on_rec_toggle(self):
        """勾选框被点：把开关请求发给板子。

        ★ 板子才是权威：它收到 0x5C 会翻转自己的录音开关，然后回 [0x5B][0/1]。
          所以这里发出去的只是"请求"，勾选框最终以板子的回报为准；板子要是
          没回话（没接线/没上电），就退回去并在日志里说清楚。
        """
        if fpga_rec is None:
            self.rec_var.set(self.rec_on)
            messagebox.showerror(APP_NAME, "缺 fpga_rec.py（要和本程序放在同一目录）。")
            return
        if not self.is_open():
            self.rec_var.set(self.rec_on)
            messagebox.showwarning(APP_NAME, "串口未打开，没法通知板子。")
            return
        self.txq.put((bytes([fpga_rec.REC_CMD]), "录音开关"))
        self.rec_pending = time.time()
        self.rec_status_var.set("已发出开关指令…")

    def _rec_feed(self, data):
        """把收到的字节喂给录音解析器。

        返回 True = 这批字节算录音回传，别再当文本打到接收区里。
        ★ 为什么必须挡：录音帧是二进制（0x5A + 5 字节掩码 + 2 字节时间），
          当文本打出来全是乱码。而且录音期间板子是【不回显】的（见
          uart_packet_echo.v 里的 echo_push），所以这时线上除了录音帧
          本来也不该有别的字节。
        ★ 为什么每批都要喂：一帧 8 个字节会被串口切成好几批送来，只喂
          "带头字节的那批"会让帧永远凑不齐。
        """
        if self.rec is None:
            return False
        was_on = self.rec_on
        # ★ V31：效果/音色是不是变了 —— 板子上按按钮、或者别的地方改了，
        #   界面那排开关要跟着动（转成 UI 任务，不能在这个线程里碰控件）。
        fx_before = (self.rec.fx_state, self.rec.tone_sel)
        new, st = self.rec.feed(data)
        if (self.rec.fx_state, self.rec.tone_sel) != fx_before:
            # ★ V31.1：板子回报了 = 刚才那次操作确认了（不管结果是不是我们想要的，
            #   界面都必须以板子为准），所以把"待确认"标记清掉，并记一行日志。
            if getattr(self, "fx_pending", None):
                self.fx_pending = None
                self.uiqueue.put(lambda: self._log_sys(
                    "板子回报：效果 %s / 音色 %s  ✓ 已确认"
                    % (fpga_rec.fx_summary(self.rec.fx_state),
                       fpga_rec.tone_name(self.rec.tone_sel))))
            self.uiqueue.put(self._update_fx_ui)
        if st is not None and bool(st) != self.rec_on:
            self.rec_on = bool(st)
            self.rec_pending = 0.0
            self.rec_var.set(self.rec_on)
            if self.rec_on:
                # ★ 清空是 RecParser 在解析流里按顺序做的（见 fpga_rec.feed）：
                #   板子的"开始"回报和这一遍最早的事件可能落在同一个串口批次里，
                #   整批解析完再清会把刚录到的头几个事件一起清掉。
                n = self.rec.cleared
                if n:
                    self._log_sys("板子回报：录音已开始（已清空上一遍的 %d 个事件）" % n)
                else:
                    self._log_sys("板子回报：录音已开始")
            else:
                self._log_sys("板子回报：录音已停止（本遍 %d 个事件 / %.1f 秒）"
                              % (len(self.rec.events), self.rec.duration_ms / 1000.0))
        if new or st is not None:
            self._rec_info()
        return was_on or self.rec_on

    def _rec_info(self):
        """刷新那行状态文字。"""
        if self.rec is None:
            self.rec_status_var.set("缺 fpga_rec.py")
            return
        n = len(self.rec.events)
        sec = self.rec.duration_ms / 1000.0
        fx = ""
        if self.rec.states:
            cur = fpga_rec.fx_summary(self.rec.fx_state)
            if cur != "全关":
                fx = "  效果:" + cur
        if self.rec_on:
            self.rec_status_var.set("● 录音中  {} 个事件 / {:.1f} 秒{}".format(n, sec, fx))
        elif n:
            self.rec_status_var.set("已录 {} 个事件 / {:.1f} 秒{}".format(n, sec, fx))
        else:
            self.rec_status_var.set("未录音")

    def _rec_timeout_check(self):
        """发出 0x5C 之后板子一直不回 0x5B —— 把勾选框退回去。"""
        if self.rec_pending and (time.time() - self.rec_pending) > REC_ACK_TIMEOUT:
            self.rec_pending = 0.0
            self.rec_var.set(self.rec_on)
            self._rec_info()
            self._log_sys("板子没回报录音状态（没等到 0x5B），开关没生效。")

    def _synth_rec(self, tag):
        """把回传的事件合成 MIDI。成功返回文件路径，失败返回 None（已弹过框）。"""
        if fpga_rec is None:
            messagebox.showerror(APP_NAME, "缺 fpga_rec.py（要和本程序放在同一目录）。")
            return None
        if self.rec is None or not self.rec.events:
            messagebox.showwarning(
                APP_NAME, "还没录到东西。\n\n先点「录音」或按板子上的按键 10 开始录，"
                          "弹一段再关掉，然后回来合成。")
            return None
        path = fpga_rec.auto_midi_path(tag)
        try:
            n_on, dur = fpga_rec.events_to_midi(self.rec.events, path,
                                                states=self.rec.states, bpm=REC_BPM)
        except Exception as exc:
            messagebox.showerror(APP_NAME, "合成 MIDI 失败:\n{}".format(exc))
            return None
        self._log_sys("合成 {}: {} 个事件 -> {} 个音, {:.1f} 秒".format(
            os.path.basename(path), len(self.rec.events), n_on, dur))
        self._log_sys("   {}".format(path))
        # 效果：板子回传了开关的时间轴，这里翻成了 MIDI 的 program change / CC
        if self.rec.states:
            self._log_sys("效果（按时间轴还原，接近板子但不是逐位一致）:")
            prev = 0
            for t_ms, st in self.rec.states:
                self._log_sys("   {:7.1f}s  {}".format(t_ms / 1000.0,
                                                       fpga_rec.fx_summary(st)))
                prev = st
            self._log_sys("   （延音锁存不用单独还原：它已经体现在音长上了）")
        return path

    def rec_play(self):
        """合成后就在这台电脑上放出来；正在放就变成「停止播放」。"""
        if self._host_playing:
            self._host_stop.set()
            return
        path = self._synth_rec("播放")
        if path:
            self._play_midi_host(path)

    def rec_export(self):
        path = self._synth_rec("导出")
        if not path:
            return
        try:
            subprocess.Popen('explorer /select,"{}"'.format(os.path.normpath(path)))
        except Exception:
            pass

    def rec_send_back(self):
        """合成后直接发回板子 —— 板子认得自己那套帧，能原样弹出来。"""
        path = self._synth_rec("回传")
        if path:
            self.start_midi_play(path)

    def rec_clear(self):
        if self.rec is not None:
            self.rec.reset()
        self._rec_info()
        self._log_sys("录音缓存已清空。")

    def _play_midi_host(self, path):
        """在上位机（这台电脑）播放 MIDI。

        优先用 mido 的默认输出口（Windows 上通常是 Microsoft GS Wavetable
        Synth）；一个输出口都没有就退回系统默认播放器 —— 总之别让它没声音。
        """
        name = os.path.basename(path)
        if mido is not None:
            port = None
            try:
                port = mido.open_output()
            except Exception as exc:
                self._log_sys("没有可用的 MIDI 输出口（{}），改用系统默认播放器。".format(exc))
            if port is not None:
                try:
                    msgs = list(mido.MidiFile(path))
                except Exception as exc:
                    self._log_sys("读 {} 失败: {}".format(name, exc))
                    msgs = []
                if msgs:
                    self._host_stop.clear()
                    self._host_playing = True
                    self.rec_play_btn.configure(text="停止播放")
                    self._log_sys("在这台电脑上播放 {}: {} 条消息".format(name, len(msgs)))
                    threading.Thread(target=self._host_play_worker,
                                     args=(port, msgs), daemon=True).start()
                    return
                try:
                    port.close()
                except Exception:
                    pass
        startfile = getattr(os, "startfile", None)
        if startfile is None:
            messagebox.showerror(APP_NAME, "没有可用的播放方式，MIDI 已保存到:\n{}".format(path))
            return
        try:
            startfile(path)
            self._log_sys("已交给系统默认播放器: {}".format(name))
        except Exception as exc:
            messagebox.showerror(APP_NAME, "打不开 {}:\n{}".format(name, exc))

    def _host_play_worker(self, port, msgs):
        sent = 0
        t0 = time.time()
        try:
            for m in msgs:
                if self._host_stop.is_set():
                    break
                if m.time and self._host_stop.wait(m.time):
                    break
                if not getattr(m, "is_meta", False):
                    port.send(m)
                    sent += 1
        except Exception as exc:
            err = str(exc)
            self.uiqueue.put(lambda e=err: self._log_sys("播放中断: " + e))
        finally:
            # ★ 收尾必须自己扛住异常：mido 的 Output 没有 panics()，
            #   写成 (port.reset, port.panics, port.close) 会在【构造元组时】
            #   就抛 AttributeError —— 那样 _host_play_done 永远发不出去，
            #   界面上就卡死在「停止播放」按不动了（踩过）。
            for name in ("reset", "panics", "close"):
                fn = getattr(port, name, None)
                if fn is None:
                    continue
                try:
                    fn()
                except Exception:
                    pass
            self.uiqueue.put(lambda: self._host_play_done(sent, time.time() - t0))

    def _host_play_done(self, sent, elapsed):
        self._host_playing = False
        self.rec_play_btn.configure(text="合成并播放")
        self._log_sys("播放结束: {} 条消息, 用时 {:.1f} 秒".format(sent, elapsed))

    def _push_history(self, raw):
        item = raw if len(raw) <= 120 else raw[:120] + "..."
        if item in self.history:
            self.history.remove(item)
        self.history.insert(0, item)
        del self.history[30:]
        self.refresh_hist_box()

    def refresh_hist_box(self):
        self.hist_box.configure(values=self.history)

    def on_history_pick(self, _event=None):
        sel = self.hist_box.get()
        if sel:
            self.tx_text.delete("1.0", "end")
            self.tx_text.insert("1.0", sel)

    def on_timer_toggle(self, force_off=False):
        if self.timer_job is not None:
            try:
                self.root.after_cancel(self.timer_job)
            except Exception:
                pass
            self.timer_job = None
        if force_off:
            self.timer_on_var.set(False)
            return
        if not self.timer_on_var.get():
            return
        if not self.is_open():
            self.timer_on_var.set(False)
            messagebox.showwarning(APP_NAME, "请先打开串口再启用定时发送。")
            return
        try:
            ms = max(20, int(self.timer_ms_var.get()))
        except ValueError:
            ms = 1000
            self.timer_ms_var.set("1000")
        self._timer_tick(ms)

    def _timer_tick(self, ms):
        if not self.timer_on_var.get() or not self.is_open():
            self.timer_job = None
            return
        self.send_once()
        self.timer_job = self.root.after(ms, lambda: self._timer_tick(ms))

    # ------------------------------------------------------------------
    # 文件 / 外部工具
    # ------------------------------------------------------------------
    def save_log(self):
        default = "{}_{}.txt".format(LOG_PREFIX, datetime.now().strftime("%Y%m%d_%H%M%S"))
        path = filedialog.asksaveasfilename(
            title="保存接收日志", initialdir=app_dir(), initialfile=default,
            defaultextension=".txt", filetypes=[("文本文件", "*.txt"), ("所有文件", "*.*")])
        if not path:
            return
        try:
            content = self.rx_text.get("1.0", "end-1c")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(content)
            self.status_var.set("日志已保存: {}".format(path))
            self._log_sys("日志已保存到 {}".format(path))
        except Exception as exc:
            messagebox.showerror(APP_NAME, "保存失败:\n{}".format(exc))

    def open_bluetooth_settings(self):
        try:
            os.startfile("ms-settings:bluetooth")
            self._log_sys("已打开 Windows 蓝牙设置页面")
        except Exception as exc:
            self._log_sys("打开蓝牙设置失败: {}".format(exc))

    def open_device_manager(self):
        try:
            subprocess.Popen(["mmc", "devmgmt.msc"], shell=False)
        except Exception as exc:
            self._log_sys("打开设备管理器失败: {}".format(exc))

    def show_help(self):
        messagebox.showinfo(
            "使用说明",
            "1. 先在此电脑上配对蓝牙设备 (点「蓝牙设置」)。\n"
            "2. 配对后蓝牙 SPP 会生成一个 COM 口, 点「刷新」,\n"
            "   勾选「仅蓝牙」可只列出蓝牙口。\n"
            "3. 选择端口 -> 设置波特率 -> 点「打开串口」。\n"
            "4. 下方发送区输入内容, 点「发送」或按 Ctrl+Enter。\n"
            "5. 接收区支持 HEX 显示、时间戳、暂停、保存日志。\n\n"
            "MIDI: 发送区最上面那行可以直接组装 MIDI 消息并按原始字节发出去。\n"
            "   · 选类型(Note On / CC / SysEx / 实时...) -> 通道 -> 参数\n"
            "   · 音符号可以写 C4 / C#4 / Db4, 也可以直接写数字(中央 C = C4 = 60)\n"
            "   · 右边实时显示组装出的 hex, 「发送MIDI」直接发, \n"
            "     「填入发送区」则把 hex 填进去, 方便你改几个字节再发\n"
            "   · 「帧头」/「帧尾」: 给按帧解析的自定义协议用。\n"
            "     单条 MIDI 只加帧尾; 发送 MIDI 文件时帧头帧尾都是「整段一次」。\n"
            "     注意 0xFF 在 MIDI 规范里是 System Reset, 真 MIDI 设备\n"
            "     会被复位。可写多个字节, 如 FF 或 0D 0A。\n\n"
            "发送MIDI文件: 点它选一个 .mid / .midi / .kar 文件,\n"
            "   对话框默认开在程序目录的 MIDI 文件夹。\n"
            "   · ★「转FPGA帧」默认勾上 —— 这是本板子(Gowin 音乐工程)唯一能响的方式:\n"
            "       先把 .mid 折成乐谱帧, 再整帧发出去。转换包含四件事:\n"
            "         1) MIDI 音符号 -> 板子键号(0x00~0x23, 走置换表, 不是 MIDI-48)\n"
            "         2) 同一时刻的多个音 -> 和弦(0xC0 ... 0xC1)\n"
            "         3) 时值 -> 0xC2, 速度 -> 0xC4/0xC5\n"
            "         4) 套上帧头 0xAA、帧尾 0xFF\n"
            "       整帧发完板子自己演奏, 蓝牙就不用管了。\n"
            "       ★ 帧头 AA 和帧尾 FF 两个都要有: AA 负责开窗收谱, FF 负责关窗开演。\n"
            "         只勾帧尾 = 窗口没开 = 一个字节都进不去 = 一声不响。\n"
            "   · 不勾「转FPGA帧」= 老行为, 把裸 MIDI 原样转发给别的设备。\n"
            "   · 「按原速」只在裸 MIDI 转发时可用。\n"
            "   · 「移调」: 单位是【半音】, 填 -24..+24 之间的整数, 或填「自动」。\n"
            "       板子只有 36 个键 = 3 个八度 (A2..G#5), 曲子经常落不进去。\n"
            "       移调就是在生成帧时把每个键号整体加上这个数。填「自动」时\n"
            "       程序只按整八度挪 (保持原调); 填具体数字就完全照你给的来,\n"
            "       超出 0..35 的音会被丢掉并在日志里报「丢掉的音 = N」。\n"
            "       ★ 用法: 换几档 -> 看哪一档「丢掉的音」最少、又最接近原调。\n"
            "       例: 一份谱子音域是 A#3..A#5, 填 0 会丢 68 个音,\n"
            "           填 -12 一个不丢但低了八度, 填 -2 一个不丢且只低 2 个半音\n"
            "           —— 最后这个通常最好听。\n"
            "   · 上面「通道」那一栏是 MIDI 协议自己的通道号 (1~16, 就是\n"
            "     90 3C 64 里 90 的低 4 位), 给标准 MIDI 设备分轨用的,\n"
            "     板子根本不看这个字节, 跟移调没有任何关系。\n"
            "   · 发送过程中按钮变成「停止发送」, 再点一次即可中断\n\n"
            "演奏录音回传: 把你在板子上手弹的东西录下来, 在这台电脑上合成并播放。\n"
            "   · 「录音 (板子按键10)」和板子上那块 16 键按钮板的【按键 10】\n"
            "     是同一个开关 —— 点勾选框(发 0x5C)或者按板子按键 10 都能开关,\n"
            "     另一边会自动跟着变(板子用 0x5B 回报状态, 以它为准)。\n"
            "   · 录音时板子每 10ms 采一次【门控】(按下 | 延音锁存), 一变就回传\n"
            "     [0x5A][36位门控掩码 5字节][间隔 2字节]; 同时【不回显】。\n"
            "     ★ 间隔那一格是把【当前这一拍】也算进去的 —— 少算一拍的话\n"
            "       每个事件都会短 10ms, 几百个事件累积就是好几秒的漂移。\n"
            "     ★ 回传的是「门控」而不是「包络输出」: 包络输出带着 1.5 秒释音尾巴,\n"
            "       尾巴没走完就重击同一个键时掩码连着不变, 两下会被并成一个长音。\n"
            "   · 释音尾巴由上位机按板子的常数补: 松手后 8ms 消抖 + 衰减\n"
            "     (默认音色约 1.46 秒, 笛子约 0.25 秒)。\n"
            "   · 效果开关也一起回传 [0x5D][8位]: 滑音/延音/音色/颤音/延迟/笛子/\n"
            "     混响/合唱。变了才发, 开始录音时强制发一次。合成时翻成 MIDI 的\n"
            "     program change + CC(混响 91 / 合唱 93 / 延迟 94 / 颤音 1) +\n"
            "     弯音(滑音) —— 接近板子, 但不是逐位一致。\n"
            "     ★ 音色挑的是【持续型 + 正弦感】的 82(Lead3 calliope) /\n"
            "       79(Ocarina) / 73(Flute): GM 钢琴自带衰减, 按住会越弹越轻,\n"
            "       而板子按住期间是恒定满幅(ENV_SUSTAIN_LV=255, 衰减那一段根本\n"
            "       不成立); 也不能用方波/锯齿(80/81), 那太尖锐 —— 板子的默认\n"
            "       音色是 sine_rom 出来的纯正弦。\n"
            "   · 「合成并播放」: 回传的事件 -> MIDI, 就在这台电脑上放出来\n"
            "     (没装 MIDI 输出口就交给系统默认播放器)。\n"
            "   · 「导出MIDI」: 只生成文件并在资源管理器里选中它,\n"
            "     落在程序目录的 MIDI 文件夹, 文件名是 录音_日期时间.mid。\n"
            "   · 「发回板子」: 合成后直接整帧发回板子让它自己弹一遍。\n"
            "   · 手弹和弦也能录 —— 掩码里有几位就是同时按下几个键。\n\n"
            "提示: 蓝牙 SPP 常见波特率 9600 / 115200;\n"
            "HEX 发送支持 01 02 AB 或 0102AB 两种写法。")

    def show_about(self):
        messagebox.showinfo(
            "关于",
            "{}\n版本 {}\n\n基于 Python + tkinter + pyserial\n"
            "配置文件: {}\n日志目录: {}".format(
                APP_NAME, APP_VERSION, os.path.join(app_dir(), CONFIG_NAME), app_dir()))

    # ------------------------------------------------------------------
    # 退出
    # ------------------------------------------------------------------
    def on_close(self):
        if self._closing:
            return
        self._closing = True
        self._midi_stop.set()          # 让正在发送的 MIDI 文件停下来
        try:
            save_config(self._collect_config())
        except Exception:
            pass
        try:
            self.close_port("程序退出", silent=True)
        except Exception:
            pass
        try:
            self.root.destroy()
        except Exception:
            pass


def _trace(msg):
    """启动心跳, 写到程序目录 spp_start.log。

    --windowed 打包后没有控制台, 万一启动就挂, 双击是「毫无反应」,
    什么线索都没有。有这份心跳至少能看出死在哪一步。
    """
    try:
        with open(os.path.join(app_dir(), "spp_start.log"), "a", encoding="utf-8") as fh:
            fh.write("[{}] {}\n".format(datetime.now().strftime("%H:%M:%S.%f")[:-3], msg))
    except Exception:
        pass


def _install_null_streams():
    """--windowed 打包后 sys.stdout / sys.stderr 是 None。

    任何往它们写东西的库(第三方库、异常打印)都会抛
    AttributeError: 'NoneType' object has no attribute 'write',
    表现就是双击毫无反应。这里补上空流兜底。
    """
    class _NullStream(object):
        def write(self, *args):
            return 0

        def writelines(self, *args):
            pass

        def flush(self):
            pass

        def isatty(self):
            return False

        def close(self):
            pass

    if sys.stdout is None:
        sys.stdout = _NullStream()
    if sys.stderr is None:
        sys.stderr = _NullStream()


def main():
    try:
        os.remove(os.path.join(app_dir(), "spp_start.log"))
    except OSError:
        pass
    _trace("main 进入")
    _install_null_streams()
    _trace("stdout/stderr 兜底完成")

    # 高 DPI 下让界面清晰
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception as exc:
        _trace("DPI 设置跳过: {!r}".format(exc))
    _trace("DPI 完成")

    try:
        root = tk.Tk()
    except Exception as exc:
        _trace("tk.Tk() 失败: {!r}".format(exc))
        raise
    _trace("tk.Tk() 成功")
    install_excepthook(root)

    if serial is None:
        _trace("pyserial 缺失, 中止")
        root.withdraw()
        messagebox.showerror(APP_NAME, "缺少 pyserial 依赖。\n请先执行: python -m pip install pyserial")
        root.destroy()
        return
    _trace("pyserial 就绪, 版本 {}".format(getattr(serial, "__version__", "?")))
    if mido is None:
        _trace("mido 缺失 (发送MIDI文件不可用)")
    else:
        try:
            import importlib.metadata as _md
            _mido_ver = _md.version("mido")
        except Exception:
            _mido_ver = "?"
        _trace("mido 就绪, 版本 {}".format(_mido_ver))
    _trace("MIDI 默认目录: {}".format(midi_folder()))

    icon = os.path.join(app_dir(), "spp.ico")
    if os.path.exists(icon):
        try:
            root.iconbitmap(default=icon)
        except Exception as exc:
            _trace("图标加载跳过: {!r}".format(exc))
    _trace("图标步骤完成")

    SppAssistant(root)
    _trace("界面构建完成, 进入 mainloop")
    root.mainloop()
    _trace("mainloop 退出")


if __name__ == "__main__":
    main()
