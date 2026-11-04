# -*- coding: utf-8 -*-
"""
============================================================
 弹琴.py  ——  IDLE 里按 F5 就跑，不用命令行
============================================================

用法（就三步）：

  1) 用 IDLE 打开这个文件
  2) 改下面【要改的两行】：COM 口、曲子
  3) 按 F5

曲子两种写法，任选其一：

  A. 直接敲简谱（推荐，写小段落最方便）
         SONG = "1 1 5 5 6 6 5 | 4 4 3 3 2 2 1"

  B. 从文件读（曲子长、或者想存着反复用）
         SONG_FILE = r"D:\我的曲子\小星星.txt"

简谱写法：
    1 2 3 4 5 6 7   中音 do ~ si（do = 147Hz，和 播放.bat 那版小星星【同一个音高】）
    1. 2. 3.        高音（数字后跟一个点）—— 整段升八度就每个数字都加点
    1, 2, 3,        低音（数字后跟一个逗号）
    0               休止符
    1_  1__         延长 1 拍 / 2 拍（一个数字 = 1 拍 = 0.5 秒）
    |               小节线：会在那里插一个换气休止符（不想停顿就别写 |）

    ★ 音高基准和 播放.bat / twinkle.py 完全一致，两个入口发出去的字节一模一样。
        中音 1~7 = 板子键号 5~11      高音 1.~7. = 17~23
      想要标准音高（中音 1 = 中央 C）就在谱子开头加一行 #base=15。
      整首升/降八度也是这行：升一个八度 #base=17，降一个八度就用默认。

找不到 COM 口时：把下面 DO_SCAN 改成 True 再按 F5，会列出所有串口并逐个
试着发一包，哪个收到板子回传就是哪个口。
"""

import os
import sys

# ============================================================
# ★★★ 每次只改这两行 ★★★
# ============================================================
COM_PORT = "COM12"          # 你的蓝牙串口，例如 "COM19"；填 "" = 只预览不发送
SONG     = ("1 1 5 5 6 6 5 | 4 4 3 3 2 2 1 | "
            "5 5 4 4 3 3 2 | 5 5 4 4 3 3 2 | "
            "1 1 5 5 6 6 5 | 4 4 3 3 2 2 1")

# 想从文件读就填路径（填了它优先，SONG 会被忽略）：
SONG_FILE = ""
# SONG_FILE = r"D:\我的曲子\小星星.txt"

# 找不到 COM 口时改成 True，按 F5 会扫描所有串口并试发
DO_SCAN = False
# ============================================================

HERE = os.path.dirname(os.path.abspath(__file__))
MUSIC = os.path.dirname(HERE)            # ...\music


def load(name):
    """按文件路径直接加载 tools\\ 或同目录下的脚本"""
    import importlib.util
    for d in (HERE, os.path.join(MUSIC, "tools")):
        p = os.path.join(d, name)
        if os.path.isfile(p):
            spec = importlib.util.spec_from_file_location("_x_" + name[:-3], p)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    raise FileNotFoundError(f"找不到 {name}（应该在 {HERE} 或 {MUSIC}\\tools）")


def main():
    print("=" * 60)
    print(" 弹琴 —— 简谱发给 FPGA")
    print("=" * 60)

    jp = load("jianpu.py")

    if DO_SCAN:
        scan = load("scan_ports.py")
        ports = sorted(p.device for p in scan.list_ports.comports())
        print("本机串口：", ", ".join(ports) if ports else "（一个都没有）")
        print("逐个试发一包，看哪个收到板子回传...")
        for port in ports:
            data, note = scan.try_port(port, 9600)
            tag = "  <== 就是它！" if data else ""
            print(f"  {port:<8} {len(data):>3} 字节  {note}{tag}")
        print("\n把上面带 <== 的那个填进 COM_PORT，再把 DO_SCAN 改回 False。")
        return 0

    # ---- 取简谱 ----
    if SONG_FILE:
        if not os.path.isfile(SONG_FILE):
            print(f"[ERR] 找不到简谱文件: {SONG_FILE}")
            return 1
        with open(SONG_FILE, encoding="utf-8") as f:
            text = f.read()
        print(f"读入 {SONG_FILE}")
    else:
        text = SONG
        print("简谱：")
    for ln in text.strip().splitlines():
        print("   " + ln.strip())

    # ---- 解析 ----
    seq = jp.parse(text)
    if not seq:
        print("[ERR] 没解析出音符。例子：1 1 5 5 6 6 5")
        return 1
    jp.preview(seq)
    body = jp.to_bytes(seq)
    print("\n板子字节：" + " ".join("%02X" % b for b in body))

    # ---- 发送 ----
    if not COM_PORT:
        print("\n[OK] COM_PORT 是空的，只预览没发送。")
        return 0

    print(f"\n发往 {COM_PORT} ...")
    try:
        jp.send(COM_PORT, body)
    except Exception as e:
        print(f"\n[ERR] 发送失败: {e}")
        print("     1) 口对不对？把 DO_SCAN 改成 True 按 F5 扫一下")
        print("     2) 蓝牙在 Windows 里显示【已连接】了吗？")
        print("     3) 串口助手 / 上次没关的口是不是还占着？")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
