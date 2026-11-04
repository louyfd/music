# -*- coding: utf-8 -*-
"""
扫描本机所有串口，自动找出能收到 FPGA 回传的那个口。

用法：在 IDLE 里打开本文件按 F5，或命令行 python scan_ports.py
原理：FPGA 收到 0xFF 结束符后会把整包原样回传，所以"有回传"的口就是对的。
"""
import time

import serial
import serial.tools.list_ports as list_ports

BAUDS = (9600, 115200)      # FPGA 是 9600，115200 只是陪跑对照
TRIES = 2                   # 蓝牙口第一次打开经常失败，重试一次
PKT = bytes([12, 12, 19, 19, 21, 21, 19, 17, 17, 16, 16, 14, 14, 12]) + b'\xff'


def try_port(port, baud):
    """发一包，返回 (收到的字节, 说明文字)"""
    note = ""
    for _ in range(TRIES):
        try:
            ser = serial.Serial(port, baud, timeout=1.5, write_timeout=1.5)
        except Exception as e:
            note = "打不开：%s" % e
            time.sleep(0.4)
            continue
        try:
            time.sleep(0.4)
            ser.reset_input_buffer()
            try:
                ser.write(PKT)
                ser.flush()
            except Exception as e:
                return b"", "写失败：%s" % type(e).__name__
            return ser.read(64), "已发送"
        finally:
            ser.close()
    return b"", note


def main():
    ports = sorted(p.device for p in list_ports.comports())
    print("本机串口：", ", ".join(ports) if ports else "（一个都没有）")
    print("-" * 66)

    found = []
    for port in ports:
        for baud in BAUDS:
            data, note = try_port(port, baud)
            if data:
                print("%-6s %-6d 收到回传 %2d 字节  <== 就是它！" % (port, baud, len(data)))
                found.append((port, baud))
                break
            print("%-6s %-6d %s" % (port, baud, note))

    print("-" * 66)
    if found:
        for port, baud in found:
            print("==> 把 1.py 改成：COM_PORT = %r    BAUD = %d" % (port, baud))
    else:
        print("==> 没有任何串口收到回传，按顺序检查：")
        print("    1) 先把手机的蓝牙关掉 —— 这种模块同一时间只能连一个主机")
        print("    2) 确认蓝牙模块已上电、FPGA 已经下载好程序")
        print("    3) Windows 设置 -> 蓝牙和其他设备 -> 设备，")
        print("       点开 BT04-A / BT04-E02 / JDY-31-SPP，能点“连接”就点一下")
        print("    4) 再跑一次本脚本")


if __name__ == "__main__":
    main()
