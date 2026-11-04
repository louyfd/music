#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_sine_mi.py
生成 Gowin BSRAM 初始化文件 sine_table.mi
    - 深度 256
    - 位宽 16 (有符号)
    - 格式：AddrHex，地址 16 进制，数据 16 进制
用法:
    python gen_sine_mi.py            # 输出 sine_table.mi
    python gen_sine_mi.py out.mi     # 指定输出文件名
"""

import math
import sys

DEPTH = 256                          # 地址深度
WIDTH = 16                           # 数据位宽（bit）
FULL_SCALE = (1 << (WIDTH - 1)) - 1  # 32767
OUT_FILE = sys.argv[1] if len(sys.argv) > 1 else "sine_table.mi"


def gen_sine_table(depth=DEPTH, full_scale=FULL_SCALE):
    """生成一个周期的正弦表，16 位有符号，转成补码无符号表示"""
    table = []
    for i in range(depth):
        v = int(round(full_scale * math.sin(2.0 * math.pi * i / depth)))
        if v > 32767:
            v = 32767
        elif v < -32768:
            v = -32768
        if v < 0:
            v = (1 << WIDTH) + v   # 负数转补码
        table.append(v & 0xFFFF)
    return table


def write_mi(path, table, depth=DEPTH, width=WIDTH):
    addr_digits = 1                          # AddrHex 地址通常不补零，写 1 位起
    data_digits = (width + 3) // 4           # 16 bit -> 4 位 hex

    with open(path, "w", encoding="ascii", newline="\n") as f:
        f.write("#File_format=AddrHex\n")
        f.write(f"#Address_depth={depth}\n")
        f.write(f"#Data_width={width}\n")
        for i, v in enumerate(table):
            f.write(f"{i:x}:{v:0{data_digits}X}\n")


def main():
    table = gen_sine_table()
    write_mi(OUT_FILE, table)
    print(f"已生成 {OUT_FILE}: depth={DEPTH}, width={WIDTH}")

    print("前 8 项：")
    for i in range(8):
        v = table[i]
        signed = v if v < 32768 else v - 65536
        print(f"  [{i:3d}] {v:04X}  ({signed})")


if __name__ == "__main__":
    main()
