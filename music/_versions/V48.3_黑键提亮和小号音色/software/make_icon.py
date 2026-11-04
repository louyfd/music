# -*- coding: utf-8 -*-
"""生成 spp.ico —— 蓝色圆角底 + 白色蓝牙符文, 纯标准库手写 ICO (多尺寸)。

运行: python make_icon.py
输出: spp.ico (16/32/48/64/128/256)
"""

import os
import struct

SIZES = [16, 32, 48, 64, 128, 256]
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "spp.ico")

# 背景渐变: 上浅蓝 -> 下深蓝
TOP = (0x45, 0x9B, 0xF5)
BOTTOM = (0x14, 0x5F, 0xC8)

# 蓝牙符文六段线 (归一化坐标)
GLYPH = [
    ((0.50, 0.10), (0.76, 0.32)),
    ((0.76, 0.32), (0.26, 0.68)),
    ((0.26, 0.68), (0.50, 0.90)),
    ((0.50, 0.90), (0.76, 0.68)),
    ((0.76, 0.68), (0.26, 0.32)),
    ((0.26, 0.32), (0.50, 0.10)),
]
STROKE = 0.085  # 线宽 (归一化)


def in_round_rect(px, py, margin=0.015, radius=0.22):
    x0 = y0 = margin
    x1 = y1 = 1.0 - margin
    r = radius
    cx = min(max(px, x0 + r), x1 - r)
    cy = min(max(py, y0 + r), y1 - r)
    dx = px - cx
    dy = py - cy
    return dx * dx + dy * dy <= r * r


def seg_distance(px, py, a, b):
    ax, ay = a
    bx, by = b
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    seg_len2 = vx * vx + vy * vy
    t = 0.0 if seg_len2 == 0 else (wx * vx + wy * vy) / seg_len2
    t = min(1.0, max(0.0, t))
    dx = wx - t * vx
    dy = wy - t * vy
    return (dx * dx + dy * dy) ** 0.5


def in_glyph(px, py):
    half = STROKE / 2.0
    for a, b in GLYPH:
        if seg_distance(px, py, a, b) <= half:
            return True
    return False


def render(size):
    """返回 size x size 的 BGRA 行数据 (自上而下), 4x 超采样抗锯齿。"""
    ss = 4 if size <= 64 else (2 if size <= 128 else 1)
    total = float(ss * ss)
    rows = []
    for y in range(size):
        row = bytearray()
        for x in range(size):
            bg = 0
            gl = 0
            for sy in range(ss):
                py = (y + (sy + 0.5) / ss) / size
                for sx in range(ss):
                    px = (x + (sx + 0.5) / ss) / size
                    if in_round_rect(px, py):
                        bg += 1
                        if in_glyph(px, py):
                            gl += 1
            if bg == 0:
                row += b"\x00\x00\x00\x00"
                continue
            bg_cov = bg / total
            gl_cov = gl / total
            blue_cov = bg_cov - gl_cov
            k = 0.5 if size == 1 else (y / float(size - 1))
            base = tuple(TOP[i] + (BOTTOM[i] - TOP[i]) * k for i in range(3))
            # 蓝底与白符按覆盖率加权, 避免边缘出现黑边
            r = (base[0] * blue_cov + 255.0 * gl_cov) / bg_cov
            g = (base[1] * blue_cov + 255.0 * gl_cov) / bg_cov
            b = (base[2] * blue_cov + 255.0 * gl_cov) / bg_cov
            row += bytes((int(round(b)), int(round(g)), int(round(r)),
                          int(round(bg_cov * 255))))
        rows.append(bytes(row))
    return rows


def bmp_entry(size):
    """构造 ICO 内的 BMP 图像块 (BITMAPINFOHEADER + BGRA + AND mask)。"""
    rows = render(size)
    xor = bytearray()
    for row in reversed(rows):  # BMP 自下而上
        xor += row
    mask_row_bytes = ((size + 31) // 32) * 4
    and_mask = b"\x00" * (mask_row_bytes * size)
    header = struct.pack("<IiiHHIIiiII", 40, size, size * 2, 1, 32, 0,
                         len(xor), 0, 0, 0, 0)
    return header + bytes(xor) + and_mask


def build():
    images = [bmp_entry(s) for s in SIZES]
    count = len(images)
    offset = 6 + 16 * count
    out = bytearray(struct.pack("<HHH", 0, 1, count))
    for size, blob in zip(SIZES, images):
        b = 0 if size >= 256 else size
        out += struct.pack("<BBBBHHII", b, b, 0, 0, 1, 32, len(blob), offset)
        offset += len(blob)
    for blob in images:
        out += blob
    with open(OUT, "wb") as fh:
        fh.write(out)
    return len(out)


if __name__ == "__main__":
    n = build()
    print("wrote {} ({} bytes, {} sizes)".format(OUT, n, len(SIZES)))
