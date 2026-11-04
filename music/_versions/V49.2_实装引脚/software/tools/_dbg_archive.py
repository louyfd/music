# -*- coding: utf-8 -*-
"""_dbg_archive.py —— 调试 PyInstaller 归档结构（一次性工具）"""
import struct
import sys

MAGIC = b"MEI\014\013\012\013\016"


def main():
    p = sys.argv[1]
    d = open(p, "rb").read()
    off = d.rfind(MAGIC)
    magic, pkglen, tocoff, toclen, pyver = struct.unpack("!8sIIII", d[off:off + 24])
    print("file      = %d" % len(d))
    print("cookie@   = %d (file_end-24 = %d, 差 %d)" % (off, len(d) - 24, len(d) - 24 - off))
    print("pkglen    = %d" % pkglen)
    print("tocoff    = %d  toclen = %d  pyver = %d" % (tocoff, toclen, pyver))
    start = off + 24 - pkglen
    print("pkg_start = cookie_end - pkglen = %d" % start)
    print("toc abs   = %d .. %d" % (start + tocoff, start + tocoff + toclen))
    print()
    print("---- TOC 前 96 字节 ----")
    blob = d[start + tocoff:start + tocoff + toclen]
    for i in range(0, min(96, len(blob)), 16):
        hx = " ".join("%02X" % b for b in blob[i:i + 16])
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in blob[i:i + 16])
        print("  %04X  %-47s  %s" % (i, hx, asc))
    print()
    print("---- 逐个条目（按 structlen 步进）----")
    pos = 0
    n = 0
    while pos + 4 <= len(blob) and n < 8:
        (slen,) = struct.unpack("!i", blob[pos:pos + 4])
        if slen == 0:
            print("  [%d] 零长条目 -> TOC 结束" % n)
            break
        if slen < 0 or pos + 4 + slen > len(blob):
            print("  [%d] structlen=%d 不合理 -> 停" % (n, slen))
            break
        e = blob[pos + 4:pos + 4 + slen]
        keep = d[:0]  # noop
        ints = struct.unpack("!IIII", e[:16]) if len(e) >= 16 else None
        print("  [%d] structlen=%d  ints=%s  tail=%r"
              % (n, slen, ints, e[16:]))
        pos += 4 + slen
        n += 1


main()
