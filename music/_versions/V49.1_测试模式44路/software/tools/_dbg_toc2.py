# -*- coding: utf-8 -*-
"""_dbg_toc2.py —— 试两种步进，看哪种能解出正常的 TOC（一次性调试）"""
import struct
import sys

d = open(sys.argv[1], "rb").read()
MAGIC = b"MEI\014\013\012\013\016"
off = d.rfind(MAGIC)
_, pkglen, tocoff, toclen, pyver = struct.unpack("!8sIIII", d[off:off + 24])
pkg_start = len(d) - pkglen
pos0 = pkg_start + tocoff
print("pkg_start=%d toc@%d len=%d" % (pkg_start, pos0, toclen))
print("TOC 首 32 字节: " + d[pos0:pos0 + 32].hex(" "))
print()


def walk(step_includes_len: bool, limit=6):
    out = []
    pos = pos0
    end = pos0 + toclen
    while pos + 4 <= end and len(out) < limit:
        (slen,) = struct.unpack("!i", d[pos:pos + 4])
        if slen <= 0 or pos + 4 + slen > end + 8:
            out.append(("<<stop slen=%d>>" % slen, "", 0, pos))
            break
        blk = d[pos + 4:pos + 4 + slen]
        try:
            el, ep, dl, ul, cf, tc = struct.unpack("!IIIIBc", blk[:18])
        except struct.error:
            out.append(("<<short>>", "", 0, pos))
            break
        name = blk[18:].rstrip(b"\x00").decode("utf-8", "replace")
        out.append((name, tc.decode("latin1"), dl, pos))
        pos += slen if step_includes_len else (4 + slen)
    return out


for flag in (False, True):
    print("---- 步进 = %s ----" % ("structlen（含自身）" if flag else "4 + structlen"))
    for name, tc, dl, p in walk(flag):
        print("   pos=%-9d type=%-3r datalen=%-8d name=%r" % (p, tc, dl, name))
    print()
