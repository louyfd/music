# -*- coding: utf-8 -*-
"""_dbg_toc.py —— 在 exe 里定位 TOC 的真实布局（一次性调试）"""
import struct
import sys

d = open(sys.argv[1], "rb").read()
MAGIC = b"MEI\014\013\012\013\016"
off = d.rfind(MAGIC)
_, pkglen, tocoff, toclen, pyver = struct.unpack("!8sIIII", d[off:off + 24])
print("file=%d cookie@=%d pkglen=%d tocoff=%d toclen=%d" % (len(d), off, pkglen, tocoff, toclen))
print("file_len - pkglen = %d" % (len(d) - pkglen))
print("cookie@ - toclen   = %d  (= TOC 结束位置)" % (off - toclen))
print()

# 扫所有出现 'spp_assistant' 的地方
needle = b"spp_assistant"
pos = 0
hits = []
while True:
    i = d.find(needle, pos)
    if i < 0:
        break
    hits.append(i)
    pos = i + 1
print("'spp_assistant' 出现 %d 次，前 8 个位置: %s" % (len(hits), hits[:8]))
for i in hits[:3]:
    print()
    print("---- 位置 %d 前 24 字节 / 后 16 字节 ----" % i)
    print("  前: " + d[max(0, i - 24):i].hex(" "))
    print("  名: " + d[i:i + 20])
    print("  后: " + d[i + len(needle):i + len(needle) + 16].hex(" "))
