# -*- coding: utf-8 -*-
"""_check_exe_content.py —— 证明"新功能真的打进 exe 了"

为什么要它
----------
PyInstaller 打出来的 exe 是**压缩归档**，直接在 exe 里搜字符串是搜不到的
（`grep`/字节搜都会给出 0 次，看起来像"没打进去"）—— 但那只是压缩造成的假象。
这个脚本按 PyInstaller 自己的归档格式**解出里面的 .pyc 再搜**，
这样"打进去了没有"才有真凭实据，不再靠"打包脚本说成功"。

它做三件事：
  1) 读 exe 末尾的 CArchive cookie，列出归档目录（模块名/类型/大小）；
  2) 解出 PYZ（类型 'z'）并列出它里面的 Python 模块；
  3) 把你给的标记串在指定模块里搜一遍，打印命中次数。

用法：
  python tools/_check_exe_content.py <exe> <模块名> <串1> [<串2> ...]
例：
  python tools/_check_exe_content.py SPP_Assistant\\SPP_Assistant.exe spp_assistant 重置LCD on_lcd_reset
"""

from __future__ import annotations

import io
import os
import struct
import sys
import zlib

MAGIC = b"MEI\014\013\012\013\016"


def read_carchive(path: str):
    """返回 (data, toc)。toc = [(name, typecode, datalen, abspos)]

    ★ cookie 结构：magic(8) + pkglen(4) + toc_off(4) + toc_len(4) + pyver(4)
    ★★ 两个实测踩出来的坑（别凭印象改）：
      ① **包起点 = 文件长度 - pkglen**，`toc_off` 也是相对这个起点算的；
      ② **cookie 不在文件最后** —— 实测它后面还跟着 64 个 0 字节的填充。
         所以既不能"取最后 24 字节当 cookie"，也不能用
         `rfind(MAGIC) + 24 - pkglen` 去推包起点（那会整体偏 64 字节，
         读出来的目录是一堆乱码类型码）。
         正确做法：cookie 的位置 = **包起点 + toc_off + toc_len**。
    """
    with open(path, "rb") as fh:
        data = fh.read()
    # 先按"包起点 = 文件长度 - pkglen"试；pkglen 藏在 cookie 里，
    # 而 cookie 就在 TOC 末尾 —— 所以先用最后一个 MAGIC 取 pkglen，
    # 再用它算包起点，最后用 toc_off 反推 cookie 位置并【自检】。
    off = data.rfind(MAGIC)
    if off < 0:
        raise SystemExit("找不到 PyInstaller cookie —— 这不是 PyInstaller 打的包？")
    _, pkg_len, toc_off, toc_len, pyver = struct.unpack(
        "!8sIIII", data[off:off + 24])
    pkg_start = len(data) - pkg_len
    cookie_pos = pkg_start + toc_off + toc_len
    if data[cookie_pos:cookie_pos + 8] != MAGIC:
        # 万一 pkglen 为 0（onedir 的某些版本），退回用 cookie 位置反推
        pkg_start = off + 24 - pkg_len
        cookie_pos = pkg_start + toc_off + toc_len
    toc = []
    pos = pkg_start + toc_off
    end = pos + toc_len
    while pos < end:
        (slen,) = struct.unpack("!i", data[pos:pos + 4])
        if slen <= 0:
            break
        # ★★ 实测布局（PyInstaller 6 / pyver=309），别再凭印象改：
        #   · 记录 = [slen:4][header:14][name:slen-18]，**步进 = slen**（slen 含自身那 4 字节）
        #   · header = entrylen(4) + entrypos(4) + datalen(4) + cflag(1) + typecode(1)
        #     —— **没有 ulen 字段**（我按老结构写过 4 个 int，名字就会多/少几个字节）
        #   · entrypos 是相对【包起点】的
        blk = data[pos + 4:pos + slen]
        entrylen, entrypos, datalen, cflag, typecode = \
            struct.unpack("!IIIBc", blk[:14])
        name = blk[14:].rstrip(b"\x00").decode("utf-8", "replace")
        toc.append((name, typecode.decode("latin1"), datalen, pkg_start + entrypos))
        pos += slen
    return data, toc


def load_entry(data: bytes, pos: int, datalen: int) -> bytes:
    raw = data[pos:pos + datalen]
    try:
        return zlib.decompress(raw)
    except zlib.error:
        return raw


def read_pyz(blob: bytes):
    """PYZ（ZlibArchive）：MAGIC 'PYZ\\0' + pyc magic(4) + toc_off(4) ...

    条目布局在不同 PyInstaller 版本里差一个字段，所以**两种都试**，
    用"位置在界内 + TOC 以零长条目收尾"判定哪种对 —— 不猜。
    """
    if not blob.startswith(b"PYZ\x00"):
        return []
    toc_off = struct.unpack("!I", blob[8:12])[0]

    def parse(fmt: str):
        out = []
        pos = toc_off
        while pos + 4 <= len(blob):
            (mlen,) = struct.unpack("!i", blob[pos:pos + 4])
            if mlen == 0:
                return out                      # TOC 以零长条目收尾
            if mlen < 0 or mlen > 4096 or pos + 4 + mlen > len(blob):
                return None
            name = blob[pos + 4:pos + 4 + mlen].decode("utf-8", "replace")
            rest = pos + 4 + mlen
            try:
                if fmt == "BII":                # typcd(1) + pos(4) + len(4)
                    _, dpos, dlen = struct.unpack("!BII", blob[rest:rest + 9])
                    step = 9
                else:                           # typcd(1) + pad(3) + pos(4) + len(4)
                    _, _, dpos, dlen = struct.unpack("!BIII", blob[rest:rest + 13])
                    step = 13
            except struct.error:
                return None
            if not (0 <= dpos and 0 <= dlen and dpos + dlen <= len(blob)):
                return None
            out.append((name, dlen, dpos))
            pos = rest + step
        return None

    for fmt in ("BII", "BIII"):
        r = parse(fmt)
        if r:
            return r
    return []


def main() -> int:
    if len(sys.argv) < 4:
        print(__doc__)
        return 1
    exe, mod, marks = sys.argv[1], sys.argv[2], sys.argv[3:]
    if not os.path.isfile(exe):
        print("找不到 %s" % exe)
        return 1
    data, toc = read_carchive(exe)
    print("=" * 72)
    print("exe 内容检查: %s" % exe)
    print("  归档条目 %d 个；类型统计：" % len(toc))
    kinds = {}
    for _, t, _, _ in toc:
        kinds[t] = kinds.get(t, 0) + 1
    print("   ", " ".join("%s=%d" % (k, v) for k, v in sorted(kinds.items())))
    pyz = [e for e in toc if e[1] == "z"]
    print("  PYZ 归档: %s" % ([e[0] for e in pyz] or "（没有，可能是 onedir）"))
    if not pyz:
        print("★ 拿不到 PYZ —— 检查不了模块内容（如实说：这次证明不了）")
        return 1
    name, _, dlen, pos = pyz[0]
    blob = load_entry(data, pos, dlen)
    mods = read_pyz(blob)
    names = [m[0] for m in mods]
    print("  PYZ 里 %d 个模块；要找的 %r 在不在: %s" % (len(mods), mod, mod in names))
    if mod not in names:
        print("★ 模块 %r 不在 PYZ 里（可能名字不对）" % mod)
        return 1
    # 解出该模块的字节码，搜标记
    for n, dl, mp in mods:
        if n != mod:
            continue
        code = load_entry(blob, mp, dl)
        print("  模块 %s 解出 %d 字节（.pyc）" % (n, len(code)))
        ok = True
        for mk in marks:
            hit = code.count(mk.encode("utf-8"))
            print("    %-22s 命中 %d 次 %s" % (mk, hit, "OK" if hit else "★ 没有"))
            ok = ok and hit > 0
    # 也把 exe 里的原始字节搜一遍作对照（说明"直接搜搜不到"这件事）
    raw_hits = sum(data.count(mk.encode("utf-8")) for mk in marks)
    print("  【对照】直接在整个 exe 的字节里搜这些串: %d 次 %s"
          % (raw_hits, "（压缩归档，搜不到是正常的）" if raw_hits == 0 else ""))
    print("=" * 72)
    if ok:
        print("ALL CHECKS PASSED（标记串都能在解出来的模块里找到）")
        return 0
    print("★ 有标记串没找到 —— 别按「打好了」算")
    return 1


if __name__ == "__main__":
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8",
                                      errors="replace", line_buffering=True)
    except Exception:                                       # noqa: BLE001
        pass
    sys.exit(main())
