# -*- coding: utf-8 -*-
"""临时扫描：src/*.v 里有没有【定宽字面量装不下】的地方（会被工具悄悄截断）。

对应 Gowin 的 WARN (EX3792) : Literal value 'dXXXX truncated to fit in NN bits。
这类错误的恶劣之处：综合不报错、行为却和注释/意图不一样，而且很难查。
（top.v 里就留着一条血泪注释：`6'd64` 被截成 0 → FRAME_DIV=0 → 除零 →
  DDS/ROM/包络/混音全被优化掉，烧进去完全没声音。）

★ 必须先【剥掉注释】再扫：top.v 那条 `6'd64` 正是写在注释里当反面教材的。
"""
import os
import re
import sys

COMMENT = re.compile(r"//[^\n]*|/\*.*?\*/", re.S)
pat = re.compile(r"(\d+)'([dD])([\d_]+)")


def strip_comments(src):
    # 用等长空白替换，行号才不会错位
    return COMMENT.sub(lambda m: re.sub(r"[^\n]", " ", m.group(0)), src)


def main():
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
    bad = []
    for fn in sorted(os.listdir(root)):
        if not fn.endswith(".v"):
            continue
        src = open(os.path.join(root, fn), encoding="utf-8", errors="replace").read()
        src = strip_comments(src)
        for m in pat.finditer(src):
            w = int(m.group(1))
            v = int(m.group(3).replace("_", ""))
            if w > 0 and v >= (1 << w):
                line = src[:m.start()].count("\n") + 1
                bad.append((fn, line, m.group(0), v, w))
    if not bad:
        print("没有发现被截断的定宽字面量（已剥掉注释）")
    for fn, line, lit, v, w in bad:
        print("%-18s :%-5d %-16s 值=%d，%d 位最多装到 %d  -> 实际变成 %d"
              % (fn, line, lit, v, w, (1 << w) - 1, v % (1 << w)))
    return 1 if bad else 0


if __name__ == "__main__":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:                                   # noqa: BLE001
        pass
    raise SystemExit(main())
