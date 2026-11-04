# -*- coding: utf-8 -*-
"""_scan_stale_sim_refs.py —— 扫"检查脚本引用了 RTL 里已经不存在的名字"

为什么需要它（真事）
--------------------
testfiles/ 下十几个自检脚本都是【正则从 src/*.v 里抠常数】再断言的。这种写法的
死穴是：RTL 一改名（哪怕只是 wire 改名），脚本抠到 None，然后
  · 要么崩在某个 %d 上（看起来像环境问题），
  · 要么 check 判失败但被当噪音，
  · 最坏的情况是拿到 None 后走了某条"跳过"分支，一路全绿 —— 等于这个检查白挂。
liss_geom_check.py 就是活例子：V35.1 把 x140 改成 x120 之后，它的正则还在找
x140，于是连着好几版都是"崩"而不是"红"。

所以这个脚本只做一件事：把每个 testfiles/*.py 里【正则字面量】中出现的标识符
抓出来，逐个问一句"src/*.v 里还有这个名字吗"。没有 → 报出来。
它不判断对错，只负责在改名的那一刻把人叫醒。

用法：python testfiles/_scan_stale_sim_refs.py
      （退出码 0 = 没发现陈旧引用；1 = 有）
"""

from __future__ import annotations

import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "src")

# 正则字面量里的这些词不是 RTL 标识符，是正则语法 / Python 侧名字 / 通用词
SKIP = {
    "re", "str", "int", "float", "None", "True", "False", "utf", "ascii",
    "w", "d", "s", "b", "n", "r", "t", "v", "z", "A", "Z", "S", "D", "W",
    "name", "src", "path", "text", "line", "lines", "expr", "pat", "pattern",
    "group", "groups", "match", "search", "findall", "sub", "compile",
}


def rtl_identifiers() -> tuple[set[str], str]:
    """src 下所有 .v 里出现过的标识符（含注释，宁可宽松也不误报）。"""
    idents: set[str] = set()
    blobs = []
    for fn in sorted(os.listdir(SRC)):
        if not fn.endswith(".v"):
            continue
        with open(os.path.join(SRC, fn), "r", encoding="utf-8",
                  errors="replace") as fh:
            txt = fh.read()
        blobs.append(txt)
        idents |= set(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", txt))
    return idents, "\n".join(blobs)


def regex_literals(path: str) -> list[tuple[int, str]]:
    """抠出 re.search/findall/match/fullmatch/sub 的第一个参数字面量。"""
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        src = fh.read()
    out: list[tuple[int, str]] = []
    for m in re.finditer(r"re\.(?:search|findall|match|fullmatch|sub|split)\s*\(\s*"
                         r"(?:r|f|rf|fr)?(['\"])(.*?)\1", src, re.S):
        line = src[:m.start()].count("\n") + 1
        out.append((line, m.group(2)))
    # 「用 % 拼出来的」正则也算（很多老脚本是 r"..." % name 这种写法）
    for m in re.finditer(r"re\.(?:search|findall|match|fullmatch)\s*\(\s*"
                         r"(?:r|f|rf|fr)?(['\"])(.*?)\1\s*%\s*", src, re.S):
        line = src[:m.start()].count("\n") + 1
        out.append((line, m.group(2)))
    return out


def python_side_names(path: str) -> set[str]:
    """脚本自己用到的名字（除了正则字面量以外的部分），用来排除误报。"""
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        src = fh.read()
    stripped = re.sub(r"re\.(?:search|findall|match|fullmatch|sub|split)\s*\(\s*"
                      r"(?:r|f|rf|fr)?(['\"])(.*?)\1", "", src, flags=re.S)
    return set(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", stripped))


def tokens_of(pattern: str) -> list[str]:
    """从正则里挑出"像 Verilog 标识符"的候选。"""
    # 先把正则元字符换成空格，剩下的连续字母数字串就是候选
    flat = re.sub(r"\\[a-zA-Z]|\\[0-9]+|\[[^\]]*\]|\([^)]*\)", " ", pattern)
    flat = re.sub(r"[^A-Za-z0-9_]+", " ", flat)
    out = []
    for tok in flat.split():
        if tok in SKIP or len(tok) < 4:
            continue
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", tok):
            continue
        out.append(tok)
    return out


def main() -> int:
    if not os.path.isdir(SRC):
        print("找不到 %s" % SRC)
        return 1
    idents, _ = rtl_identifiers()
    files = sorted(f for f in os.listdir(HERE) if f.endswith(".py"))
    stale: list[str] = []
    checked = 0
    print("=" * 74)
    print("陈旧引用扫描：testfiles/*.py 的正则 里引用的 RTL 名字还在不在")
    print("RTL: %s   （现有标识符 %d 个）" % (SRC, len(idents)))
    print("-" * 74)
    for fn in files:
        if fn == os.path.basename(__file__):
            continue
        path = os.path.join(HERE, fn)
        lits = regex_literals(path)
        py_names = python_side_names(path)
        hits: list[tuple[int, str]] = []
        for line, pat in lits:
            for tok in tokens_of(pat):
                checked += 1
                if tok in idents or tok in py_names:
                    continue
                hits.append((line, tok))
        if hits:
            print("  ★ %s" % fn)
            seen = set()
            for line, tok in hits:
                if (line, tok) in seen:
                    continue
                seen.add((line, tok))
                print("      第 %-4d 行 正则里引用了 RTL 里不存在的 %-24s" % (line, tok))
                stale.append("%s:%d:%s" % (fn, line, tok))
        else:
            print("  ok %s（正则里 %d 个候选名字全部还在 RTL 里）" % (fn, len(lits)))
    print("-" * 74)
    print("共检查 %d 个候选名字；发现 %d 处可能陈旧的引用。" % (checked, len(stale)))
    if stale:
        print()
        print("★ 处理办法：不是删掉脚本里的名字，而是【先确认 RTL 是不是改名了】——")
        print("  改名了就把脚本跟着改（顺便把新名字也纳入互证），")
        print("  没改名就该看这个脚本为什么抠不到。改完重跑本扫描 + 那个脚本本身。")
        print()
        print("=" * 74)
        print("发现 %d 处陈旧引用" % len(stale))
        return 1
    print()
    print("=" * 74)
    print("ALL CHECKS PASSED  （没有发现指向已消失 RTL 名字的引用）")
    return 0


if __name__ == "__main__":
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8",
                                      errors="replace", line_buffering=True)
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8",
                                      errors="replace", line_buffering=True)
    except Exception:                                       # noqa: BLE001
        pass
    sys.exit(main())
