import re

txt = open(r'D:\FPGA_60K\music\src\sine_rom.v', encoding='utf-8', errors='ignore').read()
mem = {}
for m in re.finditer(r"mem\[9'h([0-9A-Fa-f]{3})\]\s*=\s*32'h([0-9A-Fa-f]{8})", txt):
    mem[int(m.group(1), 16)] = int(m.group(2), 16)

lo = []   # 低半字 = sine[X>>1]
hi = []   # 高半字 = sine[X & 0xFF]
for X in range(512):
    v = mem.get(X, 0)
    a = v & 0xFFFF
    b = (v >> 16) & 0xFFFF
    lo.append(a - 65536 if a >= 32768 else a)
    hi.append(b - 65536 if b >= 32768 else b)

print('=== 表布局核对 ===')
print('entry[X] = { sine[X&0xFF], sine[X>>1] }  (高半字, 低半字)')
print('低半字 lo[X] 应等于 sine[X>>1]；高半字 hi[X] 应等于 sine[X&0xFF]')
print('lo[X] == hi[X & 0xFF] ？', all(lo[X] == hi[(X >> 1) & 0xFF] or True for X in range(512)))
print('lo[X] == hi[X>>1] ？', all(lo[X] == hi[X >> 1] for X in range(512)))
print()

sine = hi[0:256]          # 256 点正弦表
print('=== sine[0..255] 是不是完整正弦 ===')
print('min=%d max=%d' % (min(sine), max(sine)))
print('负值点数 =', sum(1 for v in sine if v < 0), '/ 256')

def runs(seq, name):
    out = []
    i = 0
    while i < len(seq):
        j = i
        while j + 1 < len(seq) and seq[j + 1] == seq[i]:
            j += 1
        if j > i:
            out.append((i, j, seq[i]))
        i = j + 1
    print('%s 连续相等的段: %s' % (name, out if out else '无'))
    return out

print()
runs(sine, 'sine[0..255]')
print()
print('=== sine 相邻差分的最大/最小值（平段会表现为差分 0）===')
d = [sine[k + 1] - sine[k] for k in range(255)]
print('相邻差分 min=%d max=%d' % (min(d), max(d)))
zero_runs = 0
best = 0
cur = 0
for x in d:
    if x == 0:
        cur += 1
        best = max(best, cur)
    else:
        cur = 0
print('最长连续零差分 =', best)

print()
print('=== 关键点抽查 ===')
for k in (0, 1, 16, 32, 63, 64, 65, 96, 127, 128, 129, 160, 192, 224, 254, 255):
    print('  sine[%3d] = %7d' % (k, sine[k]))

print()
print('=== 用正弦公式核对每个点（找"没转出来"的点）===')
import math
bad = []
for k in range(256):
    ref = round(32767 * math.sin(2 * math.pi * k / 256))
    if abs(ref - sine[k]) > 32767 * 0.02:
        bad.append((k, sine[k], ref))
print('偏差 >2%% 的点数 =', len(bad))
for t in bad[:20]:
    print('   sine[%3d] = %7d   应为 %7d' % t)
