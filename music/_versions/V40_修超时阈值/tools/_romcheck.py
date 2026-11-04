import re
txt = open(r'D:\FPGA_60K\music\src\sine_rom.v', encoding='utf-8', errors='ignore').read()
mem = {}
for m in re.finditer(r"mem\[9'h([0-9A-Fa-f]{3})\]\s*=\s*32'h([0-9A-Fa-f]{8})", txt):
    mem[int(m.group(1), 16)] = int(m.group(2), 16)
print('表项数 =', len(mem))

fund = []
harm = []
for i in range(512):
    v = mem.get(i, 0)
    lo = v & 0xFFFF
    hi = (v >> 16) & 0xFFFF
    fund.append(lo - 65536 if lo >= 32768 else lo)
    harm.append(hi - 65536 if hi >= 32768 else hi)

print()
print('基波(低半字) 每 32 个地址取一个:')
for i in range(0, 512, 32):
    print('  ad=%3d (等效点%3d)  fund=%7d   harm=%7d' % (i, i // 2, fund[i], harm[i]))

print()
print('基波 min=%d max=%d' % (min(fund), max(fund)))
# 基波每 2 个地址一个点
f2 = fund[0::2]
print('基波(256点) 前1/4:', f2[:4], '... 中:', f2[60:68], '... 峰:', f2[124:132])
print('基波(256点) 后1/4:', f2[252:])
up = all(f2[i] <= f2[i + 1] for i in range(0, 127))
dn = all(f2[i] >= f2[i + 1] for i in range(127, 255))
print('前 128 点单调升 =', up, '  后 128 点单调降 =', dn)
print('负值点数 =', sum(1 for v in f2 if v < 0))
