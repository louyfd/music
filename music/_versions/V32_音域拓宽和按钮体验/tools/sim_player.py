import sys
PLAY_DIV=40; HOLD_DIV=16; BUF_SIZE=256; BYTE_STREAM=0
REST_BYTE=0xEE; CLR_BYTE=0xAA; N_FLUSH=16
st=dict(midi_keys=0,play_run=0,note_pos=0,acc_len=0,play_cnt=0,note_pulse=0,
        tx_req=0,frame_start=0,frame_len=0,wr_ptr=0,note_hold=0,armed=0,live=0)
note_buf=[0]*BUF_SIZE

def cycle(rx_valid=0, rx_data=0):
    g=st
    last = 1 if (g['play_run'] and (g['acc_len']==0 or g['note_pos']+1>=g['acc_len'])) else 0
    cur_note=g['note_pos']
    is_rest = (not BYTE_STREAM) and (note_buf[cur_note]==REST_BYTE)
    led_set = 1 if ((not BYTE_STREAM) and g['play_run'] and g['play_cnt']==0 and g['acc_len']!=0) else 0
    cur_key = note_buf[cur_note] & 0x3F
    n=dict(g); n['frame_start']=0
    if rx_valid:
        if rx_data==CLR_BYTE:
            n['armed']=1; n['wr_ptr']=0; n['frame_len']=0
        elif rx_data==0xFF:
            if g['armed']:
                n['frame_len']=g['wr_ptr']; n['frame_start']=1
            else:
                n['frame_len']=0
            n['armed']=0; n['wr_ptr']=0
        elif rx_data<=0x23 or rx_data==REST_BYTE:
            if g['armed']:
                note_buf[g['wr_ptr']]=rx_data
                if g['wr_ptr']<BUF_SIZE-1: n['wr_ptr']=g['wr_ptr']+1
    n['tx_req']=1 if g['frame_start'] else 0
    n['note_pulse']=0
    if rx_valid and (rx_data<=0x23 or rx_data==REST_BYTE):
        n['tx_req']=1
        if BYTE_STREAM: n['note_pulse']=1
    if BYTE_STREAM: n['play_run']=0
    elif g['frame_start']:
        n['acc_len']=g['frame_len']; n['play_run']=1; n['note_pos']=0; n['play_cnt']=0
    elif g['play_run']:
        if g['play_cnt']<PLAY_DIV: n['play_cnt']=g['play_cnt']+1
        else:
            n['play_cnt']=0
            if last: n['play_run']=0
            else: n['note_pos']=g['note_pos']+1
    if BYTE_STREAM:
        if n['note_pulse']:
            n['midi_keys']=1<<cur_key; n['note_hold']=12_500_000
        elif g['note_hold']!=0:
            n['note_hold']=g['note_hold']-1
            if g['note_hold']==1: n['midi_keys']=0
    else:
        if led_set:
            if is_rest: n['midi_keys']=0; n['note_hold']=HOLD_DIV
            else: n['midi_keys']=1<<cur_key; n['note_hold']=0
        elif g['note_hold']<HOLD_DIV: n['note_hold']=g['note_hold']+1
        elif g['midi_keys']!=0: n['midi_keys']=0
        if (not n['play_run']) and (n['midi_keys']!=0):
            n['midi_keys']=0; n['note_hold']=HOLD_DIV
    st.update(n)

def feed(stream):
    global note_buf
    for k in st: st[k]=0
    note_buf=[0]*BUF_SIZE
    gap=5; slot=PLAY_DIV+1
    total=len(stream)*gap+slot*(len(stream)+4)+50
    frames=[]; prev=None; ri=0
    for c in range(total):
        if ri<len(stream) and c%gap==0:
            cycle(rx_valid=1, rx_data=stream[ri]); ri+=1
        else: cycle()
        key=(st['midi_keys'] & -st['midi_keys']).bit_length()-1 if st['midi_keys'] else None
        if key!=prev: frames.append((c,key)); prev=key
    return frames

SONG=[0x05,0x05,0x09,0x09,0x0A,0x0A,0x09,REST_BYTE,
      0x08,0x08,0x07,0x07,0x06,0x06,0x05,REST_BYTE,
      0x09,0x09,0x08,0x08,0x07,0x07,0x06,REST_BYTE,
      0x09,0x09,0x08,0x08,0x07,0x07,0x06,REST_BYTE,
      0x05,0x05,0x09,0x09,0x0A,0x0A,0x09,REST_BYTE,
      0x08,0x08,0x07,0x07,0x06,0x06,0x05]

no_ee = '--no-ee' in sys.argv
lead = [] if no_ee else [REST_BYTE, REST_BYTE]
stream=[0xFF]*N_FLUSH+[CLR_BYTE]+lead+SONG+[0xFF]
print('字节流:', ' '.join('%02X'%b for b in stream))
print('前导: %d x FF + AA%s' % (N_FLUSH, '' if no_ee else ' + EE EE'))
print()
fr=feed(stream)
ons=[(c,k) for c,k in fr if k is not None]
seq=[k for _,k in ons]
want=[b for b in SONG if b!=REST_BYTE]
print('点亮序列:', ' '.join(str(k) for k in seq))
print('期望序列:', ' '.join(str(k) for k in want))
print('个数: 实际 %d / 期望 %d   一致 = %s' % (len(seq),len(want),seq==want))
if ons:
    print('第一个音出现在第 %.2f 拍（FF占1拍%s）' % (ons[0][0]/(PLAY_DIV+1), '' if no_ee else ' + EE EE占2拍=3拍'))
