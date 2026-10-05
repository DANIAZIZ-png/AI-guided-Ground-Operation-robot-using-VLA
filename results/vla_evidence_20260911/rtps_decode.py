import struct, sys, collections
f = open(sys.argv[1], 'rb'); g = f.read(24)
magic = struct.unpack('<I', g[:4])[0]
E = '<' if magic in (0xa1b2c3d4, 0xa1b23c4d) else '>'
linktype = struct.unpack(E+'I', g[20:24])[0]
SUB = {0x06:'ACKNACK',0x07:'HEARTBEAT',0x08:'GAP',0x09:'INFO_TS',0x0c:'INFO_SRC',0x0d:'INFO_REPLY_IP4',0x0e:'INFO_DST',0x0f:'INFO_REPLY',0x12:'NACK_FRAG',0x13:'HEARTBEAT_FRAG',0x15:'DATA',0x16:'DATA_FRAG',0x01:'PAD'}
by_port = collections.Counter(); by_type = collections.Counter(); ents = collections.Counter(); pk = 0; t0 = t1 = None
seqs = collections.defaultdict(list); sizes = collections.Counter()
pos = 0; data = f.read(); n = len(data)
while pos + 16 <= n:
    ts_s, ts_u, incl, orig = struct.unpack(E+'IIII', data[pos:pos+16]); pos += 16
    p = data[pos:pos+incl]; pos += incl
    t = ts_s + ts_u/1e6; t0 = t0 or t; t1 = t
    off = 14 if linktype == 1 else 16  # ethernet / linux cooked
    if linktype == 113: off = 16
    ip = p[off:]; 
    if len(ip) < 20 or ip[9] != 17: continue
    ihl = (ip[0] & 0xf)*4; udp = ip[ihl:]; dport = struct.unpack('>H', udp[2:4])[0]
    pay = udp[8:]; pk += 1; sizes[dport] += orig
    if pay[:4] != b'RTPS': by_type['non-RTPS'] += 1; continue
    by_port[dport] += 1
    i = 20; types = []
    while i + 4 <= len(pay):
        sid, flags, ln = pay[i], pay[i+1], struct.unpack(('<' if pay[i+1] & 1 else '>')+'H', pay[i+2:i+4])[0]
        nm = SUB.get(sid, hex(sid)); types.append(nm); le = '<' if flags & 1 else '>'
        body = pay[i+4:i+4+ln]
        if nm in ('HEARTBEAT','ACKNACK','DATA','GAP') and len(body) >= 8:
            rid, wid = body[0:4].hex(), body[4:8].hex()
            ents[(dport, nm, 'reader='+rid, 'writer='+wid)] += 1
            if nm == 'HEARTBEAT' and len(body) >= 24:
                fh, fl, lh, ll = struct.unpack(le+'iIiI', body[8:24]); seqs[(dport, wid)].append((fh<<32|fl, lh<<32|ll))
            if nm == 'ACKNACK' and len(body) >= 16:
                bh, bl = struct.unpack(le+'iI', body[8:16]); seqs[(dport,'ack',rid)].append(bh<<32|bl)
        i += 4 + ln
        if ln == 0 and sid not in (0x0e,): break
    by_type[' + '.join(types)] += 1
dur = (t1 - t0) if t0 else 1
print('packets %d in %.2f s  = %.0f pkt/s   (%.0f kB/s payload)' % (pk, dur, pk/dur, sum(sizes.values())/dur/1024))
print('\n-- by destination port (Pi participant):'); [print('  %6d  %6d pkt  %5.0f pkt/s  %6.0f B avg' % (p, c, c/dur, sizes[p]/c)) for p, c in by_port.most_common()]
print('\n-- by submessage pattern:'); [print('  %6d  %s' % (c, k)) for k, c in by_type.most_common(8)]
print('\n-- top endpoint pairs:'); [print('  %6d  port %d %-9s %s %s' % (c, k[0], k[1], k[2], k[3])) for k, c in ents.most_common(14)]
print('\n-- heartbeat seq ranges per (port, writer): first/last seen'); 
for k, v in list(seqs.items())[:12]:
    if k[1] != 'ack': print('  port %d writer %s: %s ... %s  (%d hb)' % (k[0], k[1], v[0], v[-1], len(v)))
    else: print('  port %d ACKNACK reader %s: base %s ... %s (%d)' % (k[0], k[2], v[0], v[-1], len(v)))
