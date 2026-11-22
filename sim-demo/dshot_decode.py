import json,sys,collections
ev=collections.defaultdict(list)
for l in open(sys.argv[1]+'/gpio.jsonl'):
    d=json.loads(l)
    if d['port']=='gpioPortB' and d['pin'] in (0,1,6,7): ev[d['pin']].append((d['t'],d['level']))
for pin,e in sorted(ev.items()):
    frames=[];cur=[];rise=None;last=None
    for t,lv in e:
        if lv==1: rise=t
        elif rise is not None:
            w=(t-rise)*1e9
            if cur and rise-last>5e-6:
                frames.append((cur[0][0],cur)); cur=[]
            cur.append((rise,w)); last=rise; rise=None
    if cur: frames.append((cur[0][0],cur))
    print('PB%d edges=%d frames=%d'%(pin,len(e),len(frames)))
    vals=collections.Counter(); first={}
    for t,c in frames:
        if len(c)!=16: vals['len%d'%len(c)]+=1; continue
        v=0
        for _,w in c: v=(v<<1)|(1 if w>940 else 0)
        val=v>>5; tel=(v>>4)&1; crc=v&15
        v12=(val<<1)|tel; ok=((v12^(v12>>4)^(v12>>8))&15)==crc
        k=(val,ok); vals[k]+=1; first.setdefault(k,t)
    for k,n in vals.most_common(6): print('  ',k,n,'first t=%s'%first.get(k))
