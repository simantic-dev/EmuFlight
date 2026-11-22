import os,sys,time,select,termios,tty
dev=sys.argv[1]; cmds=sys.argv[2:]
fd=os.open(dev,os.O_RDWR|os.O_NOCTTY); tty.setraw(fd)
def rd(t):
    end=time.time()+t; out=b''
    while time.time()<end:
        r,_,_=select.select([fd],[],[],0.5)
        if r: out+=os.read(fd,4096)
    return out
print(rd(1).decode(errors='replace'))
def send(s): os.write(fd,s.encode())
send('#'); print(rd(20).decode(errors='replace'))
for c in cmds:
    send(c+'\r\n'); print(rd(15).decode(errors='replace'))
