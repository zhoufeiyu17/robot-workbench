"""Transparent SSH byte transport with a connection-local TCP segment-size cap."""
import os,socket,sys,threading
host,port,mss=sys.argv[1],int(sys.argv[2]),int(sys.argv[3])
if not 536<=mss<=1460:raise ValueError('bounded TCP MSS required')
s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.setsockopt(socket.IPPROTO_TCP,socket.TCP_MAXSEG,mss)
s.settimeout(12);s.connect((host,port));s.settimeout(None)
def upload():
    try:
        while data:=os.read(0,65536):s.sendall(data)
        s.shutdown(socket.SHUT_WR)
    except OSError:pass
threading.Thread(target=upload,daemon=True).start()
try:
    while data:=s.recv(65536):
        view=memoryview(data)
        while view:view=view[os.write(1,view):]
finally:s.close()
