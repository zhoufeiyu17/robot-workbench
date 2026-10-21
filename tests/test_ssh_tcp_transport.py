"""Linux socket transport preserves a bidirectional stream and half-close."""
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import unittest

SCRIPT=Path(os.environ.get('RBW_SSH_TRANSPORT',Path(__file__).resolve().parents[1]/'scripts/ssh_tcp_transport.py'))


@unittest.skipUnless(sys.platform.startswith('linux'),'deployed Linux TCP_MAXSEG socket semantics')
class TcpTransportTest(unittest.TestCase):
    def test_stream_and_stdin_eof(self):
        with socket.socket() as listener:
            listener.bind(('127.0.0.1',0));listener.listen(1);listener.settimeout(5)
            received=[]
            def echo():
                with listener.accept()[0] as peer:
                    while chunk:=peer.recv(4096):received.append(chunk);peer.sendall(chunk)
            thread=threading.Thread(target=echo,daemon=True);thread.start()
            payload=bytes(range(256))*1024
            result=subprocess.run([sys.executable,str(SCRIPT),'127.0.0.1',str(listener.getsockname()[1]),'1000'],
                input=payload,capture_output=True,timeout=5)
            thread.join(1)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(result.stdout,payload)
            self.assertEqual(b''.join(received),payload)
            self.assertFalse(thread.is_alive())

    def test_invalid_segment_limit_does_not_connect(self):
        result=subprocess.run([sys.executable,str(SCRIPT),'127.0.0.1','1','0'],capture_output=True,timeout=2)
        self.assertNotEqual(result.returncode,0)
        self.assertIn(b'bounded TCP MSS required',result.stderr)


if __name__=='__main__':unittest.main()
