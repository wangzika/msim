"""Local live viewer and PNG export. Physics stays on its owning thread."""
import base64
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
import json
from pathlib import Path
import re
import threading
import time
from urllib.parse import urlsplit


class LiveView:
    def __init__(self,port,scene,output=None):
        self.lock=threading.Lock();self.state=dict(phase='starting',sim_time_s=0.,trajectory=[],gps_path=[],cloud=[])
        # Mesh collisions are retained in physics; use their actual bounds for
        # wireframe overview, explicitly labeled in the viewer.
        self.scene=scene;self.path=deque(maxlen=3000);self.gps=deque(maxlen=1000)
        self.exports=Path(output)/'exports' if output is not None else None
        owner=self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                route=urlsplit(self.path).path
                if route=='/':payload=files('msim').joinpath('data/live.html').read_bytes();mime='text/html; charset=utf-8'
                elif route=='/api/state':
                    with owner.lock:payload=json.dumps(owner.state,allow_nan=False).encode('utf-8')
                    mime='application/json'
                elif route=='/api/scene':payload=json.dumps(owner.scene,allow_nan=False).encode('utf-8');mime='application/json'
                elif re.fullmatch(r'/exports/physics-[0-9]+\.png',route) and owner.exports is not None:
                    path=owner.exports/route.rsplit('/',1)[1]
                    if not path.is_file():self.send_error(404);return
                    payload=path.read_bytes();mime='image/png'
                else:self.send_error(404);return
                self.send_response(200);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(payload)))
                self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers()
                try:self.wfile.write(payload)
                except (BrokenPipeError,ConnectionResetError):pass
            def do_POST(self):
                if self.path!='/api/export/png' or owner.exports is None:self.send_error(404);return
                try:
                    origin=self.headers.get('Origin')
                    if origin is not None and origin!=owner.url.rstrip('/'):raise ValueError('Cross-origin export denied')
                    size=int(self.headers.get('Content-Length','0'))
                    if not 0<size<=12_000_000 or self.headers.get_content_type()!='application/json':raise ValueError('Invalid export request')
                    value=json.loads(self.rfile.read(size))['image']
                    prefix='data:image/png;base64,'
                    if not isinstance(value,str) or not value.startswith(prefix):raise ValueError('PNG required')
                    data=base64.b64decode(value[len(prefix):],validate=True)
                    if len(data)<24 or not data.startswith(b'\x89PNG\r\n\x1a\n') or data[12:16]!=b'IHDR':raise ValueError('Invalid PNG')
                    width=int.from_bytes(data[16:20],'big');height=int.from_bytes(data[20:24],'big')
                    if not 0<width<=8192 or not 0<height<=8192:raise ValueError('PNG dimensions outside limits')
                    owner.exports.mkdir(parents=True,exist_ok=True)
                    name=f'physics-{time.time_ns()}.png';(owner.exports/name).write_bytes(data)
                    payload=json.dumps(dict(url='/exports/'+name)).encode('utf-8')
                    self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(payload)));self.end_headers();self.wfile.write(payload)
                except (ValueError,KeyError,TypeError):self.send_error(400,'Invalid PNG export')
            def log_message(self,*_):pass
        self.server=ThreadingHTTPServer(('127.0.0.1',port),Handler);self.server.daemon_threads=True
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.url=f'http://127.0.0.1:{self.server.server_port}/'

    def update(self,**values):
        with self.lock:
            if 'position' in values:self.path.append(values['position']);values['trajectory']=list(self.path)
            if 'gps' in values and values['gps'] is not None:self.gps.append(values['gps']);values['gps_path']=list(self.gps)
            self.state.update(values)

    def close(self):self.server.shutdown();self.server.server_close();self.thread.join(timeout=2)
