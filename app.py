#!/usr/bin/env python3
"""Local NX Fuse simulation console. Does not open cameras or connect to VR."""
import argparse
import json
import math
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from fusion import Fusion, Observation

ROOT = Path(__file__).resolve().parent


def cameras():
    # Linux nodes can be metadata/IR endpoints, not independent cameras.
    return [{'id': '/dev/'+p.name, 'name': (p/'name').read_text().strip()}
            for p in sorted(Path('/sys/class/video4linux').glob('video*')) if (p/'name').exists()]


class Simulation:
    def __init__(self):
        self.enabled = False
        self.occluded = False
        self.fusion = Fusion()
        self.lock = threading.Lock()
        self.state = {}

    def tick(self):
        now = time.monotonic()
        sway = math.sin(now*.8)*.05
        base = {'head': (sway,1.75,0), 'neck': (sway,1.5,0), 'hip': (sway,.92,0),
                'left_shoulder': (-.23+sway,1.45,0), 'right_shoulder': (.23+sway,1.45,0),
                'left_elbow': (-.36,1.15,.04), 'right_elbow': (.36,1.15,.04),
                'left_hand': (-.4,.91,.10), 'right_hand': (.4,.91,.10),
                'left_knee': (-.15,.5,.05), 'right_knee': (.15,.5,.05),
                'left_foot': (-.17,.08,.1), 'right_foot': (.17,.08,.1)}
        clean = dict(base)
        for joint in ('hip','left_knee','right_knee','left_foot','right_foot'):
            x,y,z = base[joint]
            base[joint] = (x+.075*math.sin(now*2), y, z+.045*math.cos(now))
        obs = [] if self.occluded else [Observation('room-demo',j,p,.95,now-.025) for j,p in clean.items()]
        self.state = {'time':now, 'enabled':self.enabled, 'occluded':self.occluded,
                      'mode':'simulation', 'joints':self.fusion.step(base,obs,now,self.enabled),
                      'cameras':[{'id':'room-demo','label':'Room · simulated', 'status':'occluded' if self.occluded else 'synthetic'},
                                 {'id':'bed-demo','label':'Bed · planned','status':'not connected'}]}


def make_handler(sim):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, status, value, kind='application/json'):
            data = json.dumps(value).encode() if kind == 'application/json' else value
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(data)

        def allowed(self):
            expected = f'127.0.0.1:{self.server.server_port}'
            return (self.headers.get('Host') in (expected, f'localhost:{self.server.server_port}')
                    and self.headers.get('Origin', 'http://'+expected) in
                    ('http://'+expected, f'http://localhost:{self.server.server_port}'))

        def do_GET(self):
            if not self.allowed():
                return self.send(403, {'error':'Local requests only'})
            if self.path == '/api/state':
                with sim.lock:
                    return self.send(200, sim.state)
            if self.path == '/api/cameras':
                return self.send(200, {'devices':cameras()})
            if self.path in ('/', '/index.html'):
                return self.send(200, (ROOT/'web/index.html').read_bytes(), 'text/html; charset=utf-8')
            self.send(404, {'error':'Not found'})

        def do_POST(self):
            if not self.allowed():
                return self.send(403, {'error':'Local requests only'})
            if self.path != '/api/control':
                return self.send(404, {'error':'Not found'})
            try:
                size = int(self.headers.get('Content-Length','0'))
                if not 0 < size <= 1024:
                    raise ValueError('Invalid request size')
                value = json.loads(self.rfile.read(size))
                if (not isinstance(value, dict) or not value
                        or set(value)-{'enabled','occluded'} or any(type(v) is not bool for v in value.values())):
                    raise ValueError('Expected enabled/occluded boolean values')
                with sim.lock:
                    for k,v in value.items():
                        setattr(sim,k,v)
                    sim.tick()
                self.send(200, {'ok':True})
            except (ValueError, UnicodeDecodeError):
                self.send(400, {'error':'Expected enabled/occluded booleans, at most 1024 bytes'})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8787)
    args = parser.parse_args()
    sim = Simulation()
    sim.tick()
    stop = threading.Event()
    def update():
        while not stop.wait(1/60):
            with sim.lock:
                sim.tick()
    thread = threading.Thread(target=update, daemon=True)
    server = ThreadingHTTPServer(('127.0.0.1',args.port),make_handler(sim))
    thread.start()
    print(f'NX Fuse simulation: http://127.0.0.1:{server.server_port} — no live VR output', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        thread.join()


if __name__ == '__main__':
    main()
