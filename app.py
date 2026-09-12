#!/usr/bin/env python3
"""Local NX Fuse observation tools and separate simulation; cameras require opt-in."""
import argparse
import json
import math
import os
import signal
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from fusion import Fusion, Observation, camera_only
from camera import CameraManager, enumerate_devices
from lens_setup import LensSetup
from alignment_setup import AlignmentSetup
from shadow_session import ShadowSession

ROOT = Path(__file__).resolve().parent


def cameras():
    return enumerate_devices()


class Simulation:
    def __init__(self, camera_manager=None, tracking_receiver=None):
        self.enabled = False
        self.occluded = False
        self.camera_only = False
        self.fusion = Fusion()
        self.lock = threading.Lock()
        self.state = {}
        self.camera_manager = camera_manager or CameraManager()
        self.tracking_receiver = tracking_receiver
        self.lens_setup = LensSetup()
        self.alignment_setup = AlignmentSetup(self.camera_manager, self.lens_setup, tracking_receiver)
        self.shadow_session = ShadowSession(self.camera_manager, self.lens_setup, self.alignment_setup, tracking_receiver)

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
        if self.camera_only:
            self.fusion.reset()
            anchors = {j:base[j] for j in Fusion.protected}
            poses = camera_only(obs,now,anchors)
            fused = {j:{'fused':p['position'], 'weight':p['confidence'] if p['source']=='camera' else 0.}
                     for j,p in poses.items()}
        else:
            fused = self.fusion.step(base,obs,now,self.enabled)
            for joint in fused.values():
                joint["pico"] = joint.pop("baseline")  # simulator display label only
        self.state = {'time':now, 'enabled':self.enabled, 'occluded':self.occluded,
                      'camera_only':self.camera_only,
                      'observations':[{'camera':o.camera,'joint':o.joint,'position':o.position,
                                       'confidence':o.confidence,'timestamp':o.timestamp} for o in obs],
                      'mode':'simulation', 'joints':fused,
                      'cameras':[{'id':'room-demo','label':'Room · simulated', 'status':'occluded' if self.occluded else 'synthetic'},
                                 {'id':'bed-demo','label':'Bed · planned','status':'not connected'}]}


def make_handler(sim):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, status, value, kind='application/json', filename=None):
            data = json.dumps(value).encode() if kind == 'application/json' else value
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            if filename:
                self.send_header('Content-Disposition', 'attachment; filename="'+filename+'"')
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
                return self.send(200, {'devices':sim.camera_manager.devices()})
            if self.path == '/api/tracking':
                return self.send(200, sim.tracking_receiver.snapshot() if sim.tracking_receiver else
                                 {'enabled':False, 'connected':False, 'read_only':True, 'records':[]})
            if self.path == '/api/shadow/recording':
                return self.send(200, sim.shadow_session.recording(), filename='nx-fuse-shadow.json')
            if self.path == '/api/shadow':
                return self.send(200, sim.shadow_session.snapshot())
            if self.path == '/api/alignment':
                return self.send(200, sim.alignment_setup.snapshot())
            if self.path == '/api/alignment/profile':
                profile = sim.alignment_setup.snapshot()['profile']
                return self.send(200, profile, filename='nx-fuse-alignment.json') if profile else self.send(404, {'error':'No alignment profile'})
            if self.path.startswith('/api/alignment/frame?'):
                from urllib.parse import parse_qs, urlparse
                token = parse_qs(urlparse(self.path).query).get('token', [''])[0]
                frame = sim.alignment_setup.frame(token)
                return self.send(200, frame, 'image/jpeg') if frame else self.send(404, {'error':'Frozen frame expired'})
            if self.path == '/api/calibration':
                return self.send(200, sim.lens_setup.snapshot())
            if self.path == '/api/calibration/profile':
                profile = sim.lens_setup.snapshot()['profile']
                return self.send(200, profile, filename='nx-fuse-lens-profile.json') if profile else self.send(404, {'error':'No lens profile'})
            if self.path == '/api/calibration/board':
                from lens_calibration import calibration_board_svg
                return self.send(200, calibration_board_svg((9, 6), 25).encode(), 'image/svg+xml')
            if self.path.startswith('/api/camera/frame'):
                from urllib.parse import parse_qs, urlparse
                device_id = parse_qs(urlparse(self.path).query).get('id', [None])[0]
                frame = sim.camera_manager.frame(device_id) if device_id else None
                return self.send(200, frame, 'image/jpeg') if frame else self.send(404, {'error':'No latest frame'})
            if self.path in ('/', '/index.html'):
                return self.send(200, (ROOT/'web/index.html').read_bytes(), 'text/html; charset=utf-8')
            self.send(404, {'error':'Not found'})

        def do_POST(self):
            if not self.allowed():
                return self.send(403, {'error':'Local requests only'})
            if self.path == '/api/calibration/profile':
                try:
                    size = int(self.headers.get('Content-Length','0'))
                    if not 0 < size <= 32768: raise ValueError('Lens profile must be at most 32 KiB')
                    sim.lens_setup.load_profile(json.loads(self.rfile.read(size)))
                    return self.send(200, {'ok':True})
                except (ValueError, RuntimeError, KeyError) as exc:
                    return self.send(400, {'error':str(exc)})
            if self.path == '/api/shadow/recording':
                try:
                    size = int(self.headers.get('Content-Length','0'))
                    if not 0 < size <= 128: raise ValueError('Invalid request size')
                    value = json.loads(self.rfile.read(size))
                    if not isinstance(value,dict) or set(value) != {'action'}:
                        raise ValueError('Expected recording action')
                    sim.shadow_session.recording_command(value['action'])
                    return self.send(200, {'ok':True})
                except (ValueError, RuntimeError) as exc:
                    return self.send(400, {'error':str(exc)})
            if self.path == '/api/shadow':
                try:
                    size = int(self.headers.get('Content-Length','0'))
                    if not 0 < size <= 1024: raise ValueError('Invalid request size')
                    value = json.loads(self.rfile.read(size))
                    if not isinstance(value,dict) or set(value) != {'enabled','id'}:
                        raise ValueError('Expected id and boolean enabled')
                    sim.shadow_session.set_enabled(value['id'],value['enabled'])
                    return self.send(200, {'ok':True})
                except (ValueError, RuntimeError, KeyError) as exc:
                    return self.send(400, {'error':str(exc)})
            if self.path in ('/api/calibration', '/api/alignment'):
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 1024: raise ValueError('Invalid request size')
                    value = json.loads(self.rfile.read(size))
                    if not isinstance(value, dict): raise ValueError('Expected calibration command')
                    if self.path == '/api/alignment':
                        sim.alignment_setup.command(value)
                    else:
                        sim.lens_setup.command(value, sim.camera_manager)
                    return self.send(200, {'ok':True})
                except (ValueError, RuntimeError, KeyError) as exc:
                    return self.send(400, {'error':str(exc)})
            if self.path != '/api/control':
                if self.path == '/api/camera/control':
                    try:
                        size = int(self.headers.get('Content-Length','0'))
                        if not 0 < size <= 1024:
                            raise ValueError('Invalid request size')
                        value = json.loads(self.rfile.read(size))
                        if (not isinstance(value, dict) or set(value) != {'id','enabled'}
                                or not isinstance(value['id'], str) or type(value['enabled']) is not bool):
                            raise ValueError('Expected id and boolean enabled')
                        status = sim.camera_manager.set_enabled(value['id'], value['enabled'])
                        return self.send(200, {'ok':True, 'device':status})
                    except KeyError:
                        return self.send(404, {'error':'Unknown camera'})
                    except (ValueError, UnicodeDecodeError, RuntimeError) as exc:
                        return self.send(503 if isinstance(exc, RuntimeError) else 400, {'error':str(exc)})
                if self.path == '/api/camera/estimation':
                    try:
                        size = int(self.headers.get('Content-Length','0'))
                        if not 0 < size <= 1024: raise ValueError('Invalid request size')
                        value = json.loads(self.rfile.read(size))
                        if (not isinstance(value, dict) or set(value) != {'id','enabled'}
                                or not isinstance(value['id'], str) or type(value['enabled']) is not bool):
                            raise ValueError('Expected id and boolean enabled')
                        status = sim.camera_manager.set_estimation(value['id'], value['enabled'])
                        return self.send(200, {'ok':True, 'device':status})
                    except KeyError:
                        return self.send(404, {'error':'Unknown camera'})
                    except (ValueError, UnicodeDecodeError, RuntimeError) as exc:
                        return self.send(503 if isinstance(exc, RuntimeError) else 400, {'error':str(exc)})
                return self.send(404, {'error':'Not found'})
            try:
                size = int(self.headers.get('Content-Length','0'))
                if not 0 < size <= 1024:
                    raise ValueError('Invalid request size')
                value = json.loads(self.rfile.read(size))
                if (not isinstance(value, dict) or not value
                        or set(value)-{'enabled','occluded','camera_only'} or any(type(v) is not bool for v in value.values())):
                    raise ValueError('Expected boolean controls')
                with sim.lock:
                    for k,v in value.items():
                        setattr(sim,k,v)
                    sim.tick()
                self.send(200, {'ok':True})
            except (ValueError, UnicodeDecodeError):
                self.send(400, {'error':'Expected enabled/occluded/camera_only booleans, at most 1024 bytes'})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8787)
    parser.add_argument('--tracking-socket', default=os.environ.get('NX_FUSE_TAP'),
                        help='Opt-in private Unix socket for read-only WiVRn tracking')
    args = parser.parse_args()
    from tracking import TrackingReceiver
    receiver = TrackingReceiver(args.tracking_socket) if args.tracking_socket else None
    sim = Simulation(tracking_receiver=receiver)
    sim.tick()
    stop = threading.Event()
    def update():
        while not stop.wait(1/60):
            with sim.lock:
                sim.tick()
    thread = threading.Thread(target=update, daemon=True)
    try:
        server = ThreadingHTTPServer(('127.0.0.1',args.port),make_handler(sim))
    except Exception:
        if receiver: receiver.close()
        raise
    thread.start()
    def terminate(_signal, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    print(f'NX Fuse simulation: http://127.0.0.1:{server.server_port} — no live VR output', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        sim.shadow_session.close()
        sim.camera_manager.close()
        if receiver: receiver.close()
        server.server_close()
        thread.join()


if __name__ == '__main__':
    main()
