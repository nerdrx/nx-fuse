#!/usr/bin/env python3
"""Launch the native WiVRn tracking lab with its read-only worker ready first."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import urllib.error
import urllib.request


def launch(dashboard, port=8787, preview=False):
    dashboard = Path(dashboard).absolute()
    if not dashboard.is_file() or not os.access(dashboard, os.X_OK):
        raise ValueError('Choose an executable WiVRn NX dashboard build')
    runtime = os.environ.get('XDG_RUNTIME_DIR')
    if not runtime:
        raise ValueError('A desktop session with XDG_RUNTIME_DIR is required')
    root = Path(__file__).resolve().parent
    socket_path = Path(runtime)/f'nx-fuse-{port}'/'tracking.sock'
    environment = {**os.environ, 'NX_FUSE_TAP': str(socket_path), 'NX_FUSE_PORT': str(port),
                   'NX_FUSE_WORKER': str(root/'app.py'), 'NX_DASHBOARD_PREVIEW': '1' if preview else '0'}
    model = root/'artifacts/models/pose_landmarker_lite.task'
    if model.is_file(): environment.setdefault('NX_FUSE_MODEL', str(model))
    http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def status():
        try:
            with http.open(f'http://127.0.0.1:{port}/api/tracking', timeout=2) as response:
                value = json.load(response)
            if not value.get('read_only') or not value.get('enabled') or value.get('socket') != str(socket_path):
                raise ValueError(f'Port {port} is occupied by another Fuse mode; stop it or choose --port')
            return True
        except urllib.error.HTTPError as exc:
            raise ValueError(f'Port {port} is occupied by another service; choose --port') from exc
        except urllib.error.URLError as exc:
            if not isinstance(exc.reason, ConnectionRefusedError): raise
            return False
    worker = None
    try:
        if not status():
            # A stale socket is deliberately not unlinked here; an active owner must not be displaced.
            python = root/'.venv/bin/python'
            worker = subprocess.Popen([str(python) if python.is_file() else sys.executable,
                                       str(root/'app.py'), '--port', str(port)], env=environment)
            deadline = time.monotonic()+8
            while not status():
                if worker.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError('Tracking worker did not start; inspect its message above')
                time.sleep(.1)
        return subprocess.call([str(dashboard)], env=environment)
    finally:
        if worker is not None and worker.poll() is None:
            worker.terminate()
            try: worker.wait(timeout=3)
            except subprocess.TimeoutExpired:
                worker.kill()
                worker.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dashboard', required=True)
    parser.add_argument('--port', type=int, default=8787)
    parser.add_argument('--preview', action='store_true', help='Disable VR server startup and attachment')
    args = parser.parse_args()
    if not 0 < args.port <= 65535: parser.error('port must be 1–65535')
    try: sys.exit(launch(args.dashboard, args.port, args.preview))
    except (ValueError, RuntimeError, OSError) as exc: parser.exit(1, str(exc)+'\n')
