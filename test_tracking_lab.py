import os
from pathlib import Path
import socket
import tempfile
import unittest

from tracking_lab import launch


class TrackingLabLauncherTests(unittest.TestCase):
    def test_owned_worker_stops_and_removes_socket_when_dashboard_exits(self):
        with tempfile.TemporaryDirectory() as runtime, tempfile.TemporaryDirectory() as files:
            runtime_path = Path(runtime)
            dashboard = Path(files) / 'fake-dashboard'
            dashboard.write_text('#!/bin/sh\nexit 0\n')
            dashboard.chmod(0o700)
            with socket.socket() as probe:
                probe.bind(('127.0.0.1', 0))
                port = probe.getsockname()[1]
            old_runtime = os.environ.get('XDG_RUNTIME_DIR')
            os.environ['XDG_RUNTIME_DIR'] = runtime
            try:
                self.assertEqual(launch(dashboard, port=port), 0)
            finally:
                if old_runtime is None:
                    os.environ.pop('XDG_RUNTIME_DIR', None)
                else:
                    os.environ['XDG_RUNTIME_DIR'] = old_runtime

            socket_path = runtime_path / f'nx-fuse-{port}' / 'tracking.sock'
            self.assertFalse(socket_path.exists(), 'owned worker left tracking socket behind')
            with socket.socket() as check:
                check.settimeout(.2)
                with self.assertRaises(OSError):
                    check.connect(('127.0.0.1', port))


if __name__ == '__main__':
    unittest.main()
