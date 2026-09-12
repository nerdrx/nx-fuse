"""Opt-in, read-only receiver for WiVRn's bounded NXTP datagrams."""
from collections import deque
import math
import os
from pathlib import Path
import socket
import stat
import struct
import threading
import time

HEADER = struct.Struct('<4sHHQQQBBHI')
RECORD = struct.Struct('<HHI3f4f3f3fQ')
MAX_PACKET = HEADER.size + 32 * RECORD.size
ROLES = {1: 'head', 2: 'left_hand', 3: 'right_hand', 4: 'left_aim', 5: 'right_aim',
         6: 'left_palm', 7: 'right_palm', 10: 'hip', 11: 'chest',
         12: 'left_elbow', 13: 'right_elbow', 14: 'left_knee', 15: 'right_knee',
         16: 'left_foot', 17: 'right_foot'}
ROUTES = {0: 'anchors', 1: 'BD', 2: 'HTC'}


def decode_packet(data, now_ns=None):
    """Validate shape, roles, timestamps and finite values before publishing."""
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    if len(data) < HEADER.size:
        raise ValueError('short tracking header')
    magic, version, size, sequence, host_ns, generation, route, count, flags, reserved = HEADER.unpack_from(data)
    if (magic != b'NXTP' or version != 1 or size != HEADER.size or reserved
            or route not in ROUTES or not 0 < count <= 32 or not generation
            or flags != 1 or len(data) != HEADER.size + count * RECORD.size):
        raise ValueError('invalid tracking header')
    if not now_ns - 2_000_000_000 <= host_ns <= now_ns + 100_000_000:
        raise ValueError('expired or future tracking packet')
    records = []
    seen = set()
    for index in range(count):
        role, source, pose_flags, *values = RECORD.unpack_from(data, HEADER.size + index * RECORD.size)
        sample_ns = values.pop()
        name = ROLES.get(role)
        if name is None and 0x8000 <= role < 0x8010:
            name = 'generic_' + str(role & 0x7fff)
        if name is None and 0x100 <= role < 0x120:
            name = 'bd_joint_' + str(role & 0xff)
        if name is None or role in seen or pose_flags & ~0x3f or not all(map(math.isfinite, values)):
            raise ValueError('invalid tracking record')
        if not host_ns - 5_000_000_000 <= sample_ns <= host_ns + 500_000_000:
            raise ValueError('invalid sample timestamp')
        seen.add(role)
        records.append({'joint': name, 'source': source, 'flags': pose_flags,
                        'position': values[:3], 'orientation': values[3:7],
                        'linear_velocity': values[7:10], 'angular_velocity': values[10:13],
                        'sample_time_ns': sample_ns, 'host_time_ns': host_ns,
                        'position_valid': bool(pose_flags & 1),
                        'orientation_valid': bool(pose_flags & 2), 'route': ROUTES[route]})
    return {'sequence': sequence, 'host_time_ns': host_ns, 'generation': generation,
            'route': ROUTES[route], 'records': records}


class TrackingReceiver:
    """A private socket and bounded pose-only history. Never writes to WiVRn."""

    def __init__(self, path):
        self.path = Path(path).absolute()
        parent = self.path.parent
        parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        info = parent.stat()
        if info.st_uid != os.getuid() or stat.S_IMODE(info.st_mode) & 0o077:
            raise ValueError('tracking socket needs a user-owned private directory (0700)')
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            self._socket.bind(str(self.path))  # Never unlink another receiver's socket.
            self.path.chmod(0o600)
            self._inode = self.path.stat().st_ino
            self._socket.settimeout(.2)
        except Exception:
            self._socket.close()
            raise
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._generation = self._sequence = self._host_ns = 0
        self._records = {}
        self._history = {}  # At most 63 recognized roles × 120 pose-only samples.
        self._retired = deque(maxlen=8)
        self._accepted = self._rejected = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _accept(self, packet):
        with self._lock:
            generation = packet['generation']
            if generation != self._generation:
                if generation in self._retired or packet['host_time_ns'] < self._host_ns:
                    self._rejected += 1
                    return
                if self._generation:
                    self._retired.append(self._generation)
                self._generation, self._sequence = generation, 0
                self._records.clear()
                self._history.clear()
            if packet['sequence'] <= self._sequence:
                self._rejected += 1
                return
            self._sequence, self._host_ns = packet['sequence'], packet['host_time_ns']
            self._accepted += 1
            for record in packet['records']:
                joint = record['joint']
                self._records[joint] = record
                self._history.setdefault(joint, deque(maxlen=120)).append(record)

    def _run(self):
        while not self._stop.is_set():
            try:
                data = self._socket.recv(MAX_PACKET + 1)
                self._accept(decode_packet(data))
            except socket.timeout:
                continue
            except ValueError:
                with self._lock:
                    self._rejected += 1
            except OSError:
                break

    def snapshot(self):
        now = time.monotonic_ns()
        with self._lock:
            records = [{**record, 'age_ms': max(0, (now - record['host_time_ns']) / 1e6)}
                       for record in self._records.values() if now - record['host_time_ns'] <= 500_000_000]
            return {'enabled': True, 'connected': bool(records), 'read_only': True,
                    'generation': str(self._generation), 'sequence': str(self._sequence),
                    'records': records, 'accepted': self._accepted, 'rejected': self._rejected,
                    'socket': str(self.path)}

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1)
        self._socket.close()
        try:
            if self.path.stat().st_ino == self._inode:
                self.path.unlink()
        except FileNotFoundError:
            pass
