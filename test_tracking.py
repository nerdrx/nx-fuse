import math
from pathlib import Path
import socket
import tempfile
import time
import unittest

from tracking import HEADER, RECORD, TrackingReceiver, decode_packet


def packet(sequence=1, generation=1, now=None, value=1.0, role=10, flags=3,
           quaternion=(0, 0, 0, 1)):
    now = time.monotonic_ns() if now is None else now
    return (HEADER.pack(b'NXTP', 1, HEADER.size, sequence, now, generation, 1, 1, 1, 0)
            + RECORD.pack(role, 0, flags, value, 1, 0, *quaternion, 0, 0, 0, 0, 0, 0, now))


class TrackingTests(unittest.TestCase):
    def test_wire_shape_flags_and_finite_validation(self):
        now = time.monotonic_ns()
        self.assertEqual(HEADER.size, 40)
        self.assertEqual(RECORD.size, 68)
        record = decode_packet(packet(now=now), now)['records'][0]
        self.assertEqual(record['joint'], 'hip')
        self.assertTrue(record['position_valid'])
        for data in (b'', packet() + b'x', packet(value=math.nan), packet(now=now-3_000_000_000)):
            with self.assertRaises(ValueError): decode_packet(data, now)

    def test_socket_sequence_generation_and_expiry(self):
        frame = time.monotonic_ns() - 20_000_000
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'tracking.sock'
            receiver = TrackingReceiver(path)
            sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            try:
                sender.sendto(packet(), str(path))
                deadline = time.monotonic()+1
                while not receiver.snapshot()['connected'] and time.monotonic()<deadline:
                    time.sleep(.005)
                self.assertTrue(receiver.snapshot()['connected'])
                self.assertEqual(receiver.snapshot()['records'][0]['joint'], 'hip')
                receiver._accept(decode_packet(packet()))
                self.assertEqual(receiver.snapshot()['rejected'], 1)
                receiver._accept(decode_packet(packet(generation=2)))
                self.assertEqual(receiver.snapshot()['generation'], '2')
                receiver._accept(decode_packet(packet(sequence=5, generation=1)))
                self.assertEqual(receiver.snapshot()['generation'], '2')
                with receiver._lock:
                    receiver._records['hip']['host_time_ns'] -= 600_000_000
                self.assertFalse(receiver.snapshot()['connected'])
                with self.assertRaises(OSError): TrackingReceiver(path)
                self.assertTrue(path.exists())
            finally:
                sender.close()
                receiver.close()
            self.assertFalse(path.exists())

    def test_shared_directory_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            Path(folder).chmod(0o755)
            with self.assertRaises(ValueError): TrackingReceiver(Path(folder)/'tracking.sock')

    def test_stable_anchor_requires_quiet_recent_history_and_rotates_offset(self):
        now = time.monotonic_ns()
        frame = now - 20_000_000
        frame = time.monotonic_ns() - 20_000_000
        frame = time.monotonic_ns() - 20_000_000
        with tempfile.TemporaryDirectory() as folder:
            receiver = TrackingReceiver(Path(folder) / 'tracking.sock')
            try:
                for sequence, host_time in enumerate((frame - 300_000_000, frame - 200_000_000,
                                                       frame - 100_000_000, frame), 1):
                    receiver._accept(decode_packet(
                        packet(sequence, now=host_time, value=1, role=1, flags=0x33), now))
                anchor = receiver.stable_anchor('head', frame, (0.1, 0, 0))
                self.assertEqual(anchor['generation'], '1')
                self.assertEqual(anchor['position'], [1.1, 1, 0])
                self.assertEqual(anchor['sample_time_ns'], frame)
                self.assertLessEqual(anchor['match_error_ms'], 60)
                with self.assertRaises(ValueError): receiver.stable_anchor('hip', frame, (0, 0, 0))
                with self.assertRaises(ValueError): receiver.stable_anchor('head', frame, (0.6, 0, 0))
            finally:
                receiver.close()

    def test_stable_anchor_rotates_offset(self):
        now = time.monotonic_ns()
        frame = now - 20_000_000
        q = (0, 0, math.sqrt(.5), math.sqrt(.5))
        with tempfile.TemporaryDirectory() as folder:
            receiver = TrackingReceiver(Path(folder) / 'tracking.sock')
            try:
                for sequence, host_time in enumerate((frame - 300_000_000, frame - 200_000_000,
                                                       frame - 100_000_000, frame), 1):
                    receiver._accept(decode_packet(packet(sequence, now=host_time, value=1,
                                                          role=1, flags=0x33, quaternion=q), now))
                self.assertEqual(receiver.stable_anchor('head', frame, (0.1, 0, 0))['position'],
                                 [1, 1.1, 0])
            finally:
                receiver.close()

    def test_stable_anchor_rejects_motion_rotation_stale_and_invalid_history(self):
        now = time.monotonic_ns()
        frame = now - 20_000_000
        cases = (([1, 1, 1, 1.02], [(0, 0, 0, 1)] * 4, 0x33, 'moved'),
                 ([1, 1, 1, 1], [(0, 0, 0, 1)] * 3 + [(0, 0, math.sin(math.radians(2)), math.cos(math.radians(2)))], 0x33, 'rotated'),
                 ([1, 1, 1, 1], [(0, 0, 0, 1)] * 4, 0x31, 'invalid'))
        for values, quaternions, flags, reason in cases:
            frame = time.monotonic_ns() - 20_000_000
            with tempfile.TemporaryDirectory() as folder:
                receiver = TrackingReceiver(Path(folder) / 'tracking.sock')
                try:
                    for sequence, host_time in enumerate((frame - 300_000_000, frame - 200_000_000,
                                                           frame - 100_000_000, frame), 1):
                                                              receiver._accept(decode_packet(packet(sequence, now=host_time, value=values[sequence-1],
                                                              role=1, flags=flags, quaternion=quaternions[sequence-1]), time.monotonic_ns()))
                    with self.assertRaisesRegex(ValueError, reason): receiver.stable_anchor('head', frame, (0, 0, 0))
                finally:
                    receiver.close()
        with tempfile.TemporaryDirectory() as folder:
            receiver = TrackingReceiver(Path(folder) / 'tracking.sock')
            try:
                for sequence in range(1, 5):
                    receiver._accept(decode_packet(packet(sequence, now=frame - 500_000_000,
                                                          value=1, role=1, flags=0x33), time.monotonic_ns()))
                with self.assertRaisesRegex(ValueError, 'missing|recent|hold still'):
                    receiver.stable_anchor('head', frame, (0, 0, 0))
            finally:
                receiver.close()
        frame = time.monotonic_ns() - 20_000_000
        with tempfile.TemporaryDirectory() as folder:
            receiver = TrackingReceiver(Path(folder) / 'tracking.sock')
            try:
                for sequence, host_time in enumerate((frame - 300_000_000, frame - 200_000_000,
                                                       frame - 100_000_000, frame), 1):
                    receiver._accept(decode_packet(packet(sequence, now=host_time, role=1, flags=0x33),
                                                   time.monotonic_ns()))
                receiver._accept(decode_packet(packet(5, generation=2, now=frame, role=1, flags=0x33),
                                               time.monotonic_ns()))
                with self.assertRaisesRegex(ValueError, 'missing|hold still'):
                    receiver.stable_anchor('head', frame, (0, 0, 0))
            finally:
                receiver.close()


if __name__ == '__main__': unittest.main()
