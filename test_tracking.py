import math
from pathlib import Path
import socket
import tempfile
import time
import unittest

from tracking import HEADER, RECORD, TrackingReceiver, decode_packet


def packet(sequence=1, generation=1, now=None, value=1.0):
    now = time.monotonic_ns() if now is None else now
    return (HEADER.pack(b'NXTP', 1, HEADER.size, sequence, now, generation, 1, 1, 1, 0)
            + RECORD.pack(10, 0, 3, value, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, now))


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


if __name__ == '__main__': unittest.main()
