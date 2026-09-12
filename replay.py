"""Replay calibrated synthetic/recorded poses: python3 replay.py input.jsonl.

Each line: {time: seconds, baseline: {joint: [x,y,z]}, observations: [{camera,
joint, position: [x,y,z], confidence: 0..1, timestamp: seconds}], enabled: bool}.
All positions are metres in one shared space. This does not calibrate inputs.
"""
import argparse
import json
from fusion import Fusion, Observation


def replay(lines):
    fusion = Fusion()
    for number, line in enumerate(lines, 1):
        try:
            frame = json.loads(line)
            if type(frame.get('enabled', False)) is not bool:
                raise ValueError('enabled must be boolean')
            observations = [Observation(**value) for value in frame.get('observations', [])]
            yield {'time':frame['time'], 'joints':fusion.step(frame.get('baseline', frame.get('pico')), observations,frame['time'],frame.get('enabled',False))}
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            raise ValueError(f'Invalid frame at line {number}: {exc}') from exc


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input')
    args = parser.parse_args()
    with open(args.input) as source:
        for result in replay(source):
            print(json.dumps(result, allow_nan=False, sort_keys=True))
