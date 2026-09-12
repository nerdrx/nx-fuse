"""Summarize bounded NX Fuse shadow-session recordings."""
import argparse
import json
import math
import sys


MAX_BYTES = 4 * 1024 * 1024
NOTE = 'Differences are not ground-truth accuracy; model-inferred observations only'


def _number(value, name, *, integer=False, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be numeric')
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    if integer and not isinstance(value, int):
        raise ValueError(f'{name} must be an integer')
    if minimum is not None and value < minimum or maximum is not None and value > maximum:
        raise ValueError(f'{name} out of bounds')
    return value


def _vector(value, name):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f'{name} must be a 3-vector')
    return [_number(item, name, minimum=-10000, maximum=10000) for item in value]


def _percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    position = (len(values) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def _stats(values):
    if not values:
        return None
    return {'median': _percentile(values, .5), 'p95': _percentile(values, .95),
            'rms': math.sqrt(sum(value * value for value in values) / len(values))}


def _metric_stats(values):
    if not values:
        return None
    return {'median': _percentile(values, .5), 'p95': _percentile(values, .95)}


def build_report(data):
    if not isinstance(data, dict) or not {'format', 'frames'} <= set(data) or set(data) - {'format', 'frames', 'note'}:
        raise ValueError('recording must contain format and frames')
    if data['format'] != 'nx-fuse-shadow-v1':
        raise ValueError('unsupported recording format')
    if 'note' in data and not isinstance(data['note'], str):
        raise ValueError('recording note invalid')
    frames = data['frames']
    if not isinstance(frames, list) or len(frames) > 600:
        raise ValueError('frames must be a list of at most 600 items')
    groups = {}
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict) or set(frame) != {'camera', 'generation', 'timestamp_ns', 'joints', 'metrics', 'note'}:
            raise ValueError(f'frame {index} shape invalid')
        camera, generation = frame['camera'], frame['generation']
        if not isinstance(camera, str) or not camera or len(camera) > 256:
            raise ValueError(f'frame {index} camera invalid')
        if not isinstance(generation, str) or not generation or len(generation) > 128:
            raise ValueError(f'frame {index} generation invalid')
        timestamp = _number(frame['timestamp_ns'], f'frame {index} timestamp_ns', integer=True, minimum=0, maximum=2**64-1)
        joints = frame['joints']
        if not isinstance(joints, list) or len(joints)>32:
            raise ValueError(f'frame {index} joints invalid')
        metrics = frame['metrics']
        if not isinstance(metrics, dict) or set(metrics) != {'rms_px', 'anchor_shift_m', 'match_error_ms'}:
            raise ValueError(f'frame {index} metrics invalid')
        metric_values = {key: _number(value, f'frame {index} {key}', minimum=0, maximum=1e6)
                         for key, value in metrics.items()}
        if not isinstance(frame['note'], str):
            raise ValueError(f'frame {index} note invalid')
        key = (camera, generation)
        group = groups.setdefault(key, {'camera': camera, 'generation': generation,
                                        'frames': 0, 'joints': {}, 'rms_px': [],
                                        'anchor_shift_m': []})
        if group.get('_last_timestamp') is not None and timestamp < group['_last_timestamp']:
            raise ValueError(f'frame {index} timestamp order invalid')
        group['_last_timestamp'] = timestamp
        group['frames'] += 1
        group['rms_px'].append(metric_values['rms_px'])
        group['anchor_shift_m'].append(metric_values['anchor_shift_m'])
        seen=set()
        for joint_index, item in enumerate(joints):
            if not isinstance(item, dict) or set(item) != {'joint', 'position', 'confidence', 'baseline', 'residual_m'}:
                raise ValueError(f'frame {index} joint {joint_index} shape invalid')
            name = item['joint']
            if not isinstance(name, str) or not name or len(name) > 128:
                raise ValueError(f'frame {index} joint name invalid')
            if name in seen: raise ValueError('duplicate joint in frame')
            seen.add(name)
            position = _vector(item['position'], 'position')
            _number(item['confidence'], 'confidence', minimum=0, maximum=1)
            baseline = item['baseline']
            if baseline is not None:
                baseline = _vector(baseline, 'baseline')
                residual = math.dist(position, baseline)
            else:
                residual = None
            if item['residual_m'] is not None:
                _number(item['residual_m'], 'residual_m', minimum=0, maximum=1e6)
            stats = group['joints'].setdefault(name, {'count': 0, 'baseline_paired_count': 0,
                                                       'residuals': []})
            stats['count'] += 1
            if residual is not None:
                stats['baseline_paired_count'] += 1
                stats['residuals'].append(residual)
    output_groups = []
    for group in groups.values():
        output_groups.append({'camera': group['camera'], 'generation': group['generation'],
                              'frames': group['frames'],
                              'joints': {name: {'count': values['count'],
                                                'baseline_paired_count': values['baseline_paired_count'],
                                                'residual_m': _stats(values['residuals'])}
                                         for name, values in sorted(group['joints'].items())},
                              'metrics': {'rms_px': _metric_stats(group['rms_px']),
                                          'anchor_shift_m': _metric_stats(group['anchor_shift_m'])}})
    return {'format': 'nx-fuse-shadow-report-v1', 'groups': output_groups, 'note': NOTE}


def main(argv=None):
    parser = argparse.ArgumentParser(description='Summarize an NX Fuse shadow pose log')
    parser.add_argument('path')
    args = parser.parse_args(argv)
    try:
        with open(args.path, 'rb') as stream:
            raw = stream.read(MAX_BYTES + 1)
        if len(raw) > MAX_BYTES:
            raise ValueError('input exceeds 4 MiB')
        result = build_report(json.loads(raw.decode('utf-8')))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        parser.error(str(exc))
    json.dump(result, sys.stdout, sort_keys=True)
    sys.stdout.write('\n')


if __name__ == '__main__':
    main()
