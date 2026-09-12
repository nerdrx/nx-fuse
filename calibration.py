"""Offline rigid calibration between matched 3D point samples."""
import argparse
import json
from dataclasses import dataclass

try:
    import numpy as np
except ImportError:  # Keep module importable in fusion-only environments.
    np = None


def _array(points, minimum=4):
    if np is None:
        raise RuntimeError('numpy is required for calibration')
    try:
        if any(isinstance(value, (bool, np.bool_)) for row in points for value in row):
            raise ValueError('point coordinates must be finite numbers, not booleans')
        values = np.asarray(points, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError('point coordinates must be finite 3D numbers') from exc
    if values.ndim != 2 or values.shape[1] != 3 or values.shape[0] < minimum or not np.isfinite(values).all():
        raise ValueError(f'need at least {minimum} finite 3D points')
    return values


def _well_conditioned(points):
    singular = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
    if singular[-1] <= max(singular[0] * 1e-3, 1e-9) or singular[0] / singular[-1] > 1e4:
        raise ValueError('points must be noncoplanar and well-conditioned')


@dataclass(frozen=True)
class RigidTransform:
    rotation: tuple[tuple[float, float, float], ...]
    translation: tuple[float, float, float]

    def apply(self, points):
        values = _array(points, 1)
        return values @ np.asarray(self.rotation).T + np.asarray(self.translation)


@dataclass(frozen=True)
class CalibrationReport:
    transform: RigidTransform
    rms_error: float
    max_error: float
    held_out_errors: tuple[float, ...]


def fit_rigid(model_points, target_points, validation_model_points=None, validation_target_points=None,
              *, max_rms_error=0.05, max_error=0.1):
    """Fit target ~= rotation @ model + translation; reject scale/reflection."""
    if np is None:
        raise RuntimeError('numpy is required for calibration')
    if (not isinstance(max_rms_error, (int, float)) or not isinstance(max_error, (int, float))
            or isinstance(max_rms_error, bool) or not np.isfinite(max_rms_error)
            or max_rms_error <= 0 or isinstance(max_error, bool)
            or not np.isfinite(max_error) or max_error <= 0):
        raise ValueError('error limits must be finite positive numbers')
    model = _array(model_points)
    target = _array(target_points)
    if model.shape != target.shape:
        raise ValueError('matched point arrays must have equal shape')
    _well_conditioned(model)
    _well_conditioned(target)
    model_center = model.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (model - model_center).T @ (target - target_center)
    u, _, vh = np.linalg.svd(covariance)
    rotation = vh.T @ u.T
    if np.linalg.det(rotation) <= 0:
        raise ValueError('reflection is not a rigid transform')
    translation = target_center - rotation @ model_center
    transform = RigidTransform(tuple(map(tuple, rotation.tolist())), tuple(translation.tolist()))
    errors = np.linalg.norm(transform.apply(model) - target, axis=1)
    rms = float(np.sqrt(np.mean(errors ** 2)))
    maximum = float(np.max(errors))
    if rms > max_rms_error or maximum > max_error:
        raise ValueError(f'fit error too high; check units or correspondences (rms={rms:.6g}, max={maximum:.6g})')
    held_out = ()
    if validation_model_points is not None or validation_target_points is not None:
        if validation_model_points is None or validation_target_points is None:
            raise ValueError('held-out model and target points must be provided together')
        held_model = _array(validation_model_points, 1)
        held_target = _array(validation_target_points, 1)
        if held_model.shape != held_target.shape:
            raise ValueError('held-out point arrays must have equal shape')
        held_out = tuple(np.linalg.norm(transform.apply(held_model) - held_target, axis=1).tolist())
    return CalibrationReport(transform, rms, maximum, held_out)


def _payload(data):
    required = {'camera_id', 'resolution', 'session_generation', 'fit', 'validation'}
    if not isinstance(data, dict) or set(data) != required:
        raise ValueError('JSON must contain exactly camera_id, resolution, session_generation, fit, validation')
    if not isinstance(data['camera_id'], str) or not data['camera_id'].strip():
        raise ValueError('camera_id must be a nonempty string')
    resolution = data['resolution']
    if (not isinstance(resolution, list) or len(resolution) != 2
            or any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in resolution)):
        raise ValueError('resolution must contain two positive integers')
    generation = data['session_generation']
    if ((isinstance(generation, bool) or not isinstance(generation, (str, int)))
            or (isinstance(generation, str) and not generation.strip())
            or (isinstance(generation, int) and generation < 0)):
        raise ValueError('session_generation must be nonempty string or nonnegative integer')
    for name in ('fit', 'validation'):
        if not isinstance(data[name], list) or not data[name] or any(not isinstance(pair, dict) or set(pair) != {'model', 'target'} for pair in data[name]):
            raise ValueError(f'{name} must be nonempty list of model/target pairs')
    return data


def _main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', help='JSON calibration pairs')
    args = parser.parse_args()
    with open(args.input) as source:
        data = _payload(json.load(source))
    pairs = data['fit']
    validation = data['validation']
    report = fit_rigid([p['model'] for p in pairs], [p['target'] for p in pairs],
                       [p['model'] for p in validation] if validation else None,
                       [p['target'] for p in validation] if validation else None)
    output = {'camera_id': data['camera_id'], 'resolution': data['resolution'],
              'session_generation': data['session_generation'],
              'rotation': report.transform.rotation, 'translation': report.transform.translation,
              'rms_error': report.rms_error, 'max_error': report.max_error,
              'held_out_errors': report.held_out_errors,
              'validated': max(report.held_out_errors) <= 0.1 and
                           (sum(error * error for error in report.held_out_errors) /
                            len(report.held_out_errors)) ** .5 <= 0.05}
    print(json.dumps(output, allow_nan=False, sort_keys=True))


if __name__ == '__main__':
    _main()
