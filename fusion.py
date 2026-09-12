"""Position-only research fusion. Inputs must share calibrated space and clock.

No camera model or WiVRn connection is provided by this module.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Observation:
    camera: str
    joint: str
    position: tuple[float, float, float]
    confidence: float
    timestamp: float  # host monotonic seconds, capture time (not receive time)


def number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def vector(value):
    return (isinstance(value, (tuple, list)) and len(value) == 3
            and all(number(x) for x in value))


def camera_only(observations, now, anchors=None):
    """Fuse already-calibrated camera positions without body trackers.

    Missing/rejected joints are absent, not tracked at their previous position.
    Head/controller anchors, when supplied, remain authoritative. Orientations
    and monocular depth inference are outside this position-only framework.
    """
    if anchors is None:
        anchors = {}
    if not number(now) or not isinstance(anchors, dict) or any(not vector(p) for p in anchors.values()):
        raise ValueError('Invalid anchor or time')
    sources = {}
    for o in observations:
        try:
            position, timestamp, confidence = o.position, o.timestamp, o.confidence
            key = (o.camera, o.joint)
            hash(key)
        except (AttributeError, TypeError):
            continue
        if (not vector(position) or not number(timestamp)
                or not number(confidence) or not .65 <= confidence <= 1
                or not 0 <= now-timestamp < .15):
            continue
        if key not in sources or timestamp > sources[key].timestamp:
            sources[key] = o
    result = {}
    for joint in {o.joint for o in sources.values()}:
        obs = [o for o in sources.values() if o.joint == joint]
        if any(math.dist(a.position,b.position) > .2 for a in obs for b in obs):
            continue
        scores = [o.confidence*(1-(now-o.timestamp)/.15) for o in obs]
        result[joint] = {'position':[sum(o.position[i]*s for o,s in zip(obs,scores))/sum(scores) for i in range(3)],
                         'confidence':max(scores), 'source':'camera'}
    for joint,position in anchors.items():
        result[joint] = {'position':list(position),'confidence':1.,'source':'anchor'}
    return result


class Fusion:
    """Bounded correction with stale rejection and gradual release.

    Thresholds are provisional research defaults, not measured guarantees.
    Call reset after recenter, calibration changes, or tracking session changes.
    """
    protected = frozenset({'head', 'left_hand', 'right_hand'})

    def __init__(self):
        self.reset()

    def reset(self):
        self.corrections = {}
        self.weights = {}
        self.last_observed = {}
        self.last_time = None

    def step(self, baseline, observations, now, enabled=False):
        if not number(now) or not isinstance(baseline, dict) or any(not vector(p) for p in baseline.values()):
            raise ValueError('Invalid base pose or timestamp')
        if self.last_time is not None and (now < self.last_time or now-self.last_time > .5):
            self.reset()
        dt = min(max(now - self.last_time, 0), .1) if self.last_time is not None else 0
        self.last_time = now
        if not enabled:
            self.corrections.clear()
            self.weights.clear()
            self.last_observed.clear()
            return {j: {'baseline': list(p), 'fused': list(p), 'weight': 0.0} for j, p in baseline.items()}
        # Deduplicate each source/joint: repeated frames cannot increase its vote.
        candidates = {}
        for o in observations:
            try:
                joint, position, timestamp, confidence = o.joint, o.position, o.timestamp, o.confidence
                key = (o.camera, joint)
                hash(key)
            except (AttributeError, TypeError):
                continue
            if (joint not in baseline or joint in self.protected or not vector(position)
                    or not number(timestamp) or not number(confidence)
                    or not .65 <= confidence <= 1 or not 0 <= now-timestamp < .15
                    or math.dist(position, baseline[joint]) > .5):
                continue
            if key not in candidates or timestamp > candidates[key].timestamp:
                candidates[key] = o
        result = {}
        for joint, base in baseline.items():
            obs = [o for o in candidates.values() if o.joint == joint]
            # Disagreement is uncertainty: refuse all corrections for this joint.
            if any(math.dist(a.position, b.position) > .2 for a in obs for b in obs):
                obs = []
            target, weight = (0., 0., 0.), 0.
            if obs:
                scores = [o.confidence * (1-(now-o.timestamp)/.15) for o in obs]
                total = sum(scores)
                if total > 0:
                    self.last_observed[joint] = max(o.timestamp for o in obs)
                    weight = min(.85, max(scores))
                    target = tuple((sum(o.position[i]*s for o,s in zip(obs,scores))/total-base[i])*weight for i in range(3))
            old = self.corrections.get(joint, (0., 0., 0.))
            alpha = 1-math.exp(-dt/(.12 if obs else .20))
            correction = tuple(a+(b-a)*alpha for a,b in zip(old,target))
            expired = now-self.last_observed.get(joint, -math.inf) >= .5
            if joint in self.protected or expired:
                correction = (0., 0., 0.)
            self.corrections[joint] = correction
            self.weights[joint] = self.weights.get(joint, 0)+(weight-self.weights.get(joint, 0))*alpha
            if expired:
                self.weights[joint] = 0.
            result[joint] = {'baseline': list(base), 'fused': [a+b for a,b in zip(base,correction)], 'weight': self.weights[joint]}
        for joint in set(self.corrections)-set(baseline):
            self.corrections.pop(joint, None)
            self.weights.pop(joint, None)
            self.last_observed.pop(joint, None)
        return result
