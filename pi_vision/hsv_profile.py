"""Versioned field-tuning profile; omitted values preserve the HSV baseline."""

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import yaml


DEFAULT_PROFILE = {
    'version': 1,
    'bands': {
        1: [[[0, 130, 120], [9, 255, 255]], [[170, 130, 120], [179, 255, 255]]],
        2: [[[20, 70, 100], [34, 255, 255]]],
        3: [[[86, 60, 60], [135, 255, 255]]],
        4: [[[35, 65, 65], [85, 255, 255]]],
        5: [[[0, 0, 0], [179, 255, 80]]],
    },
    'shape': {'min_area_ratio': .012, 'max_area_ratio': .4,
              'min_circularity': .3, 'min_solidity': .65,
              'min_fill': .4, 'max_aspect': 3.0},
    'blue': {'cyan_h_max': 101, 'cyan_s_max': 160, 'cyan_v_min': 170,
             'blue_h_min': 106, 'blue_s_min': 190, 'mixed_fraction': .25,
             'classify_h_min': 104, 'classify_s_min': 170, 'classify_v_min': 140},
}


def validate_profile(raw=None):
    """Reject typos, invalid bands and NaN rather than silently changing behavior."""
    if raw is None:
        raw = {}
    if not isinstance(raw, dict) or set(raw) - set(DEFAULT_PROFILE):
        raise ValueError('HSV profile: unknown fields or non-mapping input')
    result = deepcopy(DEFAULT_PROFILE)
    if type(raw.get('version', 1)) is not int or raw.get('version', 1) != 1:
        raise ValueError('HSV profile version must be 1')
    for section in ('shape', 'blue'):
        supplied = raw.get(section, {})
        if not isinstance(supplied, dict) or set(supplied) - set(result[section]):
            raise ValueError(f'HSV profile: unknown {section} fields')
        result[section].update(supplied)
        for key, value in result[section].items():
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f'HSV profile: invalid {key}')
            maximum = (179 if '_h_' in key else 255)
            if section == 'shape':
                valid = value >= 1 if key == 'max_aspect' else 0 < value <= 1
            elif key == 'mixed_fraction':
                valid = 0 < value <= .5
            else:
                valid = type(value) is int and 0 <= value <= maximum
            if not valid:
                raise ValueError(f'HSV profile: invalid {key}')
    shape = result['shape']
    if not 0 < shape['min_area_ratio'] < shape['max_area_ratio'] < 1:
        raise ValueError('HSV profile: invalid area ratios')
    bands = raw.get('bands', {})
    if not isinstance(bands, dict):
        raise ValueError('HSV profile: bands must be a mapping')
    for color, entries in bands.items():
        # JSON metadata uses string keys; YAML examples use business IDs.
        if type(color) is int:
            cid = color
        elif type(color) is str and color in ('1', '2', '3', '4', '5'):
            cid = int(color)
        else:
            raise ValueError('HSV profile: band IDs must be 1..5; 3 includes lightblue')
        if cid not in result['bands'] or not isinstance(entries, list) or not entries:
            raise ValueError('HSV profile: invalid color bands')
        for band in entries:
            if not isinstance(band, (list, tuple)) or len(band) != 2:
                raise ValueError('HSV profile: expected lower/upper HSV band')
            low, high = band
            if any(not isinstance(v, (list, tuple)) or len(v) != 3 for v in band):
                raise ValueError('HSV profile: expected HSV triples')
            for a, b, maximum in zip(low, high, (179, 255, 255)):
                if type(a) is not int or type(b) is not int or not 0 <= a <= b <= maximum:
                    raise ValueError('HSV profile: HSV bounds out of range or reversed')
        result['bands'][cid] = deepcopy(entries)
    return result


def load_profile(path):
    return validate_profile(yaml.safe_load(Path(path).read_text(encoding='utf-8')))


def profile_hash(profile):
    return hashlib.sha256(json.dumps(profile, sort_keys=True).encode()).hexdigest()
