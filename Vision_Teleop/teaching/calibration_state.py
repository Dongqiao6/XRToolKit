"""Bind a capture configuration to an explicit, immutable calibration version."""
import copy
import hashlib
from pathlib import Path

import numpy as np
import yaml


def save_config(config_path, config):
    target = Path(config_path)
    temporary = target.with_suffix(target.suffix+'.tmp')
    temporary.write_text(yaml.safe_dump(config,allow_unicode=True,sort_keys=False),encoding='utf-8')
    temporary.replace(target)


def load_bound_calibration(root, config):
    path = Path(root) / config['calibration_file']
    raw = path.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    expected = config.get('calibration_sha256')
    if expected and digest != expected:
        raise ValueError('Calibration file changed after binding; review and bind it again')
    c = yaml.safe_load(raw)
    cams = config['cameras']
    if c['camera_serials'] != [cams[r+'_serial'] for r in ('left','right')]:
        raise ValueError('Camera roles differ from calibration')
    if c['image_size'] != [cams['width'], cams['height']]:
        raise ValueError('Capture resolution differs from calibration')
    for key in ('K1','K2','R'):
        a = np.asarray(c[key],float)
        if a.shape != (3,3) or not np.isfinite(a).all():
            raise ValueError('Invalid calibration matrix: '+key)
    for key in ('D1','D2','T'):
        if not np.isfinite(np.asarray(c[key],float)).all():
            raise ValueError('Nonfinite calibration: '+key)
    r = np.asarray(c['R'])
    if not np.allclose(r.T@r,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(r),1,atol=1e-5):
        raise ValueError('Invalid calibration rotation')
    t = np.asarray(c['T'])
    if t.size != 3 or np.linalg.norm(t) <= 0:
        raise ValueError('Invalid calibration baseline')
    return c, raw, digest


def bind_calibration(root, config_path, config, calibration_path, *, reviewed, reason):
    """Persist a new binding only after validation; reviewed is an explicit decision."""
    updated = copy.deepcopy(config)
    path = Path(calibration_path).resolve()
    try:
        name = path.relative_to(Path(root).resolve()).as_posix()
    except ValueError:
        name = str(path)
    updated['calibration_file'] = name
    updated.pop('calibration_sha256',None)
    _,_,digest = load_bound_calibration(root,updated)
    updated.update(calibration_sha256=digest, calibration_review_required=not reviewed,
                   calibration_review_note=reason)
    save_config(config_path, updated)
    config.clear()
    config.update(updated)


def snapshot_calibration(root, config, session):
    if not config.get('calibration_file'):
        return dict(status='unconfigured')
    try:
        _,raw,digest = load_bound_calibration(root,config)
    except (OSError,ValueError,KeyError,yaml.YAMLError) as exc:
        return dict(status='unavailable_or_mismatched',reason=str(exc),source=config['calibration_file'])
    (Path(session)/'calibration_snapshot.yaml').write_bytes(raw)
    return dict(status='pending_review' if config.get('calibration_review_required',False) else 'bound',
                source=config['calibration_file'],sha256=digest,snapshot='calibration_snapshot.yaml',
                scope='Calibration binding only; not hand accuracy or camera immobility certification')
