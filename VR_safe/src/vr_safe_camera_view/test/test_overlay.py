"""叠加层纯函数测试：颜色、提示、红框和区域绘制。"""

import numpy as np

from vr_safe_camera_view.overlay import (OverlayState, border_bgr, hint_for,
                                         render)


def test_revoked_is_red_and_granted_is_green():
    assert border_bgr(OverlayState(permit_decision="REVOKED")) == (0, 0, 255)
    assert border_bgr(OverlayState(permit_decision="GRANTED")) == (0, 200, 0)
    assert border_bgr(OverlayState(permit_decision="SLOW")) == (0, 200, 255)


def test_hint_tells_operator_how_to_restore_only_when_possible():
    ok = OverlayState(permit_decision="REVOKED", obs_state="CLEAR")
    assert "RJ 长按 1.5s" in hint_for(ok)
    occupied = OverlayState(permit_decision="REVOKED", obs_state="OCCUPIED")
    assert "不能恢复" in hint_for(occupied)
    unknown = OverlayState(permit_decision="REVOKED", obs_state="UNKNOWN")
    assert "不能恢复" in hint_for(unknown)
    estop = OverlayState(permit_decision="REVOKED", permit_source="system_estop")
    assert "Y" in hint_for(estop)


def test_render_draws_red_border_and_zone_without_changing_shape():
    frame = np.zeros((240, 640, 3), np.uint8)
    state = OverlayState(permit_decision="REVOKED", permit_latched=True,
                         permit_reason="看不清", obs_state="OCCUPIED",
                         obs_reason="区域被占")
    zone = {"mode": "polygon",
            "corners_px": [[100, 100], [300, 100], [300, 200], [100, 200]]}
    out = render(frame, state, zone)
    assert out.shape == frame.shape
    assert tuple(out[2, 2]) == (0, 0, 255)
    assert tuple(out[150, 200]) != (0, 0, 0)


def test_unknown_without_revoke_is_not_stopping():
    state = OverlayState(permit_decision="GRANTED", obs_state="UNKNOWN")
    assert "不停止" in hint_for(state)


def test_render_draws_person_skeleton():
    frame = np.zeros((240, 640, 3), np.uint8)
    marks = [[0.5, 0.5, 1.0]] * 33
    persons = [{"state": "OCCUPIED", "landmarks": marks}]
    state = OverlayState(permit_decision="REVOKED", obs_state="OCCUPIED")
    out = render(frame, state, None, persons)
    # 至少有一个关节点被画成红色。
    assert np.any(np.all(out == (0, 0, 255), axis=2))


def test_smooth_persons_reduces_jitter_and_keeps_state():
    from vr_safe_camera_view.overlay import smooth_persons
    old = [{"id": 1, "state": "CLEAR",
            "landmarks": [[0.0, 0.0, 1.0]] * 33}]
    new = [{"id": 1, "state": "NEAR",
            "landmarks": [[1.0, 1.0, 1.0]] * 33}]
    out = smooth_persons(old, new, alpha=0.5)
    assert out[0]["state"] == "NEAR"
    assert out[0]["landmarks"][0] == [0.5, 0.5, 1.0]
