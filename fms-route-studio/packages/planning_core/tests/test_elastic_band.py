"""Elastic Band（経路洗練: 収縮＋クリアランス反発）のテスト。"""
import numpy as np
from affine import Affine

from planning_core.planners import elastic_band

H, W = 20, 60
TRANSFORM = Affine(1.0, 0.0, 0.0, 0.0, -1.0, float(H))  # col→x, row→y=H-row


def _corridor_mask(half_width: float) -> np.ndarray:
    m = np.zeros((H, W), dtype=np.uint8)
    for r in range(H):
        y = H - r - 0.5
        if abs(y - 10.0) <= half_width:
            m[r, :] = 1
    return m


def _clearance(x, y, mask):
    from scipy.ndimage import distance_transform_edt
    dt = distance_transform_edt(mask > 0)
    inv = ~TRANSFORM
    c = int(np.floor(inv.a * x + inv.b * y + inv.c))
    r = int(np.floor(inv.d * x + inv.e * y + inv.f))
    return float(dt[r, c]) if (0 <= r < H and 0 <= c < W) else 0.0


def test_eb_pushes_path_toward_center():
    mask = _corridor_mask(4.0)  # y∈[6,14], 中心 y=10
    # 下端 y=7 を這う初期経路 → 中央(余裕大)へ寄るはず
    pts = np.array([[float(x), 7.0] for x in range(5, 56, 2)], float)
    out = elastic_band(pts, mask, TRANSFORM, desired_clearance=3.5, iters=200, step=0.6)
    interior = slice(1, -1)
    assert out[interior, 1].mean() > pts[interior, 1].mean() + 0.5  # y が中央へ上昇
    # 全点が走行可能領域内
    assert all(_clearance(x, y, mask) > 0.0 for x, y in out)


def test_eb_keeps_endpoints_fixed():
    mask = _corridor_mask(4.0)
    pts = np.array([[float(x), 8.0] for x in range(5, 56, 2)], float)
    out = elastic_band(pts, mask, TRANSFORM, desired_clearance=3.0, iters=50)
    assert np.allclose(out[0], pts[0]) and np.allclose(out[-1], pts[-1])


def test_eb_smooths_zigzag_within_mask():
    mask = _corridor_mask(4.0)
    xs = np.arange(5, 56, 2.0)
    ys = 10.0 + 2.0 * ((np.arange(len(xs)) % 2) * 2 - 1)  # 8/12 のジグザグ
    pts = np.column_stack([xs, ys])

    def length(p):
        return float(np.hypot(*(np.diff(p, axis=0).T)).sum())

    out = elastic_band(pts, mask, TRANSFORM, desired_clearance=0.0, iters=100, w_smooth=0.4)
    assert length(out) < length(pts)  # 平滑化で短く
    assert all(_clearance(x, y, mask) > 0.0 for x, y in out)


def test_eb_noop_when_too_few_points():
    mask = _corridor_mask(4.0)
    pts = np.array([[5.0, 10.0], [50.0, 10.0]], float)
    out = elastic_band(pts, mask, TRANSFORM, desired_clearance=3.0)
    assert np.allclose(out, pts)
