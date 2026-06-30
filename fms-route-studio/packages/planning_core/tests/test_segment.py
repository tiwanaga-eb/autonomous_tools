import numpy as np
import pytest
from affine import Affine

from planning_core.drivable import generate_drivable, segment_drivable_base

cv2 = pytest.importorskip("cv2")  # OpenCV 未導入環境ではスキップ


def _scene():
    # 左半分=低コスト(走行可能), 右半分=高コスト。境界がきれいに割れるはず。
    n = 60
    cost = np.full((n, n), 10.0, float)
    cost[:, n // 2:] = 400.0
    t = Affine(0.5, 0, 0, 0, -0.5, float(n) * 0.5)
    return cost, t


def test_segment_otsu_splits_low_high_cost():
    cost, t = _scene()
    base = segment_drivable_base(cost, t, method="otsu")
    n = cost.shape[0]
    # 左(低コスト)はほぼ走行可能、右(高コスト)はほぼ不可
    assert base[:, : n // 2].mean() > 0.9
    assert base[:, n // 2:].mean() < 0.1


def test_segment_adaptive_runs():
    cost, t = _scene()
    base = segment_drivable_base(cost, t, method="adaptive")
    assert base.shape == cost.shape and base.dtype == bool
    assert base.any()


def test_generate_drivable_with_base_override():
    cost, t = _scene()
    base = segment_drivable_base(cost, t, method="otsu")
    mask, stats = generate_drivable(cost, t, threshold=0.0, base_override=base, min_area_m2=0.0)
    # threshold=0 でも base_override 優先で走行可能域が残る
    assert mask.any()
    assert stats["area_m2"] > 0
