import numpy as np
from rasterio.transform import from_origin

from planning_core.drivable import generate_drivable, mask_to_rgba


def _cost_left_low():
    # 左半分=低コスト(走行可)、右半分=高コスト
    cost = np.full((50, 50), 1000.0, dtype=np.float32)
    cost[:, :25] = 10.0
    transform = from_origin(0.0, 50.0, 1.0, 1.0)  # 1m セル, 左上(0,50)
    return cost, transform


def test_generate_basic_threshold():
    cost, tr = _cost_left_low()
    mask, stats = generate_drivable(
        cost, tr, threshold=100.0, close_m=0, open_m=0, min_area_m2=0
    )
    assert mask[:, :25].mean() > 0.9   # 左は走行可
    assert mask[:, 25:].mean() < 0.1   # 右は不可
    assert stats["area_m2"] > 0
    assert stats["island_count"] >= 1


def test_exclude_edit_removes_region():
    cost, tr = _cost_left_low()
    mask0, _ = generate_drivable(cost, tr, threshold=100.0, close_m=0, open_m=0, min_area_m2=0)
    # 左上の 10x10 (world座標) を除外
    edit = [{"op": "exclude", "polygon": [[0, 50], [10, 50], [10, 40], [0, 40]]}]
    mask1, _ = generate_drivable(
        cost, tr, threshold=100.0, close_m=0, open_m=0, min_area_m2=0, edits=edit
    )
    assert mask1.sum() < mask0.sum()


def test_include_edit_adds_region():
    cost, tr = _cost_left_low()
    mask0, _ = generate_drivable(cost, tr, threshold=100.0, close_m=0, open_m=0, min_area_m2=0)
    # 右側(本来不可)に include 領域を足す
    edit = [{"op": "include", "polygon": [[40, 50], [50, 50], [50, 40], [40, 40]]}]
    mask1, _ = generate_drivable(
        cost, tr, threshold=100.0, close_m=0, open_m=0, min_area_m2=0, edits=edit
    )
    assert mask1.sum() > mask0.sum()


def test_mask_to_rgba_alpha():
    mask = np.zeros((4, 4), dtype=bool)
    mask[1, 1] = True
    rgba = mask_to_rgba(mask)
    assert rgba.shape == (4, 4, 4)
    assert rgba[1, 1, 3] == 200      # 走行可 -> 不透明
    assert rgba[0, 0, 3] == 0        # 範囲外 -> 透明


def test_keep_largest_drops_islands():
    # 大きな走行可ブロック + 離れた小島。keep_largest で小島が消える。
    cost = np.full((60, 60), 1000.0, dtype=np.float32)
    cost[5:40, 5:40] = 10.0     # 大ブロック
    cost[50:54, 50:54] = 10.0   # 小島
    transform = from_origin(0.0, 60.0, 1.0, 1.0)
    without, s0 = generate_drivable(cost, transform, threshold=100, close_m=0, open_m=0, min_area_m2=0)
    with_kl, s1 = generate_drivable(cost, transform, threshold=100, close_m=0, open_m=0, min_area_m2=0, keep_largest=True)
    assert s0["island_count"] >= 2
    assert s1["island_count"] == 1
    assert with_kl.sum() < without.sum()


def test_fill_small_holes():
    # 走行可ブロックの中に小さな穴。max_hole_m2 で埋まり hole_count=0 に。
    cost = np.full((40, 40), 10.0, dtype=np.float32)
    cost[18:22, 18:22] = 1000.0  # 4x4=16m² の穴
    transform = from_origin(0.0, 40.0, 1.0, 1.0)
    no_fill, s0 = generate_drivable(cost, transform, threshold=100, close_m=0, open_m=0, min_area_m2=0)
    filled, s1 = generate_drivable(cost, transform, threshold=100, close_m=0, open_m=0, min_area_m2=0, max_hole_m2=50)
    assert s0["hole_count"] >= 1
    assert s1["hole_count"] == 0
    assert filled.sum() > no_fill.sum()
