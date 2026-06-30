import numpy as np

from planning_core.costmap import build_costmap_arrays, cost_to_rgb, cost_to_rgba
from planning_core.models import CostmapParams


def _grid(zfun, xmax=10.0, ymax=8.0, step=0.25):
    xs = np.arange(0.0, xmax + step, step)
    ys = np.arange(0.0, ymax + step, step)
    X, Y = np.meshgrid(xs, ys)
    Z = zfun(X, Y)
    return X.ravel(), Y.ravel(), Z.ravel()


def test_flat_terrain_low_cost_and_dsm():
    x, y, z = _grid(lambda X, Y: np.full_like(X, 5.0))
    out = build_costmap_arrays(x, y, z, CostmapParams(grid_size_m=0.5))
    cost = out["cost"]
    assert cost.dtype == np.float32                  # 生cost = float32
    assert out["dsm"].shape == cost.shape            # DSM 同時出力
    finite = cost[~out["nodata_mask"]]
    assert finite.max() < 1.0                        # 平坦 -> ほぼ0コスト


def test_ramp_slope_calibrated_and_obstacle():
    x, y, z = _grid(lambda X, Y: 0.5 * X)            # 0.5 m/m の斜面
    out = build_costmap_arrays(x, y, z, CostmapParams(grid_size_m=0.5, slope_limit_deg=10.0))
    interior = out["slope"][2:-2, 2:-2]
    assert 0.3 < np.nanmedian(interior) < 0.7        # np.gradient で物理 m/m が出る
    # arctan(0.5)=26.6deg > 10deg -> 不可侵セルが多数
    assert np.sum(out["cost"] >= 1e9) > interior.size * 0.2


def test_colorize_is_display_only_rgb():
    x, y, z = _grid(lambda X, Y: 0.1 * X)
    out = build_costmap_arrays(x, y, z, CostmapParams(grid_size_m=0.5))
    rgb = cost_to_rgb(out["cost"], nodata_mask=out["nodata_mask"])
    h, w = out["cost"].shape
    assert rgb.shape == (h, w, 3)
    assert rgb.dtype == np.uint8


def test_cost_to_rgba_fills_holes_and_alpha():
    cost = np.full((12, 12), 100.0, dtype=np.float32)
    nodata = np.zeros((12, 12), dtype=bool)
    # 内側の穴（点群欠損セル）
    nodata[6, 6] = True
    cost[6, 6] = 1e9
    # 外周1行は範囲外（footprint外）
    nodata[0, :] = True
    cost[0, :] = 1e9

    rgba = cost_to_rgba(cost, nodata, vmax=200.0)
    assert rgba.shape == (12, 12, 4)
    # 内側の穴は footprint 内 -> 不透明(埋められる)
    assert rgba[6, 6, 3] == 255
    # 範囲外(上端)は透明
    assert rgba[0, 0, 3] == 0


def test_sparse_interpolation_not_all_zero():
    # 疎な点群(平面 z=0.2x, slope≈11.3°) を細かいgridで → 補間DTMで slope/cost が出ること
    rng = np.random.RandomState(0)
    x = rng.uniform(0.0, 20.0, 300)
    y = rng.uniform(0.0, 20.0, 300)
    z = 0.2 * x
    out = build_costmap_arrays(x, y, z, CostmapParams(grid_size_m=0.25, slope_limit_deg=15.0))
    cost = out["cost"]
    finite = cost[cost < 1e9]
    assert finite.size > 0
    # 全0(=全緑)にならず、傾斜由来のコストを持つ
    assert finite.max() > 50.0
