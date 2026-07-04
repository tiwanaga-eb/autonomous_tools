import math

import numpy as np
import pytest

from planning_core.earthworks import cone_from, min_area_rect, place_grid, plan_piles


def test_cone_volume_height_roundtrip():
    # 体積→高さ→体積 が一致（V = π h³ / (3 tan²φ)）
    spec = cone_from(volume_m3=24.0, repose_deg=37.0)
    assert spec.volume_m3 == pytest.approx(24.0)
    back = cone_from(height_m=spec.height_m, repose_deg=37.0)
    assert back.volume_m3 == pytest.approx(24.0, rel=1e-6)
    assert spec.radius_m == pytest.approx(spec.height_m / math.tan(math.radians(37.0)))
    # 安息角が緩いほど同体積で低く広い
    flat = cone_from(volume_m3=24.0, repose_deg=25.0)
    assert flat.height_m < spec.height_m and flat.radius_m > spec.radius_m


def test_cone_requires_one_of_volume_or_height():
    with pytest.raises(ValueError):
        cone_from()


def test_min_area_rect_matches_rotated_rectangle():
    # 30°回転した 40×10 矩形 → 主方向 30°・寸法 40×10 を復元
    ang = math.radians(30)
    base = np.array([[0, 0], [40, 0], [40, 10], [0, 10]], float)
    R = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
    poly = base @ R.T
    a, _cx, _cy, w, h = min_area_rect(poly)
    dims = sorted([w, h])
    assert dims[0] == pytest.approx(10.0, abs=1e-6)
    assert dims[1] == pytest.approx(40.0, abs=1e-6)
    assert math.degrees(a) % 90 == pytest.approx(30.0, abs=0.01)


def test_place_grid_fills_rect_from_edges():
    # 40×20 矩形・間隔 5m・マージン2.5 → x: 2.5..37.5 の8列, y: 2.5..17.5 の4行 = 32点
    poly = [[0, 0], [40, 0], [40, 20], [0, 20]]
    pts = place_grid(poly, 5.0, 5.0, edge_margin=2.5)
    assert len(pts) == 32
    xs = sorted(set(round(p[0], 3) for p in pts))
    assert xs[0] == pytest.approx(2.5) and xs[-1] == pytest.approx(37.5)


def test_place_grid_stagger_offsets_alternate_rows():
    poly = [[0, 0], [40, 0], [40, 20], [0, 20]]
    pts = place_grid(poly, 5.0, 5.0, edge_margin=2.5, stagger=True)
    rows = {}
    for x, y in pts:
        rows.setdefault(round(y, 3), []).append(round(x, 3))
    ys = sorted(rows)
    assert min(rows[ys[0]]) == pytest.approx(2.5)
    assert min(rows[ys[1]]) == pytest.approx(5.0)  # 千鳥: 2行目は dx/2 オフセット


def test_plan_piles_spacing_mode_rotated_area():
    # 回転エリアでも格子が主方向に整列して敷き詰められる
    ang = math.radians(20)
    base = np.array([[0, 0], [60, 0], [60, 30], [0, 30]], float)
    R = np.array([[math.cos(ang), -math.sin(ang)], [math.sin(ang), math.cos(ang)]])
    poly = (base @ R.T).tolist()
    out = plan_piles(poly, repose_deg=37.0, volume_m3=24.0, dx=8.0, dy=8.0)
    assert out["count"] > 0
    assert out["grid_angle_deg"] % 90 == pytest.approx(20.0, abs=0.1)
    # 基部半径ぶんのマージンが既定
    assert out["edge_margin_m"] == pytest.approx(out["pile"]["radius_m"], abs=1e-6)
    # 全ての中心がエリア内
    from planning_core.earthworks.piles import _dist_to_edges, _point_in_poly
    P = np.asarray(poly, float)
    for x, y in out["centers"]:
        assert _point_in_poly(x, y, P)
        # centers/margin とも mm 丸めで出力されるため許容 5mm
        assert _dist_to_edges(x, y, P) >= out["edge_margin_m"] - 5e-3


def test_plan_piles_spread_mode_count_and_spacing():
    # 撒き出しモード: V=24m³, t=0.5m → カバー48m²/パイル, d=√48≈6.93m, n=⌊A·t/V⌋
    poly = [[0, 0], [60, 0], [60, 30], [0, 30]]
    out = plan_piles(poly, repose_deg=37.0, volume_m3=24.0, spread_thickness_m=0.5)
    assert out["suggested_spacing_m"] == pytest.approx(math.sqrt(48.0), abs=1e-3)
    assert out["n_theory"] == int(60 * 30 * 0.5 / 24.0)  # 37
    assert out["spacing"]["dx_m"] == pytest.approx(out["suggested_spacing_m"])
    assert 0 < out["count"] <= out["n_theory"]


def test_plan_piles_edge_margin_zero_allows_boundary_centers():
    poly = [[0, 0], [20, 0], [20, 10], [0, 10]]
    out = plan_piles(poly, repose_deg=35.0, height_m=1.5, dx=5.0, edge_margin_m=0.0)
    xs = [c[0] for c in out["centers"]]
    assert min(xs) == pytest.approx(0.0, abs=1e-6)  # 端（縁）から詰める
