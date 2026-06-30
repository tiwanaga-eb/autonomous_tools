"""Hybrid A* チューニング（障害物考慮ヒューリスティック / start・goal スナップ）のテスト。"""
import math

import numpy as np
from affine import Affine

from planning_core.planners import hybrid_astar
from planning_core.planners.hybrid_astar import _holonomic_field

H, W = 60, 60
T = Affine(1.0, 0.0, 0.0, 0.0, -1.0, float(H))  # col→x, row→y=H-row


def _l_corridor() -> np.ndarray:
    """L字コリドー: 水平アーム(y∈[6,14], x∈[2,52]) ＋ 垂直アーム(x∈[44,52], y∈[6,52])。"""
    m = np.zeros((H, W), np.uint8)
    for r in range(H):
        y = H - r - 0.5
        for c in range(W):
            x = c + 0.5
            if (6 <= y <= 14 and 2 <= x <= 52) or (44 <= x <= 52 and 6 <= y <= 52):
                m[r, c] = 1
    return m


def test_holonomic_field_follows_corridor():
    mask = _l_corridor()
    dist, cT = _holonomic_field(48.0, 48.0, mask, None, T, 1.0, 1e9)
    assert dist is not None
    # 角を回り込む地点(コーナー)の方が、直線距離が近い水平アーム遠端より「経路距離」は連続的に増える。
    def at(x, y):
        c, r = (~cT) * (x, y)
        return float(dist[int(math.floor(r)), int(math.floor(c))])
    # ゴール近傍 < コーナー付近 < 水平アーム遠端（経路に沿って単調に増える）
    assert at(48, 46) < at(48, 12) < at(6, 10)


def test_l_corridor_reachable():
    mask = _l_corridor()
    res = hybrid_astar(
        (5.0, 10.0, 0.0), (48.0, 48.0, math.pi / 2),
        rho=3.0, mask=mask, transform=T, xy_res=1.0,
        pos_tol=3.0, analytic_radius=10.0, max_iters=80000,
    )
    assert res is not None and res["status"] == "OK"


def test_snapping_recovers_offgrid_start():
    mask = _l_corridor()
    # start を水平アームの下(y=2, 領域外)に置く。最近傍の走行可能は y≈6。
    args = dict(rho=3.0, mask=mask, transform=T, xy_res=1.0, pos_tol=3.0, analytic_radius=10.0, max_iters=80000)
    # スナップなし → 即詰み（start から出るプリミティブが全て領域外）
    no_snap = hybrid_astar((6.0, 2.0, 0.0), (48.0, 48.0, math.pi / 2), max_snap_m=0.0, **args)
    # スナップあり → 領域内へ寄せて到達
    snapped = hybrid_astar((6.0, 2.0, 0.0), (48.0, 48.0, math.pi / 2), max_snap_m=6.0, **args)
    assert no_snap is None
    assert snapped is not None and snapped["status"] == "OK"
    # 始点が走行可能領域内へ寄っている
    x0, y0 = snapped["points"][0][0], snapped["points"][0][1]
    inv = ~T
    c = int(math.floor(inv.a * x0 + inv.b * y0 + inv.c))
    r = int(math.floor(inv.d * x0 + inv.e * y0 + inv.f))
    assert mask[r, c] != 0
