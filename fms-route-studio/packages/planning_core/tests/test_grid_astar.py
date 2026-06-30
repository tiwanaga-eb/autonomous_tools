import numpy as np
from affine import Affine

from planning_core.planners import coarsen_grid, plan_grid_astar, simplify_collinear, string_pull
from planning_core.planners.grid_astar import rc_to_world, world_to_rc


def _transform(cell=1.0, ox=0.0, oy=100.0):
    # north-up: y は上方向で減少（rasterio 既定）
    return Affine(cell, 0.0, ox, 0.0, -cell, oy)


def test_world_rc_roundtrip():
    t = _transform(cell=0.5, ox=10.0, oy=50.0)
    r, c = world_to_rc(t, 12.25, 47.75)
    x, y = rc_to_world(t, r, c)
    assert abs(x - 12.25) < 0.5 and abs(y - 47.75) < 0.5


def test_astar_routes_around_obstacle():
    n = 40
    cost = np.ones((n, n), float)
    # 中央に壁（縦）。一部に隙間を空けて迂回を強制
    cost[5:35, 20] = 1e9
    cost[18:22, 20] = 1.0  # gap
    t = _transform(cell=1.0, ox=0.0, oy=float(n))
    start = rc_to_world(t, 20, 2)
    goal = rc_to_world(t, 20, 38)
    path = plan_grid_astar(cost, t, start, goal, obstacle_value=1e9)
    assert len(path) >= 2
    assert np.allclose(path[0], start, atol=1e-6)
    assert np.allclose(path[-1], goal, atol=1e-6)


def test_drivable_hard_constraint_blocks():
    n = 20
    cost = np.ones((n, n), float)
    mask = np.zeros((n, n), dtype=np.uint8)
    mask[8:12, :] = 1  # 走行可能な水平帯のみ
    t = _transform(cell=1.0, ox=0.0, oy=float(n))
    start = rc_to_world(t, 10, 1)
    goal = rc_to_world(t, 10, 18)
    path = plan_grid_astar(cost, t, start, goal, drivable_mask=mask, obstacle_value=1e9)
    # 経路は帯内（row 8..11）に収まる
    for x, y in path:
        r, c = world_to_rc(t, x, y)
        assert 7 <= r <= 12


def test_simplify_collinear_reduces_points():
    pts = np.column_stack([np.arange(0, 10, 1.0), np.zeros(10)])
    out = simplify_collinear(pts)
    assert len(out) == 2  # 直線 → 端点のみ


def test_coarsen_reduces_grid_and_keeps_passability():
    n = 40
    cost = np.ones((n, n), float)
    mask = np.ones((n, n), np.uint8)
    cost[:, 20:24] = 1e9  # 縦の壁（factor幅以上 → 粗格子でも残る）
    mask[:, 20:24] = 0
    t = _transform(cell=0.5, ox=0.0, oy=float(n) * 0.5)
    cost_c, passable_c, t_c = coarsen_grid(cost, mask, t, factor=4, obstacle_value=1e9)
    assert cost_c.shape == (10, 10)
    assert abs(t_c.a) == 2.0  # 0.5m * 4
    # 壁の列(元col20-23 → 粗col5)は通行不可
    assert not passable_c[:, 5].any()


def test_bounded_snap_blocks_cross_fragment_fake_path():
    # 2つの分離した島。各点はそれぞれの島内だが連結していない → 偽の経路を作らない。
    n = 30
    cost = np.ones((n, n), float)
    mask = np.zeros((n, n), np.uint8)
    mask[5:9, 2:12] = 1   # 島A
    mask[20:24, 18:28] = 1  # 島B（Aと非連結）
    t = _transform(cell=1.0, ox=0.0, oy=float(n))
    start = rc_to_world(t, 7, 6)   # 島A内
    goal = rc_to_world(t, 22, 23)  # 島B内
    path = plan_grid_astar(cost, t, start, goal, drivable_mask=mask, obstacle_value=1e9, planner_cell_m=1.0)
    assert len(path) == 2  # 連結しない → フォールバック(2点)のみ＝偽経路なし


def test_bounded_snap_rejects_far_point():
    # goal が領域からはるか遠く（> max_snap_m）→ スナップ拒否で経路なし。
    n = 40
    cost = np.ones((n, n), float)
    mask = np.zeros((n, n), np.uint8)
    mask[5:35, 2:8] = 1  # 細い帯のみ走行可
    t = _transform(cell=1.0, ox=0.0, oy=float(n))
    start = rc_to_world(t, 20, 5)
    goal = rc_to_world(t, 20, 38)  # 帯から ~30m 離れている
    path = plan_grid_astar(cost, t, start, goal, drivable_mask=mask, obstacle_value=1e9, planner_cell_m=1.0, max_snap_m=4.0)
    assert len(path) == 2  # 遠すぎる goal はスナップ拒否 → 経路なし


def test_string_pull_straightens_staircase():
    # 自由空間の階段状ポリライン → 糸引きで端点間の直線（中間2点）に短絡
    n = 30
    mask = np.ones((n, n), np.uint8)
    t = _transform(cell=1.0, ox=0.0, oy=float(n))
    stair = []
    for k in range(10):
        x, y = t * (2 + k + 0.5, 15 + 0.5)
        stair.append([x, y])
        x, y = t * (2 + k + 0.5, 15 - k + 0.5)
        stair.append([x, y])
    out = string_pull(np.array(stair, float), mask, t)
    assert len(out) < len(stair)  # 短絡で点数が減る
    assert np.allclose(out[0], stair[0]) and np.allclose(out[-1], stair[-1])


def test_string_pull_respects_obstacle():
    # 障害物がある場合、見通せないので直線短絡せず曲がりを保持
    n = 30
    mask = np.ones((n, n), np.uint8)
    mask[10:20, 14:16] = 0  # 縦の壁（col14-15, row10-19）
    t = _transform(cell=1.0, ox=0.0, oy=float(n))
    # start(col15,row25)→goal(col15,row5) の直線は col15 を縦断＝壁を通る → 短絡不可
    pts = np.array([rc_to_world(t, 25, 15), rc_to_world(t, 15, 25), rc_to_world(t, 5, 15)], float)
    out = string_pull(pts, mask, t)
    assert len(out) == 3  # 中間点が保持される（直線では壁を通るため短絡不可）


def test_plan_grid_astar_coarsens_fine_grid():
    # 0.2m の細かい格子でも planner_cell_m で粗格子化して経路を返す
    n = 60
    cost = np.ones((n, n), float)
    cost[10:50, 30] = 1e9
    cost[28:32, 30] = 1.0  # gap
    t = _transform(cell=0.2, ox=0.0, oy=float(n) * 0.2)
    start = rc_to_world(t, 30, 5)
    goal = rc_to_world(t, 30, 55)
    path = plan_grid_astar(cost, t, start, goal, obstacle_value=1e9, planner_cell_m=0.8)
    assert len(path) >= 2
    assert np.allclose(path[0], start, atol=1e-6)
    assert np.allclose(path[-1], goal, atol=1e-6)
