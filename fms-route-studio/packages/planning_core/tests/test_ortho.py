"""ortho.points_to_ortho / auto_ortho_res（LAS→オルソのグリッド化）。"""
import numpy as np

from planning_core.ortho import auto_ortho_res, points_to_ortho


def test_points_to_ortho_rgb_mean_and_transform():
    """RGB 平均色・north-up transform・nodata=(0,0,0) を検証。"""
    # 2x2m を 1m 解像度で: セル(0,0)=左上(y大側)。3点を左上セルへ、1点を右下セルへ。
    x = np.array([0.2, 0.4, 0.3, 1.5])
    y = np.array([1.6, 1.8, 1.7, 0.4])
    z = np.zeros(4)
    rgb = np.array([[100, 0, 0], [200, 0, 0], [150, 0, 0], [0, 50, 250]], dtype=np.uint8)
    img, tr = points_to_ortho(x, y, z, rgb, res=1.0)
    assert img.shape == (3, 2, 2)
    assert tr.a == 1.0 and tr.e == -1.0 and tr.c == 0.2 and tr.f == 1.8  # north-up, 原点=左上
    assert img[0, 0, 0] == 150  # 左上セル: R平均 (100+200+150)/3
    assert tuple(img[:, 1, 1]) == (0, 50, 250)  # 右下セル: 単独点の色
    assert tuple(img[:, 0, 1]) == (0, 0, 0)  # 点なしセル = nodata 黒
    assert tuple(img[:, 1, 0]) == (0, 0, 0)


def test_points_to_ortho_z_gray_fallback():
    """RGB 無し → Z の正規化グレー（3band 複製・点なしセルは 0）。"""
    x = np.array([0.5, 1.5, 0.5])
    y = np.array([0.5, 0.5, 1.5])
    z = np.array([100.0, 200.0, 150.0])
    img, _tr = points_to_ortho(x, y, z, None, res=1.0)
    assert img.shape == (3, 2, 2)
    assert np.array_equal(img[0], img[1]) and np.array_equal(img[1], img[2])  # グレー複製
    lo = img[0, 1, 0]   # z=100 のセル（下段左）
    hi = img[0, 1, 1]   # z=200 のセル（下段右）
    assert hi > lo  # 標高が高いほど明るい
    assert img[0, 0, 1] == 0  # 点なしセル


def test_auto_ortho_res_scales_with_density_and_caps_dim():
    # 高密度 → 細かい解像度（下限 0.05）
    assert auto_ortho_res(10_000_000, 400 * 400) <= 0.2
    # 低密度 → 粗い解像度（上限 1.0）
    assert auto_ortho_res(1_000, 400 * 400) == 1.0
    # 巨大サイト → max_dim を超えない解像度に強制
    r = auto_ortho_res(100_000_000, 10_000 * 10_000, max_dim=8192)
    assert 10_000 / r <= 8192 + 1
