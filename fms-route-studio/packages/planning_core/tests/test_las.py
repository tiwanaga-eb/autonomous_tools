import laspy
import numpy as np

from planning_core.costmap import build_costmap_arrays
from planning_core.io import read_las_points, read_las_xyz
from planning_core.models import CostmapParams


def _write_las(path, x, y, z):
    header = laspy.LasHeader(point_format=3, version="1.4")
    header.offsets = [float(x.min()), float(y.min()), float(z.min())]
    header.scales = [0.001, 0.001, 0.001]
    las = laspy.LasData(header)
    las.x = x
    las.y = y
    las.z = z
    las.write(str(path))


def test_read_las_and_build_costmap(tmp_path):
    xs = np.linspace(0.0, 10.0, 60)
    ys = np.linspace(0.0, 8.0, 50)
    X, Y = np.meshgrid(xs, ys)
    Z = 0.1 * X  # 緩い斜面
    x, y, z = X.ravel(), Y.ravel(), Z.ravel()

    p = tmp_path / "cloud.las"
    _write_las(p, x, y, z)

    rx, ry, rz, epsg = read_las_xyz(p)
    assert rx.size == 3000
    assert abs(rx.min()) < 0.05 and abs(rx.max() - 10.0) < 0.05
    assert epsg is None  # ヘッダにCRS無し

    out = build_costmap_arrays(rx, ry, rz, CostmapParams(grid_size_m=0.5))
    assert out["cost"].dtype == np.float32
    assert out["dsm"].shape == out["cost"].shape


def test_read_las_downsamples_when_huge(tmp_path):
    """巨大LASを想定: max_points を超える入力は系統間引きで上限内に収め、
    かつ空間カバレッジ（bbox）を保つ。チャンク境界をまたぐ点数で検証。"""
    n = 50_000
    rng = np.random.default_rng(0)
    x = rng.uniform(0.0, 100.0, n)
    y = rng.uniform(0.0, 80.0, n)
    z = rng.uniform(0.0, 5.0, n)

    p = tmp_path / "big.las"
    _write_las(p, x, y, z)

    # 間引きあり: 上限を超えない & bbox はほぼ保たれる
    rx, ry, rz, _ = read_las_xyz(p, max_points=1000)
    assert rx.size <= 1000
    assert rx.min() < 5.0 and rx.max() > 95.0
    assert ry.min() < 5.0 and ry.max() > 75.0

    # 間引き無効(None)なら全点
    fx, _, _, _ = read_las_xyz(p, max_points=None)
    assert fx.size == n

    # 表示用も同様に上限内
    xyz, _rgb, _ = read_las_points(p, max_points=2000)
    assert xyz.shape[0] <= 2000 and xyz.shape[1] == 3
