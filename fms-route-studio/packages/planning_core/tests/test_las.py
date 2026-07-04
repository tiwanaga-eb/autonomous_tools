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


def test_resolve_las_epsg_priority():
    """CRS 解決の優先順位: 明示指定 > ヘッダ > 経緯度ヒューリスティック > fallback。"""
    import numpy as np

    from planning_core.io import looks_like_lonlat, resolve_las_epsg

    lon = np.array([139.5, 139.6])
    lat = np.array([35.1, 35.2])
    metric_x = np.array([30000.0, 30010.0])
    metric_y = np.array([119000.0, 119010.0])

    # 明示指定が最優先（ヘッダ・座標範囲より強い）
    assert resolve_las_epsg(6677, lon, lat, override=6675) == (6675, "specified")
    # ヘッダがあればそれを使う（経緯度らしい座標でも上書きしない）
    assert resolve_las_epsg(4326, metric_x, metric_y) == (4326, "header")
    # ヘッダ無し＋全点が経緯度範囲 → WGS84 と推定
    assert resolve_las_epsg(None, lon, lat) == (4326, "assumed_wgs84")
    # ヘッダ無し＋メートル座標 → fallback（無指定なら None=作業CRSのまま）
    assert resolve_las_epsg(None, metric_x, metric_y, fallback=6677) == (6677, "assumed_working")
    assert resolve_las_epsg(None, metric_x, metric_y) == (None, "assumed_working")
    # ヒューリスティック単体
    assert looks_like_lonlat(lon, lat) is True
    assert looks_like_lonlat(metric_x, metric_y) is False
    assert looks_like_lonlat(np.array([]), np.array([])) is False
