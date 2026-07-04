"""LAS/LAZ point cloud reading (laspy wrapper).

設計書 §11: コストマップ生成の入力。座標CRSはヘッダから取れれば返す（無ければ None）。

巨大LAS対策(§19): `laspy.read()` はファイル全体をメモリ展開するため数億点でOOMする。
本モジュールは `laspy.open()` + `chunk_iterator()` でストリーミング読みし、点数が上限を
超える場合はチャンク内で系統間引き（stride抽出）してメモリを上限内に抑える。
"""
from __future__ import annotations

import os
from pathlib import Path

import laspy
import numpy as np

# 取り込み時の点数上限。これを超えるLASは系統間引きする（環境変数 FRS_LAS_MAX_POINTS で調整）。
# ※ アップロードのバイトサイズ上限(FRS_MAX_UPLOAD_MIB)とは別概念。
# - コストマップ用(read_las_xyz): ラスタ化に十分な密度を残しつつメモリ安全な上限。
# - 3D表示用(read_las_points): 呼び出し側でより小さい値（~20万点）を指定。
DEFAULT_MAX_POINTS = int(os.environ.get("FRS_LAS_MAX_POINTS", "8000000"))
# 1チャンクあたりの読み取り点数（メモリのピークを決める）。
CHUNK_SIZE = 2_000_000


def _parse_epsg(header) -> int | None:
    try:
        crs = header.parse_crs()
        if crs is not None:
            return crs.to_epsg()
    except Exception:
        pass
    return None


def _read_las_chunked(
    path: str | Path,
    max_points: int | None,
    want_rgb: bool,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray | None, int | None]:
    """LAS をチャンク読みし (x, y, z, rgb|None, epsg|None) を返す。

    max_points を超える場合は全体に渡って等間隔(系統)間引きする。NaN/Inf は除去。
    rgb は want_rgb かつ RGB次元がある場合のみ uint8(0-255) で返す（無ければ None）。
    """
    xs: list[np.ndarray] = []
    ys: list[np.ndarray] = []
    zs: list[np.ndarray] = []
    rs: list[np.ndarray] = []
    gs: list[np.ndarray] = []
    bs: list[np.ndarray] = []

    with laspy.open(str(path)) as reader:
        header = reader.header
        epsg = _parse_epsg(header)
        total = int(header.point_count)

        # 系統間引きの間隔。total<=max_points なら 1（間引きなし）。
        stride = 1
        if max_points and max_points > 0 and total > max_points:
            stride = total // max_points  # >=2

        has_rgb = False
        seen = 0  # これまでに走査した全点数（stride位相の基準）
        for chunk in reader.chunk_iterator(CHUNK_SIZE):
            n = len(chunk)
            if stride > 1:
                # グローバル位置 (seen+i) が stride の倍数になる局所インデックスを抽出
                start = int((-seen) % stride)
                sel = np.arange(start, n, stride)
            else:
                sel = np.arange(n)
            seen += n
            if sel.size == 0:
                continue

            x = np.asarray(chunk.x, dtype=float)[sel]
            y = np.asarray(chunk.y, dtype=float)[sel]
            z = np.asarray(chunk.z, dtype=float)[sel]
            valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            if not valid.all():
                x, y, z, sel = x[valid], y[valid], z[valid], sel[valid]
            if x.size == 0:
                continue

            xs.append(x)
            ys.append(y)
            zs.append(z)

            if want_rgb:
                dims = set(chunk.point_format.dimension_names)
                if {"red", "green", "blue"} <= dims:
                    has_rgb = True
                    rs.append(np.asarray(chunk.red, dtype=np.uint32)[sel])
                    gs.append(np.asarray(chunk.green, dtype=np.uint32)[sel])
                    bs.append(np.asarray(chunk.blue, dtype=np.uint32)[sel])

    if not xs:
        empty = np.empty((0,), dtype=float)
        return empty, empty.copy(), empty.copy(), None, epsg

    x = np.concatenate(xs)
    y = np.concatenate(ys)
    z = np.concatenate(zs)

    rgb: np.ndarray | None = None
    if want_rgb and has_rgb and rs:
        r = np.concatenate(rs)
        g = np.concatenate(gs)
        b = np.concatenate(bs)
        mx = int(max(r.max(initial=0), g.max(initial=0), b.max(initial=0)))
        if mx > 255:  # 16bit格納 → 8bitへ
            r, g, b = r >> 8, g >> 8, b >> 8
        rgb = np.clip(np.stack([r, g, b], axis=1), 0, 255).astype(np.uint8)

    # stride は floor なので結果が max_points を僅かに超えうる。最終トリムで上限を保証。
    if max_points and max_points > 0 and x.size > max_points:
        keep = np.linspace(0, x.size - 1, max_points).astype(np.int64)
        x, y, z = x[keep], y[keep], z[keep]
        if rgb is not None:
            rgb = rgb[keep]

    return x, y, z, rgb, epsg


def las_header_epsg(path: str | Path) -> int | None:
    """LAS ヘッダの CRS(EPSG) だけを読む（点データは読まない＝軽量。無ければ None）。"""
    try:
        with laspy.open(str(path)) as reader:
            return _parse_epsg(reader.header)
    except Exception:
        return None


def read_las_xyz(
    path: str | Path,
    max_points: int | None = DEFAULT_MAX_POINTS,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int | None]:
    """Return (x, y, z, epsg|None). NaN/Inf 点は除去。

    巨大LASはチャンク読み＋系統間引きで max_points 以内に抑える（既定8M点）。
    max_points=None で間引き無効（小～中規模で全点が要る場合）。
    """
    x, y, z, _rgb, epsg = _read_las_chunked(path, max_points, want_rgb=False)
    return x, y, z, epsg


def read_las_points(path: str | Path, max_points: int = 200000):
    """3D表示用に LAS を間引いて読む。返値 (xyz(N,3), rgb(N,3)uint8|None, epsg|None)。

    RGB があれば 0-255 に正規化（16bit格納は8bitへ）。max_points 超は等間隔(系統)間引き。
    """
    x, y, z, rgb, epsg = _read_las_chunked(path, max_points, want_rgb=True)
    xyz = np.column_stack([x, y, z]) if x.size else np.empty((0, 3), dtype=float)
    return xyz, rgb, epsg
