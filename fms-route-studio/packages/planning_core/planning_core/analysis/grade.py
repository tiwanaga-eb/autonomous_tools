"""縦断勾配（軌跡に沿った進行方向勾配, 設計書 §10）。

canonical DSM を軌跡 XY でバイリニア・サンプル → 進行方向の dz/ds [%]。
注: 3d_grade_validator の点群平面フィット（地表最大傾斜・方向非依存）は領域勾配検査用で、
軌跡の縦断勾配にはそのまま使わない（横断勾配込みで過大評価するため）。DSM が無ければ None。
"""
from __future__ import annotations

import numpy as np


def sample_bilinear(dsm: np.ndarray, transform, xy: np.ndarray) -> np.ndarray:
    """DSM(H,W) を ワールド座標 xy(N,2) でバイリニア・サンプル。範囲外/NaN は np.nan。"""
    xy = np.asarray(xy, float)
    inv = ~transform
    cols, rows = [], []
    for x, y in xy:
        c, r = inv * (x, y)
        cols.append(c)
        rows.append(r)
    cols = np.asarray(cols) - 0.5  # セル中心基準
    rows = np.asarray(rows) - 0.5
    h, w = dsm.shape
    out = np.full(len(xy), np.nan)
    c0 = np.floor(cols).astype(int)
    r0 = np.floor(rows).astype(int)
    for i in range(len(xy)):
        cc, rr = c0[i], r0[i]
        if cc < 0 or rr < 0 or cc + 1 >= w or rr + 1 >= h:
            continue
        fx = cols[i] - cc
        fy = rows[i] - rr
        z00 = dsm[rr, cc]
        z01 = dsm[rr, cc + 1]
        z10 = dsm[rr + 1, cc]
        z11 = dsm[rr + 1, cc + 1]
        if not np.isfinite([z00, z01, z10, z11]).all():
            # NaN を含む場合は最近傍にフォールバック
            cand = [z00, z01, z10, z11]
            finite = [z for z in cand if np.isfinite(z)]
            if not finite:
                continue
            out[i] = float(np.mean(finite))
            continue
        out[i] = float(
            z00 * (1 - fx) * (1 - fy)
            + z01 * fx * (1 - fy)
            + z10 * (1 - fx) * fy
            + z11 * fx * fy
        )
    return out


def _smooth_keep_nan(z: np.ndarray, win: int) -> np.ndarray:
    """微分前のノイズ抑制。Savitzky-Golay(2次)で平滑化＝直線/2次トレンドは厳密保存（定数勾配を歪めない）。
    NaN は有限点で内挿してから平滑化し、元が欠損の点は NaN に戻す（勾配評価から除外される）。"""
    z = np.asarray(z, float)
    finite = np.isfinite(z)
    n = int(finite.sum())
    if win < 3 or n < 3:
        return z
    from scipy.signal import savgol_filter

    w = win if win % 2 == 1 else win + 1          # 奇数
    maxw = len(z) if len(z) % 2 == 1 else len(z) - 1
    w = min(w, maxw)
    if w < 3:
        return z
    idx = np.arange(len(z))
    zfill = z.copy()
    if n < len(z):
        zfill = np.interp(idx, idx[finite], z[finite])  # 欠損は一時内挿
    sm = savgol_filter(zfill, w, 2)
    sm[~finite] = np.nan
    return sm


def grade_profile(xy: np.ndarray, s: np.ndarray, dsm: np.ndarray, transform, smooth_m: float = 8.0) -> np.ndarray:
    """軌跡に沿った縦断勾配 [%]（= 100 * dz/ds）。DSM 欠損点は np.nan。

    smooth_m: 微分前に z を平滑化する窓[m]（疎な/ノイジーな DSM の偽勾配スパイクを抑制。
    車両長スケール ~8m の実勾配を見る意味でも妥当）。0 で平滑化なし。
    """
    z = sample_bilinear(dsm, transform, xy)
    s = np.asarray(s, float)
    if len(z) < 2:
        return np.zeros(len(z))
    s_safe = s.copy()
    for i in range(1, len(s_safe)):
        if s_safe[i] <= s_safe[i - 1]:
            s_safe[i] = s_safe[i - 1] + 1e-9
    if smooth_m and smooth_m > 0 and len(z) >= 3:
        ds_med = float(np.median(np.diff(s_safe)))  # 平均サンプル間隔
        win = int(round(smooth_m / max(ds_med, 1e-6)))
        if win >= 3:
            z = _smooth_keep_nan(z, win)
    dz = np.gradient(z, s_safe)
    return dz * 100.0
