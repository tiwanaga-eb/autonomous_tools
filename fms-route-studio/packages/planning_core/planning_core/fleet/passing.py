"""すれ違い点（待避所 / passing bay）の幾何（複数台制御 Phase C）。

単線コリドーの対向(head-on)は単純な区間Mutexでは捌けない（先行が対向の待機点へ突っ込む）。
待避所＝本線から横へ退避できる場所を設け、譲る側がそこへ**横オフセット**で逸れて待ち、対向車が
本線を通過したら本線へ戻る。本モジュールは「経路の指定弧長位置で横へプルアウトする経路」を生成する。

`lateral_detour(points, s_center, offset, side, ...)`:
  s_center を中心に、本線中心線から左右(side)へ offset[m] だけ横退避し、ramp で滑らかに出入りする
  経路を返す。退避区間が本線コリドーから外れていれば、対向車は本線を通れる（衝突しない）。
座標は作業CRS(メートル)。
"""
from __future__ import annotations

import numpy as np


def _cum_s(pts: np.ndarray) -> np.ndarray:
    if len(pts) < 2:
        return np.zeros(len(pts))
    return np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])))])


def _normals(pts: np.ndarray) -> np.ndarray:
    """各点の左法線（進行方向に対し左+90°）単位ベクトル (N,2)。"""
    d = np.zeros_like(pts)
    if len(pts) >= 2:
        d[1:-1] = pts[2:] - pts[:-2]
        d[0] = pts[1] - pts[0]
        d[-1] = pts[-1] - pts[-2]
    norm = np.hypot(d[:, 0], d[:, 1])
    norm[norm < 1e-9] = 1.0
    tx, ty = d[:, 0] / norm, d[:, 1] / norm
    return np.column_stack([-ty, tx])  # 左法線


def _offset_profile(s: np.ndarray, s_center: float, offset: float, ramp: float, hold: float) -> np.ndarray:
    """弧長 s における横変位プロファイル d(s)。中心 hold 区間で offset、両側 ramp で 0→offset を平滑。"""
    half = hold / 2.0
    out = np.zeros_like(s)
    for i, si in enumerate(s):
        x = si - s_center
        ax = abs(x)
        if ax <= half:
            out[i] = offset
        elif ax <= half + ramp:
            u = (ax - half) / ramp           # 0..1
            out[i] = offset * 0.5 * (1.0 + np.cos(np.pi * u))  # cos ランプ（C1連続で滑らか）
        else:
            out[i] = 0.0
    return out


def lateral_detour(points, s_center: float, offset: float, side: int = 1, *,
                   ramp: float = 8.0, hold: float = 8.0) -> np.ndarray:
    """points の弧長 s_center を中心に、左右 side(+1=左/-1=右) へ offset[m] 横退避する経路を返す。

    ramp: 出入りの遷移長[m]（長いほど緩やか＝ピーキーにならない）。hold: 退避区間の平坦長[m]。
    端点・本線部は不変。退避区間だけ法線方向へずれる。
    """
    pts = np.asarray(points, float)
    if len(pts) < 3:
        return pts
    s = _cum_s(pts)
    nrm = _normals(pts)
    d = _offset_profile(s, float(s_center), float(offset) * (1 if side >= 0 else -1), ramp, hold)
    return pts + nrm * d[:, None]


def auto_bay_center(points, frac: float = 0.5) -> float:
    """経路の弧長 frac(0..1) の位置を待避所中心の弧長として返す（既定=中央）。"""
    pts = np.asarray(points, float)
    s = _cum_s(pts)
    return float(s[-1] * max(0.0, min(1.0, frac)))
