"""走行可能領域の **CV セグメンテーション**（実験的, 設計書 §12 拡張）。

従来の「コスト閾値を手で決める」方式に対し、OpenCV で cost マップ画像を自動セグメント化して
走行可能候補（基底マスク）を作る。出力は generate_drivable の `base_override` に渡し、以降の
クリアランス収縮・連結成分フィルタ・人手 include/exclude 編集は共通パイプラインを通す。

method:
  - "otsu":     Otsu 大域自動閾値（しきい値を自動決定。低コスト=走行可能）
  - "adaptive": 局所適応閾値（地形でコスト基準が変わる現場向け。ガウシアン適応）

cv2 は遅延 import（未導入環境では明示エラー）。AI/学習モデル版は将来の差し替え口とする。
"""
from __future__ import annotations

import numpy as np


def _cost_to_uint8(cost: np.ndarray, obstacle_value: float):
    """cost を 0..255 へ正規化（有効域の最大で割る）。返値 (img, valid)。obstacle/NaN は 255(=非走行)。"""
    c = np.asarray(cost, dtype=float)
    valid = np.isfinite(c) & (c < obstacle_value)
    cmax = float(c[valid].max()) if valid.any() else 1.0
    cmax = cmax if cmax > 1e-9 else 1.0
    norm = np.clip(c / cmax, 0.0, 1.0)
    norm = np.where(valid, norm, 1.0)
    return (norm * 255.0).astype(np.uint8), valid


def segment_drivable_base(
    cost: np.ndarray,
    transform,
    *,
    obstacle_value: float = 1e9,
    method: str = "otsu",
    blur_m: float = 0.5,
    block_m: float = 5.0,
    adaptive_c: float = 5.0,
) -> np.ndarray:
    """cost マップを CV セグメントして走行可能候補（bool 基底マスク）を返す。

    blur_m: 前処理ガウシアン半径[m]。block_m/adaptive_c: 適応閾値の窓[m]/オフセット。
    """
    try:
        import cv2
    except ImportError as e:  # pragma: no cover - 環境依存
        raise RuntimeError("OpenCV(cv2) が必要です。`pip install opencv-python-headless`") from e

    gs = abs(transform.a)
    img, valid = _cost_to_uint8(cost, obstacle_value)

    k = int(round(blur_m / gs))
    k = k * 2 + 1 if k > 0 else 1  # 奇数化
    if k >= 3:
        img = cv2.GaussianBlur(img, (k, k), 0)

    if method == "adaptive":
        bs = int(round(block_m / gs))
        bs = bs + 1 if bs % 2 == 0 else bs  # 奇数
        bs = max(3, bs)
        # 低コスト=走行可能 → INV。局所平均より adaptive_c 以上低いセルを走行可能に。
        bw = cv2.adaptiveThreshold(
            img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, bs, float(adaptive_c)
        )
        base = bw > 0
    else:  # otsu（大域自動閾値）
        _, bw = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        base = bw > 0

    return base & valid
