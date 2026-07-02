"""ラスタ整合ユーティリティ。

cost / drivable(mask) / DSM を「同一グリッド前提」で結合する箇所（hybrid A* 等は
1つの transform で複数ラスタを参照する）のための co-registration 検証。
shape 一致だけでは別ゾーン/別範囲の同サイズラスタを見逃すため、transform も照合する。
"""
from __future__ import annotations


def same_grid(t1, shape1, t2, shape2, tol: float = 1e-6) -> bool:
    """2つのラスタが同一グリッド（shape ＋ affine transform）かを判定する。

    tol は相対許容（浮動小数の書込/読出誤差を吸収）。transform が None のものは不一致扱い。
    """
    if t1 is None or t2 is None or tuple(shape1) != tuple(shape2):
        return False
    a = (t1.a, t1.b, t1.c, t1.d, t1.e, t1.f)
    b = (t2.a, t2.b, t2.c, t2.d, t2.e, t2.f)
    return all(abs(x - y) <= tol * max(1.0, abs(x), abs(y)) for x, y in zip(a, b))


MISREGISTERED_MSG = (
    "走行可能領域レイヤとコストマップのグリッド（位置・解像度）が一致しません。"
    "同じコストマップから生成した走行可能領域を選ぶか、レイヤの選択（作業ゾーン）を見直してください。"
)
