"""速度プロファイル v(s)（設計 Stage 6: 追従可能 trajectory）。

経路（弧長 s・曲率 κ・ギア）から、横加速度・前後加速度・ギア別速度上限・端点/切返での停止
を満たす速度プロファイルを前後2パスで生成する。これにより「経路」を **速度付き trajectory**
として配信できる（設計の Stage 6）。停止可能距離 = v^2/(2·a_decel) の算出にも使う。
"""
from __future__ import annotations

import numpy as np


def velocity_profile(s, kappa, gears, vehicle, grades=None, min_speed_mps: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Return (v[m/s], t[s])。s:弧長配列, kappa:曲率配列, gears:'F'/'R'/None 配列, vehicle:プロファイル。

    grades: 各点の縦断勾配[%]（あれば）。下り勾配では `max_speed_downhill_loaded` で頭打ち。
    `steer_rate_profile`（速度依存の最大操舵速度）があれば、操舵が追従できる速度に制限する。
    min_speed_mps>0: **最低速度フロア**[m/s]。クロール回避の運用要件で、内部点は最低この速度を保つ。
      端点・切り返し点は0、その近傍は加減速パスが 0↔フロア を滑らかにランプ（=「付近以外」で floor）。
      操舵レート制限が floor を下回る区間では floor が優先される（dκ/ds の平滑化で追従性を確保する想定）。
    """
    s = np.asarray(s, float)
    k = np.abs(np.asarray(kappa, float))
    n = len(s)
    if n == 0:
        return np.zeros(0), np.zeros(0)

    a_lat = float(getattr(vehicle, "max_lateral_accel", None) or 1.0)
    a_acc = float(getattr(vehicle, "max_accel", None) or 0.5)
    a_dec = float(getattr(vehicle, "max_decel", None) or 1.0)
    v_fwd = float(getattr(vehicle, "max_speed_fwd", None) or 5.0)
    v_rev = float(getattr(vehicle, "max_speed_rev", None) or 2.0)

    # 横加速度によるコーナー速度上限 v=sqrt(a_lat/κ)、ギア別上限で頭打ち
    vcap = np.sqrt(a_lat / np.maximum(k, 1e-6))
    gearcap = np.array([v_rev if g == "R" else v_fwd for g in gears], float) if gears is not None else np.full(n, v_fwd)
    v = np.minimum(vcap, gearcap)

    # #2 下り勾配の速度制限: grade<0（下り）は連続降坂リターダ性能上の最大速度(積荷=保守的)で頭打ち。
    v_dn = getattr(vehicle, "max_speed_downhill_loaded", None)
    if grades is not None and v_dn:
        g = np.asarray(grades, float)
        if g.shape == v.shape:
            downhill = np.isfinite(g) & (g < -1.0)  # -1%超の下り
            v = np.where(downhill, np.minimum(v, float(v_dn)), v)

    # #3 速度依存の最大操舵速度: 操舵角 δ≈atan(Lκ) の時間変化 dδ/dt ≤ steer_rate(v) を満たす速度に制限。
    #    dδ/dt = (L/(1+(Lκ)^2))·(dκ/ds)·v なので v ≤ steer_rate·(1+(Lκ)^2)/(L·|dκ/ds|)。
    #    steer_rate は速度依存（profile）のため2反復で近似。曲率変化が速い区間でのみ効く。
    prof = getattr(vehicle, "steer_rate_profile", None)
    L = getattr(vehicle, "wheel_base", None)
    if prof and L and L > 0 and n >= 3:
        s_safe = s.copy()
        for i in range(1, n):
            if s_safe[i] <= s_safe[i - 1]:
                s_safe[i] = s_safe[i - 1] + 1e-9
        dk = np.abs(np.gradient(k, s_safe))  # |dκ/ds|
        # 操舵レート制限は「実応答長(~1.5m)にわたる曲率変化率」で律速すべき。Dubins/RSのC-S接合の
        # 段差は細かいサンプリングだと巨大スパイク化し、数点だけ速度が急落する（離散化由来）。
        # dκ/ds を**弧長1.5m窓**で平滑化（非一様サンプリングに頑健）し、過減速を防ぐ（持続的な高dκ/dsは保持）。
        span = float(s_safe[-1] - s_safe[0])
        if span > 2.0:
            grid_ds = 0.1
            ng = int(span / grid_ds) + 1
            sg = np.linspace(float(s_safe[0]), float(s_safe[-1]), ng)
            dkg = np.interp(sg, s_safe, dk)
            w = int(round(1.5 / grid_ds))
            if w % 2 == 0:
                w += 1
            if w >= 3 and ng >= w:
                dkg = np.convolve(dkg, np.ones(w) / w, mode="same")
            dk = np.interp(s_safe, sg, dkg)
        spd = np.array([float(p[0]) for p in prof])  # km/h
        rate = np.array([float(p[1]) for p in prof])  # rad/s
        order = np.argsort(spd)
        spd, rate = spd[order], rate[order]
        for _ in range(2):
            sr = np.interp(v * 3.6, spd, rate)  # 現速度での最大操舵速度
            v_steer = sr * (1.0 + (L * k) ** 2) / (L * np.maximum(dk, 1e-9))
            v = np.minimum(v, v_steer)

    # 最低速度フロア（出発/到着/切返「付近以外」）: 内部点を min_speed 以上に引き上げる。
    # 端点・切返は次で0にし、その近傍は後段の加減速パスが 0→floor へランプ＝「付近」だけ低速。
    if min_speed_mps and min_speed_mps > 0:
        v = np.maximum(v, float(min_speed_mps))

    # 停止点: 始点・終点・ギア変化(切り返し)
    if n >= 1:
        v[0] = 0.0
        v[-1] = 0.0
    if gears is not None:
        for i in range(1, n):
            if gears[i] != gears[i - 1]:
                v[i] = 0.0
                v[i - 1] = 0.0

    # 前進パス（加速制限）
    for i in range(1, n):
        ds = max(s[i] - s[i - 1], 0.0)
        v[i] = min(v[i], float(np.sqrt(v[i - 1] ** 2 + 2.0 * a_acc * ds)))
    # 後進パス（減速制限）
    for i in range(n - 2, -1, -1):
        ds = max(s[i + 1] - s[i], 0.0)
        v[i] = min(v[i], float(np.sqrt(v[i + 1] ** 2 + 2.0 * a_dec * ds)))

    # 時間積分（区間平均速度。停止近傍は下限でゼロ割回避）
    t = np.zeros(n)
    for i in range(1, n):
        ds = max(s[i] - s[i - 1], 0.0)
        avg = max((v[i] + v[i - 1]) / 2.0, 0.05)
        t[i] = t[i - 1] + ds / avg
    return v, t


def stopping_distance(v_mps: float, vehicle) -> float:
    """速度 v からの停止可能距離 [m] = v^2/(2·a_decel)。"""
    a_dec = float(getattr(vehicle, "max_decel", None) or 1.0)
    return float(v_mps) ** 2 / (2.0 * max(a_dec, 1e-6))
