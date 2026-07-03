import numpy as np

from planning_core.analysis import stopping_distance, velocity_profile
from planning_core.models.vehicle import VehicleProfile


def _veh(**kw):
    base = dict(id="V", name="v", kinematic_type="tracked_skid", overall_length=5.0, overall_width=2.5,
                overall_height=2.5, min_turning_radius=0.0, max_speed_fwd=10.0, max_speed_rev=4.0,
                max_lateral_accel=1.5, max_accel=0.5, max_decel=1.0)
    base.update(kw)
    return VehicleProfile(**base)


def test_velocity_straight_starts_and_ends_at_zero():
    s = np.arange(0, 101, 1.0)
    kappa = np.zeros_like(s)
    v, t = velocity_profile(s, kappa, [None] * len(s), _veh())
    assert v[0] == 0.0 and v[-1] == 0.0
    assert v.max() <= 10.0 + 1e-6 and v.max() > 1.0
    assert t[-1] > 0 and np.all(np.diff(t) >= 0)


def test_velocity_respects_lateral_accel_in_corner():
    s = np.arange(0, 101, 1.0)
    kappa = np.zeros_like(s)
    kappa[50] = 0.1  # R=10m コーナー → vcap=sqrt(1.5/0.1)=~3.87
    v, _ = velocity_profile(s, kappa, [None] * len(s), _veh())
    assert v[50] <= 3.9


def test_velocity_stops_at_cusp():
    s = np.arange(0, 21, 1.0)
    gears = ["F"] * 11 + ["R"] * 10  # index 11 でギア変化（切り返し）
    v, _ = velocity_profile(s, np.zeros_like(s), gears, _veh())
    assert v[11] == 0.0 and v[10] == 0.0


def test_stopping_distance_formula():
    assert abs(stopping_distance(10.0, _veh(max_decel=1.0)) - 50.0) < 1e-6


def test_velocity_min_speed_floor_in_interior_with_stop_ramps():
    """最低速度フロア: 内部は floor 以上、端点は0（付近は加減速で滑らかにランプ）。"""
    s = np.arange(0, 101, 1.0)
    kappa = np.zeros_like(s)
    floor = 1.39  # ≈5km/h
    v, t = velocity_profile(s, kappa, [None] * len(s), _veh(), min_speed_mps=floor)
    assert v[0] == 0.0 and v[-1] == 0.0           # 端点は停止
    interior = v[(s >= 5.0) & (s <= s[-1] - 5.0)]  # 端点付近(ランプ)を除く内部
    assert interior.min() >= floor - 1e-6          # 内部はフロア以上
    # フロア無しでは曲率0でも上限まで上がるが、ここでは内部が floor で律速されないこと（上限>floor）
    v0, _ = velocity_profile(s, kappa, [None] * len(s), _veh(), min_speed_mps=0.0)
    assert v0[(s >= 5.0) & (s <= s[-1] - 5.0)].max() > floor


def test_velocity_steer_rate_limit_is_tight_not_locked_in():
    """操舵レート制限は「v ≤ sr(v)·gain の最大 v」に一致すべき（旧: 初回の高速側 sr で過小固着）。

    高い dκ/ds 区間で: (1) 制約は満たす（妥当性）(2) 5% 引き上げると制約違反になる（タイト性）
    ＝過小評価で不必要に遅くしていないことを検証する。
    """
    prof = [[0, 0.44], [15, 0.16], [30, 0.04], [55, 0.01]]  # HM400 相当（速度単調減少）
    veh = _veh(kinematic_type="rigid_bicycle", wheel_base=4.35, min_turning_radius=8.9,
               max_speed_fwd=4.65, steer_rate_profile=prof)
    # 一定の高 dκ/ds を持つ弧: κ を 0→0.08 まで線形に上げる（dκ/ds=0.008/m…平滑化後も持続）
    s = np.arange(0, 201, 1.0)
    kappa = np.clip(0.0008 * s, 0.0, 0.08)
    v, _ = velocity_profile(s, kappa, ["F"] * len(s), veh)
    L = 4.35
    spd = np.array([p[0] for p in prof], float)
    rate = np.array([p[1] for p in prof], float)
    # κ 線形区間の内部（加減速ランプ・クリップ折れ点の外）で妥当性＋タイト性を確認。
    # dκ/ds は κ=0.0008·s の持続勾配（1.5m 平滑化でも不変）。
    dk = np.full_like(s, 0.0008)
    inner = (s > 30) & (s < 90)
    assert inner.any()
    gain = (1.0 + (L * np.abs(kappa)) ** 2) / (L * dk)
    sr_at = np.interp(v * 3.6, spd, rate)
    # (1) 妥当性: v は操舵が追従できる範囲
    assert np.all(v[inner] <= sr_at[inner] * gain[inner] * 1.05)
    # (2) タイト性: 操舵律速（上限未満）の点では、5% 上げると追従不能（＝過小固着していない）
    lim = inner & (v < veh.max_speed_fwd * 0.95)
    if lim.any():
        vv = v * 1.05
        sr2 = np.interp(vv * 3.6, spd, rate)
        tight = vv > sr2 * gain + 1e-9
        assert (tight[lim].sum() / lim.sum()) > 0.8
