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
