from planning_core.vehicle import list_builtins, load_builtin


def test_all_builtins_load():
    ids = {p.id for p in list_builtins()}
    assert ids == {"HD785", "HD605", "HM400", "CD110R"}


def test_hd785_rigid_bicycle():
    p = load_builtin("HD785")
    assert p.kinematic_type == "rigid_bicycle"
    assert p.wheel_base == 4.95
    assert p.spec_status == "measured"
    assert p.supports_steering_angle()


def _fp_extents(poly):
    xs = [pt[0] for pt in poly]
    return max(xs), -min(xs)  # (前端+, 後端の大きさ)


def test_hd_reference_is_rear_axle():
    """HD系の footprint 基準点は後輪車輪軸中心＝前後非対称（前端>後端、後端は車体中心より後ろ寄り）。

    ユーザー指定値: HD785 前+7.825/後-3.190, HD605 前+7.280/後-3.070。
    基準点が車体中心なら前端=後端になる（対称）ので、非対称＝後輪軸基準であることを固定する。
    """
    expect = {"HD785": (7.825, 3.190), "HD605": (7.280, 3.070)}
    for vid, (efront, erear) in expect.items():
        p = load_builtin(vid)
        assert p.footprint_polygon is not None
        front, rear = _fp_extents(p.footprint_polygon)
        assert abs(front - efront) < 1e-6, f"{vid} front {front}"
        assert abs(rear - erear) < 1e-6, f"{vid} rear {rear}"
        assert front > rear                                   # 前後非対称（後輪軸基準）
        assert front > p.wheel_base                            # 前バンパーは前輪軸より前
        # footprint_radius は基準点から最遠コーナー（前端側）と整合
        import math
        hw = p.overall_width / 2.0
        assert abs(p.footprint_radius - math.hypot(front, hw)) < 0.05


def test_cd110r_tracked_skid():
    p = load_builtin("CD110R")
    assert p.kinematic_type == "tracked_skid"
    assert p.can_turn_in_place is True
    assert p.wheel_base is None                 # 自転車モデル不可
    assert not p.supports_steering_angle()


def test_hm400_articulated_footprint():
    p = load_builtin("HM400")
    assert p.kinematic_type == "articulated"
    assert p.footprint_polygon is not None
    assert len(p.footprint_polygon) == 4
    assert all(len(pt) == 2 for pt in p.footprint_polygon)


def test_loaded_decel_is_conservative():
    """積載時減速度は空車値以下（fleet の予約距離 v²/(2·decel) を保守側にする前提）。"""
    for p in list_builtins():
        assert p.max_decel is not None
        assert p.max_decel_loaded is not None, f"{p.id}: max_decel_loaded 未設定"
        assert p.max_decel_loaded <= p.max_decel, f"{p.id}: 積載時減速度が空車値を超えている"
