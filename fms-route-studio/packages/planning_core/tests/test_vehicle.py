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
