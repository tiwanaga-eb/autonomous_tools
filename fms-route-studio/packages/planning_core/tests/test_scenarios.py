from planning_core.scenarios import build_scenarios, run_hybrid


def _scn(name):
    return next(s for s in build_scenarios() if s["name"] == name)


def test_scenarios_build():
    scns = build_scenarios()
    assert len(scns) >= 6
    assert {"haul_straight", "switchback", "hairpin", "narrow_bench"} <= {s["name"] for s in scns}


def test_haul_straight_feasible_all():
    scn = _scn("haul_straight")
    for vid in ("HD785", "HM400", "CD110R"):
        assert run_hybrid(scn, vid, allow_reverse=False, enforce_footprint=True)["ok"], vid


def test_switchback_needs_reverse():
    scn = _scn("switchback")
    assert not run_hybrid(scn, "HM400", allow_reverse=False, enforce_footprint=True)["ok"]
    assert run_hybrid(scn, "HM400", allow_reverse=True, enforce_footprint=True)["ok"]


def test_narrow_bench_vehicle_dependent():
    # 幅6m: HM400(幅3.45)は通る。HD785(幅5.53)は footprint ON だとタイトで不可になり得る。
    scn = _scn("narrow_bench")
    assert run_hybrid(scn, "HM400", allow_reverse=False, enforce_footprint=True)["ok"]
