import numpy as np

from planning_core.fleet import detect_conflicts


def _line(p0, p1, n=60):
    return np.column_stack([np.linspace(p0[0], p1[0], n), np.linspace(p0[1], p1[1], n)])


def test_crossing_conflict_detected():
    """直交する2経路は交差点で competition。弧長範囲が交点付近に出る。"""
    a = _line((0, 0), (50, 0))
    b = _line((25, -25), (25, 25))
    cs = detect_conflicts([{"points": a, "half_width": 1.7}, {"points": b, "half_width": 1.7}], cell=0.5)
    assert len(cs) == 1
    c = cs[0]
    assert c.kind == "crossing"
    assert c.a_intervals and c.b_intervals
    # A 上の競合は x=25（弧長25付近）
    iv = c.a_intervals[0]
    assert iv.s_start < 25.0 < iv.s_end


def test_parallel_shared_corridor():
    """車幅で重なる併走（y=0 と y=2、半幅1.7同士でコリドー重なる）は shared・全長で競合。"""
    a = _line((0, 0), (50, 0))
    b = _line((0, 2), (50, 2))
    cs = detect_conflicts([{"points": a, "half_width": 1.7}, {"points": b, "half_width": 1.7}], cell=0.5)
    assert len(cs) == 1 and cs[0].kind == "shared"
    iv = cs[0].a_intervals[0]
    assert iv.s_start < 5 and iv.s_end > 45  # ほぼ全長


def test_disjoint_no_conflict():
    """十分離れた経路は競合しない。"""
    a = _line((0, 0), (50, 0))
    b = _line((0, 50), (50, 50))
    cs = detect_conflicts([{"points": a, "half_width": 1.7}, {"points": b, "half_width": 1.7}], cell=0.5)
    assert cs == []


def test_width_threshold_matters():
    """半幅が小さければ離れた併走は競合しない（車幅依存）。y=0 と y=4、半幅1.7（和3.4<4）→ 競合なし。"""
    a = _line((0, 0), (50, 0))
    b = _line((0, 4), (50, 4))
    narrow = detect_conflicts([{"points": a, "half_width": 1.7}, {"points": b, "half_width": 1.7}], cell=0.5)
    wide = detect_conflicts([{"points": a, "half_width": 2.5}, {"points": b, "half_width": 2.5}], cell=0.5)
    assert narrow == []          # 3.4 < 4 → 重ならない
    assert len(wide) == 1        # 5.0 > 4 → 重なる


def test_three_routes_pairwise():
    """3経路で総当たり。十字＋併走で複数ペアが出る。"""
    a = _line((0, 0), (50, 0))
    b = _line((25, -25), (25, 25))
    c = _line((0, 2), (50, 2))
    cs = detect_conflicts([{"points": a, "half_width": 1.7}, {"points": b, "half_width": 1.7},
                           {"points": c, "half_width": 1.7}], cell=0.5)
    pairs = {(x.a, x.b) for x in cs}
    assert (0, 1) in pairs and (0, 2) in pairs and (1, 2) in pairs
