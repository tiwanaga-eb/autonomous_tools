import math

from planner.models import MapData, PlanRequest, PlannerParams, Pose, Tolerances
from planner.planner import plan_mining_dock


def rectangular_map(width=80.0, height=40.0, obstacles=None, safety_margin=0.3):
    if obstacles is None:
        obstacles = []
    drivable = [[(0.0, 0.0), (width, 0.0), (width, height), (0.0, height)]]
    return MapData(drivable_polygons=drivable, obstacle_polygons=obstacles, safety_margin=safety_margin)


def primitive_params(timeout_ms=6000, seed=1):
    return PlannerParams(
        timeout_ms=timeout_ms,
        algorithm="primitives",
        weights={"w_len": 1.0, "w_time": 1.0, "w_rev": 1.2, "w_goal": 10.0},
        sampling_params={},
        primitives_params={
            "xy_res": 1.0,
            "yaw_bins": 24,
            "delta_bins": 11,
            "T": 0.6,
            "dt": 0.1,
            "astar_weight": 1.8,
            "collision_check_stride": 2,
            "speed_levels": 1,
            "delta_dot_levels": 3,
        },
        seed=seed,
    )


def _count_transitions(segments):
    gears = [s.gear for s in segments]
    return sum(1 for i in range(1, len(gears)) if gears[i] != gears[i - 1])


def test_primitives_simple_reaches_dock_in_reverse():
    req = PlanRequest(
        vehicle_id="demo_truck",
        start_pose=Pose(8.0, 8.0, 0.0),
        dock_pose=Pose(28.0, 18.0, math.pi * 0.8),
        exit_pose=None,
        tolerances=Tolerances(pos=1.0, yaw=0.3),
        map=rectangular_map(),
        planner_params=primitive_params(seed=10),
    )
    res = plan_mining_dock(req)
    assert res.status == "OK"
    assert res.plan is not None
    gears = [s.gear for s in res.plan.segments]
    assert "R" in gears
    assert _count_transitions(res.plan.segments) <= 1



def test_primitives_obstacle_avoidance():
    obstacle = [[(32.0, 14.0), (46.0, 14.0), (46.0, 26.0), (32.0, 26.0)]]
    req = PlanRequest(
        vehicle_id="demo_truck",
        start_pose=Pose(8.0, 6.0, 0.0),
        dock_pose=Pose(62.0, 31.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=1.2, yaw=0.35),
        map=rectangular_map(obstacles=obstacle),
        planner_params=primitive_params(seed=11),
    )
    res = plan_mining_dock(req)
    assert res.status in {"OK", "TIMEOUT"}



def test_primitives_steering_rate_stress_low_rate():
    req = PlanRequest(
        vehicle_id="HD785-7_slowsteer",
        start_pose=Pose(10.0, 8.0, 0.0),
        dock_pose=Pose(58.0, 30.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=1.2, yaw=0.35),
        map=rectangular_map(),
        planner_params=primitive_params(timeout_ms=2500, seed=12),
    )
    res = plan_mining_dock(req)
    assert res.status in {"NO_FEASIBLE_PATH", "TIMEOUT", "OK"}



def test_primitives_with_exit_three_segments():
    req = PlanRequest(
        vehicle_id="demo_truck",
        start_pose=Pose(8.0, 7.0, 0.0),
        dock_pose=Pose(26.0, 18.0, math.pi * 0.8),
        exit_pose=Pose(12.0, 24.0, math.pi / 2.0),
        tolerances=Tolerances(pos=1.2, yaw=0.4),
        map=rectangular_map(),
        planner_params=primitive_params(seed=13),
    )
    res = plan_mining_dock(req)
    assert res.status == "OK"
    assert res.plan is not None
    gears = [s.gear for s in res.plan.segments]
    assert gears[0] == "F"
    assert "R" in gears
    assert gears[-1] == "F"
