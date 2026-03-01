import math

from planner.models import MapData, PlanRequest, PlannerParams, Pose, Tolerances
from planner.planner import plan_mining_dock


def rectangular_map(width=80.0, height=40.0, obstacles=None, safety_margin=0.3):
    if obstacles is None:
        obstacles = []
    drivable = [[(0.0, 0.0), (width, 0.0), (width, height), (0.0, height)]]
    return MapData(drivable_polygons=drivable, obstacle_polygons=obstacles, safety_margin=safety_margin)


def make_params(seed=7):
    return PlannerParams(
        timeout_ms=2500,
        weights={"w_len": 1.0, "w_time": 1.0, "w_rev": 2.0, "w_switch_yaw": 0.5, "w_goal": 5.0},
        sampling_params={"switch_sample_count": 1200, "yaw_bins": 32, "step_size": 0.6, "collision_stride": 2},
        yaw_pref=0.0,
        seed=seed,
    )


def count_gear_switches(segments):
    gears = [s.gear for s in segments]
    return sum(1 for i in range(1, len(gears)) if gears[i] != gears[i - 1])


def test_simple_docking_no_obstacle():
    req = PlanRequest(
        vehicle_id="HD785-7",
        start_pose=Pose(10.0, 8.0, 0.0),
        dock_pose=Pose(58.0, 30.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=0.1, yaw=0.0873),
        map=rectangular_map(),
        planner_params=make_params(seed=10),
    )
    res = plan_mining_dock(req)
    assert res.status == "OK"
    assert res.plan is not None
    assert len(res.plan.segments) == 2
    assert [s.gear for s in res.plan.segments] == ["F", "R"]
    assert count_gear_switches(res.plan.segments) == 1
    assert res.plan.metrics.dock_error_pos <= req.tolerances.pos
    assert res.plan.metrics.dock_error_yaw <= req.tolerances.yaw
    assert res.plan.metrics.dynamic_feasible is True


def test_obstacle_avoidable():
    obstacle = [[(32.0, 14.0), (44.0, 14.0), (44.0, 26.0), (32.0, 26.0)]]
    req = PlanRequest(
        vehicle_id="HD785-7",
        start_pose=Pose(8.0, 6.0, 0.0),
        dock_pose=Pose(62.0, 31.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=0.1, yaw=0.0873),
        map=rectangular_map(obstacles=obstacle),
        planner_params=make_params(seed=3),
    )
    res = plan_mining_dock(req)
    assert res.status == "OK"
    assert res.plan is not None
    assert res.plan.metrics.collision_free is True


def test_no_feasible_path_blocked():
    obstacle = [[(-1.0, 17.5), (81.0, 17.5), (81.0, 22.5), (-1.0, 22.5)]]
    req = PlanRequest(
        vehicle_id="HD785-7",
        start_pose=Pose(8.0, 8.0, 0.0),
        dock_pose=Pose(62.0, 32.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=0.1, yaw=0.0873),
        map=rectangular_map(obstacles=obstacle),
        planner_params=make_params(seed=2),
    )
    res = plan_mining_dock(req)
    assert res.status in {"NO_FEASIBLE_PATH", "TIMEOUT"}


def test_with_exit_pose_three_segments():
    req = PlanRequest(
        vehicle_id="HD785-7",
        start_pose=Pose(8.0, 7.0, 0.0),
        dock_pose=Pose(55.0, 30.0, math.pi),
        exit_pose=Pose(15.0, 35.0, math.pi / 2.0),
        tolerances=Tolerances(pos=0.1, yaw=0.0873),
        map=rectangular_map(),
        planner_params=make_params(seed=4),
    )
    res = plan_mining_dock(req)
    assert res.status == "OK"
    assert res.plan is not None
    assert len(res.plan.segments) == 3
    assert [s.gear for s in res.plan.segments] == ["F", "R", "F"]


def test_invalid_input_start_outside_drivable():
    req = PlanRequest(
        vehicle_id="HD785-7",
        start_pose=Pose(-3.0, -1.0, 0.0),
        dock_pose=Pose(50.0, 30.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=0.1, yaw=0.0873),
        map=rectangular_map(),
        planner_params=make_params(seed=11),
    )
    res = plan_mining_dock(req)
    assert res.status == "INVALID_INPUT"


def test_steering_feasible_high_rate():
    req = PlanRequest(
        vehicle_id="HD785-7",
        start_pose=Pose(10.0, 8.0, 0.0),
        dock_pose=Pose(58.0, 30.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=0.1, yaw=0.0873),
        map=rectangular_map(),
        planner_params=PlannerParams(
            timeout_ms=2500,
            weights={"w_len": 1.0, "w_time": 1.0, "w_rev": 2.0, "w_switch_yaw": 0.5, "w_goal": 5.0},
            sampling_params={
                "switch_sample_count": 1200,
                "yaw_bins": 32,
                "step_size": 0.6,
                "collision_stride": 2,
                "min_speed_threshold": 0.1,
            },
            yaw_pref=0.0,
            seed=12,
        ),
    )
    res = plan_mining_dock(req)
    assert res.status == "OK"
    assert res.plan is not None
    assert res.plan.metrics.dynamic_feasible is True
    assert res.plan.metrics.max_steering_change >= 0.0
    assert res.plan.metrics.max_steering_rate_required >= 0.0


def test_steering_infeasible_low_rate():
    req = PlanRequest(
        vehicle_id="HD785-7_slowsteer",
        start_pose=Pose(10.0, 8.0, 0.0),
        dock_pose=Pose(58.0, 30.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=0.1, yaw=0.0873),
        map=rectangular_map(),
        planner_params=PlannerParams(
            timeout_ms=2500,
            weights={"w_len": 1.0, "w_time": 1.0, "w_rev": 2.0, "w_switch_yaw": 0.5, "w_goal": 5.0},
            sampling_params={
                "switch_sample_count": 1200,
                "yaw_bins": 32,
                "step_size": 0.6,
                "collision_stride": 2,
                "min_speed_threshold": 2.0,
            },
            yaw_pref=0.0,
            seed=13,
        ),
    )
    res = plan_mining_dock(req)
    assert res.status in {"NO_FEASIBLE_PATH", "TIMEOUT"} or (
        res.plan is not None and res.plan.metrics.dynamic_feasible is False
    )


def test_road_width_rejects_narrow_area():
    req = PlanRequest(
        vehicle_id="test_roadwide",
        start_pose=Pose(5.0, 3.5, 0.0),
        dock_pose=Pose(35.0, 3.5, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=0.2, yaw=0.2),
        map=rectangular_map(width=40.0, height=7.0, obstacles=[]),
        planner_params=PlannerParams(
            timeout_ms=2000,
            weights={"w_len": 1.0, "w_time": 1.0, "w_rev": 1.0, "w_goal": 10.0},
            sampling_params={"switch_sample_count": 300, "yaw_bins": 12, "step_size": 0.6},
            yaw_pref=0.0,
            seed=5,
        ),
    )
    res = plan_mining_dock(req)
    assert res.status in {"NO_FEASIBLE_PATH", "TIMEOUT"}
