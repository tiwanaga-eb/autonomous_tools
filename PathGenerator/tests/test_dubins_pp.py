import math

from planner.models import MapData, PlanRequest, PlannerParams, Pose, Tolerances
from planner.planner import plan_mining_dock


def rectangular_map(width=80.0, height=40.0, obstacles=None, safety_margin=0.3):
    if obstacles is None:
        obstacles = []
    drivable = [[(0.0, 0.0), (width, 0.0), (width, height), (0.0, height)]]
    return MapData(drivable_polygons=drivable, obstacle_polygons=obstacles, safety_margin=safety_margin)


def _dubins_pp_params(timeout_ms=2000, seed=7):
    return PlannerParams(
        timeout_ms=timeout_ms,
        algorithm="dubins_pp",
        weights={"w_len": 1.0, "w_time": 1.0, "w_rev": 1.5, "w_goal": 5.0},
        sampling_params={"switch_sample_count": 32, "collision_stride": 2},
        dubins_pp_params={
            "kappa1_ratios": [0.5, 0.7, 1.0],
            "kappa2_ratios": [0.5, 0.7, 1.0],
            "switch_buffer_lengths": [0.0, 1.0, 2.0],
            "switch_yaw_offsets_deg": [-15.0, 0.0, 15.0],
            "arc_sample_step": 0.25,
        },
        dubins_pp_behavioral_params={
            "reverse_arc_ratio_limit": 0.35,
            "switch_sector_d_min": 4.0,
            "switch_sector_d_max": 30.0,
            "switch_sector_angle_deg": 40.0,
            "delta_neutral_limit_deg": 12.0,
            "forward_length_ratio": 1.0,
            "entry_angle_limit_deg": 25.0,
        },
        seed=seed,
    )


def _primitives_heavy_params(timeout_ms=1200, seed=8):
    return PlannerParams(
        timeout_ms=timeout_ms,
        algorithm="primitives",
        weights={"w_len": 1.0, "w_time": 1.0, "w_rev": 1.2, "w_goal": 10.0},
        sampling_params={},
        primitives_params={
            "xy_res": 0.5,
            "yaw_bins": 36,
            "delta_bins": 21,
            "T": 0.5,
            "dt": 0.05,
            "astar_weight": 1.8,
            "collision_check_stride": 1,
            "speed_levels": 2,
            "delta_dot_levels": 5,
            "max_nodes_expanded": 30000,
        },
        seed=seed,
    )


def test_dubins_pp_simple_success():
    req = PlanRequest(
        vehicle_id="demo_truck",
        start_pose=Pose(8.0, 6.0, 0.0),
        dock_pose=Pose(56.0, 30.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=0.4, yaw=0.2),
        map=rectangular_map(),
        planner_params=_dubins_pp_params(),
    )
    res = plan_mining_dock(req)
    assert res.status == "OK"
    assert res.plan is not None
    assert [s.gear for s in res.plan.segments] == ["F", "R"]


def test_dubins_pp_succeeds_when_primitives_struggle():
    obstacle = [[(30.0, 12.0), (45.0, 12.0), (45.0, 28.0), (30.0, 28.0)]]
    base_req = dict(
        vehicle_id="demo_truck",
        start_pose=Pose(8.0, 6.0, 0.0),
        dock_pose=Pose(62.0, 31.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=0.8, yaw=0.35),
        map=rectangular_map(obstacles=obstacle),
    )

    pp_res = plan_mining_dock(PlanRequest(**base_req, planner_params=_dubins_pp_params(timeout_ms=1500, seed=11)))
    prim_res = plan_mining_dock(PlanRequest(**base_req, planner_params=_primitives_heavy_params(timeout_ms=600, seed=11)))

    assert pp_res.status == "OK"
    assert pp_res.plan is not None
    assert prim_res.status in {"TIMEOUT", "NO_FEASIBLE_PATH", "OK"}
    if prim_res.status != "OK":
        assert pp_res.debug is not None and prim_res.debug is not None
        assert pp_res.debug.get("compute_time_ms", 1e9) < prim_res.debug.get("compute_time_ms", 0.0)


def test_dubins_pp_switch_yaw_limit_constraint():
    params = _dubins_pp_params(timeout_ms=1200, seed=21)
    params.dubins_pp_params["switch_yaw_offsets_deg"] = [45.0]
    params.dubins_pp_params["switch_yaw_limit_deg"] = 10.0
    req = PlanRequest(
        vehicle_id="demo_truck",
        start_pose=Pose(8.0, 6.0, 0.0),
        dock_pose=Pose(56.0, 30.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=0.4, yaw=0.2),
        map=rectangular_map(),
        planner_params=params,
    )
    res = plan_mining_dock(req)
    assert res.status in {"NO_FEASIBLE_PATH", "TIMEOUT"}
