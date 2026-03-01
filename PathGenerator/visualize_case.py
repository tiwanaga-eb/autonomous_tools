from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt

from planner.models import MapData, PlanRequest, PlannerParams, Pose, Tolerances
from planner.planner import plan_mining_dock


OUT_DIR = Path("out")


def make_params(seed=7):
    return PlannerParams(
        timeout_ms=2500,
        weights={"w_len": 1.0, "w_time": 1.0, "w_rev": 2.0, "w_switch_yaw": 0.5, "w_goal": 5.0},
        sampling_params={"switch_sample_count": 1200, "yaw_bins": 32, "step_size": 0.6, "collision_stride": 2},
        yaw_pref=0.0,
        seed=seed,
    )


def draw_pose(ax, pose: Pose, label: str, marker: str):
    ax.plot(pose.x, pose.y, marker=marker, markersize=7, label=label)
    ax.arrow(
        pose.x,
        pose.y,
        math.cos(pose.yaw) * 2.0,
        math.sin(pose.yaw) * 2.0,
        head_width=0.8,
        head_length=1.0,
        length_includes_head=True,
    )


def draw_case(request: PlanRequest, output_path: Path):
    res = plan_mining_dock(request)

    fig, ax = plt.subplots(figsize=(10, 6))

    for poly in request.map.drivable_polygons:
        xs = [p[0] for p in poly] + [poly[0][0]]
        ys = [p[1] for p in poly] + [poly[0][1]]
        ax.plot(xs, ys)

    for poly in request.map.obstacle_polygons:
        xs = [p[0] for p in poly] + [poly[0][0]]
        ys = [p[1] for p in poly] + [poly[0][1]]
        ax.plot(xs, ys)

    draw_pose(ax, request.start_pose, "start", "o")
    draw_pose(ax, request.dock_pose, "dock", "s")

    if res.plan is not None:
        draw_pose(ax, res.plan.switch_pose, "switch", "^")
        for seg in res.plan.segments:
            xs = [s.x for s in seg.states]
            ys = [s.y for s in seg.states]
            ls = "-" if seg.gear == "F" else "--"
            ax.plot(xs, ys, linestyle=ls)
        m = res.plan.metrics
        title = (
            f"status={res.status} len={m.total_length:.2f}m "
            f"time={m.total_time:.2f}s compute={m.compute_time_ms:.1f}ms"
        )
    else:
        title = f"status={res.status} reason={res.reason}"

    ax.set_aspect("equal", adjustable="box")
    ax.set_title(title)
    ax.legend(loc="best")
    ax.grid(True, linestyle=":")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def main():
    drivable = [[(0.0, 0.0), (80.0, 0.0), (80.0, 40.0), (0.0, 40.0)]]

    case1 = PlanRequest(
        vehicle_id="HD785-7",
        start_pose=Pose(10.0, 8.0, 0.0),
        dock_pose=Pose(58.0, 30.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=0.1, yaw=0.0873),
        map=MapData(drivable_polygons=drivable, obstacle_polygons=[], safety_margin=0.3),
        planner_params=make_params(seed=10),
    )

    obstacle = [[(32.0, 14.0), (44.0, 14.0), (44.0, 26.0), (32.0, 26.0)]]
    case2 = PlanRequest(
        vehicle_id="HD785-7",
        start_pose=Pose(8.0, 6.0, 0.0),
        dock_pose=Pose(62.0, 31.0, math.pi),
        exit_pose=None,
        tolerances=Tolerances(pos=0.1, yaw=0.0873),
        map=MapData(drivable_polygons=drivable, obstacle_polygons=obstacle, safety_margin=0.3),
        planner_params=make_params(seed=3),
    )

    draw_case(case1, OUT_DIR / "case1.png")
    draw_case(case2, OUT_DIR / "case2.png")


if __name__ == "__main__":
    main()
