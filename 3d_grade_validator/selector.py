"""Pick a polygon by clicking on a 2D top-down (XY) scatter of the point cloud.

Uses matplotlib's PolygonSelector, which is far more reliable than Open3D's
VisualizerWithEditing on macOS.
"""
from __future__ import annotations

import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import PolygonSelector


def pick_polygon_vertices(
    points: np.ndarray,
    rgb: np.ndarray | None = None,
    max_display_points: int = 400_000,
) -> np.ndarray:
    """Show a top-down scatter; let the user click polygon vertices; return (M, 2) XY."""
    print(
        "\n=== Polygon selection (2D top-down) ===\n"
        "  left-click        : add vertex\n"
        "  right-click       : remove last vertex\n"
        "  click 1st vertex  : close polygon\n"
        "  Esc               : reset\n"
        "  scroll wheel      : zoom at cursor\n"
        "  toolbar (pan/zoom): drag to pan / box-zoom\n"
        "  close the window  : finish\n",
        flush=True,
    )

    if len(points) > max_display_points:
        rng = np.random.default_rng(0)
        idx = rng.choice(len(points), size=max_display_points, replace=False)
        disp_xy = points[idx, :2]
        disp_rgb = rgb[idx] if rgb is not None else None
        disp_z = points[idx, 2]
    else:
        disp_xy = points[:, :2]
        disp_rgb = rgb
        disp_z = points[:, 2]

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect("equal")
    ax.set_title("Click to add polygon vertices • close shape on first vertex • close window when done")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    if disp_rgb is not None:
        ax.scatter(disp_xy[:, 0], disp_xy[:, 1], c=disp_rgb, s=1, marker=".", linewidths=0)
    else:
        ax.scatter(disp_xy[:, 0], disp_xy[:, 1], c=disp_z, cmap="terrain", s=1, marker=".", linewidths=0)

    captured: dict[str, np.ndarray | None] = {"verts": None}

    def _on_select(verts):
        captured["verts"] = np.asarray(verts, dtype=np.float64)
        print(f"  -> polygon closed with {len(verts)} vertices", flush=True)

    def _on_scroll(event):
        if event.inaxes is not ax or event.xdata is None or event.ydata is None:
            return
        scale = 1 / 1.25 if event.button == "up" else 1.25
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        ax.set_xlim(
            event.xdata - (event.xdata - xlim[0]) * scale,
            event.xdata + (xlim[1] - event.xdata) * scale,
        )
        ax.set_ylim(
            event.ydata - (event.ydata - ylim[0]) * scale,
            event.ydata + (ylim[1] - event.ydata) * scale,
        )
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("scroll_event", _on_scroll)

    # useblit=True can glitch on some macOS backends; keep it off for reliability.
    selector = PolygonSelector(ax, _on_select, useblit=False)
    _ = selector  # keep reference alive while window is open

    plt.show()

    verts = captured["verts"]
    if verts is None or len(verts) < 3:
        print("No polygon selected.", file=sys.stderr)
        raise RuntimeError(f"need at least 3 polygon vertices, got {0 if verts is None else len(verts)}")
    return verts
