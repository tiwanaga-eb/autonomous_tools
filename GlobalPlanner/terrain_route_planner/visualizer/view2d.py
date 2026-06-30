"""2D matplotlib visualization of height map, cost map, and route."""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.colors import LogNorm


def show_2d(
    height_grid: np.ndarray,
    cost_map: np.ndarray,
    route_rc: list,
    start_rc: tuple,
    goal_rc: tuple,
    geo_transform: dict,
) -> None:
    """Display height map and cost map side-by-side with route, start, and goal overlaid."""
    cell_size = geo_transform["cell_size"]
    x_min = geo_transform["x_min"]
    y_min = geo_transform["y_min"]
    rows, cols = height_grid.shape

    # Axis tick helpers
    def col_to_x(c):
        return x_min + c * cell_size

    def row_to_y(r):
        return y_min + r * cell_size

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Terrain Route Planner", fontsize=14)

    # --- Height map ---
    ax = axes[0]
    im0 = ax.imshow(
        height_grid,
        origin="lower",
        cmap="gist_earth",
        aspect="equal",
    )
    plt.colorbar(im0, ax=ax, label="Elevation (m)")
    ax.set_title("Height Map")

    # Route
    if route_rc:
        rr = [rc[0] for rc in route_rc]
        cc = [rc[1] for rc in route_rc]
        ax.plot(cc, rr, "r-", linewidth=1.5, label="Route")
        ax.plot(cc[0], rr[0], "g^", markersize=10, label="Start")
        ax.plot(cc[-1], rr[-1], "r*", markersize=14, label="Goal")

    # World-coordinate ticks
    x_ticks_col = np.linspace(0, cols - 1, min(6, cols))
    ax.set_xticks(x_ticks_col)
    ax.set_xticklabels([f"{col_to_x(c):.0f}" for c in x_ticks_col], rotation=30)
    y_ticks_row = np.linspace(0, rows - 1, min(6, rows))
    ax.set_yticks(y_ticks_row)
    ax.set_yticklabels([f"{row_to_y(r):.0f}" for r in y_ticks_row])
    ax.set_xlabel("Easting (m)")
    ax.set_ylabel("Northing (m)")
    ax.legend(fontsize=8)

    # --- Cost map ---
    ax2 = axes[1]
    cost_display = np.where(cost_map >= 1e8, np.nan, cost_map)
    vmin = np.nanmin(cost_display)
    vmax = np.nanmax(cost_display)
    if vmin <= 0:
        vmin = 1e-3
    norm = LogNorm(vmin=vmin, vmax=max(vmax, vmin * 10))
    im1 = ax2.imshow(
        cost_display,
        origin="lower",
        cmap="hot_r",
        norm=norm,
        aspect="equal",
    )
    plt.colorbar(im1, ax=ax2, label="Traversal cost (log scale)")
    ax2.set_title("Cost Map")

    if route_rc:
        ax2.plot(cc, rr, "b-", linewidth=1.5, label="Route")
        ax2.plot(cc[0], rr[0], "g^", markersize=10, label="Start")
        ax2.plot(cc[-1], rr[-1], "r*", markersize=14, label="Goal")

    ax2.set_xticks(x_ticks_col)
    ax2.set_xticklabels([f"{col_to_x(c):.0f}" for c in x_ticks_col], rotation=30)
    ax2.set_yticks(y_ticks_row)
    ax2.set_yticklabels([f"{row_to_y(r):.0f}" for r in y_ticks_row])
    ax2.set_xlabel("Easting (m)")
    ax2.set_ylabel("Northing (m)")
    ax2.legend(fontsize=8)

    plt.tight_layout()
    plt.show()
