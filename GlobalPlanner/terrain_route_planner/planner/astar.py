"""A* pathfinder on a 2D cost grid with Euclidean heuristic."""
import heapq
import sys
import numpy as np

from config import OBSTACLE_COST
from .graph import NEIGHBORS


def _snap_to_traversable(cost_map: np.ndarray, row: int, col: int, label: str) -> tuple:
    """Find the nearest traversable cell using BFS; warn if snapping was needed."""
    rows, cols = cost_map.shape
    if cost_map[row, col] < OBSTACLE_COST:
        return row, col

    print(
        f"Warning: {label} cell ({row}, {col}) is impassable — "
        "snapping to nearest traversable cell."
    )
    visited = np.zeros((rows, cols), dtype=bool)
    queue = [(0, row, col)]
    while queue:
        dist, r, c = heapq.heappop(queue)
        if visited[r, c]:
            continue
        visited[r, c] = True
        if cost_map[r, c] < OBSTACLE_COST:
            print(f"  Snapped {label} to ({r}, {c})")
            return r, c
        for dr, dc, _ in NEIGHBORS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr, nc]:
                heapq.heappush(queue, (dist + 1, nr, nc))

    print(f"Error: no traversable cell found near {label}.", file=sys.stderr)
    sys.exit(1)


def find_route(
    cost_map: np.ndarray,
    start_rc: tuple,
    goal_rc: tuple,
    cell_size: float,
) -> list:
    """Run A* on cost_map from start_rc to goal_rc; returns list of (row, col) tuples."""
    rows, cols = cost_map.shape
    start_rc = _snap_to_traversable(cost_map, start_rc[0], start_rc[1], "start")
    goal_rc = _snap_to_traversable(cost_map, goal_rc[0], goal_rc[1], "goal")

    # Priority queue: (f_score, g_score, row, col)
    open_heap = []
    heapq.heappush(open_heap, (0.0, 0.0, start_rc[0], start_rc[1]))

    g_score = {start_rc: 0.0}
    came_from = {start_rc: None}

    gr, gc = goal_rc

    def heuristic(r, c):
        return cell_size * np.hypot(r - gr, c - gc)

    visited = set()

    while open_heap:
        f, g, r, c = heapq.heappop(open_heap)

        if (r, c) in visited:
            continue
        visited.add((r, c))

        if (r, c) == goal_rc:
            # Reconstruct path
            path = []
            node = goal_rc
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return path

        cell_cost = cost_map[r, c]
        for dr, dc, diag_w in NEIGHBORS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                nb_cost = cost_map[nr, nc]
                if nb_cost >= OBSTACLE_COST:
                    continue
                tentative_g = g + diag_w * cell_size * 0.5 * (cell_cost + nb_cost)
                nb = (nr, nc)
                if tentative_g < g_score.get(nb, float("inf")):
                    g_score[nb] = tentative_g
                    came_from[nb] = (r, c)
                    h = heuristic(nr, nc)
                    heapq.heappush(open_heap, (tentative_g + h, tentative_g, nr, nc))

    return []  # no path found
