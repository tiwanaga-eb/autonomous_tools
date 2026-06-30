"""Build an 8-connected weighted grid graph from a cost map."""
import numpy as np

# 8-connectivity: (delta_row, delta_col, euclidean_weight)
_NEIGHBORS = [
    (-1, -1, 1.41421356),
    (-1,  0, 1.0),
    (-1,  1, 1.41421356),
    ( 0, -1, 1.0),
    ( 0,  1, 1.0),
    ( 1, -1, 1.41421356),
    ( 1,  0, 1.0),
    ( 1,  1, 1.41421356),
]


def build_graph(cost_map: np.ndarray) -> dict:
    """Return a dict mapping (row, col) to list of ((nr, nc), edge_cost) neighbours."""
    rows, cols = cost_map.shape
    graph = {}
    for r in range(rows):
        for c in range(cols):
            if cost_map[r, c] >= 1e8:
                continue
            neighbours = []
            for dr, dc, diag_w in _NEIGHBORS:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    neighbor_cost = cost_map[nr, nc]
                    if neighbor_cost < 1e8:
                        # Edge cost = diagonal weight * average of cell costs
                        edge_cost = diag_w * 0.5 * (cost_map[r, c] + neighbor_cost)
                        neighbours.append(((nr, nc), edge_cost))
            graph[(r, c)] = neighbours
    return graph


NEIGHBORS = _NEIGHBORS  # expose for A*
