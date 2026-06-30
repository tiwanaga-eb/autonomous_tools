# --- Truck traversability ---
SLOPE_LIMIT_DEG   = 20.0   # degrees — impassable above this
SLOPE_WEIGHT      = 3.0
ROUGH_WEIGHT      = 2.0
OBSTACLE_COST     = 1e9

# --- Rasterization ---
DEFAULT_CELL_SIZE = 1.0    # metres per grid cell
GAP_FILL_RADIUS   = 3      # cells — ndimage fill radius for empty cells

# --- LAS chunked loading ---
LAS_CHUNK_POINTS  = 5_000_000   # points per chunk (tune for available RAM)

# --- Route smoother ---
SPLINE_SMOOTHING  = 5.0
SPLINE_POINTS     = 500

# --- Visualizer ---
ROUTE_ELEV_OFFSET = 0.5    # metres above surface for 3D route line
