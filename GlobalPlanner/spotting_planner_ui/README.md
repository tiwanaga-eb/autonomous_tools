# Dump-Truck Spotting & Exit Planner — Web Prototype

Interactive web prototype for configuring and visualising autonomous dump-truck
**spotting** and **exit-path** generation inside a constrained traversable area.

Built with React + TypeScript + Vite + Zustand. SVG rendering for the map.
Planner runs entirely in the browser (no backend) and uses a boundary-aware
**Hybrid A*-lite** with automatic and manual cusp control.

```
┌──────────────────┬─────────────────────────────────┬──────────────────┐
│ Parameter panel  │           Map canvas            │  Planner status  │
│  (19 knobs)      │  • authorized boundary          │   feasibility    │
│                  │  • traversable area             │   timing         │
│ Run Planner ▶    │  • obstacles                    │   clearances     │
│                  │  • queue / dump / exit poses    │   cusp info      │
│                  │  • spotting + exit paths        │   failure reason │
│                  │  • cusp candidates              │                  │
│                  │  • footprints along path        │                  │
└──────────────────┴─────────────────────────────────┴──────────────────┘
```

## Quick start

```bash
cd spotting_planner_ui
npm install
npm run dev        # http://localhost:5173
```

Other scripts:

```bash
npm run typecheck   # tsc --noEmit
npm run lint        # eslint
npm run test        # vitest run
npm run build       # production bundle in dist/
```

## File structure

```
spotting_planner_ui/
├── README.md
├── package.json
├── tsconfig.json
├── vite.config.ts
├── eslint.config.js
├── index.html
└── src/
    ├── main.tsx
    ├── App.tsx
    ├── styles.css
    ├── types.ts                  # Pose / Polygon / PlannerParams / PlanResult
    ├── state/store.ts            # Zustand store: params + scenario + runPlanner()
    ├── scenario/defaultScenario.ts
    ├── geometry/
    │   ├── primitives.ts         # vec ops, angle normalization
    │   ├── polygon.ts            # point-in-polygon, polygon overlap / contains, distances
    │   └── footprint.ts          # truck rectangle at pose
    ├── vehicle/bicycle.ts        # kinematic bicycle model + effective wheelbase
    ├── planner/
    │   ├── motionPrimitives.ts   # 6 primitives: ±δ_max / 0 × forward / reverse
    │   ├── heap.ts               # binary min-heap (OPEN set)
    │   ├── heuristic.ts          # admissible turning-aware Euclidean lower bound
    │   ├── collision.ts          # inflated footprint vs boundary + obstacles
    │   ├── costs.ts              # edge cost composition
    │   ├── hybridAstar.ts        # main Hybrid A* loop
    │   ├── cuspSampler.ts        # polar fan in front of dump target
    │   ├── plannerOrchestrator.ts # queue→cusp→dump + dump→exit + scoring
    │   └── validator.ts          # post-hoc per-sample re-validation
    └── components/
        ├── MapCanvas.tsx
        ├── ParameterPanel.tsx
        ├── PlannerStatusPanel.tsx
        ├── BoundaryLayer.tsx
        ├── ObstacleLayer.tsx
        ├── PathLayer.tsx
        └── VehicleFootprint.tsx

tests/
├── geometry.test.ts
├── vehicle.test.ts
├── collision.test.ts
└── planner.test.ts
```

## How the planner works

1. **Two-stage decomposition.** Spotting and exit are planned separately.
   The spotting maneuver is further split into _queue → cusp_ (forward) and
   _cusp → dump_ (reverse, planned in reversed geometry then flipped).
2. **Cusp selection.**
   - **Auto:** sample N candidates on a polar fan in front of the dump target
     (radii 1.2–2.5 R_min, bearings ±60° from the dump's forward direction,
     heading ≈ 180° opposite the dump). Each candidate is validated and scored
     by the total spotting path cost.
   - **Manual:** the user-supplied `(x, y, heading)` is validated, then used as
     the cusp.
3. **Hybrid A*-lite.**
   - Continuous state `(x, y, θ)` discretised to
     `(round(x/Δx), round(y/Δy), round(θ/Δθ))` for the CLOSED set.
   - 6 motion primitives per expansion: `(steer ∈ {-δ_max, 0, +δ_max})` ×
     `(direction ∈ {forward, reverse})`, each integrated as a constant-curvature
     arc of length `Δx`.
   - Edge cost = `Δx · (1 + reversePenalty·[reverse])` + `cuspPenalty·[flip]` +
     `steeringSmoothness·|Δsteer|` + `boundaryWeight · clearanceDeficit`.
   - Heuristic = `max(euclidean, ½(euclidean + R_min·|Δheading|))`. Admissible
     because it ignores obstacles and reverse penalties.
   - Terminates on goal tolerance, timeout, or max-expansion budget.
4. **Boundary safety.** At every expansion we build the **inflated**
   footprint with `inflate = safetyBuffer + localizationErrorMargin +
   pathFollowingErrorMargin` and check
   `footprint ⊆ authorizedBoundary ∩ traversableArea` and
   `footprint ∩ obstacle = ∅`. No expansion is added otherwise.
5. **Defence-in-depth post-validation.** After reconstruction the planner
   re-runs the inflated check on every densified sample and records minimum
   clearances. If any sample fails (which would indicate a discretisation gap),
   the result is reported infeasible with a clear reason rather than silently
   smoothed.

## Engineering invariants

- The path is **never** smoothed in a way that could leave the traversable area.
  Smoothing is post-validation only — violations are rejected, not patched.
- The chosen cusp pose itself is validated with the full inflated footprint.
- Queue and exit poses are validated up front; if any is infeasible the planner
  short-circuits with `start_invalid` and an explanation.
- When no feasible path exists, the UI shows **NO FEASIBLE PATH** and a
  failure reason instead of forcing a path through.

## Assumptions & limitations of this prototype

Assumptions:
- 2-D planar world. No grade, surface friction, or load dynamics.
- Truck is modelled as a rectangle pivoting on the rear axle (kinematic bicycle).
  Real haul trucks have non-trivial rear-overhang and articulation; for
  articulated trucks the geometry would need a tractor-trailer model.
- Constant grid resolution; no on-the-fly resolution refinement near corners.
- Speed-independent kinematics; turning radius bound is conservative.

Known limitations / shortcuts vs production Hybrid A*:
- Heuristic uses an Euclidean + heading lower bound, not full **Reeds-Shepp**
  closed-form. This is admissible but loose, so explored expansions can exceed
  what a textbook implementation would need.
- No analytic expansion: a real Hybrid A* tries to connect to the goal with a
  closed-form Reeds-Shepp curve after each expansion. We rely on the search
  reaching the goal cell. For dense scenes this is fine; for tight maneuvering
  near narrow goal tolerances it costs runtime.
- Only 3 steering angles. Production planners use 5+ or continuous resampling.
- No analytic smoothing (Conjugate-gradient / iLQR post-smooth). The reconstructed
  path is "blocky" at the resolution of the grid.
- No dynamic obstacles, no time dimension.
- Single-thread, no Web Worker. UI freezes briefly on `Run Planner` for hard
  scenarios.
- Boundary polygons must be **simple** (no self-intersections, no holes). For
  holes you must encode them as separate obstacles.

## Suggested next steps toward production

1. **Replace heuristic with Reeds-Shepp closed-form length** + add **analytic
   expansion** (try RS curve to goal at every Nth node). This typically cuts
   expansions by 3-5× and yields smoother goal approaches.
2. **Move planner to a Web Worker.** The store's `runPlanner()` should post a
   message; the UI can stream progress (expansion count, best-so-far) and
   support cancellation.
3. **Conjugate-gradient or OBVP smoothing pass.** After Hybrid A*, smooth using
   a cost combining (path length, curvature, distance-to-obstacle gradient),
   with a hard inequality on boundary clearance — never accept a smoothed
   point with smaller clearance than its A* predecessor.
4. **Replace the rectangular footprint** with a swept-volume polygon (Minkowski
   sum of vehicle box and turning trajectory disc) for tighter clearance
   checks. For articulated trucks, model tractor + trailer as a linked-rectangle
   pair and check both.
5. **Path tracking with MPC.** The path-following error margin parameter
   becomes a real bound. Build a kinematic-bicycle MPC over a horizon of
   2-4 seconds, optimising `(steer, accel)` with stage costs on lateral
   deviation, heading error, and control rate; terminal cost on pose error.
   Bound the predicted footprint to remain inside boundary∪obstacle-free space
   as a hard inequality. Use real-time iteration via acados / casadi.
6. **Sensor / localization model**. Treat `localizationErrorMargin` as a
   covariance-derived margin; allocate it dynamically based on a fused EKF
   covariance trace rather than a constant.
7. **Authority handoff**. Wire `validatePath` output into a runtime monitor
   that re-checks the path each control step against the live perception
   stack and triggers a controlled-stop if clearance falls below the
   negotiated SLA.
8. **Tests beyond unit-level**. Property-based tests (fast-check) for
   boundary respect, golden-image regression on visual output, and a HIL
   loop replaying recorded mine sites.

## Controls

- **Ctrl/⌘ + scroll** — zoom map
- **Click + drag** — pan map
- **Parameter panel** — edit any of the 19 knobs and click **Run Planner**
- **Auto cusp** checkbox toggles between automatic candidate sampling and a
  manually entered cusp pose

## License

Prototype code. Use at your own risk for non-production exploration.
