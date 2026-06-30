You are a senior autonomous vehicle engineer, mining automation engineer, and full-stack software architect.

Build a web-based UI prototype for configuring and visualizing autonomous dump-truck spotting and exit-path generation inside a constrained traversable area.

Goal:
Create an interactive application where a user can modify planning parameters and see how the spotting path, cusp point, queue position, exit path, obstacles, and traversable boundary affect the generated route.

Core algorithm concept:
Use boundary-aware Hybrid A* / lattice-style path planning for low-speed dump-truck maneuvering, with automatic cusp-point generation and optional manual cusp override. The generated path must remain inside the authorized/traversable area. The full truck footprint and swept volume must be checked, not only the centerline.

Functional requirements:
1. UI canvas / map view
   - Show traversable area polygon.
   - Show authorized planning boundary.
   - Show no-go zones / obstacles.
   - Show dump target pose.
   - Show truck queue position.
   - Show generated spotting path.
   - Show generated exit path.
   - Show cusp point.
   - Show truck footprint along the path.
   - Highlight invalid path segments in red.

2. Parameter panel
   User can edit:
   - Truck length
   - Truck width
   - Minimum turning radius
   - Max steering angle
   - Safety buffer
   - Localization error margin
   - Expected path-following error margin
   - Reverse penalty
   - Cusp penalty
   - Boundary clearance weight
   - Final pose tolerance
   - Final heading tolerance
   - Max reverse distance
   - Grid resolution
   - Heading resolution
   - Planning timeout
   - Number of cusp candidates
   - Enable/disable automatic cusp generation
   - Manual cusp x/y/heading

3. Planner behavior
   - Generate a spotting path from queue pose to dump target pose.
   - Generate an exit path from dump target pose to exit lane pose.
   - Keep all path points and swept footprint inside the traversable/authorized area.
   - Reject paths that collide with obstacles or leave the valid area.
   - Automatically sample cusp candidates when auto mode is enabled.
   - Allow the user to manually set the cusp point.
   - Score paths using distance, reverse distance, number of cusps, steering smoothness, boundary clearance, and final pose error.

4. Validation output
   Show:
   - Path feasible / infeasible
   - Planning response time
   - Minimum boundary clearance
   - Minimum obstacle clearance
   - Total path length
   - Reverse distance
   - Number of cusps
   - Estimated path-following error margin
   - Reason for failure if infeasible

5. Implementation expectation
   - Make this a working prototype, not only static UI.
   - Use clean modular code.
   - Separate planner logic, geometry utilities, vehicle model, UI components, and parameter state.
   - Include comments explaining key algorithm decisions.
   - Use TypeScript if possible.
   - If using React, create reusable components:
     - MapCanvas
     - ParameterPanel
     - PlannerStatusPanel
     - VehicleFootprint
     - PathLayer
     - BoundaryLayer
     - ObstacleLayer

6. Important engineering constraints
   - Do not generate a path outside the traversable area.
   - Do not smooth a path in a way that violates the boundary.
   - Cusp point must be valid with full truck footprint clearance.
   - Queue position and exit pose must also be validated.
   - If no feasible path exists, clearly display “No feasible path” rather than forcing a path.

Deliverables:
- Working source code.
- Clear file structure.
- README with how to run.
- Explanation of planner assumptions and limitations.
- Suggested next steps for replacing the prototype planner with production-grade Hybrid A* and MPC tracking.