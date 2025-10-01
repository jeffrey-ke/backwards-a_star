# Space-Time A* for Moving Target Interception

This implements a motion planner for catching a moving target on weighted 2D terrain with obstacles. The assignment was open-ended—students chose their algorithm, heuristics, and optimizations. Completed for 16-782: Planning and Decision Making in Robotics (Fall 2025).

## Problem Statement

Given:
- 8-connected 2D grid (up to 2000×2000 cells)
- Weighted costmap with collision threshold (cells ≥ threshold are obstacles)
- Robot starting position
- Complete target trajectory as sequence of (x,y) waypoints
- Time constraint: Planner has T seconds to catch target with T-step trajectory

Objective: Minimize path cost to intercept target

**Critical constraint**: If planning takes K seconds, target moves K steps during computation. The planner must account for its own planning time when computing interception points.

## Algorithm: Backward A* in Space-Time with Temporal Synchronization

### Time Budget Management

**The core challenge**: Plan must account for time consumed during planning itself—the robot only begins executing after planning completes, while the target continues moving.

**Solution** (`planner.cpp:98-126`):

```cpp
auto time_budget = 40s;  // Fixed planning window
auto time_start = steady_clock::now();

// Initialize robot state with time_budget as temporal coordinate
const State robot_init{robotposeX, robotposeY, static_cast<int>(time_budget.count()) - 1};

// ... run A* search ...

// Enforce fixed planning duration
auto time_now = steady_clock::now();
auto time_left = time_budget - duration_cast<seconds>(time_now - time_start);
std::this_thread::sleep_until(time_now + time_left);

// Now execute first action - robot and target synchronized
action_ptr[0] = next_x;
action_ptr[1] = next_y;
```

**Key insight**: By setting the robot's initial time coordinate to `time_budget` and sleeping until exactly `time_budget` seconds have elapsed, the planner guarantees:

1. **Deterministic planning duration**: Robot always starts moving at t=40 seconds, regardless of actual search completion time
2. **Temporal synchronization**: Target advances exactly 40 steps while planning completes
3. **Correct state-space embedding**: Robot state at (x,y,40) in space-time graph corresponds to physical state when execution begins

**Why this matters**: Without fixed timing, fast planning would produce a path assuming target at position t=5, but robot wouldn't execute until t=5 (actual time). Target would have moved to position t=10, invalidating the plan. The sleep mechanism ensures the space-time coordinates in the search graph match physical execution timeline.

### Search Direction

The planner searches **backward** from target trajectory waypoints to robot start position in (x,y,t) state space.

Standard forward formulation: Robot start → Multiple time-varying goals (target positions at different times)

Backward formulation: Multiple goal states (target trajectory) → Single static goal (robot start)

**Rationale**: Backward search converts multi-goal dynamic problem into single-goal static problem. The robot's initial state becomes the sole search objective, eliminating need to track time-varying goal positions during search.

### Implementation Details

**Initialization** (`planner.cpp:101-110`):
- Create IMAGINARY_GOAL node at (0,0,0) with g=0
- Parse target trajectory into concrete goal states at their actual time indices
- Initialize robot goal state at `(robotposeX, robotposeY, time_budget - 1)`
- Connect IMAGINARY_GOAL to all trajectory waypoints with edge cost 1
- Begin A* search from IMAGINARY_GOAL

**Termination**: Search completes when robot's starting state `(x,y,time_budget-1)` is expanded

**State representation** (`state.h:13-51`):
- 3-tuple (x, y, t) encoding position and time
- In backward search: t represents "time at which robot is at this position"
- Custom Boost hash for unordered containers
- 12-byte memory footprint per state

**Action space**: 9 actions (8 cardinal/diagonal moves + wait in place)

### Data Structures

**Priority Queue** (`openlist.h:20-73`):

Boost d-ary heap (d=4) with handle-based decrease-key capability:
```cpp
boost::heap::d_ary_heap<
    tuple<State, gval, hval>,
    arity<4>,
    compare<GT>,
    mutable_<true>
>
```

Parallel `unordered_map<State, handle_type>` enables O(1) handle lookup for priority updates. Mutable heap supports in-place modifications via handle, avoiding costly erase-reinsert pattern.

**Rationale**: d=4 balances tree height (log₄ n) against cache performance. Higher arity reduces levels but increases comparison overhead per node.

**Closed Set**: `unordered_set<State, State::Hasher>` for O(1) membership queries

**State Graph** (`map.h:38-39`):
- `unordered_map<State, gval>`: Stores g-values for path reconstruction and duplicate detection
- `unordered_map<State, int>`: Tracks minimum path length for heuristic feasibility checks

### Successor Generation

**Backward mode** (`map.h:170-198`):

For IMAGINARY_GOAL: Return all concrete goal states (target trajectory waypoints)

For regular states: Apply 9 actions with dt=-1 (moving forward in time during backward search)

Edge costs: Destination cell cost from costmap

Updates g-value only if new path improves cost:
```cpp
if (new_gval < state_gval_table[successor])
    state_gval_table[successor] = new_gval
```

## Heuristics

Multiple heuristics implemented and tested. Final submission uses DijkstraHeuristic with weight=100000.

### DijkstraHeuristic (Primary)

**Function** (`heuristic.h:106-120`):
```cpp
h(x,y,t) = weight × dijkstra_cost(x,y) + t
```

Precomputes spatial cost-to-come from robot's starting position using Dijkstra's algorithm over (x,y) states ignoring time dimension.

**Preprocessing** (`heuristic.h:60-87`):
- Run Dijkstra from robot start position
- Compute cost to all reachable (x,y) cells
- **Limit path length to `max_path_len`**: Critical optimization for time-constrained interception
- Store results in Map's `_state_gval_table`
- Time complexity: O(V log V) where V = width × height
- Space: O(V)

**Path length constraint** (`heuristic.h:72-73`):
```cpp
if (pathlen > max_path_len)
    continue;  // Don't expand this state further
```

**Rationale**: The least-cost path often takes too many steps to traverse, allowing the target to escape before interception. By bounding Dijkstra expansion to `max_path_len` steps, the heuristic only considers paths that are temporally feasible—paths the robot can execute before the target disappears. This trades off cost optimality for temporal feasibility: better to catch the target via a slightly more expensive path than to compute the optimal path that arrives too late.

**Query phase**:
- O(1) lookup of precomputed distance for state's (x,y) coordinates
- Add time component t (accounts for temporal cost in backward search)
- Return ∞ if Dijkstra path length > remaining time budget (prunes infeasible states)

**Properties**:
- **Admissible**: Spatial Dijkstra ignoring time provides true lower bound on cost
- **Consistent**: h(n) ≤ c(n,s) + h(s) for all successors s (follows from Dijkstra optimality)
- **Dominant**: Strictly better than geometric metrics on weighted terrain
- **Time-aware**: Path length constraint ensures heuristic guidance respects temporal deadline

**Learning**: The weight parameter inflates heuristic for weighted A* behavior. Higher weights provide stronger guidance toward goal but sacrifice admissibility. The feasibility check at line 111-114 significantly reduces state expansions by pruning unreachable portions of space-time graph. The max_path_len parameter was essential for solving the moving target problem—without it, the planner would compute beautiful low-cost paths that miss the interception window.

### OctileHeuristic

**Function** (`heuristic.h:178-198`):
```cpp
h(x,y,t) = weight × (√2 × min(dx,dy) + |dx - dy|)
```

Computes octile distance (optimal path length on 8-connected grid with unit costs) to target position at timestep t + look_ahead.

**Parameters**:
- `look_ahead`: How many steps ahead to look in target trajectory
- Enables tracking waypoint at `trajectory[min(t + look_ahead, len(trajectory) - 1)]`

**Mode-specific behavior**:
- Backward search: Returns ∞ if t < octile_distance (cannot reach target in available time)
- Forward search: Returns 0 for IMAGINARY_GOAL

**Learning**: Look-ahead trades off between myopic (look_ahead=0, follows current target position) and terminal-focused (large look_ahead, aims for trajectory end). Small values produce direct interception paths; large values may generate inefficient trajectories. Octile distance is admissible on unit-cost grids but weighted version sacrifices optimality for speed.

### DistanceHeuristic

**Function** (`heuristic.h:129-141`):
```cpp
h(x,y,t) = weight × √(dx² + dy²)
```

Euclidean distance to target position at current timestep.

**Properties**: Admissible on unit-cost grids but underestimates cost on weighted terrain. Used primarily for debugging and baseline comparison.

### WallHeuristic

**Function** (`heuristic.h:162-167`):
```cpp
h(s) = weight / distance_to_obstacle(s)
```

Inverse distance to nearest obstacle using precomputed distance transform.

**Distance Transform** (`map.h:89-99`):
- OpenCV `distanceTransform` with `DIST_L2` and `DIST_MASK_PRECISE`
- Computes Euclidean distance from each free cell to nearest obstacle
- O(width × height) preprocessing, O(1) queries
- Applied once during Map construction

**Rationale**: Penalizes states near obstacles to generate paths with greater clearance. Can combine with other heuristics for multi-objective optimization.

**Learning**: Distance transforms provide efficient spatial proximity queries. Inverse distance creates repulsive potential field around obstacles. Requires careful weight tuning—excessive weight dominates admissible components.

### RepulsionHeuristic

**Function** (`heuristic.h:205-216`):
```cpp
h(s) = weight / max(euclidean_distance(s, robot_start), ε)
```

Inverse distance from robot's initial position. Biases search outward from start toward target region.

**Rationale**: Encourages exploration in target direction rather than circling near start. Small epsilon (1e-6) prevents division by zero at robot start.

**Learning**: Creates gradient field pushing search away from origin. Most effective when combined with goal-directed heuristics via SumHeuristic composition.

### SumHeuristic (Composition)

**Implementation** (`heuristic.h:143-156`):

Variadic template that sums multiple heuristics:
```cpp
template <typename... Heuris>
struct SumHeuristic: public Heuristic {
    array<reference_wrapper<const Heuristic>, sizeof...(Heuris)> _heuristics;
    double operator()(const State& s) const override {
        return sum of all _heuristics[i](s);
    }
};
```

**Usage**:
```cpp
SumHeuristic<DijkstraHeuristic, RepulsionHeuristic> h(dijkstra, repulsion);
```

**Learning**: Template parameter packs enable compile-time composition without virtual function overhead during evaluation. `std::reference_wrapper` avoids object slicing while maintaining polymorphic interface through base class operator().

## Implementation Details

### Path Reconstruction

**Backtracking** (`planner.cpp:24-52`):

Starting from expanded goal state, iteratively select predecessor with minimum g-value:
```cpp
while (current != start) {
    for (pred, gval in predecessors(current)) {
        if (gval < min_gval) {
            min_gval = gval
            next_state = pred
        }
    }
    path.push_back(next_state)
    current = next_state
}
```

Removes IMAGINARY_GOAL from final trajectory. Returns deque of states from goal to start (reverse order for backward search).

**Correctness**: Each state's g-value represents minimum cost path from search start. Greedy predecessor selection along minimum g-values reconstructs optimal path.

### Plan Reuse

**Static cache** (`planner.cpp:63, 92-97`):

Global `deque<State>` persists between planner function calls:
```cpp
static deque<State> soln;

if (!soln.empty()) {
    action_ptr = get_next(soln, mode);
    return;
}
// ... run A* search to populate soln ...
```

**Rationale**: Target continues moving during execution. If planner completes search in <1 second, returns cached actions on subsequent calls. Replans only when trajectory exhausted or invalidated.

### Memory Management

**Allocation strategies**:
- `_legal_states.reserve(9)`: Pre-allocate maximum successor vector size (`map.h:68`)
- Reuse same vector across all successor queries via `clear()` before repopulating
- Handle-based heap updates avoid allocator churn
- Structured bindings enable zero-copy tuple unpacking
- No dynamic allocations in search loop critical path

**Memory scaling**: Hash tables grow dynamically but never shrink. For large state spaces, memory usage is O(visited_states). Maximum theoretical states: width × height × trajectory_length.

### Efficiency Optimizations

**Distance transform precomputation** (`map.h:89-99`): One-time O(n²) cost for O(1) obstacle distance queries

**Lazy successor generation**: Successors computed during expansion, not insertion

**Early termination** (`planner.cpp:116-118`): Exits immediately upon expanding goal state

**Action parameterization** (`map.h:74-87`): Single action template with dt ∈ {-1, 0, 1} supports forward/backward/Dijkstra modes without code duplication

**g-value pruning**: `insert_update` only modifies priority queue if new path improves existing g-value

## Key Learnings from Open-Ended Implementation

**Temporal synchronization**: The time budget mechanism was critical for correct interception. Initial implementations without fixed timing produced paths that collided with where the target *was* rather than where it *would be*. The sleep-until-budget approach ensures space-time graph coordinates align with physical execution timeline.

**Time-cost tradeoff**: The max_path_len constraint in Dijkstra heuristic addresses fundamental tension between path cost and execution time. Computing globally optimal paths is counterproductive if they take too long to execute—the target escapes. This insight drove the design of a heuristic that biases toward temporally feasible solutions rather than cost-optimal ones.

**Backward search formulation**: Not covered in standard A* tutorials. Required understanding that time dimension creates asymmetry—backward converts variable goals to fixed goal.

**Space-time graph structure**: Adding time as state dimension explodes state space (V × T states) but enables reasoning about dynamic environments. Temporal monotonicity creates DAG structure in time dimension.

**Heuristic design for weighted graphs**: Euclidean distance inadmissible on costmaps. Dijkstra preprocessing trades O(V²) upfront cost for perfect heuristic guidance. Weight parameter enables tuning completeness vs. speed.

**Handle-based priority queues**: Standard library `priority_queue` lacks decrease-key. Boost mutable heap with handle map provides O(log n) updates. Critical for A* performance on dense graphs.

**Feasibility pruning vs. admissibility**: Returning ∞ for unreachable states (heuristic.h:113) prunes search space without violating admissibility—state truly unreachable implies infinite true cost.

**Memory profiling for search**: Pre-allocation and container reuse eliminate allocator overhead. Profiling showed significant runtime reduction by reserving vectors and using structured bindings over tuple copies.

**Debugging space-time graphs**: Visualizing (x,y,t) states requires projecting 3D graph to 2D. Implemented output logging to track t-slices separately for trajectory reconstruction verification.

## Performance Characteristics

The planner successfully catches targets on provided maps (up to 2000×2000 cells) within time constraints. DijkstraHeuristic with high weight factor (100000) provides strong guidance while maintaining solution quality on weighted terrain.

**Completeness**: A* is complete for finite graphs. Backward search explores all reachable states from goals.

**Optimality**: Admissible heuristic guarantees optimal solution. Weighted A* (ε > 1) sacrifices optimality for speed—provides ε-suboptimal paths.

**Scalability**: Preprocessing cost scales as O(V log V) for Dijkstra. Per-query A* cost depends on heuristic quality—dominant heuristic reduces expansions significantly.

## Build Instructions

```bash
cd code
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
./run_test ../maps/map0.txt
```

Visualize results:
```bash
cd scripts
python visualizer.py ../maps/map0.txt
```

Output trajectory written to `code/output/robot_trajectory.txt`
