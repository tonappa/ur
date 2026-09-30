# ur_automata_scan_tsp — experimental scan planner

Experimental alternative to `scan_sequence_node` (package `ur_automata_scan`),
kept separate so it can be compared with it without changing it. It chooses
**both the visit order of the waypoints and the IK configuration used at each
waypoint** to reduce the total motion time, plans the whole scan once with
MoveIt **without moving the robot**, saves the order and the full trajectory,
and replays them in later runs after checking that they still fit the cell.

`ur_automata_scan` is not modified: this package links its installed libraries
(`sphere_waypoint_generator`, `scan_sequence_planner`, `scan_recording`) and
headers (`planning_scene_client.hpp`). The candidate filters and the planner
chain of `scan_sequence_node` are static functions inside that node, so they are
copied in `src/scan_common.cpp` with the same logic.

## Method

Reference: F. Suárez-Ruiz, T. S. Lembono, Q.-C. Pham, *RoboTSP – A Fast
Solution to the Robotic Task Sequencing Problem*, arXiv:1709.09343 (ICRA 2018).
RoboTSP separates the problem in two: first the visit order (a TSP), then, for
that order, the configuration of each task (a shortest path in a layered graph).

`mode:=plan` does:

1. **IK candidates**: every valid configuration of every waypoint, with the same
   filters as `scan_sequence_node` (pitch × IK branches, scene collisions,
   whole platform visible, line of sight, slow retry, fallback points). The
   waypoints are the same list, numbered like the `#N` of its table.
2. **Visit order**, `order:=`
   - `alternate` (default, the best measured): starts from the solution of
     `scan_sequence_node` (sector order + DP), then repeats: OR-Tools TSP on
     the configurations chosen so far, DP on the new order. A step is kept
     only if the estimated cost goes down.
   - `tsp` (RoboTSP-like): OR-Tools routing solver on the waypoints, from the
     current state to the home pose. Cost of i → j = the cheapest pair of their
     candidates.
   - `gtsp` (joint search): OR-Tools on **all** the candidates, one cluster per
     waypoint (generalized TSP, one node per cluster): order and configuration
     are chosen together.
   - `sectors`: the order of `scan_sequence_node` (sector by sector), as a
     reference for the effect of the order alone.
3. **Configurations for that order**: the layered DP of `ur_automata_scan`
   (`choose_sequence`), with the same lazy check of the straight joint-space
   segments; the return to home is its last layer.
4. **MoveIt**: every segment is planned with the planner chain of
   `automata_config.yaml` (`planners`, retries, recovery pose when nothing is
   found, Pilz CIRC only from a waypoint of the sphere, as in
   `scan_sequence_node`). The planned durations are measured.
5. **Refinement**: the measured durations replace the estimates (segments
   that go to OMPL are the badly estimated ones), steps 2–4 run again and only
   new segments are planned. It stops after `refine_iterations` rounds or when
   the sequence does not change; the best measured plan is saved.

Costs are durations. The estimate of a segment is the time of a Pilz PTP
motion on the straight joint line: trapezoidal (or triangular) profile per
joint, joints synchronized on the slowest, velocity limits of the URDF and
acceleration limits of `joint_limits.yaml`, both scaled by
`trajectory_scaling_factor`. A segment whose straight line collides costs
`blocked_penalty_s` more until its real duration is known.

The orders are **heuristic** solutions (OR-Tools guided local search with a
time limit); nothing here proves them optimal.

## Usage

Bring-up and URSim as in the main README. Build (inside the container):

```bash
colcon build --packages-select ur_automata_scan_tsp --cmake-args -DCMAKE_CXX_FLAGS="-w"
source install/setup.bash
```

**Plan and save** (no motion; the robot must be in the pose the scans will start
from, normally `home`):

```bash
ros2 launch ur_automata_scan_tsp scan_tsp.launch.py mode:=plan            # order:=alternate
ros2 launch ur_automata_scan_tsp scan_tsp.launch.py mode:=plan order:=gtsp plan_file:=/home/ros/ur/recordings/scan_gtsp.yaml
```

**Load and execute** (starts paused):

```bash
ros2 launch ur_automata_scan_tsp scan_tsp.launch.py mode:=execute
ros2 service call /scan_tsp_node/start std_srvs/srv/Trigger {}
ros2 service call /scan_tsp_node/pause std_srvs/srv/Trigger {}
```

While executing, RViz shows a small camera frustum on every waypoint of the
plan (topic `/scan_waypoints_markers`): gray = not reached yet, green =
reached. The waypoints the plan does not reach are not drawn.

Launch arguments: `mode` (plan | execute), `order` (alternate | tsp | gtsp |
sectors), `plan_file` (default `/home/ros/ur/recordings/scan_tsp.yaml`),
`tsp_time_limit` (s per OR-Tools solve, default 10), `refine_iterations`
(default 3), `blocked_penalty_s` (default 10), `tsp_check_neighbours` (default
10), `start_tolerance_rad` (default 0.01), `speed` (execute only: 1.0 = as
planned, 0.5 = half speed, same path; values above 1.0 are refused). All the scan settings come from
`automata_config.yaml`, like `scan_sequence_node`.

## Results

URSim, table cell, start and end in `home`, `trajectory_scaling_factor` 0.4.
Every run reaches 77 of 82 waypoints. Motion time = sum of the planned
trajectory durations, return to home included.

| method | motion time | joint travel | OMPL segments |
|---|---|---|---|
| `scan_sequence_node`, calibration of 2026-09-29 | 129.2 s | 255.5 rad | 3 |
| `scan_sequence_node`, calibration of 2026-09-30 | 115.0 s | 223.8 rad | 5 |
| `order:=sectors` | 129.4 s | 275.8 rad | 7 |
| `order:=tsp tsp_check_neighbours:=0` | 132.6 s | 278.3 rad | 9 |
| `order:=tsp`, two runs | 92.6 s, 137.1 s | 176.0, 298.8 rad | 4, 8 |
| `order:=gtsp tsp_check_neighbours:=0` | 112.7 s | 230.7 rad | 7 |
| `order:=gtsp tsp_time_limit:=60` | 118.7 s | 250.8 rad | 6 |
| `order:=gtsp` | 104.9 s | 220.7 rad | 8 |
| **`order:=alternate`**, two runs | **90.8 s, 96.4 s** | 178.4, 186.8 rad | 1, 2 |

- `alternate` is the shortest in both runs: 21 % less motion time than the
  calibration of 2026-09-30, 30 % less than the one of 2026-09-29.
- `tsp` is not stable (92.6 s and 137.1 s with the same settings): the cost of
  i → j is the cheapest pair of candidates, which ignores that consecutive
  waypoints must use compatible IK branches.
- `gtsp` has about 1800 nodes and OR-Tools does not converge in the time limit.
- The same configuration of `scan_sequence_node` gave 129.2 s and 115.0 s in
  two calibrations: the difference comes from the OMPL segments, which are
  random.
- The PTP estimate matches Pilz; the error of the estimate comes only from the
  segments that go to OMPL.

`mode:=execute` of the 90.8 s plan (2026-09-30): loading 0.109 s, checks
0.133 s, then the motions. A plan altered on purpose (platform moved by 5 cm,
`scan_num_rings` 5 instead of 4) is refused with both reasons and the robot
does not move. With `speed:=0.5` the same plan takes 181.7 s of motion
(186.3 s measured from `/start` to the end).

## Plan file

YAML, the same keys as the recordings of `scan_sequence_node` (so
`scan_replay_node` can also play it) plus a `plan:` section:

| key | used for |
|---|---|
| `method`, `visit_order`, `metrics` | which order, what was measured |
| `waypoints` (x y z qx qy qz qw each), `waypoint_params` | recognize the waypoints |
| `joint_names`, `start_joints`, `scaling`, `max_velocity`, `max_acceleration` | recognize the robot setup and the start state |
| `camera_pose` of every waypoint motion | recognize TCP / end effector / base (forward kinematics) |
| `scene` (box of every world object) | recognize the scene |

Before executing, `mode:=execute` checks all of them and lists **every**
reason the plan does not fit: different group or end effector, different
waypoint settings or positions, different scaling or joint limits, robot not in
the start state (`start_tolerance_rad`), scene object moved (> 1 mm), missing
or new, camera pose not matching the current model (> 2 mm / 1°), any
trajectory point colliding with the current scene. Then it asks for a new
`mode:=plan` and does not move.

## Dependencies and licenses

| dependency | version | license | how |
|---|---|---|---|
| OR-Tools (C++) | 9.15.6755 | Apache-2.0 | binary archive in `/opt/ortools` (`docker/Dockerfile`, sha256 checked) |
| MoveIt 2 | Jazzy | BSD-3-Clause | apt, already used |
| yaml-cpp, Eigen, geometric_shapes | Jazzy | MIT / MPL-2.0 / BSD-3-Clause | apt, already used |

Libraries bundled in the OR-Tools archive (all loaded by `libortools.so`,
licenses checked in `/opt/ortools/share` and upstream):

| library | version | license |
|---|---|---|
| abseil-cpp | 20250814 | Apache-2.0 |
| protobuf (with utf8_range, upb) | 33.1 | BSD-3-Clause |
| re2 | 11 | BSD-3-Clause |
| HiGHS | 1.12 | MIT |
| SCIP | 10.0 | Apache-2.0 |
| SoPlex (with fmt, zstr) | 8.0 | Apache-2.0 (fmt, zstr: MIT) |
| Coin-OR Cbc, Cgl, Clp, CoinUtils, Osi | 2.10.12 / 0.60.9 / 1.17.10 / 2.11.12 / 0.108.11 | EPL-2.0 |
| Boost (static, inside SCIP/SoPlex) | 1.87 | BSL-1.0 |
| zlib | 1.3.1 | zlib |
| bzip2 | 1.0.9 | bzip2 (BSD-like) |

All are permissive except Coin-OR (**EPL-2.0**, weak copyleft): commercial use
and distribution are allowed; modified Coin-OR source files must be released
under EPL-2.0, and binaries must say where their source is available. This
package does not modify them and only uses the routing solver.

**Not used:** LKH / GLKH, the solvers of the original RoboTSP implementation:
their author distributes them "for academic and non-commercial use" and
reserves all other rights.
