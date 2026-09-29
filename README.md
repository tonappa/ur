# ur — ROS 2 workspace for the `ur_automata` cell

ROS 2 **Jazzy** workspace that drives a Universal Robots arm (the `ur_automata`
cell) with **MoveIt 2**, either in the **URSim** simulator or on the **real
robot**. All development happens inside a Docker container: the repo root is
mounted into the container and built there with `colcon`.

The ROS interface is the same in both cases: same `/joint_states`, same
`/scaled_joint_trajectory_controller/follow_joint_trajectory` action, same
controllers, same MoveIt setup. Only the **controller IP** and the declared
**robot model** change. Underneath, `ur_robot_driver` talks either to a real UR
controller or to the one simulated by URSim, and nothing above it notices.

Main application: a **spherical scan** of the object placed on the platform.
The TCP visits a grid of waypoints on a sphere centered on the object, always
pointing at it; a photo can be taken at each waypoint.

---

## Quick start (URSim)

Full sequence, from a powered-off machine to a scan. Details in the numbered
sections below.

**Host, terminal 1: simulator** (it stays in the foreground).

```bash
cd ~/ur
./start_ursim_seccomp.sh
```

Open PolyScope at http://localhost:6080: *Power on* → *Start* → *OK*, then load
the program with the **External Control** node (Host IP `192.168.56.1`, port
`50002`, see §6.2 for the one-time setup). Do not press Play yet.

**Host, terminal 2: container, build, bring-up.**

```bash
cd ~/ur
./run.sh run                      # opens a shell in the container
# inside the container:
cd /home/ros/ur
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" --executor sequential   # after a change or a fresh clone
source install/setup.bash
ros2 launch ur_automata_bringup ur_automata_bringup.launch.py
```

Now press **Play** in PolyScope (again after every bring-up restart).

**Host, terminals 3 and 4: second and third shell in the same container.**

```bash
docker exec -it ur_container bash
source /opt/ros/jazzy/setup.bash && source /home/ros/ur/install/setup.bash
```

**Calibration** (once, and again after changing the cell: platform, walls,
sphere, end effector, TCP, base pose). Terminal 3:

```bash
ros2 launch ur_automata_scan scan_sequence.launch.py record:=true 2>&1 | tee ~/ur/log/calibration.log
```

Terminal 4, when the table is shown (~1.5 min of planning):

```bash
ros2 service call /scan_sequence_node/start std_srvs/srv/Trigger {}
```

At the end check `Registrazione salvata: ... 0 recovery`.

**Work session** (every time). Terminal 3:

```bash
ros2 launch ur_automata_scan scan_replay.launch.py 2>&1 | tee ~/ur/log/replay.log
```

Terminal 4:

```bash
ros2 service call /scan_replay_node/start std_srvs/srv/Trigger {}
ros2 service call /scan_replay_node/pause std_srvs/srv/Trigger {}   # optional: stops after the current motion
```

**Shut down:** Ctrl+C in the ROS terminals, then `./run.sh down` on the host;
Ctrl+C in terminal 1 stops URSim (the script runs it in the foreground and
removes the container on exit).

---

## 0. Current status

Latest reference runs in URSim (UR5 model, table mount, scan center
`(0.133, 0.375, 0.455)`, radius 0.30 m, full sphere 4 × 15, 82 waypoints),
with `scan_sequence_node`:

| scene | reached | time | planners used | notes |
|---|---|---|---|---|
| `disk.stl` (disk only) | 82 / 82 | 124 s | PTP 81 / OMPL 2 | |
| `platform01.stl` (full platform) | 79 / 82 | 159 s | PTP 72 / OMPL 8 | wp 38, 54, 60 lost: every IK solution collides with the platform. wp 79 is reached but the stem blocks the view |
| `platform01.stl` + line-of-sight check | 78 / 82 | 171 s | PTP 72 / OMPL 8 | wp 60 and 79 dropped because the view is blocked, wp 38 and 54 fail IK |

For comparison, the first scan node (`scan_executor_node`, removed on 2026-09-29) reached 72 / 82 in 322 s
(same sphere, older scene).

**Arm swings (2026-09-29).** The scan is now visited sector by sector, the
sequence cost is the distance the elbow and the TCP actually travel, walls and a
ceiling keep the arm inside the cell, the arm may not hide any part of the
platform disk, and the TCP sits on the real lens center of the V2 end effector
(see §9.2, §9.3, §10); the scan can be recorded once and replayed (§9.4). Measured on full scans in URSim:

| run | reached | time | swings (elbow > 0.25 m) | elbow path | TCP path | OMPL segments |
|---|---|---|---|---|---|---|
| morning: rings, joint-distance cost, no walls | 78 / 82 | 163 s | not measured | – | – | 7 |
| sectors 0°, elbow + TCP cost, no walls | 78 / 82 | 143 s | 20 | 20.0 m | 30.8 m | 5 |
| + sectors 45°, walls | 78 / 82 | 163 s | 12 | 14.0 m | 27.2 m | 5 |
| + RRT*, faster DP convergence | 78 / 82 | 137 s | 13 | 11.4 m | 19.9 m | 3 |
| + whole platform visible, TCP on the lens (current) | 77 / 82 | 130 s | 12 | 12.3 m | 17.9 m | 3 |

The last two rows are not strictly comparable: before the TCP fix the waypoints
were computed for a point 21.5 mm below the camera. The waypoint lost in the
current run is in the lower hemisphere: every collision-free solution there
hides part of the platform.

What is left: 3–5 segments whose straight joint-space line collides, mostly in
the lower hemisphere near the platform stem, still go to OMPL and are the
largest motions (up to ~1.3 m of elbow travel). A set-back path (camera backs
away from the sphere, moves, comes back) was tried for them and is never free
in this cell: the arm is wedged between the platform and the table, and backing
away pushes the upper arm into the platform or the wrist into the table.

Possible next steps:

- Pilz CIRC (arc on the sphere) before OMPL on the blocked segments;
- tighter walls, sweeping them with `dry_run`;
- line-of-sight check with a cone of rays matching the camera field of view,
  instead of the single optical-axis ray;
- photo trigger at each waypoint (on hold): an MQTT "take photo" message to
  the camera side and a "photo taken" reply to the orchestrator before the
  replay moves on; the hook is marked in `scan_replay_node.cpp`;
- bring-up on the real robot (calibration, IP `192.168.1.97`, low scaling,
  diagnosing the occasional abrupt stops seen during execution).

---

## 1. Host prerequisites

- Docker Engine + the `docker compose` plugin
- `nvidia-container-toolkit` (the compose file requests an NVIDIA GPU; without
  one, remove the `deploy.resources.reservations.devices` section from
  `docker-compose.yaml`)
- X11 running (for RViz), `xauth`, `xhost`
- the host user in the `audio` and `video` groups (`run.sh` reads their GIDs)

```bash
sudo usermod -aG audio,video $USER   # then log out and back in
```

## 2. Cloning the repo

The upstream UR packages are git submodules, kept for reference and for
possible patches (the Docker image installs the `ros-jazzy-ur` `.deb` packages
anyway):

```bash
git clone <repo-url> ur
cd ur
git submodule update --init --recursive
```

## 3. Docker image and container

`run.sh` wraps `docker compose`: it derives the names from the current folder
and exports the host user's UID/GID, the X11/Pulse variables, the audio/video
GIDs and the bash-history and VSCode mounts.

```bash
./run.sh build     # build the image (cached)
./run.sh rebuild   # rebuild from scratch (--no-cache, slow)
./run.sh run       # open an interactive shell in the container
./run.sh down      # stop and remove the container
```

With the repo folder named `ur` you get:

| | value |
|---|---|
| image | `ur_ws:jazzy` |
| container | `ur_container` |
| workspace in the container | `/home/ros/ur` |
| network | `host` (needed for DDS discovery and the TCP/IP link to the robot) |
| `ROS_DOMAIN_ID` | `44` |

**Second shell in the same container** (almost always needed: bring-up in one
terminal, scan in the other). `docker exec` does not go through the entrypoint,
so you have to source by hand:

```bash
docker exec -it ur_container bash
source /opt/ros/jazzy/setup.bash
source /home/ros/ur/install/setup.bash
```

The main shell's entrypoint does it on its own: it sources ROS, sources
`install/setup.bash` if it exists, runs `apt update/upgrade` and
`rosdep install --from-paths src --ignore-src -r -y --skip-keys "moveit_resources"`.

## 4. Building the workspace (inside the container)

```bash
cd /home/ros/ur
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" --executor sequential
source install/setup.bash
```

A single package:

```bash
colcon build --packages-select ur_automata_bringup
source install/setup.bash
```

Tests (there is one gtest for the sequence planner; the other packages have no
tests):

```bash
colcon test --packages-select ur_automata_scan
colcon test-result --verbose
```

> **Warning: the config file must be rebuilt.**
> Every launch file reads `automata_config.yaml` from the **installed share**,
> not from `src/`. After every change to the YAML run
> `colcon build --packages-select ur_automata_bringup` and a fresh
> `source install/setup.bash`, otherwise you keep launching the old
> configuration.

Which package to rebuild after a change:

| you changed | rebuild |
|---|---|
| `automata_config.yaml`, URDF base pose | `ur_automata_bringup` (then restart the bring-up) |
| scene code, meshes | `ur_automata_scene` |
| SRDF, MoveIt configs | `ur_automata_moveit_config` |
| scan nodes | `ur_automata_scan` |

## 5. The single config file

`src/automata_robot/ur_automata_bringup/config/automata_config.yaml` is where
*every* parameter lives: bring-up, MoveIt, scene and scan all read it. Three
sections:

```yaml
robot:
  ip: 192.168.56.101          # UR controller (URSim or real)
  type: ur5                   # ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e
  base_xyz: [0.0, 0.0, 0.0]   # base pose in the world frame (m, rad), read by the xacro,
  base_rpy: [0.0, 0.0, 0.0]   # so driver and MoveIt stay consistent. If non-zero, the scene
                              # adds a mounting wall behind the base. Example of a wall-mounted
                              # robot with the pan axis toward the platform:
                              # xyz [0.133, -0.25, 0.455], rpy [-1.5708, 0, 0]
                              # and planning.home_pose_name / lower_home_pose_name: home_wall

planning:
  group: ur_manipulator
  global_frame: world
  end_effector_link: ee_automata_tcp
  trajectory_scaling_factor: 0.4
  home_pose_name: home
  lower_home_pose_name: lower_scan_ready
  use_moveit: true
  use_scene: true
  use_rviz: true

scan:
  center: [0.133, 0.375, 0.455]
  radius: 0.30
  platform_sim: false
  platform_mesh: platform01.stl
  ...
```

Notes on `planning`:

- `end_effector_link: ee_automata_tcp` is the link defined at the end of
  `ur_automata.urdf.xacro` (offset `xyz="0 0.052 0.1745"` from `tool0`, the lens center of the V2 end effector) and must
  match the tip link of the group in the SRDF.
- `home_pose_name` and `lower_home_pose_name` are `group_state`s defined in
  `ur_automata_moveit_config/config/ur_automata.srdf` (`home`, `up`,
  `lower_scan_ready`, `home_wall`). They are the start pose and the recovery
  poses for the upper and lower hemisphere. If you change a name here, it must
  exist there. They are joint values, so they depend on `robot.base_rpy`: use
  `home` / `lower_scan_ready` for the table mount, `home_wall` for the wall mount.
- `trajectory_scaling_factor` is passed to both `setMaxVelocityScalingFactor`
  **and** `setMaxAccelerationScalingFactor` in the scan nodes.
- `use_moveit`, `use_rviz`, `use_scene` are the defaults of the launch arguments
  with the same names and can be overridden from the command line.

The YAML comments also keep the history of the scan positions and mounts that
were tried, with their results.

---

## 6. Bring-up in simulation (URSim)

URSim is the official UR simulator: it runs in its own Docker container and
exposes a complete PolyScope controller. It is not Gazebo: there is no physics
of the environment, but the controller behavior (programs, External Control,
safety) is the real one. It is the most faithful way to test everything before
using the real robot.

### 6.1 Start URSim (on the host, **not** inside the container)

```bash
./start_ursim_seccomp.sh
```

The script:

- downloads the *External Control* URCap v1.0.5 (`.jar`) into `~/.ursim/e-series/urcaps`
- creates the Docker network `ursim_net` (`192.168.56.0/24`, gateway `192.168.56.1`)
- starts `universalrobots/ursim_e-series:5.25.1` with the fixed IP **192.168.56.101**
- mounts the PolyScope programs and settings under `~/.ursim/e-series` (persistent)
- uses `--security-opt seccomp=unconfined`: this is the workaround for kernels
  ≥ 6.15, where `URControl` fails with ENOSYS on `socket()`. The simulator does
  not start with the official `start_ursim.sh` script.

PolyScope interface: browser at **http://localhost:6080** (noVNC) or a VNC
client on `localhost:5900`.

### 6.2 Prepare PolyScope

1. Turn the robot on: *Power on* → *Start* → *OK* (the arm must show
   `RUNNING`, not `IDLE`).
2. *Installation* → *URCaps* → **External Control**:
   - **Host IP**: `192.168.56.1` (the gateway of `ursim_net`, i.e. the host
     running the ROS driver)
   - **Custom port**: `50002`
3. *Program* → add the **External Control** node to the program and save it.
4. Press **Play** *only after* starting the ROS bring-up (step 6.3): the
   program fails if it cannot find the driver listening.

### 6.3 Start the bring-up

In the container, with `robot.ip: 192.168.56.101` in `automata_config.yaml`:

```bash
ros2 launch ur_automata_bringup ur_automata_bringup.launch.py
```

What starts:

- `ur_automata_control.launch.py` → the upstream driver's `ur_control.launch.py`
  with our URDF (`ur_automata.urdf.xacro`), our controllers
  (`ur_automata_controllers.yaml`) and `scaled_joint_trajectory_controller`
  active
- `ur_automata_moveit.launch.py` → `move_group` + RViz with the MotionPlanning
  panel (if `use_moveit`/`use_rviz` are true)
- `scene.launch.py` from `ur_automata_scene` → publishes the planning scene
  (table, platform, object) through `/apply_planning_scene` (if `use_scene`)

Command-line overrides of the top-level arguments:

```bash
ros2 launch ur_automata_bringup ur_automata_bringup.launch.py \
  use_moveit:=true use_rviz:=false use_scene:=true
```

Now press **Play** in PolyScope. The driver log must show
`Robot connected to reverse interface. Ready to receive control commands.`

### 6.4 Quick check

```bash
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 action list | grep follow_joint_trajectory
```

In RViz: drag the interactive marker, *Plan*, then *Execute*. The arm moves in
PolyScope too.

---

## 7. Bring-up on the real robot

### 7.1 Network

Connect the PC to the UR controller over Ethernet and put them on the same
subnet. On the teach pendant: *Settings* → *System* → *Network*, static IP (the
cell uses `192.168.1.97`). From the host:

```bash
ping 192.168.1.97
```

### 7.2 Kinematic calibration (once per robot)

Every UR leaves the factory with its own measured DH parameters: without them
the error at the tool can reach a few millimeters. Do this **once** for each
physical robot, with the robot on and reachable:

```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.1.97 \
  target_filename:="/home/ros/ur/src/automata_robot/ur_automata_bringup/config/ur5e_calibration.yaml"
```

The resulting YAML must then be passed to the bring-up (see below). Without it,
the nominal values from `ur_description` are used: fine for URSim, **not** for
the real robot.

### 7.3 Configuration

In `automata_config.yaml`:

```yaml
robot:
  ip: 192.168.1.97
  type: ur5e            # the actual model in the cell
```

then rebuild `ur_automata_bringup` (see §4).

The calibration file has no entry in the YAML, so launch the two levels
separately. This is the only clean way to pass arguments to the control level,
because the top-level launch file does not re-declare them:

```bash
# terminal 1 — driver + controllers
ros2 launch ur_automata_bringup ur_automata_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.97 \
  kinematics_params_file:=/home/ros/ur/src/automata_robot/ur_automata_bringup/config/ur5e_calibration.yaml

# terminal 2 — MoveIt + RViz (+ scene)
ros2 launch ur_automata_bringup ur_automata_moveit.launch.py \
  ur_type:=ur5e use_rviz:=true use_scene:=true
```

`ur_type` **must be the same in both commands**: otherwise the kinematic model
in `move_group` does not match the TF published by the driver and the
trajectories come out wrong without any obvious error.

Useful arguments of `ur_automata_control.launch.py`:

| argument | default | notes |
|---|---|---|
| `ur_type` | from YAML | UR model |
| `robot_ip` | from YAML | controller IP |
| `kinematics_params_file` | nominal from `ur_description` | YAML from `ur_calibration` |
| `headless_mode` | `false` | `true` = the driver sends the URScript directly, without the External Control program on the pendant |
| `tf_prefix` | `""` | prefix on joints and links |
| `initial_joint_controller` | `scaled_joint_trajectory_controller` | MoveIt expects this one |

### 7.4 Power-on sequence

1. Turn on the controller and release the brakes (*Power on* → *Start*).
2. Load the program with the **External Control** node (Host IP = IP of the PC
   running ROS, port `50002`).
3. Start the ROS bring-up.
4. Press **Play** on the pendant.

> **Every time you restart the bring-up you must press Play again**: when the
> driver stops, the URScript program on the controller ends and has to be
> restarted by hand.

### 7.5 First motion

- Lower `trajectory_scaling_factor` to `0.05`–`0.1` for the first moves.
- Set the PolyScope *speed slider* to 20–30%: the
  `scaled_joint_trajectory_controller` respects it and slows down the
  trajectory without deforming it.
- Keep the emergency stop within reach; check that the MoveIt scene (table,
  platform, object) really matches the physical layout before running a full
  scan.

---

## 8. Simulation vs real robot: what to change

| | URSim | Real robot |
|---|---|---|
| `robot.ip` | `192.168.56.101` | controller IP (e.g. `192.168.1.97`) |
| `robot.type` | `ur5` (URSim starts with `ROBOT_MODEL=UR5`) | the real model, e.g. `ur5e` |
| `kinematics_params_file` | nominal default is fine | **required**: the YAML from `ur_calibration` |
| `trajectory_scaling_factor` | `0.4` is fine | start from `0.05`–`0.1` |
| `planning.use_rviz` | `true` | `true` to check, `false` in production |
| `scan.platform_sim` / `platform_mesh` | either | `false` + the STL that matches the real platform |
| External Control | Play in PolyScope via noVNC | Play on the teach pendant, after every bring-up |
| start | `./start_ursim_seccomp.sh` + bring-up | bring-up only |

Everything else (controllers, MoveIt, scene, scan nodes) stays the same.

---

## 9. Running the scan

With the bring-up running, in a second container shell:

```bash
ros2 launch ur_automata_scan scan_sequence.launch.py
```

The node:

1. generates the waypoints on the sphere (`scan.center`, `scan.radius`,
   hemisphere, direction, equator exclusion) and puts them in visit order:
   ring by ring, or sector by sector with `sectors` > 0 (§9.3);
2. **discards waypoints whose view is blocked**: it casts a ray from the
   waypoint to the center against the scene objects and drops the waypoint if
   something is in the way (for example the platform stem). See §9.2;
3. **enumerates offline** the IK solutions of each remaining waypoint (TRAC-IK
   in `Speed` mode, several pitch values and several seeds per branch),
   discarding those that collide with the scene or where the arm itself blocks
   the camera→center view;
4. picks the sequence with a **layered dynamic program (DP)**: for every
   waypoint it chooses one IK solution so that the arm
   travels as little as possible in space (elbow path + TCP path along each
   joint-space segment, plus a small joint term);
5. executes waypoint by waypoint, trying the `planners` chain and falling back
   to a nearby point if a waypoint is unreachable;
6. goes back to `home` and prints a summary (reached / failed, causes, time,
   how many times each planner was used, how many waypoints were dropped for a
   blocked view, arm swings measured on the executed trajectories).

**The scan always starts paused.** To start or pause it:

```bash
ros2 service call /scan_sequence_node/start std_srvs/srv/Trigger {}
ros2 service call /scan_sequence_node/pause std_srvs/srv/Trigger {}
```

If the node runs in an interactive terminal (not through `ros2 launch`), keys
also work: **SPACE** = pause/resume, **Q** = quit.

To save a run for later comparison:

```bash
ros2 launch ur_automata_scan scan_sequence.launch.py 2>&1 | tee ~/ur/log/<name>.log
# summary lines, ANSI colors stripped:
sed 's/\x1b\[[0-9;?]*[A-Za-z]//g' ~/ur/log/<name>.log \
  | grep -E 'Sequenza:|Previsione:|Scan complete|Planner usati|Tempo scansione|Sbracciate:' | sort -u
```

> The waypoint markers (spheres + orientation arrows, colored by status) are
> published on `/scan_waypoints_markers`. The bring-up's `automata.rviz` config
> already has the **Scan waypoints** display on that topic, so you do not need
> to add it by hand. If you launch RViz with another config, create the display.

### 9.1 Scan parameters (`scan:`)

**Geometry**

| parameter | effect |
|---|---|
| `center` | sphere center in the `planning.global_frame` frame, in meters (top face of the platform disk) |
| `radius` | sphere radius. This is the hardest constraint: increasing it quickly pushes waypoints out of reach |
| `hemisphere` | `upper` \| `lower` \| `full` (upper first, then lower) |
| `direction` | `latitudinal` (ring by ring) \| `longitudinal` (meridian by meridian) |
| `num_rings`, `points_per_ring` | density in latitudinal mode |
| `num_arc`, `points_per_arc` | density in longitudinal mode |
| `equator_exclusion_upper_deg`, `equator_exclusion_lower_deg` | forbidden band around the equator: the robot cannot point in line with the support. The two hemispheres can use different values |
| `stagger_rings` | even rings shifted by half a step → more uniform coverage |
| `adaptive_rings` | points per ring scale with `sin(theta)`: fewer near the pole, more near the equator |

**TCP orientation**

| parameter | effect |
|---|---|
| `lock_pitch` | `true`: TCP X axis horizontal, no rotation about Y. `false`: free pitch, the node searches for an offset with a valid IK |
| `pitch_search_range_deg`, `pitch_search_step_deg` | range and step of the pitch search |
| `pitch_xparallel_bias` | `0.0` = any pitch is fine; `0.05` = mild preference for X parallel; `>0.3` = X parallel almost always |
| `occlusion_check`, `occlusion_disk_radius`, `occlusion_margin` | discards IK solutions where an arm link hides part of the platform disk from the camera. Sight lines go from the camera to the disk center and to 24 points on its rim (QR codes all around the platform, on top and below) and are checked against the real arm collision meshes, in both hemispheres. `occlusion_margin` = how close a link may come to a line of sight. |

**IK and planning**

| parameter | effect |
|---|---|
| `enum_ik_timeout` | IK timeout during the enumeration phase (`scan_sequence_node`). Worst-case cost of the phase = waypoints × pitch values × seeds × timeout |
| `planners` | chain used by `scan_sequence_node`: each segment tries the planners in order and stops at the first that succeeds. Default `[pilz_ptp, ompl]`; the full chain `[pilz_circ, pilz_ptp, stomp, ompl]` was measured slower (262 s vs 143 s) without reducing the segments that fall back to OMPL |
| `ompl_algorithm` | used when planning with OMPL (`RRTConnect`, `RRTstar`, `PRM`, …) |
| `planning_time`, `planning_attempts` | time of one OMPL request and parallel attempts in it. 15 s here: with RRTstar every OMPL segment uses the whole time to shorten the path, and it is only paid in the calibration run |
| `retry_planning_times`, `retry_ompl_algorithm` | when nothing is found in `planning_time`, try again with each of these times (e.g. `[30.0, 60.0]`) and with this OMPL algorithm (default `RRTConnect`) before going to the recovery pose. RRTstar grows one tree from the start and can miss a narrow passage for minutes (wp 13 under the platform: nothing in 15 + 30 + 60 s); RRTConnect grows trees from both ends and gets through. Planning time only matters in the calibration run, the replay does not plan |
| `fallback_search`, `fallback_radius_mm` | offline search for an alternative point on the sphere near a waypoint that has no valid IK solution |

Allowed entries in `planners`:

| entry | what it does | when it fails |
|---|---|---|
| `pilz_circ` | arc on the sphere centered at `scan.center`: the TCP stays on the sphere and the camera frames the object for the whole segment | when not starting from a point on the sphere (skipped from `home` and from the recovery poses), near singularities, or if the arc collides |
| `pilz_ptp` | straight line in joint space, deterministic, ~10 ms | when the line goes through an obstacle |
| `stomp` | starts from the same line and deforms it until it is collision-free: smooth, repeatable path | when the obstacle is too large for a local deformation |
| `ompl` | OMPL with `ompl_algorithm`: RRTConnect returns the first path it finds (fast, often long), RRTstar keeps shortening it for the whole `planning_time` | RRTstar can miss narrow passages: see `retry_planning_times` |

The final summary prints a `Planner usati:` line with the count per planner:
this is the metric used to compare two runs. Many segments on `ompl` mean
large, unpredictable motions.

> `pilz_*` **ignores** the path constraint used when `lock_pitch: false`.
> `pilz_circ` is the exception: it uses its own path constraint (the arc
> center), which the node sets and removes around that single attempt.
> `stomp` needs `stomp_planning.yaml` in `ur_automata_moveit_config/config`.

**Dry run.** To see the planned sequence and its forecast without moving the
robot (bring-up running, no Play needed):

```bash
ros2 launch ur_automata_scan scan_sequence.launch.py dry_run:=true 2>&1 | tee ~/ur/log/<name>_dry.log
```

It stops after the DP (~1 min) and prints the `Previsione:` line (forecast arm
swings, elbow path, segments that will need OMPL), the list of those segments
and the **arm envelope**: the box holding the arm links in every configuration
the scan needs (chosen waypoint solutions, start, `home`, `lower_scan_ready`),
with a proposed `walls:` block (that box + 0.10 m). Walls there cut none of the
needed configurations but keep the arm from swinging out. Use it to compare
settings before a real run.

### 9.2 Line-of-sight check

Before the IK enumeration, `scan_sequence_node` casts the segment
waypoint → `scan.center` against the world objects of the planning scene
(meshes triangle by triangle, primitive shapes through `geometric_shapes`). If
the segment hits something, the waypoint gets the `OCCLUDED` status: no IK
search and no fallback, orange marker in RViz, and a separate counter in the
summary (`Vista coperta: N waypoint scartati`). Fallback candidates go through
the same check.

Ignored by the check: `support_center`, `artefact`, the margin keep-outs
(`platform_margin`, `table_margin`) and hits within 3 cm of the center (the
disk right under the object, crossed by every lower-hemisphere ray a few
millimeters from the center). The check is always on and uses a single ray
along the optical axis, not the full camera field of view.

This is different from `occlusion_check` (§9.1), which looks at the *robot
arm* hiding the platform for a given IK solution. The line-of-sight check on
scene objects stays on the single center ray: the platform's own stem hides part
of the bottom rim from below anyway, on its side.

### 9.3 Sectors and arm swings

The classic order walks each ring all the way around the object (360°). Going
around, the arm has to switch configuration (elbow, wrist) somewhere, usually
behind the object, and that switch is a big, visible swing. With `sectors: 4`
every hemisphere is split into 4 azimuth slices (front = robot side, side,
back, other side) and visited slice by slice, rings in serpentine order inside
the slice.

The DP cost of a segment is what we want to keep small: the path of the elbow
plus the path of the TCP along the straight joint-space line (the path of Pilz
PTP), computed with forward kinematics, plus a small joint term.

| parameter | effect |
|---|---|
| `sectors` | slices per hemisphere; `0` = classic ring-by-ring order |
| `sector_offset_deg` | rotation of the slices (counterclockwise seen from above). `0` = sector 0 centered on the robot side (front / right / back / left); `45` = boundaries on the robot → center axis (front-right / back-right / back-left / front-left) |
| `joint_cost_weight` | DP segment cost = elbow path + TCP path (m) + this × joint distance (rad). The joint term keeps big wrist spins from being free |
| `swing_threshold_m` | a motion is reported as an arm swing when the elbow travels more than this |

The summary line `Sbracciate:` counts the executed motions whose elbow path is
above the threshold and lists them, with the planner that produced each one.
In the forecast, blocked segments (straight line in collision) are listed as
going to OMPL.
Recoveries and the final return to `home` are not counted.

Tried and dropped on 2026-09-29: per-joint weights with an IK-branch penalty in
the DP cost (more swings than the plain joint distance); a home pose per
sector (the DP never chose it when it was optional: a detour always makes the
arm travel more); a set-back path on blocked segments, i.e. the camera backs
away from the sphere by 5-15 cm, moves and comes back with three PTP motions
(never free in this cell: backing away pushes the upper arm into the platform
or the wrist into the table).

### 9.4 Calibration and replay

Planning a scan takes ~1.5 min before the robot moves (IK enumeration and
sequence), and every blocked segment waits for OMPL. For work sessions the scan
is **recorded once** and then **played back** with the same motions and timing
every time.

**Calibration** (once, and again after changing anything in the cell):

```bash
ros2 launch ur_automata_scan scan_sequence.launch.py record:=true 2>&1 | tee ~/ur/log/calibration.log
ros2 service call /scan_sequence_node/start std_srvs/srv/Trigger {}
```

A segment that OMPL cannot solve in `planning_time` is tried again with
`retry_planning_times` before going to the recovery pose: the calibration may
take longer, the replay does not. At the end of a complete scan every executed
motion (waypoints, recoveries, return to `home`) is saved with its full timing to `scan.recording_file`
(default `recordings/scan_sequence.yaml` in the repo). An interrupted scan is
not saved. If the run needed recoveries they are recorded too, and the summary
suggests running the calibration again.

**Work session:**

```bash
ros2 launch ur_automata_scan scan_replay.launch.py
ros2 service call /scan_replay_node/start std_srvs/srv/Trigger {}
ros2 service call /scan_replay_node/pause std_srvs/srv/Trigger {}   # stops after the current motion
```

Before moving, `scan_replay_node` refuses the recording if:

- scan center, radius, planning group or end-effector link differ from
  `automata_config.yaml`;
- the current robot model does not put the camera where it was recorded at
  the end of every waypoint motion (2 mm / 1°): TCP, end effector or base pose
  changed;
- any recorded point collides with the current planning scene (walls, platform
  or anything else changed).

Then it moves to the first recorded point with a planned motion and plays the
recorded motions one after the other. MoveIt refuses a motion if the robot is
not where it starts. The recorded speed is the one of the calibration
(`trajectory_scaling_factor`); the PolyScope speed slider still slows it down.

### 9.5 Scene

`platform_sim: true` builds the platform as a disk + 3 cylindrical legs.
`false` loads the STL file named by `platform_mesh` from
`ur_automata_scene/meshes/` (exported in mm, so scaled ×0.001). The mesh is
placed so that the top face of the scanned disk matches `scan.center`: moving
the platform only means changing `scan.center`. Switching STL files only needs
a rebuild of `ur_automata_bringup` (it is a YAML key), not of the scene package.

| `platform_mesh` | content |
|---|---|
| `disk.stl` | only the Ø300 × 4 mm disk close to the robot. There is **no support** under the disk in the scene |
| `platform01.stl` | the full rotating platform: two disks at ±0.20 m from the axis, hub, stem, base with the motor on the side away from the robot; ~28k triangles. The base ends 4.9 cm below the table top (the platform is 0.504 m tall, `center.z` is 0.455); harmless for planning |

The object to scan is `meshes/ceramic_model.obj`, placed at `scan.center`.

`platform_margin` and `table_margin` (meters, `0` = off) add two
semi-transparent yellow keep-out zones: a cylinder around the disk
(radius + margin, thickness + 2·margin) and a slab *margin* tall on the table
under the sphere, starting at y = 0.12 so it does not touch the base. MoveIt's
collision check is binary, so this is how a safety margin is obtained; it
applies to the IK candidate filter and to the planners, paths included. A thick
object cannot be "skipped" between two checks the way a 4 mm plate can.
Careful under the disk: there are ~28 cm between table and platform and the arm
goes in edgewise, so a 2 cm margin on both is enough to lose the lowest
waypoints.

`walls` (`back_y`, `left_x`, `right_x`, `top_z`, meters, `0` = no wall) adds
the cell walls behind and beside the robot, never in front, and a ceiling at
height `top_z` (it must stay above the highest waypoint, `center.z + radius`,
plus the end effector). They limit the space where
the planners can wander, they do not shorten the paths inside it.
Constraints: `back_y` no further than −0.42 (at `home` the elbow reaches
y −0.37), `right_x` not below 0.75 (at `lower_scan_ready` the end effector
reaches x 0.71).

The farthest point of the sphere from the base is at `|scan.center| + radius`:
with this end effector the practical UR5e limit is ~0.91 m, so with
`radius: 0.30` the center must stay within ~0.60 m of the base.

---

## 10. Repo layout

```
docker/                  Dockerfile, entrypoint, Python requirements
docker-compose.yaml      ros_dev service: GPU, X11, host network, workspace mount
run.sh                   build/rebuild/run/down wrapper
start_ursim_seccomp.sh   URSim start with the seccomp workaround
recordings/              recorded scans for scan_replay_node (written by the calibration run)

src/automata_robot/
  ur_automata_bringup/         single config + top-level launch files (control, moveit, bringup) + RViz
  ur_automata_description/     cell URDF/xacro, end-effector mesh, visualization-only launch
  ur_automata_moveit_config/   SRDF, kinematics (TRAC-IK), limits, OMPL/Pilz/STOMP pipelines, MoveIt controllers, generated launch files
  ur_automata_scene/           planning scene: table, platform, object, walls; STL/OBJ meshes
  ur_automata_scan/            spherical waypoint generation, sequence planner (DP), executor nodes,
                               scan recording + replay node, gtest

src/utils/                     upstream UR submodules (driver and description), jazzy branch — read-only
```

View the model only, without driver or MoveIt:

```bash
ros2 launch ur_automata_description display.launch.py ur_type:=ur5e
```

Try MoveIt without robot and without URSim (mock hardware):

```bash
ros2 launch ur_automata_moveit_config demo.launch.py
```

The end effector is the `ee_automata_V2.stl` mesh attached to `tool0`; the TCP
`ee_automata_tcp` is at `xyz = (0, 0.052, 0.1745)` from `tool0` (lens center of V2). Changing the
end-effector version means updating both the mesh **and** the TCP offset in
`ur_automata.urdf.xacro`.

---

## 11. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| I edit `automata_config.yaml` and nothing changes | launch files read the installed share: `colcon build --packages-select ur_automata_bringup` + `source install/setup.bash` |
| The driver starts but the robot does not move | **Play** on External Control is missing: press it again after every bring-up restart. Alternatively `headless_mode:=true` |
| `Can't accept new action goals. Controller is not running` | the program on the controller is not running (same issue as above) |
| URSim does not start, `URControl` errors out | kernel ≥ 6.15: use `./start_ursim_seccomp.sh`, not the official script |
| PolyScope does not see the URCap | the `.jar` must be in `~/.ursim/e-series/urcaps`; old `.urcap` files are no longer recognized by PolyScope 5.25 |
| External Control does not connect from URSim | Host IP must be `192.168.56.1` (the `ursim_net` gateway), not `127.0.0.1` |
| `No kinematics solver instantiated for group ur_manipulator` | the application node did not load `kinematics.yaml`: the `ur_automata_scan` launch files pass it under `robot_description_kinematics`, a hand-written node must do the same |
| Plausible trajectories but wrong positions | `ur_type` differs between control and moveit, or the calibration file is missing on the real robot |
| I cannot see the waypoint markers in RViz | RViz must run with `automata.rviz` (the bring-up does this) and the **Scan waypoints** display must be enabled; the topic is `/scan_waypoints_markers` |
| Many waypoints fail IK | `radius` too large or `center` too far from the base: the sphere leaves the arm's reach |
| Some waypoints are `OCCLUDED` | a scene object (e.g. the platform stem) is between the waypoint and the center: expected, see §9.2 |
| Many planning failures | raise `planning_time` and `planning_attempts`, or add `ompl` at the end of `planners` |
| Plan fails in a few ms with `INVALID_MOTION_PLAN`, move_group says `ValidateSolution: Computed path is not valid` | the planner found a path but checked it with too large a step and it grazes a thin obstacle: lower `longest_valid_segment_fraction` in `config/ompl_planning.yaml` (currently 0.001 ≈ 1.5°) |
| move_group warns `Cannot find planning configuration ... kConfigDefault` | the entry is missing from `planner_configs` in `ompl_planning.yaml`: OMPL ignores `scan.ompl_algorithm` and uses RRTConnect |
| The node only reports a generic FAILURE | the real causes (Pilz limits, ValidateSolution, OMPL unable to solve) are in the move_group log: `ls -t /home/ros/.ros/log/move_group_*.log \| head -1` inside the container |
| `scan_replay_node` refuses to start | it prints why: config (center, radius, end effector) differs from the recording, the camera poses do not match the current robot model, or a recorded point collides with the current scene. Run the calibration again (`record:=true`) |
| RViz does not open from the container | `xhost +local:docker` (already done by `./run.sh run`) and `DISPLAY` set on the host |
