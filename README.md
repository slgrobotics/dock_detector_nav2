Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)

-----------------------

# Named LiDAR docks for Nav2

**Contents:**

- [What is this package for?](#what-is-this-package-for)
- [Technical overview](#technical-overview)
- [Build](#build)
- [Catalogue and configuration](#catalogue-and-configuration)
- [Selection and freshness](#selection-and-freshness)
- [Approach and arrival tracing](#approach-and-arrival-tracing)
- [Verification](#verification)

-------------

## What is this package for?

A robot may know how to drive around a room but still need help finding the exact
spot where it should park. This package connects its navigation software, Nav2,
to the LiDAR dock detector.

For example, when Seggy is told to use the garage dock, this connection asks the
detector to find that named bay. It passes fresh location reports back to Nav2,
which handles driving to the bay, reversing into it, and trying again if docking
fails. Reports from the wrong dock or an old search are rejected. This package
provides the connection; Nav2 remains in charge of the docking motion.

**Related packages and data:**
- https://github.com/slgrobotics/dock_detector
- https://github.com/slgrobotics/dock_detector_interfaces
- https://github.com/slgrobotics/dock_detector/tree/main/media

## Technical overview

`dock_detector_nav2::LidarDock` derives from Nav2's SimpleNonChargingDock. It
connects a normal DockRobot dock_id to a Python dock detector catalogue, retaining
Nav2 staging, reverse approach, retries, actions and noncharging undocking.
No custom BT node or modification of Nav2 source is required.

## Build

Verified against the installed compatibility overlay 1.5.3-21-g4cc5cae3:

```bash
source /opt/ros/lyrical/setup.bash
source ~/nav2_compat_ws/install/setup.bash
cd ~/robot_ws
colcon build --packages-select dock_detector_interfaces dock_detector dock_detector_nav2 articubot_one
source install/setup.bash
```

Use the same overlay order at runtime. Plugin ABI compatibility with other Nav2
builds is not established. The plugin links exported Nav2 libraries; the Nav2
source tree is not modified or compiled by this build.

## Catalogue and configuration

Seggy simulation now enables this plugin through its normal bringup. See
[Seggy simulation docking](../articubot_one/docs/seggy_docking/README.md) for the
persistent robot-specific catalogue/tuning files and automatic launch-time merge.
The manual generator below remains available for other integrations and inspection.


`dock_detector/config/seggy_docks.yaml` is an example single source for both
perception and Nav2's named dock database. Dock IDs are catalogue keys (not separate
UUIDs). Important: Nav2 forwards the database entry's optional `id` field to
getRefinedPose, not its lookup key. The generator explicitly writes `id: <catalogue_key>`
for every entry. Handwritten Nav2 database entries must do the same; otherwise
selection receives an empty or unknown ID and fails. Common scan/frame/timing settings are robot-wide; each dock may override
its target_shape, expected_dock_x/y/yaw_deg and nested geometry parameters. Both
trapezoid and V profiles use inward Nav2 headings. World/map alignment must be
calibrated; example world coordinates are not automatically map coordinates.

Generate a NEW full Nav2 parameter file from the existing robot configuration:

```bash
ros2 run dock_detector generate_nav2_docking_config \
  --base ~/robot_ws/src/articubot_one/robots/seggy/config/nav2_params.yaml \
  --catalogue ~/robot_ws/src/dock_detector/config/seggy_docks.yaml \
  --output /tmp/seggy-nav2-lidar-docks.yaml
```

The output is exclusive-created. It preserves other Nav2 nodes and replaces the
docking type/database with one backward noncharging LiDAR plugin. Review the
output, map alignment, base frame, local costmap and footprint topic names,
physical clearance and safety configuration before using it. Defaults: 0.035 m/s
maximum, 0.08 rad/s maximum, 0.025 m XY arrival threshold, 90 s approach timeout,
0.8 m staging distance. Seggy simulation overrides staging to 0.5 m. Near-dock trajectory-check exclusions remain Nav2's
behavior; Collision Monitor remains in the command path.

Start the detector in catalogue mode:

```bash
ros2 launch dock_detector detector.launch.py \
  params_file:=$HOME/robot_ws/src/dock_detector/config/seggy_docks.yaml \
  use_sim_time:=true
```

Restart your usual Seggy bringup with
`nav2_params_file:=/tmp/seggy-nav2-lidar-docks.yaml`. The generic navigation wrapper
now accepts this optional override and otherwise retains its existing default.
Alternatively pass `params_file` to the lower-level navigation_launch.py. Never
launch a second navigation stack or docking server alongside the existing one.
The docking server's commands are now remapped to cmd_vel_raw in both launch
modes, preserving smoother -> Collision Monitor -> mux -> controller routing.
Changes take effect after rebuilding/restarting, not in a running navigation stack.
For hardware set use_sim_time:=false and calibrate catalogue poses/geometry.

Ordinary Nav2 requests then work:

```bash
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
  "{use_dock_id: true, dock_id: sim_trapezoid_bay, navigate_to_staging_pose: true}" --feedback
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
  "{use_dock_id: true, dock_id: sim_v_bay, navigate_to_staging_pose: true}" --feedback
ros2 action send_goal /undock_robot nav2_msgs/action/UndockRobot \
  "{dock_type: lidar_dock, max_undocking_time: 60.0}" --feedback
```

These are commissioning commands, not commands exercised in the implementation
verification. Existing `opennav_docking_bt` DockRobot/UndockRobot nodes can call
these same actions. Battery/charging semantics are not implemented by this plugin.
The inherited arrival check is XY distance, not yaw/dwell/contact confirmation.

## Selection and freshness

Catalogue mode starts idle. Nav2 starts acquisition after staging. On the first
getRefinedPose(pose,id), the plugin asynchronously calls SelectDock with the ID
and a random acquisition token. It returns false until acknowledgement and a
fresh identified observation arrive. There is no blocking spin inside the plugin.
It enforces a steady-clock selection deadline in addition to Nav2's perception
timeout. Unknown or empty IDs fail explicitly; pose-only DockRobot goals are not
supported. Selection clears pending scans and measurement/filter state. Scans
predating selection are discarded.

The plugin consumes `dock_detector/detection` (DockDetection), not anonymous
PoseStamped messages. It checks dock ID, acquisition token, measurement timestamp,
finite position, quaternion normalization, ROS-time age and steady receipt age.
Duplicates/out-of-order observations do not refresh the watchdog. Filtering runs
once per new measurement and resets on every acquisition. Stale detection blocks
arrival checks as well as pose refinement. Detection loss follows Nav2's retry
behavior, with no experimental-controller cached-goal grace.

Stop/cancel/deactivate clears local state immediately and sends a best-effort
conditional release. A delayed release cannot cancel a different token's session.
Detector unavailability may prevent remote release; local stale/identity gates
remain closed, and the next selection replaces the remote acquisition.
Use one docking task owner per robot; do not run the experimental controller
concurrently. This is not a fleet reservation or access-control service.

Plugin parameters (under its configured type ID, e.g. lidar_dock):

- selection_service: dock_detector/select_dock (relative to robot namespace)
- detection_topic: dock_detector/detection
- selection_timeout: 3.0 seconds
- external_detection_timeout: 0.5 seconds
- filter_coef: 0.1 (0 = use newest measurement)
- inherited staging/threshold/direction parameters

All external detection translations and rotations must be identity. This is
validated during configure; the parent plugin's camera-oriented defaults must be
overridden, as the generator does. rotate_to_dock must be false. This plugin
requires continuous detection rather than blind backward docking.

For multiple robots, run one detector/docking server per namespace. Catalogue IDs
may be shared; topics and TF must be isolated. The detector launch passes its
parameter dictionary independently of namespace. Frame strings are configured
explicitly; namespacing does not rename them. Shared bay allocation remains a
mission/fleet concern. Detections of geometrically identical nearby bays can still
be ambiguous within a prior; catalogue identity is association, not a physical ID.

## Approach and arrival tracing

Add `trace_enabled: true` under `docking_server.ros__parameters.lidar_dock`
in the selected Seggy docking settings overlay. Restart the docking server to
load the setting. The default is false.

The server logs `DOCK: approach` at most twice per second when it has a fresh,
refined pose. A successful arrival check logs `DOCK: arrival` immediately.
Each line includes the dock ID, frame, and `scan_age_ms`, the elapsed ROS time
from the LiDAR scan to the log sample. Robot and parking coordinates are in metres
with three decimal places. `dx_cm` and `dy_cm` are robot minus detected position
in the logged frame. `xy_cm` is their planar distance.
`yaw_deg` is robot heading minus the desired parking heading, with the
180-degree adjustment applied for backward docking.

With pose-based docking, arrival requires both the filtered planar distance
to be below `docking_threshold` (metres) and the absolute filtered yaw error
to be at or below `docking_threshold_yaw` (degrees, default 5.0).
The yaw tolerance also applies when stall detection supplies the contact check.
Compare `filtered_yaw_deg` with `threshold_yaw_deg` in the trace.
The trace is sampled before the stop command and any settling.

## Verification

Use isolated ROS domains for communication tests so they cannot reach Seggy:

```bash
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR=/tmp/dock-plugin-verification
colcon test --packages-select dock_detector_nav2 --event-handlers console_direct+
colcon test-result --verbose
python3 -m pytest -q src/dock_detector/test
ROS_DOMAIN_ID=88 python3 -m pytest -q src/dock_detector/test/integration_named_docks.py
ROS_DOMAIN_ID=89 python3 -m pytest -q src/dock_detector/test/integration_namespaces.py
ROS_DOMAIN_ID=90 python3 -m pytest -q src/dock_detector_nav2/test/integration_server.py
```

The C++ harness loads the exported library through pluginlib and uses a fake
selection service; the separate Python integration test uses the real detector
service and a stub geometry matcher. Geometry accuracy has separate tests.
The real-server smoke test also configures, activates, deactivates and cleans up
DockingServer with the generated configuration, without sending an action goal.
That server has its normal command publisher but receives no motion request.
The other test harnesses create no velocity publishers. Full DockRobot/UndockRobot,
BT execution, physical stopping and multi-robot commissioning remain unvalidated.

-----------------------

Back to [Main Project Home](https://github.com/slgrobotics/articubot_one/wiki)
