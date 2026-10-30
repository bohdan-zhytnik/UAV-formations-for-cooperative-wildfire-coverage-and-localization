# One-drone RViz capability demo

This demo demonstrates ROS visualization, MRS position commands and localization, and a small custom thermal mapping pipeline. It uses one x500, one flat world, and one stationary 600 K patch. No extra artificial noise or Kalman filter is added.

## Start and try it

From Ubuntu, enter the usual MRS Apptainer with `~/mrs_apptainer/wrapper.sh`. Inside it:

```bash
cd ~/user_ros_workspace/src/fire_observation_demo/tmux
./start.sh
```

If the demo is already running, attach instead:

```bash
# Inside the existing container
tmux -L mrs_fire attach -t fire_demo

# Or from Ubuntu
tmux -S /tmp/apptainer/tmp/tmux-1000/mrs_fire attach -t fire_demo
```

1. Choose the tmux **commands** window with Ctrl+A then `w`. Run `./takeoff.sh` and let the drone finish its climb.
2. In RViz, find **Fire demo controls** on the left. The selected drone is **uav1 (blue)**. Set **Flight altitude** to **5 m** and check **Enable map commands**.
3. Select **2D Goal Pose** on the toolbar (the standard `g` shortcut also selects it). Click near **x=8, y=0** on the ground and drag to choose heading. Release to send the goal. The bottom status bar gives the cursor position; the grid's small cells are one metre.
4. Check the panel's command message. A green marker appears only after MRS accepts the reference. The drone flies there and holds the accepted reference. You can alternatively run `./observe_fire.sh` in the commands window to reach the patch's vicinity precisely.
5. Once the complete hot patch fits inside the thermal view, a red boundary, **Fire 1** label, red centre, and small yellow ellipse appear. The thermal image gains a green outline and white detected-centre dot.
6. Send another nearby goal, for example **(8.5, 0.4)** at 5 m. Watch the footprint, flight trail, and fire estimate change. Setting altitude to **6 m** and sending another goal demonstrates the footprint growing with height.
7. **Hover / cancel current target** requests normal MRS hover. Changing altitude alone sends no flight command. Unchecking **Enable map commands** ignores future map clicks; it does not stop an already accepted flight.
8. Land from the commands window with `./land.sh`. Use `./takeoff.sh` to fly again. **Clear fire estimate and flight trail** clears those displays; observations rebuild them automatically.

Ground clicks select horizontal position. **Flight altitude means z in `uav1/world_origin`**, approximately height above the flat ground in this scene. It is not a general terrain-following command. The demo accepts goals only within x/y ±20 m and altitude 1–9 m; MRS applies its own limits too.

The demo starts on the ground and map commands start disabled. The adapter never arms, changes flight mode, or takes off automatically. It requires fresh connected, armed, offboard status before forwarding a goal.

## Read the view

| Appearance | Meaning |
|---|---|
| Green flat surface and blue start square | Known stationary terrain reference layer |
| Blue arrow, `uav1` label, blue trail | Drone attitude/name and recent estimated flight path |
| Cyan outline on the ground | RGB camera footprint |
| Orange outline on the ground | Thermal camera footprint |
| Red outline | Latest complete observed fire boundary |
| Red sphere and `Fire 1` label | Rolling estimate of the fire centre and source drone |
| Yellow ellipse | Illustrative 2-sigma repeatability around the estimated centre |
| Green sphere, vertical line and label | Latest RViz map goal accepted by MRS |
| Green outline in thermal image | Complete detected hot region used for an update |
| Orange outline in thermal image | Edge-clipped region; centre update skipped |
| Muted fire markers and last-seen age | Stored estimate, currently outside view or not fully visible |

The two optical centres are 8 cm apart, so footprints and images differ slightly. The ellipse can be small in this ideal scene: it is separate from the physical fire boundary. Expand **Fire observation demo** in Displays to toggle its layers. The original MRS diagnostic displays remain available in their groups. Use the Views panel to switch between Orbit and Top-down Orthographic if desired.

Zoom in near the fire to inspect the yellow ellipse: its radius can be only a few centimetres in this ideal scene.

The main map is fixed to `uav1/world_origin`. The view target is also stationary, rather than following the drone. Gazebo is the simulated physical world; RViz shows ROS data. Fire markers are generated from sensor measurements and do not create objects in Gazebo.

## What computes the fire position?

1. Read `mono16` raw thermal pixels and multiply counts by **0.02 K**.
2. Select pixels above **450 K**. Keep the largest connected region with at least 50 pixels.
3. If the region touches the image edge, retain the previous estimate. A partial patch would bias its centre.
4. Use `CameraInfo` and the camera-to-world TF **at that image's timestamp** to construct viewing rays. Intersect them with the known plane `z=0`. Missing historical TF causes the observation to wait or be skipped, rather than substituting the drone's current pose.
5. Average the projected hot pixels to obtain one centre measurement. Project the contour to obtain the boundary.
6. Keep a rolling mean of the latest **30** complete measurements, updating at up to **2.5 Hz** of simulation time. Old measurements eventually leave the window, so a moved target would eventually move the estimate.

The detector receives no configured fire position or Gazebo fire ground truth. The validation script separately knows the patch's expected position to check the result.

The yellow ellipse combines observed centre scatter and an approximate pixel-resolution term, limits the effective sample count to five, and retains a **3 cm per-axis floor**. It is an illustrative repeatability measure, **not calibrated uncertainty**. It does not model GPS bias, camera calibration error, or all correlations between frames. No 95% accuracy claim is implied by its 2-sigma label. No noise is injected to make the ellipse look larger.

## What belongs to MRS, and what belongs to this package?

MRS/PX4/Gazebo provide the simulated vehicle, dynamics, sensor streams/bridges, vehicle-state estimation, transforms, flight controllers and trackers, safety checks, and services for takeoff/reference/hover/landing.

Our package adds the thermal detector, geometric projection, rolling fire estimate, fire/footprint/trail markers, and goal adapter/control dock. These demonstrate how a thesis project can use the existing MRS interfaces. The fire estimator is custom demonstration code, not an upstream MRS fire-detection feature.

The map separates the stationary terrain reference from the fire observation layer. Drone localization currently comes from the configured MRS GPS/rangefinder estimator. We have **not added visual SLAM using terrain features**. Later, a terrain-localization system could provide drone poses while fire mapping stays a separate, changing layer. Several drones would additionally need shared-frame alignment, measurement association/fusion, and coordinated planning.

## ROS interfaces worth exploring

| Topic/service | Type | Purpose |
|---|---|---|
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz goal input; its frame is transformed, z is replaced by panel altitude |
| `/fire_demo/goal_settings` | `std_msgs/msg/String` JSON | Selected UAV, next-goal altitude, enable flag |
| `/fire_demo/command_status` | `std_msgs/msg/String` | Accepted/rejected commands and explanations |
| `/fire_demo/accepted_goal` | `geometry_msgs/msg/PoseStamped` | Latest MRS-accepted destination in world coordinates |
| `/fire_demo/fire_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Centre with illustrative x/y covariance |
| `/fire_demo/fire_boundary` | `geometry_msgs/msg/PolygonStamped` | Latest complete projected hot-region boundary |
| `/fire_demo/fire_status` | `std_msgs/msg/String` JSON | Validity, visibility, source, observation count, centre, covariance, last-seen age |
| `/fire_demo/markers` | `visualization_msgs/msg/MarkerArray` | Terrain, drone/name, footprints, fire centre/boundary/ellipse |
| `/fire_demo/goal_markers` | `visualization_msgs/msg/MarkerArray` | Latest accepted destination |
| `/fire_demo/path/uav1` | `nav_msgs/msg/Path` | Up to 1,500 trail poses, sampled after at least 10 cm movement |
| `/fire_demo/detection_image` | `sensor_msgs/msg/Image` | Thermal colour image with detection annotations |
| `/fire_demo/reset_map` | `std_srvs/srv/Trigger` | Clear rolling fire estimate and flight trail |
| `/fire_demo/action` | `std_msgs/msg/String` | Panel actions: `hover` or `reset_map` |
| `/uav1/control_manager/reference` | `mrs_msgs/srv/ReferenceStampedSrv` | Existing MRS stamped position/heading command |
| `/uav1/control_manager/hover` | `std_srvs/srv/Trigger` | Existing MRS hover command |

Map/estimate publishers use transient-local durability so a new viewer receives their last sample. Images use sensor-data QoS. After a clear or when the fire leaves view, a previously published pose can remain cached: consumers must check `fire_status`, the pose timestamp, and the last-seen age. An unavailable estimate is reported as `centre_m: null`; current visibility is a separate flag.

Examples in a terminal **inside the existing container**:

```bash
cd ~/user_ros_workspace/src/fire_observation_demo/tmux
source ../scripts/env.sh
ros2 topic echo /fire_demo/fire_status
ros2 topic echo /fire_demo/fire_pose --once
ros2 service call /fire_demo/reset_map std_srvs/srv/Trigger '{}'
```

Node parameters are in `config/demo.yaml`; RViz displays are in `tmux/rviz.rviz`. Restart the `fire_mapping` launch after changing its parameters. Changes to the C++ controls require rebuilding the package and restarting RViz.

## Validation

```bash
# Inside the container; use the same middleware as the demo
cd ~/user_ros_workspace/src/fire_observation_demo/tmux
source ../scripts/env.sh
python3 ../scripts/test_fire_geometry.py

# Read mapping outputs while already hovering over the patch; no flight command
python3 ../scripts/validate_rviz_demo.py

# Explicit flight exercise: takes off if needed, tests goal handling, flies, lands
python3 ../scripts/validate_rviz_demo.py --exercise-flight
```

The flight exercise uses `/goal_pose`, including a goal in `uav1/fixed_origin` that the adapter must transform into the world frame. It checks disabled/grounded/invalid-frame/out-of-bounds goal handling, observes the fire from two viewpoints, verifies both footprints, boundary size, centre accuracy against simulation ground truth, and landing. Outputs are `artifacts/rviz_demo_check.json` and `artifacts/fire_detection.png`.

If a command fails, read the panel message and the **fire_mapping** tmux window. If flight diagnostics stop, inspect **core**. Existing upstream timing warnings and the earlier native core crash are not fixed by these additions.

During a long test session PX4 eventually reported **Preflight Fail: Battery unhealthy** and rejected arming. The takeoff helper correctly stopped. For this simulated demo, use `./kill.sh` and then `./start.sh` to start a fresh vehicle; do not disable the arming checks. The previous RViz flight trail and fire estimate rebuild after restarting.
