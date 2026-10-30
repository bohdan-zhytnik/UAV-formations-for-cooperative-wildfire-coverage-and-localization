# Fire observation prototype

**GitHub/source-backup guide: [what to commit and how to rebuild](docs/GITHUB.md).**

**Current demo: [one shared survey region, two drones and six fires](docs/SHARED_SURVEY.md).**
Use RViz's **Survey** tab and **Survey Region (r)** tool to split a rectangle,
search both halves, automatically claim discovered fires and schedule owned
fire revisits with a 20-simulation-second target.

**Milestone 4: [cooperative observation and looping waypoint routes](docs/COOPERATIVE_OBSERVATION.md).**
Use RViz's **Missions** tab or `tmux_fleet/missionctl.sh` to assign routes,
search areas and detected fires. Missions start explicitly after manual takeoff;
manual goals, hover and landing pause automation until explicit resume.

**Milestone 2: [two drones, three fires and one shared fire map](docs/MULTI_FIRE_DEMO.md).**
The fleet scene is configured in `config/fleet/fires.yaml`. Each camera detects
all hot regions; one shared node maintains stable IDs and bounded estimates.
RViz shows the shared map and optional individual observations.

A separate copy of the MRS one-drone scenario, extended with an x500 carrying downward RGB and thermal cameras. Original installed MRS models, worlds, launch files, and the Apptainer wrapper are not edited.

**RViz fire mapping and click-to-fly instructions: [docs/RVIZ_DEMO.md](docs/RVIZ_DEMO.md).**

**Two-drone manual flight and shared RViz: [docs/FLEET_DEMO.md](docs/FLEET_DEMO.md).**
Start that separate scenario from `tmux_fleet/`. It adds a drone selector,
independent takeoff/hover/landing and map goals, selected camera views, and a
shared `common_origin` frame. Both drones start on the ground.

The fleet world currently requests **quarter real-time speed (`0.25`)**. See
[tmux_fleet/README.md](tmux_fleet/README.md) for the exact configuration file,
restart instructions, and temporary speed commands.

This is a capability demonstration: MRS supplies localization, flight controllers/trackers, diagnostics, and command services. Our package supplies the thermal fire detector, flat-ground projection, rolling centre estimate, visualization markers, and RViz-to-MRS command adapter. No SLAM or Kalman filter is added.

## Scene and sensors

- Drone spawn: Gazebo `(0, 0, 0.15)`, heading zero; it settles onto its landing gear.
- Fire surrogate: orange/red **2 m × 2 m** ground patch at `(8, 0)`, **600 K**.
- Background ground: **293.15 K**. Blue square marks the drone's starting point.
- Both cameras: **320 × 240**, **10 Hz**, horizontal field of view **60°**.
- RGB and thermal optical centres are 8 cm apart; their images are not pixel-aligned.
- Camera assembly: **0.20 kg**, modelled as a fixed payload with box inertia, 7 cm below the original base link. Total rendered model mass: **2.208308 kg**. MRS nominal mass: **2.21 kg**.
- Thermal raw image: `mono16`, **0.02 K per count**. Convert raw pixel values with `kelvin = raw * 0.02`; Celsius is `kelvin - 273.15`.
- Thermal colour preview: fixed 290–650 K colour scale, for display only. Preserve raw images for estimation.

This is a stationary coloured hot target, not a combustion, smoke, heat-transfer, or fire-growth simulation. Camera and mount dimensions/mass are prototype assumptions.

## Launch

The package has already been built and tested in this workspace. For a fresh container session, start the usual `./wrapper.sh` from Ubuntu. Then, **inside Apptainer**:

```bash
cd ~/user_ros_workspace/src/fire_observation_demo/tmux
./start.sh
```

Stop any other Gazebo simulation first. This scenario uses the normal ROS domain/router port and its own Gazebo Transport partition `mrs_fire_demo`, tmux socket `mrs_fire`, and session `fire_demo`.

Unlike the original automatic example, this prototype starts **on the ground**. In its `commands` window:

```bash
./takeoff.sh
```

Wait for takeoff to finish and for `MpcTracker` / `Se3Controller` in the status/core output. Then:

```bash
./observe_fire.sh
```

This sets a position reference at **(8, 0, 5 m)** in `uav1/world_origin`, over the patch. The cameras should show the target after the drone arrives and stops tilting. It holds the accepted reference until another command or a safety action changes it.

Land with:

```bash
./land.sh
```

The takeoff helper re-enables output, requests offboard mode, arms, confirms hardware state, and calls the MRS takeoff service. It stops on a rejected request. No arming checks are bypassed. `waitForCore` alone does not perform these actions.

## Tmux and RViz

Use Ctrl+A then `w` to choose a window. Select `commands` to issue mission commands; leave the process windows running. Ctrl+A then `d` detaches without stopping the simulation.

RViz opens with navigation displays, RGB, and an annotated thermal view. The **Fire demo controls** panel selects `uav1`, the next goal's altitude (1–9 m), and whether map commands are enabled. After taking off, enable commands, choose **2D Goal Pose**, then click and drag on the ground to select position and heading. A new `goal_bridge` node transforms that goal into `uav1/world_origin` and calls the stamped MRS reference service. The selected altitude replaces the ground click's z coordinate; changing the altitude field alone does not move the drone.

The blue drone arrow, label, flight trail, RGB footprint (cyan), thermal footprint (orange), detected fire boundary (red), fire centre/ID, yellow illustrative uncertainty ellipse, and green accepted destination are shown in the main view. The fire appears only after the thermal camera sees it. The map stays fixed while the drone moves. The **Hover** button calls normal MRS hover; **Clear** resets the fire estimate and trail, which rebuild from new measurements.

The `fire_rviz` executable starts standard RViz, restores the image docks, and adds our control panel. The `fire_mapping` tmux window contains the detector and goal adapter logs. No flight starts automatically.

Reattach from an existing container terminal:

```bash
tmux -L mrs_fire attach -t fire_demo
```

You can also attach from your Ubuntu terminal, without opening another writable-overlay container:

```bash
tmux -S /tmp/apptainer/tmp/tmux-1000/mrs_fire attach -t fire_demo
```

Stop this scenario from a free terminal **inside the existing container**:

```bash
cd ~/user_ros_workspace/src/fire_observation_demo/tmux
./kill.sh
```

The separate `work` session on the same socket is a test/maintenance terminal; it can remain available while the demo is restarted. Avoid a second wrapper launch against an already-mounted writable overlay; use terminals in the running environment.

## Camera interfaces

| ROS topic | Message | Purpose |
|---|---|---|
| `/uav1/rgb_down/image_raw` | `sensor_msgs/msg/Image` (`rgb8`) | RGB measurements |
| `/uav1/rgb_down/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB calibration |
| `/uav1/thermal_down/image_raw` | `sensor_msgs/msg/Image` (`mono16`) | Quantitative thermal measurements |
| `/uav1/thermal_down/camera_info` | `sensor_msgs/msg/CameraInfo` | Thermal calibration |
| `/uav1/thermal_down/image_color` | `sensor_msgs/msg/Image` (`bgr8`) | Human-readable thermal preview |

Optical frames: `uav1/rgb_down/optical` and `uav1/thermal_down/optical`. Static transforms link them to `uav1/fcu`. Optical conventions are +Z forward, +X right, +Y down. At a level attitude, both look toward the ground.

The RGB stream uses the MRS spawner's existing image bridge. The extra camera launch bridges the thermal image and calibration, publishes both optical transforms, and makes the preview. Both sensor poses and their TFs are explicit: update both together if moving a camera.

To examine metadata without printing the pixel arrays:

```bash
ros2 topic echo /uav1/thermal_down/image_raw --once --no-arr --qos-reliability best_effort
ros2 topic echo /uav1/rgb_down/camera_info --once --qos-reliability best_effort
```

The existing state interfaces remain available, including `/uav1/estimation_manager/odom_main`, `/uav1/control_manager/reference`, `/tf`, and `/clock`. Sensor consumers must use compatible QoS, image timestamps, camera calibration, TF, and simulation time. Ground truth is for evaluation, not an input to a realistic fire detector.

## Project files

- `tmux/session.yml`: copied scenario structure; added cameras; omitted i3-only layout and automatic takeoff.
- `tmux/config/`: local copies of MRS scenario configuration plus nominal platform mass.
- `models/x500_fire.sdf.jinja`: copy of the installed x500, extended with the camera payload.
- `ROMFS/`: copy of the installed firmware resource scripts; adds `4001_x500_fire` using x500 settings.
- `worlds/fire_patch.sdf`: the custom ground and hot patch.
- `config/gazebo_server.config`: copied server plugins with **Ogre2** sensor rendering.
- `launch/cameras.launch.py`: thermal bridges, preview, and optical TFs.
- `scripts/thermal_preview.py`: display-only thermal colour mapping.
- `scripts/fire_mapper.py`: image-time TF, threshold detection, flat-ground projection, rolling mean, footprints, fire markers, and drone trail.
- `scripts/fire_geometry.py`: projection and bounded rolling-estimate geometry, independent of ROS.
- `scripts/goal_bridge.py`: RViz goals/settings to stamped MRS references, with state/limit checks and response reporting.
- `config/demo.yaml`, `launch/demo.launch.py`: demo node parameters and launch.
- `src/fire_rviz.cpp`: standard RViz startup, image activation, and demo controls dock.
- `scripts/rviz_layout.py`: regenerates the saved dock geometry using Qt offscreen.
- `scripts/prepare.py`: resolves package paths inside the container and writes ignored runtime configuration.
- `scripts/check_model.py`: renders the actual Jinja model and checks mass and sensor structure.
- `scripts/inspect_cameras.py`: subscribes to real images/calibration, saves snapshots, and checks the target temperature.
- `scripts/test_fire_geometry.py`: downward/tilted-camera, invalid-ray, and rolling-estimate regressions.
- `scripts/validate_rviz_demo.py`: inspect mapping outputs; optional explicitly requested flight exercise.
- `artifacts/`: snapshots and validation evidence, excluded from Git.

If you change the model, world, or Gazebo configuration, restart the scenario. The package is installed with symlinks, so those resource changes do not need a rebuild in this workspace. Changes to installed program lists or package/build metadata do require one.

Build only this package from the workspace root:

```bash
cd ~/user_ros_workspace
colcon build --packages-select fire_observation_demo --symlink-install
source install/setup.bash
```

Validate measurements while hovering above the patch:

```bash
python3 ~/user_ros_workspace/src/fire_observation_demo/scripts/inspect_cameras.py --require-fire
```

The inspection writes RGB, raw thermal, colour thermal, and a side-by-side image into `artifacts/`, plus a JSON report. The raw PNG retains integer counts, not Kelvin units.

## Limits

This prototype has ideal camera geometry and no added image noise. Thermal temperatures are a simplified Gazebo rendering model. The ground is assumed to be `z=0` in `uav1/world_origin`. The target's thin raised surface can introduce a small projection bias. Existing MRS/PX4 state estimation can still produce position variation.

The centre is a rolling mean of up to 30 complete observations, not a Kalman filter. The yellow ellipse shows an illustrative 2-sigma repeatability measure with a 3 cm floor; it is not a calibrated confidence bound or a claim about real-world accuracy. Patch size is represented separately by its red boundary. Edge-clipped detections do not update the centre. The last estimate remains when the fire is no longer visible, with a last-seen label. Only the largest hot connected region is tracked, always as `Fire 1`.

The fixed terrain layer is known flat geometry, and drone pose comes from MRS; the package does not implement terrain-based visual localization or SLAM. There is no combustion, fire spread, multi-fire association, multi-drone fusion, or formation controller yet.

Your original environment showed timing warnings and a previous MRS core crash. The new scenario does not repair those upstream issues. If diagnostics stop, inspect `core`; do not repeatedly send takeoff or movement requests. Multiple rendered cameras can lower the real-time factor on this notebook.

## Sources

The x500, firmware scripts, and original scenario are copied from the locally installed CTU MRS packages; retain their upstream comments and attribution. The intended project license is BSD-3-Clause.

- https://github.com/ctu-mrs/mrs_uav_gazebo_simulator/tree/ros2
- https://gazebosim.org/api/sensors/8/thermalcameraigngazebo.html
- https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/thermal_camera.sdf
