# Two-drone manual flight controls

For looping routes, assigned search areas and stale-fire revisits, see
[Milestone 4: cooperative observation](COOPERATIVE_OBSERVATION.md). Manual
goals, hover and landing pause an active mission on the addressed drone.

The fleet now includes milestone 2. See [MULTI_FIRE_DEMO.md](MULTI_FIRE_DEMO.md)
for the three-fire scene, shared mapping and observation interfaces. The manual
flight workflow below is unchanged.

This milestone adds **two independently controlled x500 drones to one Gazebo
world and one shared RViz map**. It keeps the existing one-drone scenario in
`tmux/` available. No original installed MRS files or Apptainer wrapper are
changed.

Both drones start on the ground. `uav1` is blue, at `(0, -6)` m; `uav2` is green,
at `(0, 6)` m. Three configurable fire surrogates are generated from
`config/fleet/fires.yaml`.
Each drone carries its own downward RGB and thermal cameras. The fleet model
uses **5 Hz** cameras, at the original 320 × 240 resolution, to reduce rendering
work on the notebook. Camera mount, mass, calibration and thermal scaling remain
the same as in the one-drone demo.

The fleet world currently requests a **real-time factor of 0.25** at startup. See
[the simulation-speed README](../tmux_fleet/README.md) for the configuration
location, how to apply changes, and how to inspect the achieved speed.

## Start and attach

Stop another simulation first. This scenario has its own Gazebo partition and
tmux socket, but uses the normal ROS domain and Zenoh router port. Run this
**inside the existing MRS Apptainer**:

```bash
cd ~/user_ros_workspace/src/fire_observation_demo/tmux_fleet
./start.sh
```

Detach with **Ctrl+A, then d**. Reattach from an existing container terminal:

```bash
tmux -L mrs_fire_fleet attach -t fire_fleet
```

Or reattach directly from an Ubuntu terminal:

```bash
tmux -S /tmp/apptainer/tmp/tmux-1000/mrs_fire_fleet attach -t fire_fleet
```

Use **Ctrl+A, then w** to choose a window. `commands` is a free terminal.
`px4_api`, `core`, and `status` each have a pane for each drone. Leave those
processes running. `fleet_control` contains the shared TF connector, command
router, and separate command and detection nodes for both drones.

## Control in RViz

1. In **Two-drone controls**, choose `uav1 (blue)` or `uav2 (green)`.
2. Click **Take off selected**. Wait for the climb to finish and for the status
   window to show `MpcTracker` / `Se3Controller`.
3. Choose **Next goal altitude** (1–6 m) and enable map goals for that drone.
4. Select **2D Fleet Goal**, or press **g** with the map focused. Click and drag
   on the map: the click gives horizontal position; the drag gives heading.
5. Choose the other drone and repeat. A drone holds its previous accepted goal
   while you control the other one.

Selecting a drone switches both camera views and the displayed command/fire
status. Each drone remembers its own altitude and map-enable checkbox. Selecting
a drone or changing altitude alone sends no movement command. The custom RViz
tool puts the selected drone, altitude, enable state and clicked pose in **one
message at mouse release**, so switching selection afterward cannot redirect an
already issued click.

RViz keeps four image subscriptions connected and shows only the selected
drone's RGB/thermal pair. Seeing RViz subscribed to both drones in the ROS graph
is expected. Keeping the subscriptions fixed avoids an installed RViz Image
display failure when changing an active display's topic. Our small `FleetImage`
display also clears disconnected image callbacks before their subscriber is
destroyed, covering repeated cleanup during shutdown. It inherits the stock
RViz Image viewer; the workaround lives in this package.

**Hover selected** and **Land selected** affect that drone. **Hover both drones**
requests hover for both; each MRS response is independent. Takeoff re-enables
output, requests offboard mode, arms, confirms hardware state, and requests MRS
takeoff. An acknowledgement means MRS accepted the action; it does not mean the
physical climb or landing has finished. Wait until landing has disarmed the
vehicle before taking off again.

The 6 m altitude ceiling leaves room below the rangefinder estimator's 10 m
limit for this example. Keep separated flight areas initially. MRS's existing
MpcTracker collision avoidance remains enabled; manual commands still need
sensible separation. The demo adds no coordinated route planner. To view the
same fire, send one drone over it while the other remains well away.

## Control in the commands terminal

These commands use the same per-drone services as the panel and require an
explicit vehicle name:

```bash
./fleetctl.sh status
./fleetctl.sh select uav2
./fleetctl.sh takeoff uav1
./fleetctl.sh hover uav1
./fleetctl.sh land uav1
./fleetctl.sh takeoff uav2
./fleetctl.sh hover_all
./fleetctl.sh land uav2
```

`select` also changes the RViz selector and camera views. Every action carries a
unique request ID. The terminal waits for that request's acknowledgement, so an
old successful takeoff message cannot be mistaken for a new result. One drone's
takeoff preparation or service response does not block the other's command node.

After landing both drones, stop the session from a free terminal inside the
container with:

```bash
cd ~/user_ros_workspace/src/fire_observation_demo/tmux_fleet
./kill.sh
```

## What the shared view means

The fixed RViz frame is **`common_origin`**. Both MRS estimators use the same
GNSS world origin, matching the Gazebo world's origin. The installed
`mrs_tf_connector` connects both MRS TF trees using their `world_origin` frames.
This retains the 12 m launch separation. It avoids incorrectly placing each
drone's local starting point at the centre of the map.

Paths, drone arrows and labels, start pads, and accepted targets use each
drone's blue/green colour. RGB footprints are cyan; thermal footprints are
orange. Each camera projects its own measurements using its own optical TF.
World-ground markers and the fire truth patch are for orientation/evaluation;
the detector still estimates position from thermal images, calibration and TF.

Fire estimates are now maintained by a shared-map node. Each camera supplies
independent raw observations; matching observations from another drone update
the same global fire entry. Optional observation displays remain available for
comparison. See [MULTI_FIRE_DEMO.md](MULTI_FIRE_DEMO.md) for the map's assumptions
and limits. Formation flight and autonomous missions remain later milestones.

## Useful interfaces

Replace `uav1` with `uav2` for the second drone.

| Interface | Purpose |
|---|---|
| `/uav1/rgb_down/image_raw`, `/camera_info` under the same camera prefix | RGB image and calibration |
| `/uav1/thermal_down/image_raw`, `/camera_info` under the same camera prefix | Raw `mono16` thermal data and calibration; kelvin = count × 0.02 |
| `/fire_fleet/uav1/detection_image` | Annotated thermal view |
| `/fire_fleet/uav1/fire_status`, `/fire_fleet/uav1/observations` | Current observer status and raw region batches |
| `/fire_fleet/fire_map/status`, `/fire_fleet/fire_map/markers` | Shared estimates, boundaries, IDs and RViz display |
| `/fire_fleet/uav1/markers`, `/path`, `/goal_markers` under the same prefix | RViz displays |
| `/fire_fleet/uav1/accepted_goal` | Reference accepted by MRS, in `uav1/world_origin` |
| `/uav1/hw_api/status` | Connection, armed and offboard state |
| `/uav1/control_manager/reference` | Stamped MRS reference service |
| `/uav1/control_manager/hover`, `/uav1/uav_manager/takeoff`, `/uav1/uav_manager/land` | Normal MRS flight services |
| `/fire_fleet/status` | JSON summary for both drones |
| `/fire_fleet/map_goal` | Addressed JSON from the custom RViz tool |
| `/fire_fleet/action` | Addressed JSON actions with request IDs |
| `/fire_fleet/uav1/action_ack` | Matching per-request action result |

The incoming map goal is expressed in `common_origin`; the per-drone adapter
transforms it to that drone's `world_origin` before calling MRS. Unknown drones,
disabled commands, unready hardware, invalid coordinates, missing transforms,
altitudes outside 1–6 m and goals outside the ±20 m demo square are rejected.

## Configuration and validation

`config/fleet/scenario.yaml` defines names, colours, spawn positions and command
altitude limit. `fires.yaml` defines scene truth; `fire_mapping.yaml` defines
perception settings. `network.yaml` tells MRS about both robots; `tf_connector.yaml`
connects their world frames. `tmux_fleet/session.yml` launches the scenario.
`fleet_cameras.launch.py` and `fleet.launch.py` instantiate per-drone nodes.
The fleet RViz executable and goal-tool plugin are separate from the original
one-drone RViz executable.

The SDF world and saved RViz groups also contain the pads/names/colours; changing
the number or names of drones requires updating those resources as well. This
milestone is deliberately configured for two drones.

From `tmux_fleet/`, inside Apptainer, inspect the running scenario with:

```bash
python3 ../scripts/validate_fleet.py
```

The following check **flies the simulated drones**. Start with both disarmed
on their launch pads and leave the controls alone during the check:

```bash
python3 ../scripts/validate_fleet.py --exercise-flight
```

It checks camera switching in the actual RViz ROS graph, shared TF alignment,
command rejection, addressed movement even after switching selection, separate
takeoff/hover/landing, and camera estimates of the same fire. It writes
`artifacts/milestone2_check.json`. Flight-check failure records the failure and stops
the sequence; inspect both vehicles and use the normal hover/land controls as
appropriate before retrying.
