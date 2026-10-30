# Milestone 4: cooperative observation and waypoint supervision

The current fleet adds [one shared survey rectangle and six fires](SHARED_SURVEY.md),
including automatic discovery ownership. This page describes the earlier
waypoint and independently configured-area workflow, which remains available.

This extends the existing **fire_fleet** scenario: two drones, three hot patches,
one shared fire map, and explicit manual control. There are two mission modes:

| Mode | Behavior |
|---|---|
| Waypoint route | Fly through your ordered points, stop at each point for a camera inspection, then continue. Repeat the route or hold at the final point. |
| Cooperative | Each drone searches its assigned rectangle and prioritizes stale fires in that rectangle or explicitly assigned to it. Fire tasks use the detected shared map, with one drone assigned per fire. |

No mission starts on launch, arms a drone, or takes off automatically. Select,
take off, and wait for normal MpcTracker flight before pressing **Start**.
Manual flight remains available independently for each drone.

## Start and attach

Inside your usual MRS Apptainer:

```bash
cd ~/user_ros_workspace/src/fire_observation_demo/tmux_fleet
./start.sh
```

After detaching:

```bash
tmux -L mrs_fire_fleet attach -t fire_fleet
```

If the previous fleet is still running after updating the code, stop that fleet
and start it again so its launch includes the new mission coordinator and RViz
controls. Your current world requests `real_time_factor: 0.25` (quarter speed);
this milestone preserves the value in `worlds/fire_fleet.sdf`.
For less CPU load, launch with `FIRE_GZ_HEADLESS=true ./start.sh`; RViz and the
simulated cameras still work.

## Make a route in RViz

1. Select **uav1** or **uav2** at the top of **Two-drone controls**.
2. Open the **Missions** tab. Pause an existing mission before editing its route.
3. Use **Clear** to remove the preset, or **Preset** to restore its initial points.
   **Preset** also selects **Waypoint route** as the start mode.
4. Set **Next goal altitude** above the tabs. Check **Add route points with
   2D Fleet Goal (g)**. Press `g`, click and drag on the map to add each point
   and heading. Repeat `g` for each point. This checkbox makes clicks edit the
   route; **they do not command flight**. The map-goal enable checkbox is not
   needed for route editing.
5. The table and coloured map labels show the order, with a line connecting
   the points. **Undo** removes the last point. Points use `common_origin`;
   keep RViz's Fixed Frame set to that frame.
6. Choose **Repeat route** and the inspection dwell, then **Apply repeat / dwell
   settings**. These changes require a paused mission. An individual point can
   have a different dwell when configured using YAML or the terminal.
7. Take off using the **Manual** tab, wait for the climb to finish, select
   **Waypoint route**, then press **Start selected**. Starting turns off route
   editing. The drone flies to point 1, inspects, visits point 2, and so on.

Changing the selected drone or altitude alone sends no movement. Each drone has
its own route and execution state. Switching selection also turns off route
editing, so check it again deliberately when editing another drone's route.

The stock routes keep the drones apart: uav1 observes the southern pair of
patches around `(8, -0.8)` and `(8, -2.5)`, while uav2 observes the northern patch
around `(12, 6)` and `(12, 7)`, at 5 m. They are operator-authored example routes,
not positions secretly supplied to the detector.

## Inspect a fire rather than just a point

A plain waypoint is complete when the drone settles within **0.4 m** for
**1 simulated second**, then spends its dwell there and receives a fresh camera
frame. It records the IDs actually observed during that stop. A plain point can
be inspected even when no fire is visible: that is useful for searching.

For a point that **must observe a particular mapped fire**, set its `fire_ids`.
The same drone must produce a new, complete observation of every required fire
after arriving. A stale map entry or the other drone's measurement is
insufficient. If a fire is clipped or not visible from that point, adjust the
point/altitude. After the dwell plus 12 simulated seconds without the required
observation, the mission pauses and requests hover.

Example, from the `commands` tmux window after detecting the fire and checking
its current ID:

```bash
./missionctl.sh pause uav1
./missionctl.sh clear uav1
./missionctl.sh append uav1 8 -0.8 5 --dwell 5 --fires 1 2
./missionctl.sh append uav1 8 -2.5 5 --dwell 5 --fires 1
./missionctl.sh options uav1 --loop
./missionctl.sh start uav1
```

IDs depend on discovery order. Use the IDs in the shared map, not the scene's
patch names. To supervise several fires, add appropriate inspection points and
loop the route. To fly once and hold at the last point, use `--once`.

## Cooperative search and stale observations

Each drone has a rectangular search area. The coordinator generates a simple
back-and-forth grid through it. At every task boundary it checks the shared map:

* Prefer eligible fires whose latest complete observation is older than the
  revisit threshold, initially **20 simulated seconds**. Older observations
  have priority; distance breaks ties.
* An explicit fire assignment overrides rectangular-area eligibility. It does
  not stop that drone searching its rectangle between inspections.
* A fire already claimed by another drone is excluded. A fire task flies over
  the estimated centre at the area's altitude and requires that drone's fresh
  complete observation before completion.
* If there is no eligible stale fire, inspect the next grid point and repeat
  coverage indefinitely. A fire continuously visible to either drone stays
  fresh and needs no extra revisit.

Configure areas and start from tmux, with both drones already airborne:

```bash
./missionctl.sh area uav1 7 -3 9 1 --spacing 1 --altitude 5
./missionctl.sh area uav2 11 5 13 7 --spacing 1 --altitude 5
./missionctl.sh revisit uav1 20
./missionctl.sh revisit uav2 20
./missionctl.sh start_all
```

Or choose **Cooperative search / revisit** and **Start selected** in RViz;
**Start both: cooperative** requests it independently for both drones. Each
drone must already be ready; if one rejects the start, the other can still start.

Optional explicit assignments, after the map contains those IDs:

```bash
./missionctl.sh assign uav1 1 2
./missionctl.sh assign uav2 3
./missionctl.sh assign uav1          # clear uav1's explicit assignments
```

The RViz fire-ID field accepts comma-separated IDs and an empty field clears
that drone's assignments. Area, assignments, and revisit changes affect the
**next task**. They do not redirect a reference already in progress. An assigned
fire cannot be taken from another drone while it is actively inspecting it.

The map's IDs are estimates, not persistent identities across a map reset.
Resetting the shared map clears assignments and pauses cooperative missions;
recheck the new IDs and explicitly start/resume. Waypoint missions with explicit
fire IDs also pause. Plain waypoint routes do not depend on map IDs.

## Manual control and pausing

**Hover**, **Land**, a valid enabled manual fleet goal, or **Pause selected and
hover** cancels that drone's mission ownership. The other drone continues its
own mission. Press **Resume selected** or run:

```bash
./missionctl.sh pause uav1
./missionctl.sh resume uav1
./fleetctl.sh hover_all
```

Resume retries the interrupted point/task; Start begins the route/search grid
again. Route completion counters and recent inspection history remain available.
Resume requires a previously started mission. Take off again manually after
landing before starting/resuming.

When using manual map goals, leave route editing unchecked and enable map goals
for the selected drone. A manual destination replaces its autonomous target.
Missions never resume simply because a manual action has finished.

The vehicle adapter serializes its MRS reference requests and drains any already
sent reference before sending a queued hover/land. A late service response or
old coordinator task cannot regain mission ownership. A lost response can delay
the queued action: the status explains the pending response rather than claiming
hover has already happened. Direct calls to MRS services bypass this demo's
ownership arbitration; use the fleet controls for manual intervention while a
demo mission is active.

If the coordinator disappears for 5 real seconds, restarts, simulation time
rewinds, or normal flight/position becomes unavailable, an active mission pauses
and requests hover. Recovery requires an explicit Start/Resume. Mission starts
also require fresh hardware, TF, and normal MpcTracker flight.

## Separation and limits

MRS collision avoidance stays enabled. In addition, before granting a mission
leg the coordinator checks its horizontal line segment against the other
drone's current position, accepted/pending destination and reserved leg. It
requires **6.5 m horizontal separation** from airborne or unconfirmed vehicles,
even if their altitudes differ. A vehicle confirmed by fresh hardware status
as disarmed and within 0.5 m of this flat ground is treated as a stationary
obstacle instead: the planned 3D segment must stay **2 m** away from it. A
parked drone therefore does not reserve a full airborne traffic corridor.
Unknown/stale status, an outstanding reference or an active reservation retains
the conservative traffic check. Both distances are in `missions.yaml`.

The status may say **Waiting for separation from uav2**; this reason also appears
in the command status and tmux log. Pressing Start again reports the current
blocking state instead of starting another mission. Choose separated areas,
adjust the routes, or manually move the blocking drone away. This conservative
reservation scheme does not find detours, resolve every deadlock, optimize a
fleet tour, or guarantee separation when an operator subsequently flies across
an autonomous route. MRS remains the flight-control collision-avoidance layer.
The demonstration intentionally uses simple spatial division rather than close
formation flight.

Waypoints and search bounds are limited to x/y ±20 m and altitude 1–6 m. Routes
have at most 100 points, grids at most 200. Dwell is 1–60 simulated seconds;
the revisit threshold is 3–600 simulated seconds. Travel timeout is initially
90 simulated seconds. Dwell, arrival, observation age and travel time all use
**simulation time**, including during slowed playback; coordinator/hardware liveness
uses real time.

## Configuration, files and topics

Saved startup defaults: **[../config/fleet/missions.yaml](../config/fleet/missions.yaml)**.
Edit this file to preserve your routes/areas across restarts. Runtime GUI/CLI
edits are in memory. You can also load a route from your own YAML:

```yaml
waypoints:
  - position: [8.0, -0.8, 5.0]
    heading: 0.0
    dwell_sim_s: 5.0
    fire_ids: [1, 2]
  - position: [8.0, -2.5, 5.0]
    dwell_sim_s: 3.0
```

```bash
./missionctl.sh route uav1 /path/inside/container/my_route.yaml
./missionctl.sh status
./missionctl.sh --help
```

| File | Role |
|---|---|
| `scripts/mission_logic.py` | ROS-independent route state machine, grid generation, stale-fire choice and corridor checks |
| `scripts/mission_coordinator.py` | Shared assignments, reservations, heartbeat, mission status and markers |
| `scripts/fleet_vehicle.py` | Per-drone execution, readiness, inspection completion and manual takeover |
| `src/fleet_rviz.cpp`, `src/fleet_goal_tool.cpp` | Mission panel and route editing using the existing goal tool |
| `tmux_fleet/missionctl.sh`, `scripts/mission_cli.py` | Explicit tmux commands with request-ID acknowledgements |
| `launch/fleet.launch.py` | Launches the coordinator and passes mission defaults to each vehicle |
| `tmux_fleet/rviz.rviz` | Adds the coloured mission marker layer |

| Topic | Meaning |
|---|---|
| `/fire_fleet/mission/status` | Combined JSON: active mode/phase, route, next target, recent inspections, areas, fire assignments and reservations |
| `/fire_fleet/mission/markers` | Route lines, numbered points, area rectangles, and active task lines in `common_origin` |
| `/fire_fleet/mission/config`, `/config_ack` | Runtime configuration and correlated acknowledgements; the latter suffix is relative to `/fire_fleet/mission` |
| `/fire_fleet/uavN/mission_status` | One drone's execution state and observed fire IDs per completed visit |
| `/fire_fleet/action`, `/fire_fleet/uavN/action_ack` | Existing addressed command protocol, extended with mission start/pause/resume |

These JSON interfaces are prototype application messages. The coordinator sends
task proposals, while each vehicle adapter alone calls its existing MRS
`control_manager/reference` service. Detection and mapping still use only camera
measurements, calibration and TF. The coordinator consumes that shared map;
it never reads the Gazebo scene's `fires.yaml`. MRS supplies state estimation,
tracking/control, diagnostics and collision avoidance; the mission system is
our application layer on top of MRS.

Tests: `scripts/test_mission_logic.py` exercises mission state and geometry;
`scripts/validate_missions.py` performs real Gazebo flights through the public
fleet command interfaces. See [VALIDATION.md](VALIDATION.md) for actual results.
