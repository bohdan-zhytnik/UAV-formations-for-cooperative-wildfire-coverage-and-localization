# Two drones, one survey rectangle, six fires

This demo divides one operator-selected rectangle into equal halves. Each drone
searches its half, automatically owns fires it discovers, and returns to its
own fires while continuing coverage. It uses the existing two x500 drones,
RGB/thermal cameras, flat terrain, shared fire map and MRS flight controllers.

## Start

Inside MRS Apptainer:

```bash
cd ~/user_ros_workspace/src/fire_observation_demo/tmux_fleet
./start.sh
```

Attach after detaching:

```bash
tmux -L mrs_fire_fleet attach -t fire_fleet
```

Both drones start landed, with missions idle. Take off each drone using the
**Manual** tab and wait for normal MpcTracker flight before starting a survey.
You can select/apply a region while they are still on the ground.

The world still requests **0.25 real-time speed** with a **0.005 s** physics
step. The extra three fires are static hot surfaces; they add no drones or
cameras. For less load, `FIRE_GZ_HEADLESS=true ./start.sh` keeps RViz and cameras
but omits the separate Gazebo GUI. The fleet now uses `GZ_IP=127.0.0.1` for
local Gazebo transport, so it does not depend on a Wi-Fi address staying fixed.

## Define a region in RViz

1. Open **Two-drone controls → Survey**. This tab applies to both drones;
   the selected-drone dropdown still chooses the cameras/manual controls.
2. Press **r**, or select **Survey Region** in the RViz toolbar. Drag from one
   ground corner to the opposite corner. Releasing the mouse fills the four
   coordinate fields. Keep RViz Fixed Frame **common_origin**.
3. Alternatively, type minimum x/y and maximum x/y directly.
4. Choose a split direction, survey altitude, coverage spacing and revisit
   target. Automatic splitting bisects the longer side; the two halves have
   equal area. Uav1 receives the lower x/y half, uav2 the upper half.
5. Pause both missions if necessary, then press **Apply region and split**.
   This replaces the shared plan and clears its coverage/ownership assignments.
   Existing detected fires inside the rectangle can be claimed again using
   their recorded first observer. Applying/drawing sends no flight command.
6. Once both drones are airborne and ready, press **Start both: cooperative
   survey**. They begin their independent search/inspection cycles.

An invalid region is rejected without changing the current plan. Pause both
missions and wait for pending references before replacing an active region.
Resizing does not move either drone automatically. Start explicitly afterwards.

## What the split and dots mean

The default rectangle is **x = 5..17 m, y = -5..9 m**. Its horizontal split is
at **y = 2 m**:

| Drone | Observed ground half | Flight-centre y range |
|---|---|---|
| uav1, blue | x 5..17, y -5..2 | -3.1..0.1 m |
| uav2, green | x 5..17, y 2..9 | 3.9..7.1 m |

Each half currently has **12 search points** at 5 m altitude, with spacing no
larger than 3 m. Search runs back and forth across rows and reverses at the
end of a sweep. A fire revisit does not lose the next search-point index.

Coloured rectangles outline the observed halves. Dots show the generated
flight points, which become dimmer after inspection. The active task line
connects the drone to its destination. **Coverage N/12** counts distinct search
points inspected since the region was applied; it is planned sampling coverage,
not a dense per-pixel ground-visibility or occupancy map.

Flight centres are inset **1.9 m** from the half boundaries. Thus the two flight
lanes remain at least 3.8 m apart, while the downward camera footprints overlap
across the shared line. The ground halves themselves have no gap. Altitude,
spacing and region size are validated against the demo's flat-ground,
heading-zero, 60-degree 4:3 camera model. A region too narrow to fit the inset,
or an altitude too low to cover the split with the configured separation, is
rejected. This is not a planner for arbitrary camera orientations or terrain.

The waypoint table in **Missions** still defines the separate **Waypoint route**
mode. Its two preset rows do not determine the shared survey grid.

## Automatic ownership and fire revisits

The shared mapper records the **first accepted complete thermal observation**
that creates a fire track. That drone becomes the owner when the fire lies
inside the survey rectangle and can be inspected from that drone's half.
Transit sightings or old observations from a different partition cannot claim
a fire that is out of reach from that half; it remains unclaimed until the
other drone obtains a complete observation. This also makes changing the split
direction usable with an existing shared map. Clipped regions do not create a complete fire
track. If detections arrive together, mapper processing order resolves discovery
order. Each global fire ID has one owner; another drone seeing it does not
create a second fire or steal ownership.

The Survey table and map labels show the owner, the age of **that owner's own
last complete camera observation**, and the time remaining to the revisit
target. Another drone's camera can still improve the shared map, but does not
reset the owner's revisit timer. Existing explicit manual assignments can
override ownership; automatic discovery does not overwrite an existing owner.

The initial target is **20 simulated seconds**, about 80 real seconds at a
factor of 0.25. The planner estimates travel/inspection time and starts returning
before a fire becomes due. It uses an assumed planning speed of 1 m/s and a
2-second return margin, rather than waiting until the full interval has passed.

When measured fire outlines fit into one camera view, nearby owned fires can
share one inspection viewpoint. Completion requires fresh complete observations
of **all** those IDs from the owner after arrival. This uses measured boundaries,
not scene truth. Otherwise fires receive separate visits. Targets are kept
inside the owner's flight half, including near the shared boundary.

Each stop requires settling within 0.4 m for 1 simulated second, then a fresh
camera frame and a 1-second survey dwell. Fire tasks additionally require the
named fires to be fully observed. Failure to observe the required fire within
the allowed inspection time pauses that drone and requests hover.

**Twenty seconds is a scheduling target, not a hard real-time guarantee.** A
large region, many separated fires, flight dynamics or a blocked leg can make
it impossible. Negative due times and red **OVERDUE** labels expose this.
Inspection bursts are bounded to one round of owned fires before another
coverage stop, so overdue work cannot freeze the area search indefinitely.
The planner also avoids repeatedly visiting a fire that was just observed.

Ownership persists when a fire leaves view. Resetting the shared map clears its
IDs and claims and pauses cooperative missions; new detections create new
claims. Recheck the map and explicitly restart/resume. Pausing one drone does
not automatically hand its fires to the other drone.

For a fresh discovery demonstration, while both drones are landed/paused:

```bash
ros2 service call /fire_fleet/fire_map/reset std_srvs/srv/Trigger '{}'
./missionctl.sh survey 5 -5 17 9 --spacing 3 --revisit 20
```

The first command clears measured fire tracks; the second also resets survey
coverage. Take off and explicitly start both missions afterwards.

## Manual control

Manual map goals, **Hover**, **Land**, and **Pause both and hover** cancel mission
ownership for the affected drone. The other drone continues its own tasks.
Use **Resume selected** to retry the interrupted task, or **Start both** to
restart both search indices. A recovered network/heartbeat never resumes a
mission automatically. Take off manually again after landing.

MRS collision avoidance remains enabled with your configured **2 m radius**.
Our coordinator retains your **3.5 m airborne route separation** and **2 m
stationary-obstacle clearance** for confirmed parked drones. Reservations can
delay conflicting legs; this demo does not compute arbitrary detours.

## Terminal control

From the fleet `commands` window:

```bash
./missionctl.sh survey 5 -5 17 9 --axis auto --altitude 5 --spacing 3 --revisit 20
./missionctl.sh start_all
./missionctl.sh status
./missionctl.sh pause uav1
./missionctl.sh resume uav1
./fleetctl.sh hover_all
```

Both missions must be paused for the first command, and both drones ready in
normal flight for `start_all`. Configuration and actions return correlated
acknowledgements. `area uav1 ...` explicitly returns to the older independent
per-drone-area mode and disables the shared-region planner.

Runtime GUI/CLI edits stay in memory. Save startup defaults in
**[../config/fleet/missions.yaml](../config/fleet/missions.yaml)** under `survey`.
The existing per-drone preset routes remain available under `uavs`.

## Six-fire scene and implementation

Scene truth is configured in **[../config/fleet/fires.yaml](../config/fleet/fires.yaml)**:

| Patch | Centre x/y, m | Size, m | Temperature |
|---|---|---|---|
| A | 8, -2.5 | 1 × 1 | 480 K |
| B | 8, 0 | 2 × 2 | 560 K |
| C | 12, 6 | 3 × 2 | 640 K |
| D | 14, -2.5 | 1.5 × 1.5 | 520 K |
| E | 7, 6 | 1.5 × 1 | 600 K |
| F | 15, 6 | 1 × 1.5 | 500 K |

These names are evaluation truth, not detected IDs. IDs depend on discovery
order. The older three-fire scene is saved as `config/fleet/fires_three.yaml`.
Scene edits need a Gazebo restart. Original installed MRS and Apptainer files
and the separate one-drone scenario are unchanged.

The detector/mapper/planner never read fire truth. The shared survey uses
operator bounds, TF, and measured shared-map centres/outlines/observer times.
`scripts/survey_logic.py` supplies the geometry, ownership and task selection;
`mission_coordinator.py` integrates it with reservations and status. The mapper
adds `first_observer` and `first_observed_sim_s` to each track.

Important interfaces:

| Topic | Content |
|---|---|
| `/fire_fleet/survey/selection` | RViz drag result; fills fields only |
| `/fire_fleet/mission/config` | `set_survey` JSON with bounds/settings; applies a plan |
| `/fire_fleet/mission/status` | Shared plan, halves, coverage, owners and owner-specific deadlines |
| `/fire_fleet/mission/markers` | Areas, sampling dots, active tasks and ownership labels |
| `/fire_fleet/fire_map/status` | Image-derived tracks and first-observer provenance |

Validation uses `scripts/test_survey_logic.py`, `scripts/test_multi_fire.py`,
`scripts/validate_shared_survey.py --exercise-flight`, and the actual RViz tool
interaction executable `survey_tool_check`. See [VALIDATION.md](VALIDATION.md)
for measured results and limits.
