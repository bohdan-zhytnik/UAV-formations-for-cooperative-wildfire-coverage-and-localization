# Milestone 2: two drones, three fires, one shared map

The current scene has [six fires and a shared survey-region workflow](SHARED_SURVEY.md).
This page records the earlier three-fire milestone; its scene configuration is
preserved in `config/fleet/fires_three.yaml`.

Start the usual fleet session inside Apptainer:

```bash
cd ~/user_ros_workspace/src/fire_observation_demo/tmux_fleet
./start.sh
```

Both drones start on the ground and use the existing manual takeoff, hover,
landing and click-to-fly controls. The world still requests real-time factor
**0.25**, with a **0.005 s** physics step. Actual speed depends on notebook load.

To save the cost of the separate Gazebo GUI, start a stopped fleet with:

```bash
FIRE_GZ_HEADLESS=true ./start.sh
```

RViz and both simulated camera pairs remain available in this mode; the cameras
still need rendering. The existing tmux attach commands are unchanged:

```bash
# Inside Apptainer:
tmux -L mrs_fire_fleet attach -t fire_fleet
# Or from Ubuntu:
tmux -S /tmp/apptainer/tmp/tmux-1000/mrs_fire_fleet attach -t fire_fleet
```

## Configure the scene

Edit [../config/fleet/fires.yaml](../config/fleet/fires.yaml):

| Scene name | Centre x, y [m] | Size [m] | Temperature [K] |
|---|---|---|---:|
| `patch_a` | 8, -2.5 | 1 × 1 | 480 |
| `patch_b` | 8, 0 | 2 × 2 | 560 |
| `patch_c` | 12, 6 | 3 × 2 | 640 |

These are stationary coloured hot surfaces on flat ground, rather than a flame,
smoke or combustion model. Keep patches separated by cool ground so thresholding
can distinguish them. The first two fit together in a downward camera view near
`(8, -0.8, 5)` m; the third is farther away.

At startup, `scripts/prepare_fleet.py` combines this file with the terrain,
launch pads and physics in `worlds/fire_fleet.sdf`. It writes the generated world
to **`tmux_fleet/runtime/fire_fleet.sdf`**, and `session.yml` launches that file.
Edit the source YAML or world template, then restart after landing both drones.
Runtime files are regenerated; do not use them as the place to save changes.

The scene file is **evaluation truth**. Neither the camera observers nor the
shared-map node receives its positions, dimensions, temperatures or model names.
Detected IDs are assigned in discovery order, so `Fire 1` need not be `patch_a`.

## A short demonstration

1. Select `uav1`, take off and wait for `MpcTracker`. Set goal altitude to 5 m,
   enable map commands, and send it near `(8, -0.8)` in RViz.
2. The annotated thermal view should show two separate complete hot regions.
   Two fire entries appear in the shared map and the **Shared fire map** table.
3. Move `uav1` away. The fire entries stay in the map; their last-seen ages grow.
   Return it near its pad `(0, -6)` and land.
4. Take off `uav2` and observe the same pair. Their existing IDs remain, and the
   contributor column now lists both drones.
5. Send `uav2` near `(12, 6, 5)` to observe the larger, hotter third patch.
   Return near `(0, 6)` and land. All three map entries remain available.

Both drones can still fly independently. Keep separated flight areas, and
observe the nearby patch cluster one drone at a time initially. Existing MRS
collision avoidance remains enabled; this milestone adds no route coordinator.

## Read RViz

The shared layer shows a labelled centre, projected boundary and yellow
uncertainty ellipse for every mapped fire. The label and bottom table report
centre coordinates, ground area, mean/maximum temperature, last-seen age and
contributing drones. `Visible from` lists current complete observers; a past
contributor remains listed after its camera leaves the fire.

In **Displays**, each drone group also contains **raw camera observations
(optional)**, disabled initially to keep the shared map readable. Enable these
to compare blue/green individual outlines and centres with the shared estimate.
Amber outlines are clipped image-edge regions. The camera's `r1`, `r2`, etc.
are frame-local region numbers, distinct from persistent global fire IDs.

Partial regions are displayed but do not create a global track or update its
centre. A centre update needs the complete connected hot region inside the
image. Previously mapped fires remain when their current observation is partial
or absent.

## What the estimate means

Every qualifying connected region above 450 K is projected through camera
calibration and the optical transform **at the image timestamp** onto the known
flat ground. There is no substitution of the latest drone pose for an unavailable
historical transform.

The shared mapper matches nearby centres, with an area-compatibility check and
one-to-one assignment within each image. The default distance gate is 0.9 m.
New regions in one frame cannot collapse into the same new track. Duplicate and
out-of-order frames do not count as additional evidence.

Each fire keeps at most **15 complete observations per drone**, with equal
weight for each contributing drone. Thus a faster camera cannot dominate the
centre just by sending more frames. The ellipse uses bounded sample spread,
pixel projection resolution and a 3 cm floor. It shows **illustrative 2-sigma
repeatability, not calibrated position accuracy**. Localization bias can remain;
more frames do not guarantee a more accurate centre.

The boundary is the latest complete measured outline, translated onto the
rolling centre. Its area is measured from that polygon. Mean temperature uses
the bounded source-balanced history; maximum temperature is the maximum within
that history. These are apparent simulated surface temperatures.

Tracks keep their IDs until a map reset or a rewind of simulation time. There
is a configurable 64-track cap.
Touching hot patches, large localization errors, splitting/merging fires and
changing fire shapes need more advanced association and are outside this static
demonstration. There is no Kalman filter or SLAM.

Perception settings live in
[../config/fleet/fire_mapping.yaml](../config/fleet/fire_mapping.yaml), separately
from scene truth. Drone configuration remains in `config/fleet/scenario.yaml`.

## ROS interfaces

| Topic/service | Purpose |
|---|---|
| `/fire_fleet/uav1/observations` | Timestamped JSON batch of all image-derived regions; replace `uav1` with `uav2` |
| `/fire_fleet/uav1/observation_markers` | Optional raw observation markers |
| `/fire_fleet/uav1/fire_status` | Current observer counts and regions, schema version 2 |
| `/fire_fleet/uav1/detection_image` | Annotated thermal image |
| `/fire_fleet/fire_map/status` | Shared JSON map, schema version 1 |
| `/fire_fleet/fire_map/markers` | Shared RViz markers |
| `/fire_fleet/fire_map/reset` | `std_srvs/srv/Trigger`: clear all shared fire tracks |

An observation batch contains `source`, `frame`, `stamp_ns`, `sequence` and
`observations`. Each region includes `region_id`, `partial`, `centre_m`,
`boundary_m`, `area_m2`, `mean_kelvin`, `max_kelvin`, `hot_pixels` and
`pixel_covariance_xy_m2`. The frame is `common_origin`; timestamps use simulation
time in nanoseconds. Observation topics are reliable and volatile; retained map
status and markers use reliable, transient-local QoS.

The shared status contains a `fires` list. Each entry has an `id`, geometry,
temperature, covariance, observation count, bounded window size,
`contributing_uavs`, `visible_from` and `last_seen_age_s`. It does not publish a
single ambiguous `/fire_pose` for a scene containing several fires.

Inspect the retained map:

```bash
ros2 topic echo /fire_fleet/fire_map/status --once --qos-durability transient_local
```

Clear the shared map explicitly:

```bash
ros2 service call /fire_fleet/fire_map/reset std_srvs/srv/Trigger '{}'
```

New complete images can immediately rebuild it. Existing `./fleetctl.sh
reset_map uav1` clears that observer's recent data and trail; it retains the
shared map. Choosing another drone never clears the shared map.

## Validation

Run geometry, detection, association and scene-generation regressions:

```bash
cd ~/user_ros_workspace/src/fire_observation_demo
python3 scripts/test_fire_geometry.py
python3 scripts/test_multi_fire.py
```

From `tmux_fleet/`, inspect the running ROS graph with:

```bash
python3 ../scripts/validate_fleet.py
```

To exercise the simulated flights, start with both drones at their launch pads:

```bash
python3 ../scripts/validate_fleet.py --exercise-flight
```

The check uses real camera images, starts with an empty shared map, detects two
fires in one image, tests independent takeoff/movement/hover/landing, verifies
shared IDs across both drones, visits the third fire and checks out-of-view
retention. Only validation reads scene truth to evaluate position, area and
temperature errors. It writes **`artifacts/milestone2_check.json`** and leaves
both drones landed with map goals disabled after success. A failed check records
its result and stops the sequence; inspect vehicle status before retrying.

Wall-time budgets default to a multiplier of 3 for a slow notebook. Increase
`--timeout-scale` if needed. Takeoff's setpoint warmup uses two simulated seconds;
communication and hardware-freshness checks remain based on elapsed real time.
