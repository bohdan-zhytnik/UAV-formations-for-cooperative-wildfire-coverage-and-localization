# Validation on Rob's notebook

## Shared region, automatic ownership and six fires, 29 October 2026

Version **0.5.0** built successfully inside the MRS base image. The original
installed MRS one-drone session, x500 model, Gazebo server configuration and
Apptainer wrapper still match the protection manifest. The demo preserves
the user's 0.25 requested speed, 0.005 s physics step, 2 m MRS avoidance radius,
3.5 m mission separation, and two drones with their existing camera rates.

* **45 unit/behavior tests passed:** 4 geometry, 13 multi-fire, 15 mission logic,
  4 ownership/editor, and 9 shared-survey checks. New cases cover contiguous
  equal halves, ground coverage at the split/outer edges, separated flight
  lanes, region validation, stable first-observer ownership, rotated-camera
  conservative view fitting, grouped observations, anticipatory owner-specific
  revisits, continued coverage under overdue work, and an old/transit observer
  unable to inspect a fire from a newly chosen half. The final eligibility
  guard also reproduced all six ownership decisions from actual flight data.
* The actual **RViz SurveyRegion plugin** passed a mouse-event interaction
  check on its own render panel. A drag produced ordered `common_origin`
  ground bounds. The check sends selection previews, not flight/configuration
  commands, and restores the default bounds afterwards. Process exit was 0.
* The final **23-check Gazebo exercise passed**. Both drones searched their
  halves, detected all six real thermal targets, automatically claimed one
  stable owner per ID, completed all **12/12** search points in each half while
  performing revisits, and confirmed owned fires with their own fresh complete
  camera observations. Manual hover paused uav1 while uav2 continued. Both
  returned to their pads, landed and disarmed. The final run resumed already
  airborne drones after a controls reload; cold-start empty-map behavior and
  independent manual takeoffs were exercised in the preceding run.
* Accepted/rejected region edits were checked through the same public command
  interface used by RViz. Invalid bounds left the old region intact; replacing
  an active survey was rejected. All active task destinations stayed in their
  assigned flight half. Minimum sampled horizontal vehicle separation during
  the exercise was **5.646 m**.
* No truth positions were preloaded into perception or planning. Evaluation
  matched each observed track to one configured scene target.

| Scene patch | Centre error at six-fire checkpoint |
|---|---:|
| A | 0.017 m |
| B | 0.022 m |
| C | 0.088 m |
| D | 0.237 m |
| E | 0.026 m |
| F | 0.079 m |

These are checkpoint errors in this flat simulation, not calibrated uncertainty
or a claim about converged/real-world accuracy. IDs follow discovery order.
Three fires belonged to each drone in this run.

The owner-specific observation ages were monitored during the latter part of
the patrol. Per-fire maxima in that monitoring window ranged from **14.8 to
23.0 simulated seconds**. Thus the 20-second target was occasionally missed;
RViz's overdue indicators expose this. It is a scheduling target rather than
a hard deadline guarantee. The flight report's lifetime maximum ages also
include the manual-pause/return/landing phase and should not be interpreted as
patrol revisit intervals.

The first run encountered a Wi-Fi transport interruption. The fleet environment
now pins Gazebo transport to `127.0.0.1` for this local notebook demo. A combined
view that assumed the wrong image-axis orientation was corrected to use the
smaller camera extent in both directions; the final run completed with that
check. Nearby southern fires shared complete inspection views, while the wider
northern pair received separate visits.

After verification, test-generated trails and fire tracks were cleared and
coverage reset to 0/12 for a fresh user demonstration. Both drones were left
landed with missions idle.

Evidence: `artifacts/shared_survey_check.json`, `shared_survey_flight.log`,
`shared_survey_monitored_ages.json`, `survey_tool_drag.log`,
`shared_survey_build.log`, `shared_survey_original_files.json`, and the two
`shared_survey_ready_*.json` snapshots. Usage: [SHARED_SURVEY.md](SHARED_SURVEY.md).

## Preset route blocked by a parked drone: correction, 29 October 2026

Live diagnosis found uav2 waiting for separation while uav1 was disarmed near
`(3.3, 1.1, 0.15)`. Uav2's route from its northern pad passed within the
6.5 m horizontal traffic margin, so the original reservation rule blocked it
even though uav1 was parked and the flight was clear of the vehicle body.
The live mission was also in cooperative mode: the old Preset button loaded
points without selecting waypoint mode.

* Mission status now includes confirmed arming and fresh connected hardware
  information. The coordinator uses a 2 m 3D stationary-obstacle margin for a
  confirmed disarmed vehicle within 0.5 m of the flat ground. Armed, unknown,
  stale or outstanding-reference states retain the conservative traffic rule.
* Preset selects Waypoint route in RViz. Blocking reasons appear in command
  status/tmux; repeated Start reports the actual active phase and reason.
* The package rebuilt successfully. All 15 mission-logic and 4 ownership/editor
  tests passed, including the captured geometry, retained airborne clearance,
  low paths through parked vehicles, and unknown/stale hardware states.
* `scripts/validate_parked_route.py --exercise-flight` reproduced the actual
  blocking geometry with uav1 still parked. Uav2 then flew to both preset points,
  inspected with fresh camera frames, completed one loop, and paused in hover.
  Uav1 remained disarmed. The test left uav2 airborne and paused for the user to
  resume, with uav2 selected in RViz.

Evidence: `artifacts/preset_route_diagnosis.json`, `preset_route_fix_build.log`,
`preset_route_fix_flight.log`, and `preset_route_fix_check.json`. Demo controls
and RViz were reloaded; Gazebo and MRS flight controllers continued running.
The shared camera map rebuilt as part of the demo-control reload.

## Cooperative observation, 29 October 2026

Version **0.4.0** built successfully inside the base MRS image, with our user
workspace mounted and the user's writable Apptainer overlay untouched.

* **33 unit/behavior checks passed:** the existing 4 geometry and 12 multi-fire
  checks, 13 mission-logic checks, and 4 vehicle-ownership/editor checks.
  These cover route order, repeat/once behavior, settling and fresh images,
  required per-drone fire observations, timeout behavior, explicit resume,
  stale session rejection, search-grid coverage, stale-fire assignments,
  intersecting corridors, pending manual destinations, late reference replies,
  missing coordinator heartbeats, and paused-only route editing.
* The existing **59-check Gazebo manual-flight regression passed** with the
  coordinator running but missions idle. It rechecked aligned common frames,
  actual RViz selection/camera connections, addressed goals, independent
  takeoff/hover/landing, three-fire detection and shared IDs across cameras.
* The new **44-check Gazebo mission exercise passed** using the public fleet
  command interfaces. Both drones flew their two-point routes, inspected with
  fresh cameras, completed a loop, and detected all three fires. Hover paused
  uav1 while uav2 continued inspecting. A manual destination and explicit resume
  worked. After moving both drones away, cooperative mode revisited separately
  assigned stale fires and required each drone's own fresh complete detection.
  Both continued to further search/inspection tasks. Resetting the shared map
  paused both cooperative missions. Each drone then returned to its own pad,
  landed and disarmed.
* CLI route clear, append, pop, preset and option changes were exercised while
  paused; the example routes and 20-second revisit thresholds were restored.
  Runtime route editing did not start a mission.
* The installed original MRS one-drone session, x500 model, Gazebo server
  configuration and existing Apptainer wrapper match the protection manifest.

The world used the user's existing requested factor **0.25**, physics step
**0.005 s**, and headless Gazebo with RViz and both camera pairs active.
The milestone did not change the world-speed setting. Mission timing uses
simulation time. The flight exercises use the same addressed protocol as the
RViz tools; the GUI route-edit gesture itself was not automated.

Evidence in `artifacts/`:

* `milestone4_manual_check.json`, `milestone4_manual_regression.log`
* `milestone4_check.json`, `milestone4_mission_flights.log`
* `milestone4_cooperative_live_status.json`, `milestone4_original_files.json`
* `milestone4_build.log`

The cooperative checkpoint recorded uav1 confirming fires 1 and 2 and uav2
confirming fire 3, followed by distinct search tasks. These IDs describe this
run's discovery order. The coordinator used detected map coordinates, never
the scene truth. The conservative reservations are a demonstration of spatial
work division, not a general collision-free fleet planner or deadlock solver.

Usage: [COOPERATIVE_OBSERVATION.md](COOPERATIVE_OBSERVATION.md).

## Three-fire shared map, 29 October 2026

- Built version 0.3.0 successfully inside the MRS base image with the shared
  workspace mounted. The user's writable Apptainer overlay was not modified.
- Four existing geometry regressions and twelve multi-fire regressions passed.
  The new tests cover all connected hot regions, clipped regions, temperatures,
  one-to-one association, reversed observation order across drones, stable IDs,
  replay/out-of-order rejection, out-of-view retention, bounded source-balanced
  histories, invalid-batch atomicity, the track limit and configurable scene
  generation with preserved physics settings.
- The full Gazebo flight exercise passed **59 checks**. It started with an empty
  shared map, detected two complete fires in one real simulated thermal image,
  then associated the second drone's observations with the same two IDs.
  It detected the third fire separately and retained all three tracks when
  the cameras left the scene. No observation frames were rejected in this run.
- Manual-control regression checks passed: disabled/unknown-drone rejection,
  actual Qt selector connection and camera subscriptions, coordinate conversion,
  addressed goals after switching selection, independent takeoff and hover,
  simultaneous flight in separated areas, and landing one drone while the
  other remained airborne. Both returned to their own pads and disarmed.
- The scene was generated from `config/fleet/fires.yaml`. The perception nodes
  received only calibration, transforms and images, not evaluation truth.

| Truth patch | Shared ID in this run | Centre error | Estimated area | Mean / maximum temperature |
|---|---:|---:|---:|---:|
| A: 1 m², 480 K | 1 | 0.064 m | 0.986 m² | 478.02 / 478.02 K |
| B: 4 m², 560 K | 2 | 0.158 m | 3.966 m² | 560.26 / 560.26 K |
| C: 6 m², 640 K | 3 | 0.297 m | 5.945 m² | 642.50 / 642.50 K |

A and B list both `uav1` and `uav2` as contributors. C was observed by `uav2`.
The table records the validation checkpoint; C then had two complete samples,
so it is not a claim about a converged or calibrated estimate. IDs depend on
discovery order. Temperature differences include the existing simulated thermal
rendering/quantization behaviour. The uncertainty ellipse remains illustrative.

- Kept the requested real-time factor at **0.5** and the physics step at
  **0.005 s** in both template and generated world. Validation used
  `FIRE_GZ_HEADLESS=true`; RViz and simulated cameras stayed active. A passive
  end-of-run sample reported achieved factor **0.159**, so the notebook still
  runs below the requested pace under load. Existing upstream timing and
  optional-ground-truth warnings remain.
- Protected original-file hashes still match for the wrapper, stock x500,
  stock one-drone session and Gazebo server configuration.

Evidence: [full flight report](../artifacts/milestone2_check.json),
[two-fire RViz view](../artifacts/milestone2_two_fires_rviz.png),
[three-fire overview](../artifacts/milestone2_three_fires_rviz.png), and
[original-file hashes](../artifacts/milestone2_original_files.json).
Usage and interfaces: [MULTI_FIRE_DEMO.md](MULTI_FIRE_DEMO.md).

## Two-drone manual fleet, 22 October 2026

- Built the separate `fleet_rviz` executable and `FleetGoal` RViz plugin with
  colcon; all package targets compiled successfully.
- The final simulated flight exercise passed **47 recorded checks**. It checked
  both independent adapters and observers, actual RViz camera subscriptions when
  switching selection, rejection of disabled/unknown-drone commands, repeated
  takeoff after landing, and correctly addressed goals even when the selector
  immediately switches to the other drone.
- Both MRS world origins coincide in `common_origin`; the two launch locations
  retain their 12 m separation. Accepted map coordinates match the addressed
  drone's world coordinates within the integration check's 8 cm tolerance.
- Both drones flew concurrently in separate areas. `uav1` hovered, returned and
  landed while `uav2` remained airborne and subsequently accepted another goal.
  Both returned to their own pads and disarmed.
- Each independent camera observer located the same `(8, 0)` m fire surrogate:
  `uav1` centre `(8.088, 0.059)` m, error **10.6 cm**; `uav2` centre
  `(8.160, -0.102)` m, error **18.9 cm**. These are observations from this run,
  including the existing MRS/Gazebo localization behaviour, not an accuracy
  guarantee or a fused fire estimate.
- The four existing geometry regressions still passed. The original one-drone
  model rendered with seven sensors and total link mass 2.208308 kg. Python,
  XML, YAML and shell syntax checks passed for the added resources.
- Recorded original-file hashes still match for the Apptainer wrapper, installed
  x500 model, stock one-drone session and Gazebo server configuration.
- A direct interaction check exposed an installed RViz Image display failure in
  `message_filters::removeCallback` when reconnecting a camera topic. The fleet
  UI now keeps four fixed image displays subscribed and switches visible docks.
  Six actual map clicks while alternating selection passed after this fix:
  every message captured the selected UAV, altitude, enable state and shared
  frame correctly, with no GUI crash. The visible RGB images showed the correct
  blue launch pad for `uav1` and green launch pad for `uav2` after selection.
  A package-local `FleetImage` subclass also clears the disconnected callback
  before deleting its subscriber, making repeated image cleanup safe without
  editing the installed RViz library. The final build passed the six actual
  click checks again and then exited on Ctrl+C with **exit code 0**.
- Camera rendering is 5 Hz per camera in the separate fleet model. Existing
  upstream timing/optional-ground-truth and Zenoh shutdown warnings remain; notebook CPU usage is
  high with Gazebo, RViz, two PX4 instances and both MRS stacks running.

Evidence: [fleet flight report](../artifacts/fleet_check.json),
[actual RViz goal-tool messages](../artifacts/fleet_tool_check.json),
[RViz shutdown check](../artifacts/fleet_shutdown_check.json),
[RViz map](../artifacts/fleet_rviz_view.png), and
[original-file checks](../artifacts/fleet_original_files_check.json).
Instructions: [FLEET_DEMO.md](FLEET_DEMO.md).

## Initial camera prototype, 16 October 2026

- Built `fire_observation_demo` with colcon inside the existing Apptainer environment.
- Rendered the custom x500 Jinja model: both camera sensors plus rangefinder present; total link mass 2.208308 kg.
- Started the custom Gazebo world, spawned x500_fire, and started PX4/MRS without changing installed MRS files.
- Received RGB images, raw mono16 thermal images, preview images, and calibration for both cameras.
- Confirmed optical frame IDs and positive camera focal lengths.
- Successfully enabled control output, selected offboard mode, armed, took off, and sent a stamped reference to (8,0,5) in uav1/world_origin.
- While observing the patch, recorded background ~292.98 K, hottest pixels ~601.38 K, and 12,747 thermal pixels above 500 K. The target is configured at 600 K.
- Fixed RViz's startup dock visibility in the package's own fire_rviz launcher. Confirmed one RViz subscriber per image, mapped camera-render windows, and visually inspected snapshots of both actual RViz image panels.
- Compared original wrapper, x500 template, one-drone session, and Gazebo server configuration against recorded SHA-256 hashes: unchanged.
- Original MRS timing warnings still occur; upstream timing issues and the earlier stock-core crash are outside this prototype's fixes.

Evidence is in `../artifacts/`: camera_check.json, camera_pair.png, rgb.png, thermal_raw.png, thermal_preview.png, rviz_rgb.png, and rviz_thermal.png.

The scene has a static coloured/thermal surrogate; it does not implement a fire-state estimator or formation coordinator.


## RViz fire mapping and flight controls, 20 October 2026

- Updated package built successfully with colcon, including native RViz controls.
- Four geometry regressions passed: downward optical axes, tilted-camera projection, invalid rays, and rolling-window memory/uncertainty floor.
- Fresh simulated flight exercise passed **21 reported checks**, plus invalid-altitude/unsupported-UAV and marker-visibility assertions. Covered RViz panel/topic connection, takeoff, command rejection, fixed-origin-to-world conversion, arrival at goals, two viewpoints, both footprints, fire boundary/centre/ellipse, hover, out-of-view retention, return to start, landing/disarm, and destination cleanup.
- First centre error: **5.03 cm**. Second-view error: **3.43 cm**. These describe this ideal simulated run, not a real-world accuracy guarantee.
- Projected boundary: **3.929 m²**, compared with the configured 4 m² patch.
- Visually inspected the actual native RViz map render and annotated thermal image. Confirmed 21 yellow ellipse pixels in the map screenshot: the small ellipse is visible and no longer covered by the centre sphere. Zooming makes it easier to see in this ideal scene.
- Rebuilt RViz exits on Ctrl+C. Both new Python nodes returned exit code 0 in isolated SIGINT tests; the installed Jazzy context's already-shut-down race is handled narrowly.
- A long earlier run reached PX4's Battery unhealthy preflight rejection. Takeoff stopped; restarting the scenario restored a fresh vehicle, and the full final exercise passed without bypassing checks.
- Original wrapper, upstream x500, stock one-drone session, and Gazebo server configuration still match recorded SHA-256 hashes.
- Left running with the drone landed/disarmed near the starting point, map commands disabled, and the last-seen fire layer retained.

Evidence: [flight report](../artifacts/rviz_demo_check.json), [thermal detection](../artifacts/fire_detection.png), [RViz map](../artifacts/rviz_fire_map.png), [visual check](../artifacts/visual_check.json), [shutdown checks](../artifacts/shutdown_check.json), and [original-file hashes](../artifacts/original_files_check.json). Instructions: [RVIZ_DEMO.md](RVIZ_DEMO.md).

The centre is a rolling mean with no Kalman filter and no added noise. The ellipse is illustrative repeatability with a 3 cm floor. Known terrain and fire observations are separate map layers; terrain-based visual localization/SLAM is not implemented. Existing upstream timing warnings remain outside this demo's changes.
