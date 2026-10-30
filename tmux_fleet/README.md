# Fleet simulation speed

Current workflow: [shared survey region and six fires](../docs/SHARED_SURVEY.md).
Use the **Survey** tab to define one region for both drones.

Mission instructions: [cooperative observation and waypoint supervision](../docs/COOPERATIVE_OBSERVATION.md).
The same fleet now includes an RViz **Missions** tab and `./missionctl.sh`.
Both drones still start on the ground with missions idle.

The two-drone demo currently requests a **real-time factor of 0.25** at startup:
one simulated second takes approximately four real seconds. Both drones share
this setting because they run in the same Gazebo world. Under heavy load, the
achieved factor can be lower than the requested value.

## Where the setting is configured

Edit **[../worlds/fire_fleet.sdf](../worlds/fire_fleet.sdf)**, inside the world's
`<physics>` element:

```xml
<physics name="default_physics" type="ignored">
  <max_step_size>0.005</max_step_size>
  <real_time_factor>0.25</real_time_factor>
</physics>
```

On Ubuntu, the source file is:

```text
/home/rob/mrs_apptainer/user_ros_workspace/src/fire_observation_demo/worlds/fire_fleet.sdf
```

Inside the usual Apptainer workspace, it is:

```text
~/user_ros_workspace/src/fire_observation_demo/worlds/fire_fleet.sdf
```

At startup, `scripts/prepare_fleet.py` reads this template and adds the hot
patches from `config/fleet/fires.yaml`. It writes
`tmux_fleet/runtime/fire_fleet.sdf`, preserving the template's physics settings.
The `gazebo` window in [session.yml](session.yml) launches that generated world.
Save speed changes in **`worlds/fire_fleet.sdf`**, rather than the generated
runtime file. This workspace's symlink installation links installed resources
to the source files.

Keep `max_step_size` at `0.005` seconds when changing playback speed. That is
the physics time step; `real_time_factor` controls how quickly simulated time
advances relative to real time.

| Requested factor | Real seconds for one simulated second |
|---|---:|
| `1.0` | 1 |
| `0.5` | 2 |
| `0.25` | 4 |
| `0.1` | 10 |

## Apply a file change

Gazebo reads the world setting when it starts. Editing the SDF does not change
an already running world. After landing both drones, restart the fleet from a
terminal inside the existing Apptainer:

```bash
cd ~/user_ros_workspace/src/fire_observation_demo/tmux_fleet
./kill.sh
./start.sh
```

The existing symlink installation needs no rebuild for this world-file edit.
If using a different installation that copies resources, rebuild the package
first from `~/user_ros_workspace`:

```bash
colcon build --packages-select fire_observation_demo --symlink-install
source install/setup.bash
```

## Change the running world's speed

For a temporary change without restarting, run this from a fleet tmux terminal
inside Apptainer, where `GZ_PARTITION=mrs_fire_fleet` is already configured:

```bash
gz service -s /world/default/set_physics \
  --reqtype gz.msgs.Physics \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'real_time_factor: 0.5'
```

A successful request returns `data: true`. This changes the current world;
the SDF remains the startup configuration for future runs. Use the same command
with `real_time_factor: 1.0` to request normal speed temporarily, or edit the
SDF to make that the startup default again.

Read the achieved speed with:

```bash
gz topic -e -t /world/default/stats -n 1
```

Inspect the returned `real_time_factor`. A requested factor of `0.5` is a target,
so this change alone does not establish that simultaneous flight is stable.

This configuration applies to the fleet demo. The original one-drone world and
installed MRS/Apptainer files are unchanged. Full flight instructions are in
[../docs/FLEET_DEMO.md](../docs/FLEET_DEMO.md); three-fire mapping is described in
[../docs/MULTI_FIRE_DEMO.md](../docs/MULTI_FIRE_DEMO.md).
