# Your MRS Apptainer and Gazebo guide

Prepared for Rob on 13 October 2026.

This guide combines your local MRS documentation with inspection of `/home/rob/mrs_apptainer` and the software installed in its base image. I surveyed the documentation collection and examined the Apptainer, ROS 2 workspace, simulation, API, configuration, and launch sections in detail. The collection also contains hardware manuals, academic resources, and unrelated material; those are not prerequisites for your daily simulation workflow.

Commands labelled **Ubuntu host** belong in your normal notebook terminal. Commands labelled **Inside Apptainer** belong after starting the wrapper, or in a terminal opened by the VS Code instance launched from that container.

## 1. What you have installed

| Component | Your setup | What it does |
|---|---|---|
| Apptainer | Version 1.5.4 | Runs the container on your Ubuntu notebook |
| Base image | `/home/rob/mrs_apptainer/images/mrs_uav_system.sif` | Contains the prepared operating-system libraries and robotics software |
| Container operating system | Ubuntu 24.04 | Supplies the userspace environment inside the image |
| ROS | ROS 2 Jazzy | Connects robotics programs through topics, services, and other interfaces |
| MRS Gazebo package | `mrs_uav_gazebo_simulator`, package version starts with `2:2.0.0` | Connects simulated drones to the MRS UAV system |
| Gazebo | `gz sim`, version 8.15.0 | Runs the simulated world, physics, and sensors |
| Alternative simulator | `mrs_multirotor_simulator` | Provides another simulation backend for MRS development |
| Persistent overlay | `overlays/mrs_uav_system.img`, 2 GiB logical capacity | Stores additional system software, including your VS Code installation |
| Your source repository | `user_ros_workspace/src/mrs_core_examples` | Contains C++ and Python examples |

The Gazebo executable was checked after sourcing ROS. The command named `gazebo` was absent; your installation uses the modern `gz sim` command. Do not interpret an old tutorial's `gazebo: command not found` as evidence that your simulator is missing.

The package and launch-file checks succeeded. I did not start a GUI simulation or test flight commands during this inspection. Your successful VS Code launch is confirmed by your report; the base-image inspection intentionally did not remount your active writable overlay.

## 2. Understand the host, container, and workspace

Your **host** is the Ubuntu installation that boots your notebook. It owns the actual files, kernel, display, GPU drivers, network interfaces, and hardware.

Your **container image** supplies a prepared Linux userspace: programs, libraries, ROS, MRS packages, and simulator dependencies. Apptainer runs those programs using the host's Linux kernel. It does not boot a second kernel or a complete virtual machine.

Your **workspace** contains the code you are developing. The wrapper mounts that directory into the container. Editing a file there changes the same underlying file on the notebook.

```text
Ubuntu notebook
│
├── mrs_apptainer/images/mrs_uav_system.sif
│     Prepared ROS + MRS + Gazebo software
│
├── mrs_apptainer/overlays/mrs_uav_system.img
│     Persistent additions, including VS Code
│
├── mrs_apptainer/user_ros_workspace
│     Your source code and build output
│             │ bind mount
│             └── visible inside as /home/rob/user_ros_workspace
│
└── /tmp/apptainer/home
      Used as /home/rob inside the container
```

The word “container” also appears in ROS 2 documentation. A **ROS component container** is a process that hosts multiple ROS nodes. It is a different concept from your **Apptainer container**, which supplies their runtime environment.

## 3. Your wrapper settings

The normal entry point is `/home/rob/mrs_apptainer/wrapper.sh`.

| Setting | Current value | Practical meaning |
|---|---|---|
| `CONTAINER_NAME` | `mrs_uav_system.sif` | Selects your base image |
| `CONTAINED` | `true` | Uses a separate home directory instead of binding your normal home as the container home |
| `CLEAN_ENV` | `true` | Cleans most host environment variables before starting the container |
| `OVERLAY` | `true` | Loads the persistent additions |
| `OVERLAY_NAME` | `mrs_uav_system.img` | Selects the overlay file |
| `WRITABLE` | `false` | Does not use a writable sandbox-directory image |
| `FAKEROOT` | `false` | Runs as your regular user |
| `USE_NVIDIA` | `false` | Does not request NVIDIA library integration through `--nv` |
| `DETACH_TMP` | `true` | Maps container `/tmp` to host `/tmp/apptainer/tmp` |

`CONTAINED=true` controls the home-directory mapping in this wrapper. It is not a promise that every other host path is inaccessible: Apptainer has other default bind behavior, and the wrapper explicitly exposes the workspace and graphics connections.

For ordinary work, use the wrapper. A bare `apptainer shell images/mrs_uav_system.sif` may omit your overlay, workspace mount, graphics configuration, and shell initialization.

## 4. Where to save your work

| Inside the container | On your notebook | Persistence |
|---|---|---|
| `~/user_ros_workspace` | `/home/rob/mrs_apptainer/user_ros_workspace` | Normal persistent host files |
| `/opt/mrs/host` | `/home/rob/mrs_apptainer/mount` | Normal persistent host files |
| `~` outside the workspace mount | `/tmp/apptainer/home` | Temporary host location; can be cleared |
| `/tmp` | `/tmp/apptainer/tmp` | Temporary host location; can be cleared |
| System files changed through the overlay | `overlays/mrs_uav_system.img` | Persist with that overlay |

**Keep source code, simulation sessions, recordings, and important configuration under `~/user_ros_workspace`.**

The wrapper does not put your actual notebook home at the container's home path. Both prompts can show `rob`, and both homes can be spelled `/home/rob`, while referring to different storage.

Your VS Code settings and extensions normally live under the container home. In this setup that means `/tmp/apptainer/home`, unless you choose another location. They may survive leaving and reopening the container but are vulnerable to temporary-directory cleanup and reboot policies.

The README suggests deleting `/tmp/apptainer` as a troubleshooting reset. That also deletes this temporary home and its VS Code settings. Back up anything valuable there before using such a reset; it is not a routine step.

The outer `mrs_apptainer` repository ignores the user workspace contents. Committing the outer repository does not automatically back up your code. Keep your own packages in their own Git repositories and push or back them up separately.

## 5. Starting and identifying the environment

**Ubuntu host:**

```bash
cd /home/rob/mrs_apptainer
./wrapper.sh
```

Your prompt should show `[MRS Apptainer]`.

**Inside Apptainer:**

```bash
echo "$APPTAINER_NAME"
echo "$ROS_DISTRO"
ros2 pkg prefix --share mrs_uav_gazebo_simulator
```

Expected image name: `mrs_uav_system.sif`; expected ROS distribution: `jazzy`.

The package command should return:

```text
/opt/ros/jazzy/share/mrs_uav_gazebo_simulator
```

Leave the interactive shell with `exit` after stopping your simulation and closing the applications you started there.

## 6. Working in VS Code

**Inside Apptainer:**

```bash
code ~/user_ros_workspace
```

Because you launch this VS Code from the container, it uses the container's runtime environment. Its ordinary integrated Bash terminal should also run there. Check that terminal with:

```bash
echo "$APPTAINER_NAME"
echo "$ROS_DISTRO"
command -v colcon
```

Extensions, tasks, and terminal profiles can customize how commands launch, so check the environment if a tool behaves differently from the surrounding shell.

For a more durable VS Code profile, you can instead use:

```bash
mkdir -p ~/user_ros_workspace/.vscode-user-data
mkdir -p ~/user_ros_workspace/.vscode-extensions
code --user-data-dir ~/user_ros_workspace/.vscode-user-data \
     --extensions-dir ~/user_ros_workspace/.vscode-extensions \
     ~/user_ros_workspace
```

Close existing VS Code windows first when switching profiles. These directories can contain account information and extensions; exclude them from your package repositories.

VS Code does not need `sudo` for daily editing. Run builds as `rob` as well, so generated files stay owned by your user.

## 7. Why Gazebo can run inside Apptainer

Gazebo is a Linux application. It needs compatible libraries, CPU resources, graphics access for its GUI and rendering, and connections to the robotics programs it simulates.

Your setup supplies these pieces:

1. **Compatible software:** the image includes ROS Jazzy, the MRS Gazebo integration, modern Gazebo, and related dependencies.
2. **CPU and memory:** container processes run on the notebook's CPU using the host kernel and available memory.
3. **Display access:** the wrapper forwards `DISPLAY` and binds the X11 socket directory and your `.Xauthority` file when available. Gazebo's GUI can appear on your normal desktop even though its process runs inside Apptainer.
4. **Graphics access:** the wrapper binds `/dev/dri`. This exposes graphics device nodes; actual acceleration depends on your hardware and compatible driver libraries. NVIDIA integration is a separate wrapper option and is currently disabled.
5. **Robotics communication:** the launch session starts the simulator, bridge, autopilot-related processes, hardware API, MRS core, and visualization with matching configuration.

Apptainer supplies the environment. Gazebo supplies the simulated world. MRS supplies the UAV estimation and control system.

You can use Gazebo to develop and test drone behavior without a physical aircraft: inspect sensors, test waypoints, debug controllers, try different models or worlds, and repeat experiments. How much a simulation predicts a real aircraft depends on the model, sensor behavior, delays, and operating conditions.

## 8. What the complete simulated drone system does

The installed example launches more than a 3D window:

```text
Your mission program or manual command
                    │
                    ▼
            MRS ControlManager
                    │
              Reference tracker
                    │
              Feedback controller
                    │
                Hardware API
                    │
          Simulated autopilot / vehicle
                    │
           Gazebo dynamics and sensors
                    │
              State estimation
                    └──────── feedback to MRS control
```

This diagram shows the conceptual feedback loop. The bridge and the hardware API provide different connections; not every signal travels through the same path.

Important components:

- **Gazebo:** simulates the world, vehicle dynamics, and configured sensors.
- **Drone spawner:** creates a drone model from the configured templates and starts the associated simulation processes.
- **PX4 SITL:** runs autopilot software on the computer; SITL means software in the loop.
- **MRS PX4 hardware API:** connects MRS to the autopilot interface.
- **EstimationManager:** provides the estimated vehicle state used for feedback.
- **Tracker:** turns a high-level goal into a feasible motion reference.
- **Controller:** uses the state estimate and reference to compute control output.
- **UavManager:** coordinates procedures such as takeoff and landing.
- **ConstraintManager and GainManager:** manage motion limits and controller gains.
- **RViz:** displays ROS information such as frames, estimates, references, and markers. It is a visualizer; Gazebo provides the simulated world in this scenario.
- **Zenoh router:** supports discovery and communication for this session's selected ROS middleware.

You usually develop a high-level mission node first. Replacing a controller or estimator is a more specialized task; the repository contains plugin examples for those cases.

## 9. Start your installed one-drone Gazebo example

Start from a container shell opened with the wrapper.

**Inside Apptainer:**

```bash
cd "$(ros2 pkg prefix --share mrs_uav_gazebo_simulator)/tmux/one_drone"
./start.sh
```

This starts a tmux session named `simulation` on the tmux socket named `mrs`.

The installed `session.yml` defines windows for:

| Window | Purpose |
|---|---|
| `zenoh_router` | Starts `rmw_zenohd` |
| `gazebo` | Starts Gazebo and spawns the drone |
| `px4_api` | Starts the MRS PX4 API |
| `core` | Starts the MRS UAV core |
| `status` | Shows MRS status |
| `autostart` | Runs automatic flight-start logic |
| `takeoff` | Waits for the core, then runs the arming/offboard script |
| `rviz` | Starts RViz with simulation time |
| `layout` | Tries to arrange GUI windows with an i3-specific helper |

**This example is configured to attempt automatic takeoff.** Its takeoff script arms `uav1` and requests offboard mode; the autostart node handles the subsequent procedure when conditions are satisfied. Starting it is not just opening Gazebo.

Your installed scenario selects `uav1`, the `x500` model, simulation mode, a rangefinder, and the `gps_garmin` state estimator. After takeoff it configures `Se3Controller` with `MpcTracker`.

The copied world configuration defines a horizontal polygon from -50 to 50 metres in its `world_origin` frame and vertical bounds of 0.5 to 15 metres. These are the example's configured safety bounds, not an instruction to fly at their edges.

The layout window's `~/.i3/layout_manager.sh` may be absent on a normal Ubuntu desktop. An error confined to this optional window does not by itself establish that the simulator or control system failed. Inspect the Gazebo, core, and status windows separately.

## 10. Navigating and stopping tmux

The installed MRS tmux configuration uses **Ctrl+A** as its prefix. Press Ctrl+A, release the keys, then press the next key.

| Keys | Action |
|---|---|
| Ctrl+A, then `w` | Choose a window |
| Ctrl+A, then `n` | Next window |
| Ctrl+A, then `p` | In this MRS configuration, paste from the clipboard |
| Alt+arrow | Move between panes |
| Ctrl+A, then `d` | Detach; programs continue running |
| Ctrl+A, then `k` | Open the MRS session-kill confirmation menu |

The `p` binding is customized, so standard tmux shortcut lists do not all apply unchanged. Use the window chooser when in doubt.

If detached, reattach from a terminal in the same container environment:

```bash
tmux -L mrs attach -t simulation
```

You can also stop the example by running its `kill.sh` from that scenario directory. Detaching is different from stopping: a detached simulator can still consume CPU and keep its processes alive.

The Gazebo and multirotor examples both use the `simulation` session name and `mrs` socket. Stop one scenario before starting another unchanged example.

## 11. Using ROS commands alongside the simulation

The installed session sets:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export UAV_NAME=uav1
export USE_SIM_TIME=true
```

An independently opened VS Code terminal does not inherit variables that were exported later in another terminal. Set these explicitly in the terminal from which you interact with the simulation:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export UAV_NAME=uav1
export USE_SIM_TIME=true
```

The session already starts its Zenoh router. Do not start a duplicate router automatically whenever you open another terminal.

Basic observations:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 topic echo /uav1/estimation_manager/odom_main --once
ros2 topic hz /uav1/estimation_manager/odom_main
```

Stop continuous observation commands with Ctrl+C.

Inspect a service and its interface before using it:

```bash
ros2 service type /uav1/control_manager/goto
ros2 interface show mrs_msgs/srv/Vec4
```

Your installed `Vec4` request contains a `goal` array with four numbers.

**Simulation only, after successful takeoff and with no other mission commanding the UAV:**

```bash
ros2 service call /uav1/control_manager/goto_relative \
  mrs_msgs/srv/Vec4 "{goal: [1.0, 0.0, 0.0, 0.0]}"
```

This requests a small relative displacement along the relevant world-frame x direction. It does not mean “forward along the nose” for every heading. Inspect the response and verify the behavior in status/RViz.

Normal hover and landing requests, when the services are present:

```bash
ros2 service call /uav1/control_manager/hover std_srvs/srv/Trigger '{}'
ros2 service call /uav1/uav_manager/land std_srvs/srv/Trigger '{}'
```

A service request can be received and still be rejected because the vehicle state, safety area, or active tracker does not allow it. Read the returned success flag and message.

## 12. Coordinates, namespaces, and time

`uav1` is the vehicle namespace: `/uav1/control_manager/...` targets that drone. Another drone normally has its own namespace.

The frame tells you how to interpret a position or velocity:

| Frame | Meaning |
|---|---|
| `uav1/fcu` | Vehicle body frame |
| `uav1/local_origin` | Frame established at the starting position and orientation |
| `uav1/stable_origin` | Intended to remain continuous across estimator changes |
| `uav1/world_origin` | Configured global/local world frame when available |
| Sensor frames | Coordinates specific to a camera, lidar, or rangefinder |

Some frames depend on the active estimator and sensors. Do not assume every frame exists in every scenario.

For your own code, use stamped reference messages/services with explicit frame IDs and timestamps. The simple `goto` family is convenient for manual interaction but omits a message header.

There is also a specific MRS pitfall: repeatedly calculating a new hover reference from the latest measured position can let the reference drift. For maintaining or incrementing a commanded pose, the documentation recommends using the existing control reference. Actual measured position is still appropriate when your task really depends on it, such as reacting to nearby objects.

Simulation uses a simulated clock. ROS nodes need the `use_sim_time` parameter enabled to use `/clock`. `USE_SIM_TIME=true` is a convention consumed by MRS launch files; it does not automatically set that parameter for every arbitrary ROS node.

For a standalone node that supports normal ROS parameters, a typical invocation is:

```bash
ros2 run YOUR_PACKAGE YOUR_EXECUTABLE --ros-args -p use_sim_time:=true
```

Replace those uppercase placeholders with a real package and executable. Mixing wall time with simulation time can cause stale-message checks and synchronization problems. A paused simulation can also make time-dependent tools appear to wait indefinitely.

## 13. Build packages from the workspace root

Your intended layout is:

```text
user_ros_workspace/
├── src/
│   └── mrs_core_examples/
├── build/
├── install/
└── log/
```

Inspection found existing `build`, `install`, and `log` directories **under `user_ros_workspace/src`**, while no root-level `install` directory was present. This suggests a previous build was run from `src`.

The wrapper's shell initialization automatically sources:

```text
~/user_ros_workspace/install/setup.bash
```

It does not automatically source `~/user_ros_workspace/src/install/setup.bash`. Consequently, previously compiled examples may not be found in a fresh shell.

For a new build, use the workspace root. Leave the old generated directories alone until the new build is working; there is no need to delete them just to read this guide.

**Inside Apptainer, preferably in a fresh terminal:**

```bash
cd ~/user_ros_workspace
colcon list
colcon build --packages-up-to example_waypoint_flier \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
               -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
ros2 pkg prefix example_waypoint_flier
```

`--packages-up-to` builds the selected example and workspace dependencies needed by it. Use `colcon build` without that selection when you want all discovered packages.

`RelWithDebInfo` keeps optimization enabled while retaining debugging information. `--parallel-workers 2` limits the number of concurrently built packages; it does not necessarily limit compiler threads inside a single package. If memory remains a problem, reduce both forms of parallelism, for example with `CMAKE_BUILD_PARALLEL_LEVEL=2` for CMake-based builds.

The documentation describes an optional MRS shell wrapper that adds `colcon init` and `presource_ros`. In my check of your base image with the mounted MRS Bash configuration, `colcon` was the regular `/usr/bin/colcon`, and `presource_ros` was absent. Do not assume those custom helpers are installed. Ordinary `colcon build` from the correct directory is the relevant starting point here.

After building, source the workspace in each already-open terminal that needs the new packages. The wrapper automatically sources the root-level install in later shells when it exists.

If a C++ editor cannot find headers, point its compile-command configuration at the appropriate generated `build/<package>/compile_commands.json`. That file describes the actual compiler include paths and options inside the container.

## 14. Your example waypoint flier

You already have the `mrs_core_examples` repository. You do not need to clone a second copy to follow this example.

After a successful root-level build and sourcing the workspace:

```bash
cd ~/user_ros_workspace/src/mrs_core_examples/cpp/example_waypoint_flier/tmux
./start.sh
```

Stop the earlier simulation first. This example's local `session.yaml` launches the **MRS multirotor simulator**, not Gazebo. Its visualization window is RViz. Therefore, not seeing a Gazebo window in this example is expected.

The example includes service commands for moving to the first waypoint, starting waypoint following, and stopping new waypoint submission. Use the commands supplied in that session, and inspect their service responses. The source README explicitly says that stopping waypoint following does not interrupt the flight to the current waypoint.

Follow the actual session file and installed service names: this local repository contains some `session.yml` files and some `session.yaml` files, and some names differ from the documentation's older examples.

## 15. Gazebo versus the MRS multirotor simulator

| Choice | Useful for | What you should expect |
|---|---|---|
| Gazebo scenario | World geometry, simulated sensors, vehicle/autopilot integration | Gazebo GUI plus ROS tools such as RViz |
| MRS multirotor scenario | Control and mission development using the MRS dynamics simulator | Primarily ROS visualization such as RViz |

Both are present in your base image. The lighter simulator is useful when you do not need Gazebo's world and sensor features; actual performance depends on the scenario.

To try the installed MRS example after stopping other scenarios:

```bash
cd "$(ros2 pkg prefix --share mrs_multirotor_simulator)/tmux/mrs_one_drone"
./start.sh
```

The installed MRS example also contains automatic takeoff and a commands window running `goto.py`; inspect the scenario before assuming it merely hovers.

## 16. Customize a scenario without editing installed packages

Installed files under `/opt/ros/jazzy/share` are useful templates. Store your working scenario under the mounted workspace.

**Inside Apptainer:**

```bash
mkdir -p ~/user_ros_workspace/simulations
cp -r "$(ros2 pkg prefix --share mrs_uav_gazebo_simulator)/tmux/one_drone" \
      ~/user_ros_workspace/simulations/gazebo_one_drone
cd ~/user_ros_workspace/simulations/gazebo_one_drone
code .
```

Run that copy with `./start.sh` after stopping the previous scenario.

Files to understand:

- `session.yml`: starts programs and exports scenario variables.
- `config/custom_config.yaml`: overrides selected MRS defaults.
- `config/world_config.yaml`: describes the MRS world origin and safety area.
- `config/network_config.yaml`: configures the MRS core's network-related settings.
- `config/automatic_start.yaml`: configures automatic start behavior.
- `takeoff.sh`: the scenario's arming/offboard commands.

The Gazebo **SDF world** defines simulated objects and physics. The MRS **world configuration YAML** defines coordinate and safety information. They serve different purposes.

Your installed `simulation.launch.py` accepts `world_file`, `spawner_config`, `gz_headless`, and other launch arguments. To see the installed arguments:

```bash
ros2 launch mrs_uav_gazebo_simulator simulation.launch.py --show-args
```

For example, in your copied session, changing `gz_headless:=false` to `gz_headless:=true` omits the Gazebo GUI. RViz is a separate window and will still be started unless you also change that part of the session. Headless mode can still need rendering support for simulated cameras or other rendered sensors.

The spawner request selects the model and optional sensors. The current example spawns `1 --x500 --enable-rangefinder`. Adding another model requires compatible simulator resources and an appropriate MRS platform configuration, not just an arbitrary name.

For MRS parameters, inspect what your installed system exposes:

```bash
ros2 run mrs_uav_core get_public_params.py \
  > ~/user_ros_workspace/public_mrs_parameters.yaml
```

Keep custom YAML overrides small. The documentation describes custom configuration as having priority over platform, world, network, and default settings in the parameter-loading pipeline. Exact behavior depends on the node and parameter loader involved.

When editing config or launch files that belong to a built ROS package, rebuild that package so its installed files are updated. Editing a source YAML does not always change the copy actually loaded from `install/`. Standalone copied scenario files used directly by the session do not require a ROS rebuild just because you edited them.

## 17. Installing additional software

Your earlier errors had two separate causes:

- The `.sif` base filesystem was read-only.
- Running `sudo` inside your normal container could not gain root privileges; it reported ownership and `no new privileges` errors.

A writable overlay gives system modifications somewhere to persist, but it does not itself grant your user root privileges.

Your successful VS Code installation used this host-side script:

```text
/home/rob/mrs_apptainer/scripts/install_vscode.sh
```

It is run with host `sudo`, starts a contained Apptainer command with the overlay, and installs `code` inside that environment. It does not install the VS Code package into the host operating system.

Do not run it again just because you want to open VS Code. Use `code` inside the ordinary wrapper session.

For later system-package installation, stop applications using this overlay and perform a deliberate maintenance command as root from the host. Avoid using `sudo ./wrapper.sh` for normal editing or builds because root-run processes can create files with inconvenient ownership and different home paths.

The overlay has a logical capacity of 2 GiB and currently uses roughly 1.4 GiB of host disk blocks. This is not a measurement of its remaining internal free space. A sparse file can consume fewer host blocks than its logical capacity. Workspace source and build output live in the bind-mounted directory, outside the overlay capacity.

Do not recreate or format `mrs_uav_system.img`: that would erase the installed additions. Back it up before changing it or attempting a resize.

The official Apptainer guide explains the overlay mechanism and sparse-file behavior: [Persistent overlays](https://apptainer.org/docs/user/latest/persistent_overlays.html).

## 18. Multiple terminals and the overlay

An ext3 overlay image should not be independently mounted writable by several container launches at once. Apptainer may report that the overlay is already in use.

For your current workflow, start the wrapper once, launch VS Code inside it, and use additional integrated terminals or tmux panes from that running environment. This avoids repeatedly launching the wrapper against the same writable image.

For more advanced use, a completed overlay can be mounted read-only by appending `:ro` to its path. That is a deliberate runtime configuration change, and requires all writable users to stop first. Do not mix a writable maintenance mount with concurrent readers of the same filesystem image.

Always close VS Code and stop the simulation before installing or updating system packages through the overlay.

## 19. Troubleshooting in the order that helps

### A. `ros2` or `colcon` is missing

First verify that you are inside the wrapper environment:

```bash
echo "$APPTAINER_NAME"
echo "$ROS_DISTRO"
```

In a manually created shell, sourcing `/opt/ros/jazzy/setup.bash` may restore ROS paths, but use the wrapper for normal work so mounts and shell settings are also configured.

### B. Your own package is not found

Check the workspace location and sourcing:

```bash
cd ~/user_ros_workspace
colcon list
ls install/setup.bash
source install/setup.bash
ros2 pkg prefix example_waypoint_flier
```

Your existing output under `src/install` is particularly relevant here. Rebuild from the workspace root before expecting the wrapper to source it automatically.

### C. The GUI is missing or reports display errors

Inside the container:

```bash
echo "$DISPLAY"
ls /tmp/.X11-unix
ls /dev/dri
```

The wrapper prints a message when a requested mount source, such as `.Xauthority`, is absent. Whether that matters depends on your desktop/X11/XWayland setup. Start the wrapper from your local graphical desktop session and preserve the exact GUI error when asking for help.

The wrapper currently uses `xhost +` while it runs and `xhost -` when it returns. This is broad display-access configuration inherited from the repository. Treat it as part of the wrapper's implementation; do not add more `xhost +` commands as a routine fix.

### D. Gazebo is slow

Start with one drone and minimal sensors. Inspect CPU and memory load, and compare with the MRS multirotor example. Disable an unnecessary GUI or expensive sensors in a copied scenario when appropriate.

`USE_NVIDIA=false` means the wrapper does not enable NVIDIA integration. Your notebook model name alone does not establish which GPU is rendering the application. Check host driver operation and actual renderer information before changing that setting.

Old documentation contains Gazebo Classic commands such as `gz physics`. Your installed simulator is modern Gazebo, so do not assume those speed-control commands apply. Inspect the installed simulator and supported settings before using an older recipe.

### E. ROS commands cannot see the simulation

Confirm the Zenoh router window is running and that your terminal uses `RMW_IMPLEMENTATION=rmw_zenoh_cpp`. Check that terminal and session use compatible ROS domain and router configuration if you changed them.

If you previously ran ROS CLI commands with another middleware, a cached ROS daemon may also need restarting with `ros2 daemon stop` before retrying under the correct environment. Avoid launching duplicate routers as a first response.

### F. The drone never takes off

Inspect the `status`, `core`, `gazebo`, `px4_api`, `autostart`, and `takeoff` windows. Possible causes include a missing drone, unavailable sensor/odometry data, autopilot connection failure, rejected arming/offboard requests, or an unready core.

Do not repeatedly send flight commands without reading the status and service response. The session is already attempting its configured automatic start sequence.

### G. A modified YAML seems to have no effect

Determine whether the launch file reads your source file, a copied standalone scenario file, or an installed package file. Check the launch argument path and rebuild the package if it uses an installed copy.

### H. Overlay is already in use

Keep using the terminals inside your existing container, or close the applications and stop the existing session before relaunching the wrapper. Do not delete the overlay or alter its locks while processes are using it.

## 20. Backups and reproducibility

Your complete working environment consists of more than the `.sif` file:

1. Base image: the prepared software stack.
2. Overlay: extra installed system software such as VS Code.
3. Workspace: source, simulation scenarios, and optionally recordings/build output.
4. Wrapper and mount configuration: how the host connects to the container.
5. Important editor settings: if you keep them in the temporary home, back them up separately.

Stop processes using the overlay before copying it. Preserve sparse files when your backup tool supports it. Back up source repositories frequently; generated `build`, `install`, and `log` directories can usually be recreated, while uncommitted source changes cannot.

Do not replace your base image with an arbitrary new `stable` image and assume an old overlay remains compatible. The overlay can contain packages and libraries built against the earlier base. Keep the old pair together, test updates separately, and record the exact image/version used for experiments.

For a repeatable customized software environment, a documented Apptainer recipe is better long-term than undocumented manual package changes. The repository contains `recipes/user_modifications_from_existing_img`, but its current example also installs Sublime Text and uses older installation instructions; review and tailor it before running it.

## 21. A practical learning sequence

1. Learn which terminal is the host and which is inside Apptainer.
2. Open VS Code on the mounted workspace and identify where saved files live.
3. Start the installed one-drone Gazebo example and inspect all its tmux windows.
4. Observe ROS nodes, topics, odometry, and service responses from a correctly configured terminal.
5. Stop the scenario cleanly.
6. Build `example_waypoint_flier` from the workspace root and source the result.
7. Run its separate multirotor scenario and understand the waypoint services.
8. Copy a scenario into the workspace and change one configuration item at a time.
9. Develop a mission node before moving on to custom trackers, controllers, or estimators.

## 22. Local references and verification limits

These are the most relevant sources on your notebook:

- [Apptainer repository README](/home/rob/mrs_apptainer/README.md)
- [Your wrapper configuration](/home/rob/mrs_apptainer/wrapper.sh)
- [Container Bash initialization](/home/rob/mrs_apptainer/mount/apptainer_bashrc.sh)
- [Mount/home initialization](/home/rob/mrs_apptainer/mount/99-mrs_env.sh)
- [VS Code installer](/home/rob/mrs_apptainer/scripts/install_vscode.sh)
- [ROS 2 workspace guide](/home/rob/mrs_docs/docs/10-prerequisites/25-ros2/20-workspace-build.md)
- [Gazebo simulation guide](/home/rob/mrs_docs/docs/60-simulations/01-gazebo/01-howto.md)
- [Drone spawner guide](/home/rob/mrs_docs/docs/60-simulations/01-gazebo/04-drone_spawner.md)
- [MRS multirotor guide](/home/rob/mrs_docs/docs/60-simulations/10-mrs/20-with_mrs_system.md)
- [UAV ROS API](/home/rob/mrs_docs/docs/40-api/01-uav_ros_api.md)
- [Frames of reference](/home/rob/mrs_docs/docs/40-api/20-frames_of_reference.md)
- [Custom configuration](/home/rob/mrs_docs/docs/40-api/10-custom_configs.md)
- [Relative command pitfalls](/home/rob/mrs_docs/docs/65-howto-usecases/relative_commands/index.md)
- [Zenoh networking](/home/rob/mrs_docs/docs/50-features/100-zenoh-network/index.md)
- [Local waypoint example](/home/rob/mrs_apptainer/user_ros_workspace/src/mrs_core_examples/cpp/example_waypoint_flier/README.md)
- [Local waypoint scenario](/home/rob/mrs_apptainer/user_ros_workspace/src/mrs_core_examples/cpp/example_waypoint_flier/tmux/session.yaml)

Documentation checkout inspected: `9385faf150ad36a59e64279720d19dcabd58696a`.

Many local pages explicitly warn that ROS 2 content may be outdated; some still contain ROS 1 commands, old package names, or TODO sections. This guide prefers the installed launch scripts, interface definitions, and your actual wrapper when they disagree with those examples.

Verified directly: image OS, ROS distribution, Gazebo version, package availability, installed scenario scripts, service interface definition, shell setup, wrapper configuration, and host workspace layout. Instructions for building, starting GUI simulations, issuing flight requests, and backing up files are documented workflows; they were not executed as part of preparing this guide. No existing source files or simulation configuration were changed while writing it.
