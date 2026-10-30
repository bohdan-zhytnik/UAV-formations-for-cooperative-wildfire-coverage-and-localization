# What to keep on GitHub

Use **fire_observation_demo as the root of the project repository**. It contains
our application source, configuration, resources, launchers, tests and guides.
It runs on top of an installed MRS/ROS/Gazebo environment; that environment is
not part of the project's source tree.

## Keep these files

| Content | Why it belongs in Git |
|---|---|
| `src/`, `scripts/`, `launch/` | RViz tools, perception, mapping, mission planning, launchers and verification scripts |
| `config/` | Six-fire scene, shared survey defaults, mapping settings and network/TF configuration |
| `models/`, `worlds/` | Custom drone/camera templates and source world descriptions |
| `ROMFS/` | Required PX4 startup scripts and our custom airframe definitions; these are text resources, not a generated firmware image |
| `tmux/`, `tmux_fleet/` | Session definitions, helper commands, local MRS settings and saved `.rviz` layouts |
| `docs/`, `README.md` | Operator guides and the written validation record |
| `CMakeLists.txt`, `package.xml`, `fleet_tools.xml` | Build/dependency information and RViz plugin registration |
| `.gitignore` | Excludes generated/local files when the package is its own repository |

Keep existing attribution/copyright comments in the copied MRS/PX4 resources.
The source package is small; copying only Python/C++ files would omit resources
needed to launch the simulation.

## Excluded by the project .gitignore

* `artifacts/`: raw camera snapshots, flight reports, logs, temporary test
  utilities and old source snapshots. The written results in `docs/VALIDATION.md`
  remain versioned. If a figure or example report is useful to readers, select
  it and copy it to a deliberate location under `docs/`.
* `tmux/runtime/` and `tmux_fleet/runtime/`: generated worlds/spawner settings,
  including paths specific to the local installation. The prepare scripts
  regenerate these from versioned source configuration.
* `build/`, `install/`, `log/` and similar generated workspace directories.
* Python bytecode, virtual environments, caches, compiled libraries, editor
  temporary/backup files and crash dumps.
* ROS bag/MCAP recordings, PX4 logs and Apptainer image/overlay binaries.

Image assets and `.rviz`, YAML, SDF and Jinja source files are deliberately not
globally ignored. A future model texture or documentation screenshot can still
be committed. Nothing has been deleted from the working directory.

## Useful files outside the package

These were created during the environment setup and are worth keeping as an
optional, separate environment backup:

* `mrs_apptainer/MRS_APPTAINER_GUIDE.md`: the general host/container/workspace
  guide. It contains notebook-specific paths and describes the setup inspected
  at the time; it is not a runtime dependency of this package.
* `mrs_apptainer/scripts/install_vscode.sh`: the optional editor installer. It
  expects to live in the MRS Apptainer repository, beside its images/overlays.
  Keep that location or adapt its path handling if copying it elsewhere.

For example, retain these in an environment repository or documentation archive.
The ROS demo does not require VS Code. A base-image build of the package was
tested without the user's writable overlay.

`mrs_core_examples` is a separate source checkout, not a dependency of this
package. Unchanged upstream MRS/Apptainer files can be obtained from upstream;
record the upstream revision and image release/tag when preparing a reproducible
environment. Keep a separate local backup of a personalized wrapper/overlay if
needed, rather than committing machine-specific runtime state as project code.

## Existing repository on this notebook

The current Git repository is rooted at `/home/rob/mrs_apptainer`, not inside
the package. Its original workspace ignore rules hid the entire source tree.
They now make a narrow exception for `src/fire_observation_demo` and exclude
workspace `build/`, `install/` and `log/` directories.

This lets you include the package in a fork of the environment repository if
that is the layout you choose. A separate project repository can instead have
the package contents directly at its root. The package `.gitignore` works for
both layouts. Avoid accidentally creating/staging an embedded repository as a
submodule when you intend to publish ordinary source files.

No project repository was initialized, and nothing was staged, committed or
uploaded by this preparation.

## Rebuild after cloning

Inside the prepared MRS Apptainer, place the package at:

```text
~/user_ros_workspace/src/fire_observation_demo/
```

Then rebuild the generated workspace output:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
cd ~/user_ros_workspace
colcon build --packages-select fire_observation_demo --symlink-install
source install/setup.bash
cd src/fire_observation_demo/tmux_fleet
./start.sh
```

The workspace placement matters: `scripts/env.sh` finds `install/setup.bash`
relative to the package's location. Runtime files and recordings are not needed
to clone/build; model and PX4 startup resources are needed. The dependency list
is in `package.xml`, and [SHARED_SURVEY.md](SHARED_SURVEY.md) describes operation.
