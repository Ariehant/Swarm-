# Three-drone PX4 / Gazebo SITL swarm

A complete source project for three x500 quadcopters, three Python companion
containers, a telemetry-sharing mission coordinator, and a manual command API.
The default mission takes off into a 12 m equilateral triangle, flies three GPS
centroid waypoints, translates to a marked landing triangle, descends together,
and releases PX4 AUTO.LAND together.

**Fresh Ubuntu PC:** extract the full ZIP, open a terminal in `swarm_sitl`,
and run `bash setup.sh --start`. It installs the host dependencies, Docker and
Compose, builds/downloads the container dependencies, runs application tests,
and launches the simulation disarmed. See `DEPENDENCIES.md` for supported
systems, options and troubleshooting. Start the mission explicitly after
checking readiness:

```bash
bash scripts/docker.sh compose exec coordinator python scripts/swarmctl.py status
bash scripts/docker.sh compose exec coordinator python scripts/swarmctl.py start
```

**Validation status:** controller and MAVLink loopback tests have been run.
Docker image builds and a full PX4/Gazebo flight have **not** been run in the
authoring environment, which has no Docker executable. See `VALIDATION.md`.
This is a runnable implementation supplied for SITL validation, not a claim of
flight-tested performance. No measured flight logs are fabricated or included.

## 1. Technology stack and hardware interpretation

| Layer | Choice | Role |
|---|---|---|
| Simulator | Gazebo Harmonic, Ubuntu 22.04 | Physics, motors, IMU, magnetometer, GPS and world |
| Autopilot | PX4 tag `v1.16.0`, three SITL processes | Estimation, position/attitude control, arming and failsafes |
| Vehicle | PX4's pre-built `x500`, airframe 4001 | Standard quadcopter model and tuning |
| Companion runtime | Python 3.11 image, `pymavlink==2.4.49` | One independent offboard agent per drone |
| Coordination | Python standard library; UDP JSON, 20 Hz | Peer-state distribution, centroid planner, barriers |
| Operator interface | HTTP JSON plus `swarmctl.py` | Start, GPS override, hold, resume, target landing, abort |
| Containerization | Multi-stage Dockerfile and Docker Compose v2 | All simulation and control processes |
| Intended real FC | Pixhawk 6 family with PX4 | Hardware target for a later integration phase |
| Intended real SBC | Raspberry Pi 5, 64-bit Linux | Hardware target for each companion process |

SITL runs the PX4 flight software on the host CPU. It does not emulate a
Pixhawk 6C/6X MCU, Pi 5 peripherals, radio interference, thermal throttling or
hardware timing. The `companion` image is based on a multi-architecture Python
image; the default simulator is explicitly `linux/amd64`. ARM64 builds of the
companion are possible, but have not been validated here.

Pymavlink is the middleware in this implementation. ROS 2, MAVROS, MAVSDK,
micro-XRCE-DDS and a DDS agent are not required. This removes a redundant
translation layer for a mission that needs GPS telemetry and MAVLink offboard
messages. PX4 itself owns the simulator bridge.

## 2. Project files, in reading order

| File | Purpose |
|---|---|
| `setup.sh` | Automatic fresh-Ubuntu host setup, image builds and application tests |
| `DEPENDENCIES.md` | One-command setup and dependency inventory |
| `config/host-packages.txt` | Host apt utility package list |
| `scripts/docker.sh` | Local Docker wrapper with sudo when required |
| `Dockerfile` | PX4/Gazebo build stage and companion runtime stage |
| `docker-compose.yml` | Five services and shared network namespace |
| `requirements.txt` | Direct Python dependency version |
| `config/sitl.params` | Explicit SITL-only PX4 parameter overrides |
| `config/mission.json` | GPS origin, route, geometry and limits |
| `scripts/make_world.py` | Generate the world and six home/landing markers |
| `scripts/launch_sim.sh` | Start Gazebo, then three separately rooted PX4 instances |
| `swarm/common.py` | Geographic conversions, formation math and telemetry checks |
| `swarm/coordinator.py` | State machine, peer distribution and operator API |
| `swarm/agent.py` | Per-drone MAVLink telemetry, global targets and mode commands |
| `scripts/swarmctl.py` | Operator command-line client |
| `scripts/analyze.py` | Flight log metrics and synchronization measurements |
| `docker-compose.gui.yml` | Optional containerized Linux desktop viewer |
| `tests/test_controller.py` | Mission logic, interlocks, geometry and overrides |
| `tests/test_protocol.py` | Actual MAVLink packet encode/decode over UDP loopback |
| `ARCHITECTURE.md` | Control equations, state transitions, routing and limitations |
| `VALIDATION.md` | Executed checks and required SITL acceptance procedures |

## 3. Host requirements

Use a Linux x86-64 workstation with Docker Engine and the Compose plugin.
The installer accepts Ubuntu 22.04/24.04/26.04 hosts; all PX4 dependencies live inside
the Ubuntu 22.04 image. Plan for roughly 16 GB RAM, several CPU cores and
30 GB or more free disk as a starting resource budget, not a measured minimum.
Initial builds need access to Docker registries, GitHub, Ubuntu/OSRF package
repositories and PyPI. Runtime model assets come from the checked-out PX4
submodule rather than downloading from Gazebo Fuel during flight.

The default is headless software rendering. Avoid building/running all three
simulated vehicles on a Pi 5 as the first validation setup; use the workstation
for physics and treat companion containers as logical Pi application instances.

Check prerequisites:

```bash
docker version
docker compose version
```

## 4. Build and launch

On a fresh PC, use `bash setup.sh --start` from the extracted project folder.
It performs the installation, build and test steps automatically. The manual
Docker commands below are for an already-configured host. If Docker needs sudo,
use `bash scripts/docker.sh compose ...` instead of `docker compose ...`.

Extract the project ZIP and enter the directory containing `Dockerfile`:

```bash
unzip Three_Drone_PX4_Gazebo_SITL.zip
cd swarm_sitl
mkdir -p logs
docker compose build
docker compose up -d
docker compose ps
docker compose logs -f sim coordinator drone1 drone2 drone3
```

The first build compiles PX4 and installs Gazebo; it can take substantial time.
For a machine with limited build RAM:

```bash
BUILD_JOBS=2 docker compose build sim
```

`sim` becomes container-healthy when Gazebo and the three PX4 processes have
launched. This is **not** the flight-readiness check. The coordinator remains
in `WAIT`; it will not arm automatically. Wait for telemetry, GPS and home
initialization, then inspect status:

```bash
docker compose exec coordinator python scripts/swarmctl.py status
```

There must be three peers, IDs 1, 2 and 3. Each needs fresh `pos_age`, `gps_age`,
`hb_age`, `land_age`, a 3D GPS fix, `home: true`, no fault, `ground: true` and
`armed: false`. Reported positions must be within 3 m of their marked homes.
PX4's normal arming checks still apply after these application checks.

Launch the autonomous mission:

```bash
docker compose exec coordinator python scripts/swarmctl.py start
```

Expected progress is `PRESTREAM`, `OFFBOARD`, `ARM`, `TAKEOFF`, `AUTO`,
`APPROACH`, `DESCEND`, `LAND`, `DONE`. Read status at any point. `DONE` requires
all three vehicles to report landed and disarmed with fresh telemetry.

No container uses automatic restart. For another run after all vehicles have
landed, stop and recreate the whole stack; do not restart one flight process
inside a live mission:

```bash
docker compose down
docker compose up -d
```

Logs persist under `./logs`; each simulator start has a separate runtime folder.
The runtime folder also keeps each vehicle's PX4 parameters, console output and
ULog tree. A new run does not reuse old parameter files.

## 5. Manual waypoint injection and overrides

Commands apply to the triangle's **centroid**. Each drone retains its assigned
vertex. The planner always approaches an injected waypoint with bounded speed;
it never teleports its reference to the requested coordinate.

```bash
# Pause centroid progression; maintain formation around the current reference.
docker compose exec coordinator python scripts/swarmctl.py hold

# Fly to a new GPS centroid at 10 m above the configured flat ground.
docker compose exec coordinator python scripts/swarmctl.py waypoint \
  --lat 47.397832 --lon 8.545726 --agl 10

# Resume the interrupted autonomous waypoint, keeping its original index.
docker compose exec coordinator python scripts/swarmctl.py resume

# Go to the designated landing triangle, settle, descend, then land together.
docker compose exec coordinator python scripts/swarmctl.py land

# Abort the route and request landing at current positions.
docker compose exec coordinator python scripts/swarmctl.py abort
```

Waypoint/hold/resume are accepted in `AUTO`, `MANUAL` and `HOLD`. A manual
waypoint ends in `HOLD`; use `resume` or `land` explicitly. Normal target landing
is also accepted during `TAKEOFF` or `APPROACH`. `abort` is available in any
nonterminal phase. It cannot be undone by `resume` or `start`.

An operator hold has a configurable 120 s timeout, after which the mission
aborts and lands. A stalled movement phase has a 180 s timeout. These deadlines
prevent indefinite mission execution after loss of progress.

Equivalent HTTP calls from the host:

```bash
curl http://127.0.0.1:8080/status
curl -X POST http://127.0.0.1:8080/command \
  -H 'Content-Type: application/json' \
  -d '{"action":"waypoint","lat":47.397832,"lon":8.545726,"agl":10}'
```

Accepted commands return HTTP 200; invalid coordinates, missing readiness or
wrong mission phases return an error. HTTP acceptance acknowledges the planner
change, not completion of flight. The API binds to host loopback and has no
authentication. Keep it local to this simulation host.

`land` and `abort` differ intentionally: `land` navigates to the configured
destination; `abort` stops mission progression and lets each PX4 land where it
is. An abort does not guarantee formation or simultaneous touchdown.

## 6. GPS route and world editing

Edit `config/mission.json`, then recreate the stack. The configuration is mounted
read-only inside containers. Do not hot-edit it midflight: each process loads
its configuration once at startup.

Defaults:

* Origin: latitude 47.397742, longitude 8.545594, ground 488 m AMSL.
* Triangle: side 12 m, first vertex north, two trailing vertices southwest/southeast.
* Takeoff altitude: 8 m above the configured flat ground.
* Cruise reference: 1.2 m/s horizontally and 0.6 m/s vertically.
* GPS waypoints: centroid north, northeast, then east of the start.
* Landing centroid: east of home, with three distinct landing pads.
* Descent staging height: 2 m AGL, followed by native PX4 AUTO.LAND.

`make_world.py` retains PX4's default world and creates named blue home discs and
green landing discs. The pads are visual-only so they do not alter the contact
surface. Exact GPS coordinates for all six pads are saved in
`logs/sim-*/markers.json`. The three homes and landing targets are derived from
the same triangle offsets used by the controller, avoiding axis mismatches.

The setpoint altitude is **AMSL**, calculated as `origin.amsl + agl`. The API's
`agl` means height above the configured flat plane, not terrain-following height
and not height above each vehicle's potentially different home. This project
assumes a flat, obstacle-free world. Keep the route within the configured
150 m origin radius; validation includes the formation's outer vertices.

## 7. Optional Gazebo GUI

The simulation itself remains containerized and headless. On a Linux desktop
using X11 or XWayland, provide the current session's valid `DISPLAY` and
`XAUTHORITY` cookie path, then launch the optional GUI container:

```bash
export DISPLAY=:0
export XAUTHORITY=/absolute/path/to/your/session/Xauthority
docker compose -f docker-compose.yml -f docker-compose.gui.yml up -d gui
```

Use the actual values from your desktop session; `:0` is only an example.
Select `x500_0`, `x500_1`, `x500_2` in the scene tree. Model entities
`home_1..3` and `landing_1..3` identify the pads. The shared `GZ_PARTITION`
connects this viewer to the existing world. A functioning desktop/X cookie is
required; this file is not a Windows/macOS remote-desktop configuration.

Stop the optional viewer with:

```bash
docker compose -f docker-compose.yml -f docker-compose.gui.yml stop gui
```

## 8. Logs and reproducibility

| Output | What it establishes |
|---|---|
| `coordinator-*.jsonl` | Mission transitions, accepted commands, peer snapshots and formation errors |
| `drone1-*.jsonl`, etc. | Estimated positions, setpoints, mode commands, ACKs, PX4 status and ground transitions |
| `sim-*/gazebo.log` | Gazebo startup and transport errors |
| `sim-*/px4_*/console.log` | PX4 startup and preflight messages |
| `sim-*/px4_*/log/` or nested runtime ULog directory | PX4 `.ulg` flight data, depending on PX4's runtime layout |
| `sim-*/markers.json` | Exact generated home/landing coordinates |
| `sim-*/PX4_COMMIT`, `PX4_SUBMODULES`, `OS_PACKAGES` | Actual fetched source revisions and installed OS packages |
| `/app/PYTHON_PACKAGES` in companion image | Resolved Python dependency versions |

Locate ULogs without assuming a date subdirectory:

```bash
find logs -name '*.ulg'
docker compose exec coordinator cat /app/PYTHON_PACKAGES
```

Run analysis with the four matching JSONL files from **one run**, replacing
the illustrative names below with actual names:

```bash
docker compose exec coordinator python scripts/analyze.py \
  /logs/coordinator-RUN.jsonl \
  /logs/drone1-RUN.jsonl /logs/drone2-RUN.jsonl /logs/drone3-RUN.jsonl
```

The report includes maximum and RMS sampled edge error, minimum separation,
first liftoff skew, land-command skew, touchdown skew, final phase and abort
reasons. Null timing values mean there was insufficient evidence. JSONL
snapshots are 5 Hz, while control is 20 Hz; brief between-sample excursions may
be missed. Use PX4 ULogs and independent Gazebo pose recordings for more rigorous
evaluation. The report measures estimator-based formation, not ground truth.

The PX4 release and primary Python package are version-pinned. Ubuntu/OSRF apt
packages, base-image tags and transitive Python dependencies are not fully
digest-locked; the build records their resolved versions. Preserve the actual
Docker images for repeatable experiments:

```bash
docker image inspect three-drone-sim:1.0 three-drone-companion:1.0 > logs/images.json
docker save three-drone-sim:1.0 three-drone-companion:1.0 -o swarm-images.tar
```

## 9. Tests

After building, the application tests can run without starting the simulator:

```bash
docker run --rm three-drone-companion:1.0 python -m unittest discover -s tests -v
```

The kinematic follower in the mission test is an idealized test double. It
checks mission sequencing and must not be interpreted as Gazebo flight evidence.
Follow the additional procedures in `VALIDATION.md` for actual SITL acceptance.

## 10. Troubleshooting

| Symptom | Checks |
|---|---|
| Build stops with compiler killed | Reduce `BUILD_JOBS`; check RAM and disk |
| Gazebo never becomes ready | Read `gazebo.log`; inspect OSRF install, software rendering and model/resource paths |
| Model spawn fails | Confirm generated `gz_env.sh` exports `PX4_GZ_MODELS`; the launcher sources it before standalone PX4 startup |
| Missing or mismatched telemetry | Verify IDs 1/2/3 and ports 14540/14541/14542; no second process may bind those ports |
| Healthy container, `start` rejected | Container health is not FC health; inspect `/status`, home coordinates and age fields |
| Stuck OFFBOARD or ARM | Inspect command ACKs and `px4_status`; keep normal preflight checks enabled |
| GPS home mismatch | Check SDF origin, ENU pose mapping, shared mission config and AMSL altitude |
| Offboard loss while host is busy | Inspect loop delays; maintain approximately real-time simulation and enough CPU headroom |
| `paused_by_error: true` | A vehicle is lagging or formation error is above the soft threshold; inspect positions and modes |
| No `DONE` after LAND | Read fresh ground/disarmed states; stale telemetry is never treated as successful landing |
| Restarted agent never rejoins | Intentional sequence/fault protection: land, then recreate the whole stack |

## 11. Scope of real hardware work

This project models the application split expected for a Pixhawk 6 plus Pi 5
vehicle, but is not a ready-to-fly hardware deployment. Determine whether the
actual controller is a 6C, 6X or another variant before selecting firmware.
Real deployment requires a serial/Ethernet MAVLink transport adapter, hardware
parameter configuration, RC/manual recovery policy, validated estimator quality,
independent separation sensing, and a synchronized multi-host clock/network
design. The current agents intentionally accept only the local SITL endpoints.

The agents use one host's monotonic clock for scheduling. Containers share that
clock; three physical Pi 5 boards do not. Simply moving these containers onto
three boards would invalidate the synchronization and lease assumptions.
See `ARCHITECTURE.md` for the necessary interface changes.

Ordinary GNSS cannot guarantee a rigid formation. Formation precision on real
vehicles depends on relative localization quality (for example, validated RTK
or other relative sensing), latency, wind, dynamics and controller tuning.

## 12. Upstream references

* PX4 multi-vehicle Gazebo usage: https://docs.px4.io/v1.16/en/sim_gazebo_gz/multi_vehicle_simulation
* Gazebo installation and standalone mode: https://docs.px4.io/v1.16/en/sim_gazebo_gz/index
* Offboard mode and supported setpoints: https://docs.px4.io/v1.16/en/flight_modes/offboard
* PX4 parameter reference: https://docs.px4.io/v1.16/en/advanced_config/parameter_reference
* MAVLink global setpoints: https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT
* Pinned PX4 source: https://github.com/PX4/PX4-Autopilot/tree/v1.16.0
* MAVLink routing source: https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink
* Standalone spawn source: https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim

PX4/Gazebo/pymavlink and their bundled assets retain their upstream licenses.
