# Validation record and SITL acceptance procedure

## Dependency installer addition

Added `setup.sh`, a host package manifest, `scripts/docker.sh`, and
`DEPENDENCIES.md` on 2026-10-06. Shell syntax and installer argument handling
were checked. Unsupported-host rejection was exercised without installing
packages. The existing flight-control source files are unchanged by this
addition. No host apt installation or Docker image build was executed here.

## Checks executed during preparation

Date: 2026-10-06. Environment: Linux x86-64, Python 3.12. Tests use Python
standard library unittest and pymavlink 2.4.49 installed into an isolated
temporary dependency directory. The delivered Docker runtime uses Python 3.11.

* Sixteen controller/protocol tests passed; actual console output is in
  `tests/last_run.txt`.
* Python source compilation passed.
* `bash -n scripts/launch_sim.sh` passed.
* Compose YAML parsed successfully with all five services and anchor inheritance.
  This is a YAML check, not `docker compose config` or a container launch.
* MAVLink tests encoded and decoded actual UDP packets for system/component
  addressing, global frame, latitude scaling, AMSL altitude, type mask,
  scheduled landing, stale-plan fallback and latched no-rearm behavior.
* Ideal-follower controller test completed the full mission state sequence.
  Its movement is a test double, not physics or a PX4 simulation.
* Pinned PX4 v1.16.0 startup/CMake sources were inspected for MAV_SYS_ID mapping,
  offboard ports, standalone model names, spawn poses, generated Gazebo
  environment and the explicit `px4_gz_plugins` build dependency.
* World generation was executed using `default.sdf` from the model submodule
  revision recorded by PX4 v1.16.0 (`e05f4312d3f28aa621157610584a4870406cb6d3`).
  XML checks confirmed the ENU GPS origin and all six named home/landing markers.

**Not executed:** Docker build, Gazebo launch, PX4 integration flight, GUI,
ARM64 image build, physical Pi/Pixhawk deployment, measured synchronization or
formation flight performance. Docker is not available in the authoring
environment. The package does not contain purported successful SITL flight logs.

## Before the first flight

```bash
docker compose config
docker compose build
docker run --rm three-drone-companion:1.0 python -m unittest discover -s tests -v
docker compose up -d
docker compose ps
docker compose exec coordinator python scripts/swarmctl.py status
```

Confirm all three peers are present and fresh, IDs and homes are distinct,
Gazebo has exactly one world and three x500 models, and no vehicle is armed.
Check the generated home/landing coordinates. Keep Gazebo unpaused at 1x
simulation speed and ensure the host has sufficient CPU headroom.

## Baseline mission test

1. Start one mission with `swarmctl.py start`.
2. Observe the all-vehicle OFFBOARD and ARM barriers, then takeoff.
3. Observe every autonomous waypoint and the final marked landing triangle.
4. Confirm `DONE` with fresh disarmed and landed telemetry for all vehicles.
5. Run `analyze.py` on matching logs and preserve the report and PX4 ULogs.

Suggested initial acceptance targets, which must be measured rather than
assumed: no ABORT; all landing vertices reached; sampled edge error within
1.5 m during settled cruise; pair distance above 5 m throughout; land-command
skew under 0.2 s, liftoff skew under 0.5 s and touchdown skew under 1 s. These
are provisional engineering targets, not verified results. Use independent
Gazebo pose data to assess ground-truth positions, especially during transients.

## Manual trajectory test

During AUTO, issue `hold`, then a valid `waypoint`, then `resume`. Verify a
continuous reference, maintained vertex assignment, no instantaneous position
jump, hold on manual arrival and resumption of the interrupted autonomous
waypoint. Try a waypoint outside the radius or altitude envelope: it must be
rejected without changing the current trajectory. Use normal `land` to verify
navigation to designated pads before descent.

## Failure tests (separate fresh runs)

Perform these only against this simulated stack. Save separate logs for each.

| Injection | Command/example | Expected behavior |
|---|---|---|
| Missing companion before start | `docker compose stop drone3` | Start is rejected; no launch |
| Companion process failure during AUTO | `docker compose kill -s SIGKILL drone2` | PX4 2 loses offboard input; coordinator aborts other agents |
| Coordinator failure during AUTO | `docker compose kill -s SIGKILL coordinator` | Each agent latches landing after the coordinator lease expires |
| Operator abort | `swarmctl.py abort` | Individual current-position land; final state FAILED rather than successful mission DONE |
| Unsupported/out-of-range command | Invalid GPS or wrong mission phase | HTTP error; current valid plan unchanged |

Verify every FC's actual mode change and landing from ULogs; command logs alone
are not proof of recovery. A disconnected/failed vehicle must never be counted
as landed. Do not restart processes to continue the same flight after a fault.
End the test, inspect logs, then recreate the whole stack.

## Remaining engineering limits

The application uses sampled estimated position and does not prove collision
avoidance. Real GNSS bias, disturbances, PX4 tuning, simulation time drift,
network jitter and CPU scheduling remain integration risks. A telemetry-safe
application state machine is useful but is not a substitute for measured
end-to-end SITL behavior or hardware validation.
