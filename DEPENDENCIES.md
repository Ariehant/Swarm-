# Fresh-PC dependency installation

## One command

Extract the entire project ZIP with the desktop archive manager, open a terminal
in its `swarm_sitl` folder, then run as your normal Ubuntu user:

```bash
bash setup.sh --start
```

Enter your sudo password when Ubuntu requests it. The script installs host
dependencies, downloads/builds the simulator and companion images, runs the
application tests and starts the containers. The drones stay disarmed in WAIT.
An internet connection and administrator/sudo access are required.

Supported installer targets: **Ubuntu 22.04, 24.04 and 26.04, x86-64/amd64,
native Linux with systemd**. The containers continue to use Ubuntu 22.04 for
PX4/Gazebo regardless of the host version. This installer is not for Windows,
macOS, ARM/Pi hardware deployment, or non-systemd environments. It rejects
unsupported systems before installing packages.

## What downloads automatically

| Location | Dependencies | How they are obtained |
|---|---|---|
| Host | CA certificates, curl, GnuPG, Git, unzip, Python 3 | Ubuntu apt, `config/host-packages.txt` |
| Host | Docker Engine/CLI, containerd, Buildx, Compose plugin | Official Docker apt repository with its signing key |
| Simulator image | Ubuntu 22.04, PX4 toolchain, Gazebo Harmonic and libraries | Docker base image and pinned PX4 `Tools/setup/ubuntu.sh --no-nuttx` |
| Simulator image | PX4 v1.16.0, recursive submodules, x500 models, worlds and custom Gazebo plugins | GitHub checkout and compilation in `Dockerfile` |
| Companion image | Python 3.11, pymavlink 2.4.49 and its Python dependencies | Python base image and `requirements.txt` |
| Verification | Docker hello-world image | Docker registry |

You do not need a host installation of PX4, Gazebo, ROS, MAVSDK or Python pip
packages for this project. The dependency files have separate roles:
`setup.sh` installs the host; `config/host-packages.txt` lists its utilities;
`Dockerfile` installs/builds the simulator; `requirements.txt` installs the
companion Python libraries. Rebuilding follows the same existing version pins;
it does not turn the dependency set into a complete digest lockfile.

## Options

```bash
bash setup.sh                     # Install, build and test; do not launch containers.
bash setup.sh --start             # Also launch the disarmed simulation.
bash setup.sh --jobs 2 --start    # Reduce PX4 build parallelism.
bash setup.sh --install-only     # Install and verify host tools/Docker only.
bash setup.sh --check            # Read-only report; no sudo/install/build.
bash setup.sh --help
```

The script reports RAM, CPU count and free space and chooses up to four build
jobs, or at most two below 12 GB RAM. Prefer approximately 16 GB RAM and at
least 30 GB free disk for the initial build. These are guidelines, not validated
minimums. It warns on low resources rather than changing swap or disk layouts.
The free-space precheck uses the default Docker filesystem; if your Docker data
root is customized, check its capacity separately.

Installation/build output is retained in `logs/setup-*.log`. Resolved image
metadata is saved as `logs/setup-images.json`. Downloads can be large and builds
can take substantial time. A failed download or build returns a nonzero exit
code and does not print a success result. Fix the reported issue and rerun;
apt packages and Docker layers are reused where available. `--pull` checks for
updated base images on build, so a rerun may download newer base-image layers.

## Running after setup

Use the provided local Docker wrapper so you do not need to log out and back in
or add your account to the Docker group:

```bash
bash scripts/docker.sh compose exec coordinator python scripts/swarmctl.py status
# Once all three drones report ready:
bash scripts/docker.sh compose exec coordinator python scripts/swarmctl.py start

# Watch process logs:
bash scripts/docker.sh compose logs -f
# After the mission has landed, stop the stack:
bash scripts/docker.sh compose down
```

The wrapper uses the local Docker socket explicitly, with sudo only when
needed. It does not use a remote Docker context, change your group membership,
or change Docker socket permissions. Normal `docker compose ...` commands in
README.md remain usable if your account already has Docker access; otherwise
prefix them with `bash scripts/docker.sh` in place of `docker`.

## Existing-PC behavior

If Docker CLI, Compose and Buildx are already present, the installer reuses
them rather than migrating the installation. It enables/starts the local
`docker` systemd service and checks the actual daemon. Rootless Docker and
Docker Desktop-only setups are outside this installer's native Engine path.

If the official installation path is needed but conflicting distribution
packages are present, the script reports their exact names and stops. It does
not automatically remove runtimes, containers, images, volumes or data. Review
Docker's official migration instructions for that PC before changing an
existing installation. Existing official Docker repository entries are reused;
broken keys or entries must be corrected if apt reports an error.

The script uses apt package signature checks and Docker's HTTPS-hosted signing
key, with a dedicated signed repository entry. It does not execute a downloaded
shell script directly on the host.

## GUI and validation scope

Default startup is headless. The optional GUI still needs your desktop session's
DISPLAY/XAUTHORITY values, as explained in README.md; the installer does not
install a desktop environment or select an X11 cookie automatically.

Installer shell syntax and argument handling have been checked. A full fresh
Ubuntu installation, Docker build and Gazebo flight have not been executed in
the authoring environment. Successful setup runs the existing controller and
MAVLink tests; those tests are not a substitute for the SITL acceptance steps
in VALIDATION.md.

Official installation references, checked 2026-10-06:

* https://docs.docker.com/engine/install/ubuntu/
* https://docs.docker.com/compose/install/linux/
