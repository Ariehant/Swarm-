#!/usr/bin/env bash
# Fresh Ubuntu workstation bootstrap. Run as your normal user: bash setup.sh
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: bash setup.sh [--start] [--install-only] [--check] [--jobs N]

Default: install host dependencies, verify Docker, build both images, run tests.
  --start          Also launch the simulation, disarmed in WAIT.
  --install-only   Install/verify host dependencies without building images.
  --check          Read-only host/dependency report; install/build nothing.
  --jobs N         PX4 build parallelism (1-64); default chosen from CPU/RAM.
  -h, --help       Show this help.

Supported: native Ubuntu 22.04 / 24.04 / 26.04, x86-64, systemd, internet.
Run from the extracted complete project. sudo is requested when necessary.
EOF
}

die() { echo "ERROR: $*" >&2; exit 1; }
warn() { echo "NOTICE: $*" >&2; }

start=0
install_only=0
check_only=0
jobs=""
while (($#)); do
  case "$1" in
    --start) start=1; shift ;;
    --install-only) install_only=1; shift ;;
    --check) check_only=1; shift ;;
    --jobs)
      [[ $# -ge 2 && "$2" =~ ^[1-9][0-9]?$ ]] || die '--jobs requires an integer from 1 to 64'
      ((10#$2 <= 64)) || die '--jobs must be at most 64'
      jobs="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; die "Unknown option: $1" ;;
  esac
done
((start == 0 || install_only == 0)) || die '--start and --install-only cannot be combined'
((check_only == 0 || (start == 0 && install_only == 0))) || die '--check cannot be combined with installation/start options'

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$project_dir"
[[ -f /etc/os-release ]] || die 'This installer requires Ubuntu Linux'
# shellcheck disable=SC1091
source /etc/os-release
[[ "${ID:-}" == ubuntu ]] || die "Supported OS: Ubuntu, detected ${ID:-unknown}"
case "${VERSION_ID:-}" in
  22.04|24.04|26.04) ;;
  *) die "Unsupported Ubuntu release: ${VERSION_ID:-unknown}" ;;
esac
[[ "$(uname -m)" == x86_64 && "$(dpkg --print-architecture)" == amd64 ]] ||
  die 'The full simulator requires an x86-64 PC; this installer does not install the swarm on ARM/Pi 5'
[[ -d /run/systemd/system ]] || die 'A native Ubuntu host with systemd is required'
for required in Dockerfile docker-compose.yml config/host-packages.txt scripts/docker.sh; do
  [[ -f "$required" ]] || die "Missing $required; extract the complete project ZIP first"
done

ram_kib="$(awk '/MemTotal:/ {print $2}' /proc/meminfo)"
cpu_count="$(nproc)"
if [[ -z "$jobs" ]]; then
  jobs=4
  ((ram_kib >= 12*1024*1024)) || jobs=2
  ((cpu_count >= jobs)) || jobs="$cpu_count"
fi
echo "Host: Ubuntu $VERSION_ID amd64; CPUs=$cpu_count; RAM=$((ram_kib/1024)) MiB; build jobs=$jobs"
((ram_kib >= 16*1024*1024)) || warn '16 GB RAM is recommended; lower-memory builds may fail or run slowly'
docker_space_dir=/var/lib/docker
[[ -d "$docker_space_dir" ]] || docker_space_dir=/var/lib
free_kib="$(df -Pk "$docker_space_dir" | awk 'END {print $4}')"
echo "Available space on $docker_space_dir filesystem: $((free_kib/1024/1024)) GiB"
((free_kib >= 30*1024*1024)) || warn 'Plan for at least 30 GB free disk for images/build cache; this is a guideline'

if ((check_only)); then
  missing=0
  for cmd in curl gpg git unzip python3 docker; do
    if command -v "$cmd" >/dev/null 2>&1; then
      echo "FOUND: $cmd"
    else
      echo "MISSING: $cmd"
      missing=1
    fi
  done
  if command -v docker >/dev/null 2>&1; then
    docker compose version || missing=1
    docker buildx version || missing=1
    if ! docker --host unix:///var/run/docker.sock info >/dev/null 2>&1; then
      warn 'Local Docker daemon is stopped, missing or needs sudo; --check does not elevate privileges'
      missing=1
    fi
  fi
  exit "$missing"
fi

mkdir -p logs
setup_log="$project_dir/logs/setup-$(date -u +%Y%m%dT%H%M%SZ)-$$.log"
exec > >(tee -a "$setup_log") 2>&1
echo "Installation/build log: $setup_log"
trap 'echo "Setup failed at line $LINENO. Fix the reported error and rerun; see $setup_log" >&2' ERR
root=()
if ((EUID != 0)); then
  command -v sudo >/dev/null 2>&1 || die 'sudo is required; ask the PC administrator to install it'
  sudo -v
  root=(sudo)
fi

# Use the explicit package manifest; no curl-piped shell installer.
mapfile -t host_packages < <(sed '/^[[:space:]]*#/d; /^[[:space:]]*$/d' config/host-packages.txt)
"${root[@]}" apt-get update
"${root[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${host_packages[@]}"

if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1 || ! docker buildx version >/dev/null 2>&1; then
  # Never automatically remove another container runtime or its data.
  conflicts=()
  for pkg in docker.io docker-compose docker-compose-v2 docker-doc docker-buildx podman-docker containerd runc; do
    if [[ "$(dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null || true)" == 'install ok installed' ]]; then
      conflicts+=("$pkg")
    fi
  done
  ((${#conflicts[@]} == 0)) || die "Existing conflicting packages: ${conflicts[*]}. Resolve using DEPENDENCIES.md, then rerun."

  # Reuse an existing official Docker repository instead of creating duplicate entries.
  if ! grep -Rqs 'download.docker.com/linux/ubuntu' /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
    tmp_dir="$(mktemp -d)"
    trap 'rm -rf -- "$tmp_dir"' EXIT
    curl --fail --show-error --silent --location --retry 3 --connect-timeout 20 \
      https://download.docker.com/linux/ubuntu/gpg -o "$tmp_dir/docker.asc"
    gpg --batch --show-keys "$tmp_dir/docker.asc" >/dev/null
    "${root[@]}" install -m 0755 -d /etc/apt/keyrings
    "${root[@]}" install -m 0644 "$tmp_dir/docker.asc" /etc/apt/keyrings/docker.asc
    codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}"
    [[ "$codename" =~ ^[a-z]+$ ]] || die 'Could not determine Ubuntu repository codename'
    cat > "$tmp_dir/docker.sources" <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $codename
Components: stable
Architectures: amd64
Signed-By: /etc/apt/keyrings/docker.asc
EOF
    # Avoid overwriting an unrelated pre-existing file with this basename.
    [[ ! -e /etc/apt/sources.list.d/swarm-docker.sources ]] || die 'Unexpected existing swarm-docker.sources; inspect it before rerunning'
    "${root[@]}" install -m 0644 "$tmp_dir/docker.sources" /etc/apt/sources.list.d/swarm-docker.sources
  fi
  "${root[@]}" apt-get update
  "${root[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y \
    docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
fi

"${root[@]}" systemctl enable --now docker
docker_run=(bash "$project_dir/scripts/docker.sh")
"${docker_run[@]}" version
"${docker_run[@]}" compose version
"${docker_run[@]}" buildx version
"${docker_run[@]}" info >/dev/null
"${docker_run[@]}" run --rm hello-world
if ((install_only)); then
  echo 'Host dependencies ready. Run bash setup.sh to build the project.'
  exit 0
fi

"${docker_run[@]}" compose config --quiet
# Build each distinct image once; all three agents reuse the companion image.
"${docker_run[@]}" compose build --pull --build-arg "BUILD_JOBS=$jobs" sim
"${docker_run[@]}" compose build --pull coordinator
"${docker_run[@]}" run --rm three-drone-companion:1.0 python -m unittest discover -s tests -v
"${docker_run[@]}" image inspect three-drone-sim:1.0 three-drone-companion:1.0 > logs/setup-images.json
if ((start)); then
  "${docker_run[@]}" compose up -d --no-build
  "${docker_run[@]}" compose ps
fi
cat <<'EOF'
Setup and application tests completed.
Start containers:  bash scripts/docker.sh compose up -d --no-build
Check readiness:   bash scripts/docker.sh compose exec coordinator python scripts/swarmctl.py status
Start mission:     bash scripts/docker.sh compose exec coordinator python scripts/swarmctl.py start
The mission starts only after the explicit start command and vehicle readiness checks.
EOF
