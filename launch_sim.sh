#!/usr/bin/env bash
set -Eeuo pipefail
rm -f /tmp/swarm-ready
px4_dir=/opt/PX4
build_dir="$px4_dir/build/px4_sitl_default"
run_dir="/logs/sim-$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$run_dir"
cp /opt/PX4_COMMIT /opt/PX4_SUBMODULES /opt/OS_PACKAGES "$run_dir/"
cp /app/config/mission.json "$run_dir/mission.json"
export GZ_SIM_RESOURCE_PATH="" GZ_SIM_SYSTEM_PLUGIN_PATH=""
# This also exports PX4_GZ_MODELS, required by PX4's standalone spawn command,
# and GZ_SIM_SERVER_CONFIG_PATH, which supplies the upstream sensor plugins.
source "$build_dir/rootfs/gz_env.sh"
python3 /app/scripts/make_world.py \
  "$px4_dir/Tools/simulation/gz/worlds/default.sdf" /tmp/swarm.sdf \
  --manifest "$run_dir/markers.json"
pids=()
cleanup() {
  rm -f /tmp/swarm-ready
  if ((${#pids[@]})); then
    kill -TERM "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 0' TERM INT
gz sim -r -s --headless-rendering /tmp/swarm.sdf > "$run_dir/gazebo.log" 2>&1 &
pids+=("$!")
ready=0
for ((attempt=0; attempt<90; attempt++)); do
  kill -0 "${pids[0]}" || { cat "$run_dir/gazebo.log"; exit 1; }
  if timeout 2 gz service -l | grep -q '/world/swarm/create'; then
    ready=1
    break
  fi
  sleep 1
done
[[ "$ready" == 1 ]] || { cat "$run_dir/gazebo.log"; exit 1; }
export PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=swarm
export PX4_GZ_STANDALONE=1 HEADLESS=1
mapfile -t poses < <(python3 - <<'PY'
from swarm.common import load_config, offsets
for n,e,u in offsets(load_config()).values():
    print(f'{e},{n},0.2,0,0,0')
PY
)
for instance in 0 1 2; do
  work="$run_dir/px4_$instance"
  mkdir -p "$work"
  (
    cd "$work"
    export PX4_GZ_MODEL_POSE="${poses[$instance]}"
    exec "$build_dir/bin/px4" -i "$instance" -d "$build_dir/etc" -s etc/init.d-posix/rcS
  ) > "$work/console.log" 2>&1 &
  pids+=("$!")
  sleep 2
done
for pid in "${pids[@]}"; do kill -0 "$pid"; done
touch /tmp/swarm-ready
echo "World and three PX4 processes launched. Logs: $run_dir"
# A child exit stops the entire simulator; restarting midflight is deliberately disabled.
wait -n "${pids[@]}"
exit 1
