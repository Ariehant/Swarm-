#!/usr/bin/env bash
# Explicit local-daemon wrapper; works immediately without a docker-group relogin.
set -Eeuo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_dir"
command -v docker >/dev/null 2>&1 || { echo 'Run bash setup.sh first.' >&2; exit 1; }
if (($# == 0)); then
  echo 'Usage: bash scripts/docker.sh compose <arguments> OR <docker command>' >&2
  exit 2
fi
if ((EUID == 0)) || [[ -w /var/run/docker.sock ]]; then
  exec docker --host unix:///var/run/docker.sock "$@"
fi
exec sudo docker --host unix:///var/run/docker.sock "$@"
