#!/usr/bin/env bash
# Avant-AirSim on 84: wait for archive, extract, start on non-default ports, smoke test.
set -euo pipefail

BASE="/data/linux/workspace/Avant-AirSim"
ARCHIVE="${BASE}/Avant-AirSim-linux-x86_64.tar.gz"
ARCHIVE_URL="${ARCHIVE_URL:-http://10.229.20.133:8999/downloads/Avant-AirSim-linux-x86_64.tar.gz}"
EXPECTED_SIZE="${EXPECTED_SIZE:-15426648457}"
PRODUCT="${BASE}/Avant-AirSim"
LOG="${BASE}/smoke.log"

CARLA_PORT="${CARLA_PORT:-2200}"
AIRSIM_PORT="${AIRSIM_PORT:-41463}"
MAP="${MAP:-XianFlightBase}"

exec > >(tee -a "$LOG") 2>&1
echo "=== avant_airsim_84_smoke $(date -Is) ==="
echo "CARLA_PORT=$CARLA_PORT AIRSIM_PORT=$AIRSIM_PORT MAP=$MAP"

wait_for_archive() {
  echo "Waiting for $ARCHIVE ($EXPECTED_SIZE bytes)..."
  while true; do
    if [[ -f "$ARCHIVE" ]]; then
      local sz
      sz=$(stat -c '%s' "$ARCHIVE" 2>/dev/null || stat -f '%z' "$ARCHIVE")
      echo "  archive: $sz / $EXPECTED_SIZE"
      if [[ "$sz" -eq "$EXPECTED_SIZE" ]]; then
        echo "Archive complete."
        return 0
      fi
    else
      echo "  archive: missing"
    fi
    sleep 30
  done
}

extract_product() {
  if [[ -x "${PRODUCT}/run.sh" ]]; then
    echo "Product already extracted: $PRODUCT"
    return 0
  fi
  echo "Extracting $ARCHIVE ..."
  cd "$BASE"
  tar -xzf "$ARCHIVE"
  [[ -x "${PRODUCT}/run.sh" ]] || { echo "run.sh missing after extract"; exit 1; }
  chmod +x "${PRODUCT}/run.sh" "${PRODUCT}/CarlaAir.sh" "${PRODUCT}/CarlaUE4.sh" 2>/dev/null || true
  chmod +x "${PRODUCT}/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" 2>/dev/null || true
}

setup_python() {
  local conda_base="${CONDA_BASE:-/data/linux/workspace/miniconda3}"
  local tarball="${PRODUCT}/env_setup/carla_python_module.tar.gz"
  [[ -f "$tarball" ]] || { echo "missing $tarball"; return 1; }
  [[ -f "${conda_base}/etc/profile.d/conda.sh" ]] || { echo "missing conda at $conda_base"; return 1; }
  # shellcheck source=/dev/null
  source "${conda_base}/etc/profile.d/conda.sh"
  conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main 2>/dev/null || true
  conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r 2>/dev/null || true
  conda create -n carlaAir python=3.10 -y -q 2>/dev/null || true
  conda activate carlaAir
  pip install -q numpy msgpack-rpc-python opencv-contrib-python Pillow pygame
  pip install -q --no-build-isolation airsim
  local site conda_lib
  site="$(python -c 'import site; print(site.getsitepackages()[0])')"
  conda_lib="$(python -c 'import sys; print(sys.prefix)')/lib"
  rm -rf "${site}/carla" "${site}/carla.libs"
  tar xzf "$tarball" -C "$site"
  cp -n "${site}/carla.libs/"*.so* "$conda_lib/" 2>/dev/null || true
  AERIAL_PY="$(which python)"
  export AERIAL_PY
  python -c "import carla, airsim; print('python ok:', carla.__file__)"
}

stop_avant() {
  if [[ -x "${PRODUCT}/run.sh" ]]; then
    CARLA_PORT="$CARLA_PORT" AIRSIM_PORT="$AIRSIM_PORT" \
      bash "${PRODUCT}/run.sh" stop 2>/dev/null || true
  fi
}

start_sim() {
  stop_avant
  cd "$PRODUCT"
  nohup env CARLA_PORT="$CARLA_PORT" AIRSIM_PORT="$AIRSIM_PORT" \
    OFFSCREEN=1 AUTO_TRAFFIC=0 QUALITY=Epic RES=1280x720 \
    ./run.sh start "$MAP" >"${BASE}/sim.log" 2>&1 &
  echo "sim pid=$!"
}

wait_ports() {
  local i
  for i in $(seq 1 90); do
    if ss -ltn 2>/dev/null | grep -q ":${CARLA_PORT} " && \
       ss -ltn 2>/dev/null | grep -q ":${AIRSIM_PORT} "; then
      echo "Ports listening after ${i}0s"
      return 0
    fi
    sleep 10
  done
  echo "Ports not ready; tail sim.log:"
  tail -50 "${BASE}/sim.log" || true
  return 1
}

smoke_rpc() {
  local py="${AERIAL_PY:-/data/linux/workspace/miniconda3/envs/carlaAir/bin/python}"

  "$py" - <<PY
import carla
import airsim

c = carla.Client("127.0.0.1", ${CARLA_PORT})
c.set_timeout(60.0)
sv = c.get_server_version()
w = c.get_world()
map_name = w.get_map().name.split("/")[-1]
print(f"CARLA ok: server={sv} map={map_name}")

drone = airsim.MultirotorClient(ip="127.0.0.1", port=${AIRSIM_PORT}, timeout_value=30)
drone.confirmConnection()
if not drone.ping():
    raise RuntimeError("AirSim ping failed")
print("AirSim ok: ping=True")
PY
}

main() {
  mkdir -p "$BASE"
  wait_for_archive
  extract_product
  "${PRODUCT}/run.sh" status || true
  setup_python || true
  start_sim
  wait_ports
  smoke_rpc
  echo "=== SMOKE PASS $(date -Is) ==="
  echo "CARLA=127.0.0.1:${CARLA_PORT}  AirSim=127.0.0.1:${AIRSIM_PORT}  MAP=$MAP"
  echo "External: 10.229.20.84:${AIRSIM_PORT}"
}

main "$@"
