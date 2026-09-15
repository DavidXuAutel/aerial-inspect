#!/usr/bin/env bash
# Create conda env `aerial_sim` on 10.229.20.84 for WAM + AirSim eval.
# Run ON the GPU host:  bash scripts/setup_sim_env_84.sh
set -euo pipefail

ENV_NAME="${ENV_NAME:-aerial_sim}"
CONDA="${CONDA:-$HOME/miniforge3/bin/conda}"
PY="$HOME/miniforge3/envs/${ENV_NAME}/bin/python"
PIP="$HOME/miniforge3/envs/${ENV_NAME}/bin/pip"

INSPECT_ROOT="${INSPECT_ROOT:-$HOME/Projects/aerial-inspect}"
WAM_ROOT="${WAM_ROOT:-$HOME/Projects/aerial-wam-v2}"
VGOAL_ROOT="${VGOAL_ROOT:-$HOME/Projects/aerial-vgoal-wam}"

echo "=== aerial_sim setup on $(hostname) ==="
echo "CONDA=$CONDA ENV=$ENV_NAME"

if [[ ! -x "$CONDA" ]]; then
  echo "ERROR: miniforge not found at $CONDA"
  exit 1
fi

if ! "$CONDA" env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  echo "[1/6] create conda env $ENV_NAME (python 3.10)"
  "$CONDA" create -n "$ENV_NAME" python=3.10 -y
else
  echo "[1/6] conda env $ENV_NAME exists"
fi

echo "[2/6] install PyTorch (CUDA 12.4 wheel)"
"$PIP" install -U pip wheel
"$PIP" install torch torchvision --index-url https://download.pytorch.org/whl/cu124

echo "[3/6] AirSim client + sim_verify deps"
"$PIP" install -r "$WAM_ROOT/experiments/aerial/sim_verify/requirements.txt"

echo "[4/6] WAM eval extras (depth head / vgoal)"
"$PIP" install einops addict timm pyyaml ultralytics opencv-python-headless

echo "[5/6] aerial-inspect + vgoal"
"$PIP" install -e "$INSPECT_ROOT[dev]"
"$PIP" install -r "$VGOAL_ROOT/requirements.txt"

echo "[6/6] smoke import"
export PYTHONPATH="$WAM_ROOT"
"$PY" - <<'PY'
import torch
print("torch", torch.__version__, "cuda", torch.cuda.is_available())
import airsim
import einops, addict, timm, yaml
print("airsim ok")
from experiments.aerial.scripts import wam_vgoal_eval
print("wam_vgoal_eval import ok")
PY

cat > "$INSPECT_ROOT/scripts/env_sim_84.sh" <<EOF
#!/usr/bin/env bash
# source scripts/env_sim_84.sh
export AERIAL_WAM_ROOT="$WAM_ROOT"
export AERIAL_VGOAL_ROOT="$VGOAL_ROOT"
export PYTHONPATH="\$AERIAL_WAM_ROOT:\${PYTHONPATH:-}"
export PYTHON_BIN="$PY"
export AIRSIM_HOST="\${AIRSIM_HOST:-127.0.0.1}"
export AIRSIM_PORT="\${AIRSIM_PORT:-41451}"
export SIM_DEVICE="\${SIM_DEVICE:-cuda}"
export SIM_DETECTOR="\${SIM_DETECTOR:-open_vocab}"
alias aerial-py="\$PYTHON_BIN"
echo "[env_sim_84] PYTHON_BIN=\$PYTHON_BIN"
echo "[env_sim_84] AIRSIM=\$AIRSIM_HOST:\$AIRSIM_PORT cuda=\$(\$PYTHON_BIN -c 'import torch; print(torch.cuda.is_available())')"
EOF
chmod +x "$INSPECT_ROOT/scripts/env_sim_84.sh"

echo ""
echo "=== Done ==="
echo "  source $INSPECT_ROOT/scripts/env_sim_84.sh"
echo "  bash $INSPECT_ROOT/scripts/run_sim_pipeline.sh configs/missions/bridge_default.yaml"
