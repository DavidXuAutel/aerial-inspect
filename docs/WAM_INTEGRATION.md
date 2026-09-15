# WAM 集成说明

## 环境变量

| 变量 | 默认 | 说明 |
|------|------|------|
| `AERIAL_WAM_ROOT` | `~/Projects/aerial-wam-v2` | WAM 仓库根目录 |
| `AERIAL_VGOAL_ROOT` | `~/Projects/aerial-vgoal-wam` | vgoal 检测与 search_planner |
| `QWEN_DECOMPOSE_URL` | — | 可选，`http://<h100>:8000/api/decompose` |

## 复用的 WAM 组件

| WAM 路径 | 用途 |
|----------|------|
| `experiments/aerial/scripts/wam_vgoal_deploy.py` | 真机闭环：检测 + 导航 + 可选录制 |
| `experiments/aerial/deploy/orin_deploy_recorder.py` | 高清 JPEG + traj.jsonl |
| `experiments/aerial/rl/env/pixhawk_env.py` | 真机环境（经 deploy 脚本） |
| Phase-2 toward_g ckpt | 默认 actor（与 vgoal deploy 一致） |

## 调用方式（M1）

本仓 **不 import WAM 训练代码**；通过子进程调用 deploy 脚本，避免依赖 torch/CUDA 版本冲突。

```bash
# 单航点抵近（示例）
python -m experiments.aerial.scripts.wam_vgoal_deploy \
  --vgoal-repo "$AERIAL_VGOAL_ROOT" \
  --visual-prompt "bridge" \
  --goal-x 120.5 --goal-y -30.2 --goal-z 45.0 \
  --offboard --run --max-steps 200 \
  --record-auto
```

端到端闭环：**语义 SEARCH → 自动估计桥中心 → replan 环绕 → APPROACH → SURVEY → 离线重建**。

| 步骤 | 产物 | 命令 |
|------|------|------|
| 预规划 | `phase_plan.json` | `aerial-inspect plan ...` |
| SEARCH | `phase_runs.json`, `traj.jsonl` | `run_wam_search.sh` |
| 估计 + 环绕 | `detected_centroid.json`, `waypoints.json` | `aerial-inspect replan-survey --export-wam` |
| APPROACH / SURVEY | 采集 `run_*` | `run_wam_approach.sh`, `run_wam_survey.sh` |
| 重建 | `artifacts/models/*` | `run_offline_reconstruct.py` |

```bash
aerial-inspect plan configs/missions/bridge_default.yaml -o artifacts/bridge_river_001
aerial-inspect export-wam-phases artifacts/bridge_river_001

bash scripts/run_wam_search.sh artifacts/bridge_river_001
aerial-inspect replan-survey artifacts/bridge_river_001 --export-wam

bash scripts/run_wam_approach.sh artifacts/bridge_river_001
bash scripts/run_wam_survey.sh artifacts/bridge_river_001

# 一键
bash scripts/run_mission_pipeline.sh configs/missions/bridge_default.yaml

# 真机
MOCK_CAMERA=0 bash scripts/run_mission_pipeline.sh configs/missions/bridge_default.yaml
```

桥中心从 SEARCH 的 `traj.jsonl` 中 tracker 锁定的 `goal_rel` + 位姿反算世界坐标（median）。YAML 中 `bridge_centroid_xyz` 仅作 debug 覆盖（`plan --include-survey`）。

## 不纳入 WAM 主线的内容

- `aerial_inspect/survey/orbit_planner.py` — 测绘航线，与 `AdaptiveSubgoalGenerator` 无关
- `aerial_inspect/reconstruct/` — COLMAP 离线管线
- `aerial_inspect/mission/orchestrator.py` — 产品 FSM

这些仅存在于 **aerial-inspect** 仓库。
