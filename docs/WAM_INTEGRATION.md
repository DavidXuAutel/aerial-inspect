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

Survey 阶段使用本仓生成的 `wam_waypoints.json`，由 `aerial-inspect export-wam` 写出，再由 `scripts/run_wam_survey.sh` 顺序执行。

## 不纳入 WAM 主线的内容

- `aerial_inspect/survey/orbit_planner.py` — 测绘航线，与 `AdaptiveSubgoalGenerator` 无关
- `aerial_inspect/reconstruct/` — COLMAP 离线管线
- `aerial_inspect/mission/orchestrator.py` — 产品 FSM

这些仅存在于 **aerial-inspect** 仓库。
