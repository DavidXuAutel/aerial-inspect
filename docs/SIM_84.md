# 10.229.20.84 AirSim 仿真验收

- **主机**: `10.229.20.84`
- **用户**: `ubantu`（注意拼写，不是 ubuntu）
- **GPU**: RTX 4090 D

仿真走 **`wam_vgoal_eval` + AirSim**，不走真机 `wam_vgoal_deploy`。

## 前置条件（84 上）

1. AirSim 已启动、无僵尸进程
2. 仓库已在 `~/Projects/`：
   - `aerial-wam-v2`
   - `aerial-inspect`
   - `aerial-vgoal-wam`
3. **WAM Python 环境** — 已存在，**不要重复装 `aerial_sim`**：
   - venv: `/data/linux/workspace/venvs/aerial-wam`（`torch 2.6+cu124`，`airsim`，约 5.7G）
   - 安装记录: `/data/linux/workspace/aerial_setup.log`（今日 `setup_aerial.sh`）
   - 激活: `source scripts/env_sim_84.sh`
   - 旧版仓库: `/data/linux/workspace/aerial-wam-v2`；**新脚本用** `~/Projects/aerial-wam-v2`
   - AirSim 渲染在 **`10.229.20.125:41451`**（84 是 eval 客户端）
4. `bridge_default.yaml` 的 `search_area.center_xy` 与当前地图桥区一致

## 环境变量

```bash
export AERIAL_WAM_ROOT=~/aerial-wam-v2
export AERIAL_VGOAL_ROOT=~/Projects/aerial-vgoal-wam
export SIM_DETECTOR=open_vocab    # 开放词表找 bridge
export SIM_DEVICE=cuda
export SIM_SEARCH_PATTERN=lawnmower
```

## 一键验收

```bash
cd ~/Projects/aerial-inspect
source scripts/env_sim_84.sh
/data/linux/workspace/venvs/aerial-wam/bin/pip install -e .

bash scripts/run_sim_pipeline.sh configs/missions/bridge_default.yaml
```

等价分步：

```bash
aerial-inspect plan configs/missions/bridge_default.yaml -o artifacts/bridge_river_001
aerial-inspect export-wam-phases artifacts/bridge_river_001
aerial-inspect export-sim artifacts/bridge_river_001

aerial-inspect sim-search artifacts/bridge_river_001
aerial-inspect replan-survey artifacts/bridge_river_001 --export-wam

aerial-inspect export-sim artifacts/bridge_river_001   # 刷新 approach 航段
aerial-inspect sim-approach artifacts/bridge_river_001
aerial-inspect sim-survey artifacts/bridge_river_001
```

## 产物路径

| 阶段 | 路径 |
|------|------|
| SEARCH traj | `artifacts/<id>/sim_runs/search/traj.jsonl` |
| 桥中心 | `artifacts/<id>/detected_centroid.json` |
| 环绕航点 | `artifacts/<id>/waypoints.json` |
| SURVEY | `artifacts/<id>/sim_runs/survey/wp_*/traj.jsonl` |

## 验收标准

| 项 | 条件 |
|----|------|
| 检测 | `detected_centroid.json` → `status: ok` |
| 航点 | `waypoints.json` 48 点 |
| SURVEY | 48 个 `wp_*` 目录均有 `traj.jsonl` |

## 从本机同步代码到 84

```bash
ssh ubantu@10.229.20.84 mkdir -p ~/Projects
rsync -avz --exclude .venv --exclude .git --exclude artifacts \
  ~/Projects/aerial-inspect/ ubantu@10.229.20.84:~/Projects/aerial-inspect/
rsync -avz --exclude .venv --exclude .git --exclude artifacts \
  ~/Projects/aerial-wam-v2/ ubantu@10.229.20.84:~/Projects/aerial-wam-v2/
rsync -avz --exclude .venv --exclude .git \
  ~/Projects/aerial-vgoal-wam/ ubantu@10.229.20.84:~/Projects/aerial-vgoal-wam/
```

建议配置 SSH 公钥，避免在脚本里写密码：

```bash
ssh-copy-id ubantu@10.229.20.84
```

## 常见问题

| 问题 | 处理 |
|------|------|
| `insufficient tracked frames` | 调 `SIM_DETECTOR`、确认 AirSim 场景有桥、`--search-area-half-m` 覆盖桥区 |
| CUDA OOM | `export SIM_DEVICE=cpu`（慢） |
| 坐标不对 | 改 `configs/missions/bridge_default.yaml` 的 `search_area` |
