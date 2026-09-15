# 10.229.20.84 桥梁仿真（本机渲染 + 外部连接）

- **主机**: `10.229.20.84` · 用户 `ubantu`
- **GPU**: RTX 4090 D
- **场景**: `env_airsim_16` **河段桥区**（`bridge_default.yaml` 搜索中心约 `[-1020,-220]`），与 125 上 Phase-2 长跑航线区不同
- **后端**: `wam_vgoal_eval` + AirSim（非真机 `wam_vgoal_deploy`）

## 拓扑

```
Mac / H100 客户端 ──► 10.229.20.84:41451 (AirSim RPC, 0.0.0.0 绑定)
                         └── 本机 UE 渲染 (bridge zone)
```

**不要**再连 `10.229.20.125:41451`（125 是另一条评测线）。

## 84 上一次性准备

```bash
cd ~/Projects/aerial-inspect
git pull origin main

# 从 125 只同步渲染参数 settings.json；场景为 bridge 用 env_airsim_16
SRC_PASS='***' bash scripts/sync_renderer_from_125.sh

# 启动本机桥梁渲染器（自动 patch 外部绑定 + 跳过 vulkaninfo 依赖）
bash scripts/start_renderer_84.sh

source scripts/env_sim_84.sh
/data/linux/workspace/venvs/aerial-wam/bin/pip install -e .
```

验证 RPC（本机）：

```bash
python3 -c "import socket;socket.create_connection(('127.0.0.1',41451),5);print('local ok')"
```

验证 RPC（局域网，在 Mac 上）：

```bash
python3 -c "import socket;socket.create_connection(('10.229.20.84',41451),5);print('external ok')"
```

## Mac 外部连接

```bash
cd ~/Projects/aerial-inspect
source scripts/mac_env_sim_84.sh   # AIRSIM_HOST=10.229.20.84

# 拉一张桥区检查图（84 上需已 start_renderer_84）
python3 scripts/bridge_scene_check.py --host "$AIRSIM_HOST" \
  -o /tmp/bridge_scene_check.jpg
```

SSH 到 84 跑 pipeline 时仍用 `env_sim_84.sh`（`127.0.0.1`）。

## 环境变量

| 变量 | 84 本机 | Mac / 外部 |
|------|---------|------------|
| `AIRSIM_HOST` | `127.0.0.1` | `10.229.20.84` |
| `AIRSIM_PORT` | `41451` | `41451` |
| `AIRSIM_BIND_IP` | `0.0.0.0`（settings patch） | — |
| `AERIAL_WAM_ROOT` | `~/Projects/aerial-wam-v2` | 同左 |
| `SIM_DETECTOR` | `open_vocab` | 同左 |

## 一键验收（84 上）

```bash
source scripts/env_sim_84.sh
bash scripts/run_sim_pipeline.sh configs/missions/bridge_default.yaml
```

桥区截图检查：

```bash
python3 scripts/bridge_scene_check.py \
  --out artifacts/bridge_scene_check.jpg
```

## 产物

| 阶段 | 路径 |
|------|------|
| 场景检查图 | `artifacts/bridge_scene_check.jpg` |
| SEARCH traj | `artifacts/bridge_river_001/sim_runs/search/traj.jsonl` |
| 桥中心 | `artifacts/bridge_river_001/detected_centroid.json` |

## 常见问题

| 问题 | 处理 |
|------|------|
| Mac 连不上 `:41451` | 84 上 `bash scripts/patch_airsim_external_bind.sh` 后重启 renderer；查防火墙 |
| `vulkaninfo: not found` | 已用 `recover_renderer_bridge.sh` 跳过；不影响启动 |
| 画面无桥 | 确认 spawn 在河段坐标，见 `configs/sim/bridge_scene.yaml` |
| 仍指向 125 | 检查 `AIRSIM_HOST`，Mac 用 `mac_env_sim_84.sh` |
