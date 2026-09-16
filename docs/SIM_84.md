# 10.229.20.84 仿真（本机渲染 + 外部连接）

- **主机**: `10.229.20.84` · 用户 `ubantu`
- **GPU**: RTX 4090 D
- **场景**: `env_airsim_16`（水岸建筑区，`y≈-220`）
- **当前任务**: 建筑扫描 — `configs/missions/building_default.yaml`
- **后端**: `wam_vgoal_eval` + AirSim

## 拓扑

```
Mac / H100 客户端 ──► 10.229.20.84:41451 (AirSim RPC, 0.0.0.0 绑定)
                         └── 本机 UE 渲染 (env_airsim_16)
```

**不要**连 `10.229.20.125:41451` 做本仓 eval（125 是 Phase-2 长跑线）。

```bash
ssh cursor-84-public
```

## 84 上准备

```bash
cd ~/Projects/aerial-inspect
git pull origin main

bash scripts/sync_renderer_from_125.sh   # settings + env_airsim_16（125 已有）
bash scripts/start_renderer_84.sh

source scripts/env_sim_84.sh
/data/linux/workspace/venvs/aerial-wam/bin/pip install -e .
```

## 验证

```bash
source scripts/env_sim_84.sh
python3 scripts/bridge_scene_check.py -o artifacts/scene_check.jpg
```

## 建筑扫描 pipeline

```bash
source scripts/env_sim_84.sh
bash scripts/run_sim_pipeline.sh configs/missions/building_default.yaml
```

检测词为 `building`（YOLO-World），搜索中心 `[-801, -220]`，与 Phase-2 `outdoor_long` 水岸航线一致。

## 环境变量

| 变量 | 默认 | 说明 |
|------|------|------|
| `AIRSIM_SCENE_ID` | `env_airsim_16` | 场景目录 |
| `AIRSIM_SCENE_CONFIG` | `configs/sim/bridge_scene.yaml` | spawn / yaw |
| `SIM_YOLO_MODEL` | `yolov8s-worldv2.pt` | open-vocab 检测 |
| `BRIDGE_CAMERA_PITCH` | `-25` | 相机俯角（建筑立面） |

## 产物

| 阶段 | 路径 |
|------|------|
| 场景检查图 | `artifacts/scene_check.jpg` |
| SEARCH traj | `artifacts/building_waterfront_001/sim_runs/search/traj/` |
| 目标中心 | `artifacts/building_waterfront_001/detected_centroid.json` |

## 虎门大桥（Avant-AirSim）

Avant-AirSim `HumenCorridor` 与 OpenFly `env_airsim_16` **可同时运行**（不同端口）。

```bash
# 84 上先启动虎门（若未运行）
bash /data/linux/workspace/Avant-AirSim/start_humen.sh

cd ~/Projects/aerial-inspect
git pull   # 含 humen_scene.yaml / bridge_humen.yaml
source scripts/env_sim_84_humen.sh
python3 scripts/bridge_scene_check.py \
  -o artifacts/humen_scene_check.jpg \
  --meta-out artifacts/humen_scene_check.json

bash scripts/run_sim_pipeline.sh configs/missions/bridge_humen.yaml
```

| 变量 | 虎门默认 | OpenFly 默认 |
|------|----------|--------------|
| `AIRSIM_PORT` | **41463** | 41451 |
| `AIRSIM_VEHICLE` | `SimpleFlight` | `drone_1` |
| `AIRSIM_CAMERA` | `0` | `front_custom` |
| `AIRSIM_SCENE_CONFIG` | `configs/sim/humen_scene.yaml` | `configs/sim/bridge_scene.yaml` |

## 其他场景

| 场景 | 配置 | 备注 |
|------|------|------|
| 桥梁（OpenFly 远景） | `configs/missions/bridge_default.yaml` | `env_airsim_16`，检测 `bridge` |
| 桥梁（虎门主场景） | `configs/missions/bridge_humen.yaml` | Avant `HumenCorridor` |
| 广州 | `configs/missions/bridge_guangzhou.yaml` | 需 HF 下载 `env_airsim_gz` |

```bash
export AIRSIM_SCENE_ID=env_airsim_gz
export AIRSIM_SCENE_CONFIG=configs/sim/guangzhou_scene.yaml
HF_TOKEN=hf_... bash scripts/download_openfly_scene.sh env_airsim_gz
```
