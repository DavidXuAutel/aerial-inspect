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
# 公网（详见 docs/SSH_84_PUBLIC.md）
ssh cursor-84-public           # 推荐：ssh-84.david-x.com + Cloudflare Access
ssh cursor-84-via-125          # 备用：经 125 跳转

# 公司局域网
ssh cursor-84-lan              # ubantu@10.229.20.84

source scripts/remote_84.env   # 默认 cursor-84-public（ssh-84.david-x.com）；sync/pull 脚本用此
```

## Mac 公网连接（后续默认）

在 Mac 上 **不 SSH 进 84** 也可探测 OpenFly RPC：

```bash
source scripts/mac_env_sim_84.sh          # 41451 bridge / building
bash scripts/mac_sim_via_84.sh
```

虎门 Avant-AirSim（**41463**）：

```bash
source scripts/mac_env_sim_84_humen.sh
python3 -c "import airsim; c=airsim.MultirotorClient(ip='$AIRSIM_HOST',port=$AIRSIM_PORT); c.confirmConnection(); print('OK')"
```

一键在 84 上跑虎门采图+重建并拉回 Mac：

```bash
bash scripts/run_humen_capture_on_84.sh
# 仅同步代码 / 仅拉产物：
bash scripts/sync_to_84.sh
bash scripts/pull_from_84.sh bridge_humen_001
```

| 变量 | 默认 | 说明 |
|------|------|------|
| `AERIAL_SSH_HOST` | `cursor-84-public` | 公网 SSH（`ssh-84.david-x.com`） |
| `AIRSIM_PUBLIC_HOST` | `10.229.20.84` | Mac 直连 AirSim RPC |
| `AIRSIM_HOST` | Mac 上同 public；84 本机用 `127.0.0.1` | 见 `env_sim_84*.sh` |

84 本机执行仍用 `source scripts/env_sim_84_humen.sh`（`AIRSIM_HOST=127.0.0.1`）。

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
git pull   # 或 Mac: bash scripts/sync_to_84.sh
source scripts/env_sim_84_humen.sh
python3 scripts/bridge_scene_check.py \
  -o artifacts/humen_scene_check.jpg \
  --meta-out artifacts/humen_scene_check.json

# 椭圆航线采图 + hybrid 重建（M1）
bash scripts/run_bridge_humen_capture.sh

# 或完整 SEARCH→SURVEY 闭环
bash scripts/run_sim_pipeline.sh configs/missions/bridge_humen.yaml
```

**Mac 触发（public SSH，推荐）：** `bash scripts/run_humen_capture_on_84.sh`

**首轮结果（2026-09-16）：** 48 帧椭圆双圈，`artifacts/models/bridge_humen_001/`（425 sparse 点，待调参）。

### 虎门双侧立面主线（overnight / FORCE_SURVEY）

主线：**phase2 polyline、no-shield、scripted tip、AUTO_EVAL→auto_next**。仿真 **仅** 在需要重新采图时必需；对已有 `capture_survey` 做 denser COLMAP **不需要** AirSim。

```bash
# 84：确认 Avant 虎门 RPC
nc -z -w 2 127.0.0.1 41463 && echo ping_41463_ok

# Mac → 84 同步代码后启动 autopilot（最多约 10 轮）
bash scripts/sync_to_84.sh
ssh "$AERIAL_SSH_HOST" 'cd ~/Projects/aerial-inspect && nohup bash scripts/humen_autopilot.sh > artifacts/bridge_humen_001/humen_autopilot_outer.log 2>&1 &'

# 仅 denser 再飞一圈（改 heading_overlap 后 FORCE_SURVEY）
ssh "$AERIAL_SSH_HOST" 'cd ~/Projects/aerial-inspect && source scripts/env_sim_84_humen.sh && \
  export SIM_SURVEY_NO_SHIELD=1 SIM_SURVEY_SCRIPTED_TIP=1 && \
  FORCE_SURVEY=1 bash scripts/run_humen_sim_pipeline.sh configs/missions/bridge_humen.yaml'

# 仅 denser COLMAP（无仿真；84 有 pycolmap）
# --sequential-overlap 14 --lap-stride 8 → 同 16 帧可从 ~162 提到 ~262 pts
```

| 开关 / 配置 | 作用 |
|-------------|------|
| `SIM_SURVEY_NO_SHIELD=1` | 关 shield，避 tip 卡死 |
| `SIM_SURVEY_SCRIPTED_TIP=1` | tip 用脚本 hop，不用长 polyline |
| `heading_overlap: 0.70` | 沿跨更密 WP（overnight 曾用 0.55） |
| export skip bad WP | 单点 pose 失败不整趟 abort |
| hang watchdog | long_eval >600s 且 traj 行数 <5 则 kill |

**验收参考（v14_r8）：** SR≈93%（14/15），`recon_points=162` → denser hybrid **262**；决策 `done_pull`。稀疏仍偏近岸时再飞 denser dual-facade。

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
