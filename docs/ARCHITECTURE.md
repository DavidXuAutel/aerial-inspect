# 架构

```text
┌─────────────────────────────────────────────────────────────┐
│  Aerial Inspect（本仓）                                       │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────────┐ │
│  │ MissionOrchestrator │  │ OrbitPlanner │  │ CoverageChecker │ │
│  │ SEARCH→APPROACH→    │  │ 立面/水平环绕  │  │ 重叠/缺口补拍   │ │
│  │ SURVEY→DONE         │  └──────────────┘  └─────────────────────┘ │
│  └──────┬──────┘                                              │
│         │ waypoints + capture_spec                            │
│  ┌──────▼──────┐  ┌──────────────────────────────────────────┐│
│  │ CaptureSession│  │ ReconstructPipeline (offline, COLMAP)   ││
│  └─────────────┘  └──────────────────────────────────────────┘│
└────────────────────────────┬────────────────────────────────┘
                             │ adapters.wam_platform
┌────────────────────────────▼────────────────────────────────┐
│  aerial-wam-v2                                               │
│  wam_vgoal_deploy · OrinDeployRecorder · PixhawkDroneEnv       │
│  LatentActorDeployPolicy · ThreeZoneShield                   │
└────────────────────────────┬────────────────────────────────┘
                             │
┌────────────────────────────▼────────────────────────────────┐
│  aerial-vgoal-wam + Qwen（可选）                             │
│  detector · search_planner · /api/decompose                  │
└─────────────────────────────────────────────────────────────┘
```

## 任务 FSM

| 状态 | 行为 | WAM 模式 |
|------|------|----------|
| `SEARCH` | 区域扫描，开放词表找 `bridge` | vgoal SEARCH + toward_g |
| `APPROACH` | 抵近 standoff，估计桥中心 | vgoal APPROACH |
| `SURVEY` | 执行 `OrbitPlanner` 航点序列 | 逐 waypoint toward_g / 坐标导航 |
| `CAPTURE` | 每航点 Recorder 存 1280 图 + 位姿 | 同 SURVEY，采集开 |
| `DONE` | 写 manifest，触发离线重建 | 悬停/返航 |

## 数据流

1. `MissionSpec`（YAML）→ `plan_mission()` → `phase_plan.json`（survey 待定）
2. SEARCH：`run_wam_search.sh` → `orin_deploy/run_*` + `traj.jsonl`
3. `replan-survey`：从 traj 估计 `bridge_centroid_xyz` → `waypoints.json` + `wam_waypoints.json`
4. APPROACH + SURVEY：WAM 子进程逐阶段执行
5. `run_offline_reconstruct.py` → `artifacts/models/<id>/`
