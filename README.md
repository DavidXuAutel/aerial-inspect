# Aerial Inspect

独立产品：**语义发现目标 → 测绘式环绕采集 → 离线三维重建**。

与 [aerial-wam-v2](https://github.com/) 的关系：WAM 提供 **自主飞行底盘**（π + 安全罩 + 可选 vgoal 语义引导）；本仓拥有 **任务规划、环绕航迹、采集规范与重建管线**。

## 能力边界

| 本仓 | WAM（依赖） |
|------|-------------|
| 任务分解 / 环绕航迹 / 覆盖度 | 低级导航、避障、抵近 |
| 采集会话与质检元数据 | Orin 相机 + `OrinDeployRecorder` |
| 离线 COLMAP / 质检 | vgoal 检测、Qwen 分解（旁线仓） |

## 快速开始

```bash
cd ~/Projects/aerial-inspect
python3.10 -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"

# 环境：指向 WAM 与 vgoal 旁线仓
export AERIAL_WAM_ROOT=~/Projects/aerial-wam-v2
export AERIAL_VGOAL_ROOT=~/Projects/aerial-vgoal-wam

# 1) 预规划（SEARCH 阶段配置；环绕航点待检测后生成）
aerial-inspect plan configs/missions/bridge_default.yaml -o artifacts/bridge_river_001
aerial-inspect export-wam-phases artifacts/bridge_river_001

# 2) 语义搜索 → 自动估计桥中心 → 生成环绕航点
bash scripts/run_wam_search.sh artifacts/bridge_river_001
aerial-inspect replan-survey artifacts/bridge_river_001 --export-wam

# 3) 抵近 + 环绕采集
bash scripts/run_wam_approach.sh artifacts/bridge_river_001
bash scripts/run_wam_survey.sh artifacts/bridge_river_001

# 或一键全流程
bash scripts/run_mission_pipeline.sh configs/missions/bridge_default.yaml

# 4) AirSim 仿真（10.229.20.84 / 125）
bash scripts/run_sim_pipeline.sh configs/missions/bridge_default.yaml
# 详见 docs/SIM_84.md

# 5) 干跑（fixture traj 模拟 SEARCH 检测，不连飞控）
python scripts/run_mission_dryrun.py configs/missions/bridge_default.yaml

# 6) 离线重建（需本机安装 colmap）
python scripts/run_offline_reconstruct.py ~/Projects/aerial-wam-v2/artifacts/orin_deploy/run_xxx
```

## 目录

```
aerial-inspect/
  aerial_inspect/    # 核心库
    mission/         # 任务规格与 FSM 编排
    survey/          # 环绕 / 立面航迹与覆盖度
    capture/         # 采集会话（封装 WAM recorder）
    reconstruct/     # 离线重建与质检
    adapters/        # WAM / Qwen / vgoal 适配
  configs/missions/  # 任务模板
  scripts/           # CLI 入口脚本
  docs/              # PRD、架构、集成说明
  tests/
```

## 文档

- [产品定义与里程碑](docs/PRD.md)
- [架构](docs/ARCHITECTURE.md)
- [WAM 集成](docs/WAM_INTEGRATION.md)
- [虎门主桥整桥扫描总结](docs/HUMEN_FULL_BRIDGE.md)

## 许可

与组织内 WAM 项目保持一致（内部使用）。
