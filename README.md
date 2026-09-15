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

# 1) 从任务 YAML 生成环绕航点
aerial-inspect plan configs/missions/bridge_default.yaml -o artifacts/mission_001

# 2) 导出 WAM 可执行的 waypoint 清单（真机 / 仿真）
aerial-inspect export-wam artifacts/mission_001

# 3) 仿真干跑（仅打印 FSM，不连飞控）
python scripts/run_mission_dryrun.py configs/missions/bridge_default.yaml

# 4) 离线重建（需本机安装 colmap）
python scripts/run_offline_reconstruct.py artifacts/captures/run_xxx
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

## 许可

与组织内 WAM 项目保持一致（内部使用）。
