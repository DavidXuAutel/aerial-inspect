# 虎门主桥整桥深度扫描

日期：2026-09-19。代码：`main` @ `6e0db4d`。状态：扫描已结束，可以归档。这不是整个 Aerial Inspect 产品的完工说明。

## 结论

在 84 号机的 Avant 场景 `HumenCorridor` 里，沿主跨中线飞完一次，得到可重建的整桥深度点云和网格。

虎门大桥主桥是悬索桥，只有一层桥面。侧视图上下两条分别是桥面和主缆，不是两层行车道。

这次交付的是深度反投影点云，不是搜索、抵近、环绕采集，也还没有做正式 COLMAP 重建。

## 它在项目里的位置

Aerial Inspect 负责任务规划、测绘航迹、采集规范和离线重建。飞行底盘在 [aerial-wam-v2](https://github.com/DavidXuAutel/aerial-wam-v2)。产品定义见 [PRD.md](PRD.md)，84 仿真环境见 [SIM_84.md](SIM_84.md)。

本任务对应 `configs/missions/bridge_humen.yaml`（`mission_id: bridge_humen_001`）：在虎门走廊找到主桥，双侧立面采集，供三维建模。早期椭圆环绕和补洞扫描没有给出连续主缆，最终改为下面这一条整桥深度扫描。

## 场景与坐标

| 项 | 值 |
|----|----|
| 机器 | `10.229.20.84`，用户 `ubantu`，RTX 4090 D |
| 场景 | Avant-AirSim `HumenCorridor` |
| RPC | 本机 `127.0.0.1:41463`（不是 OpenFly 的 41451） |
| 坐标系 | AirSim 世界坐标，单位米，z 向上为正 |
| 沿桥向 | `x`。西塔约 2300 m，东塔约 3200 m |
| 横桥向 | `y`。桥面两侧峰值约 -176 m 与 -262 m |
| 高度 | `z`。桥面约 130 m，主缆约 175–200 m，塔顶约 202 m |

这不是经纬度，也不是海拔。CARLA / OpenDRIVE 坐标与这套 AirSim 坐标不是同一套，不能混用。旧锚点 `[-890, -85]` 是 OpenDRIVE 误用，已经废弃。

任务里的主跨锚点：`bridge_centroid_xyz: [2724, -217, 130]`（主航道 midspan），桥轴约 **-30.7°**。扫描中线在仓库 `configs/sim/humen_centerline.json`（桥面中线，约 838 m），不是只覆盖西塔的短段。

旧配置曾把中心钉在西塔附近 `[2400, -20]`、桥轴写成 `+26°`，立面长度又退化成 `2×radius×ellipse_aspect≈264 m`，航点只落在约 `x∈[2260,2545]`，没有沿主航道走。已改正。

## 方法

脚本：`scripts/fly_continuous_complete.py`，由 `scripts/run_complete_v6.sh` 在 84 上启动。整理与出图：`scripts/finalize_full_bridge.py`。

一次沿整条主跨飞行，每个站两侧都拍。不是分段补洞。

每个站的固定俯仰层：

| 层 | 含义 | 保留高度 |
|----|------|----------|
| underside | 梁底 | 118–130 m |
| deck | 桥面 | 120–142 m |
| mid | 中层 | 132–150 m |
| cable_lo | 下层缆 | 136–158 m |

主缆用抛物线瞄准，不再用会把跨中切掉的固定俯仰。v6 把跨中瞄准从 156 m 抬到 168 m，塔侧 188 m。之前瞄准 156 m 时，高度窗顶在约 168 m，跨中主缆被裁掉。

| 层 | 侧向距离 | 飞行高度 | 瞄准半窗 |
|----|----------|----------|----------|
| cable_hi | 48 m | 92 m | 14 m |
| cable_peak | 45 m | 100 m | 16 m |
| cable_top | 42 m | 108 m | 14 m |
| tower（仅两端） | 42 m | 115 m | 14 m |

深度为 DepthPlanar，水平视场 70°，每隔一个像素反投影，深度 2–160 m。跨中 `z ≥ 158 m` 的点只保留距中线 28 m 以内的，用来丢掉以前 look-at 打出来的假面片。过滤只用 NumPy，84 的环境里没有 SciPy。

v6 飞行：229 站，步长 4 m，3312 张，约 43 分钟。

## 结果

过滤后 **4,207,526** 点。质检 **通过**。

| 检查 | 结果 |
|------|------|
| 一次扫完整跨 | 满足 |
| 两侧立面 | 满足。桥面横向两个峰，点数接近 |
| 桥面连续 | 满足。每 40 m 都有数万点 |
| 主缆连续，含跨中 | 满足。`x ∈ [2620, 2820]` 且 `z > 168 m` 有 294,211 点。跨中每 20 m 仍有约 3 万个 `z > 170 m` 的点 |
| 主缆形状 | 上沿从塔侧约 195 m 平滑降到跨中约 177 m，再升起。跨中这段贴中线（横向中位约 5 m，高度标准差约 1 m），不是假面片 |
| 桥塔 | 两端到约 202 m，跨中没有假塔 |
| 网格 | 596,265 顶点，1,170,896 面 |

`z > 165 m` 共 1,482,070 点，`z > 175 m` 共 1,211,918 点。

## 图怎么读

先看 `看这个_整桥侧视.png`。

- 横轴 `x`：沿桥向，米。
- 纵轴 `z`：高度，米，向上为正。
- 颜色也是高度。
- 下面那条（约 125–145 m）是桥面和钢箱梁。
- 上面那条是主缆。
- 中间空隙是吊索区，点少。
- 两端竖条是桥塔。
- 红线是 `z > 140 m` 的中位数，会被下层结构拉低，**不是主缆上沿**。跨中红线掉到约 157 m，不代表主缆断了。

## 文件

本机（不进 git，`artifacts/` 已忽略）：

`artifacts/models/bridge_humen_001/`

| 文件 | 内容 |
|------|------|
| `看这个_整桥侧视.png` | 整桥侧视 |
| `看这个_中段侧视.png` | 跨中 `x ∈ [2600, 2800]` |
| `看这个_整桥网格.png` | 网格俯视和侧视 |
| `看这个_整桥QC.png` | 俯视 + 侧视 |
| `看这个_整桥点云.ply` | v6 点云，151 MB |
| `FULL_BRIDGE_complete.ply` | 与上一份相同 |
| `bridge_mesh.ply` | 网格，62 MB |
| `README_看这里.json` | 点数摘要 |

84 上同一套在 `~/Projects/aerial-inspect/artifacts/bridge_humen_001/full_bridge_complete/`。原始扫描在 `deck_scan_complete_v6/complete_points.ply`。84 磁盘很满，不要把 84 当长期存放处。

## 已知限制

靠近两座塔的下层缆（约 150–165 m）偏稀。那些高度大多进了更高的主缆层。桥面和主缆在塔附近是满的。

网格来自泊松重建加密度裁剪，是预览，不是测绘级模型。没有 RTK，不承诺厘米精度。

## 复现

84 上 Avant `HumenCorridor` 已在 41463 监听：

```bash
cd ~/Projects/aerial-inspect
# 默认读 configs/sim/humen_centerline.json（主航道桥面中线）
bash scripts/run_complete_v6.sh
```

Mac 上同步代码、拉回图和点云：

```bash
source scripts/remote_84.env
bash scripts/sync_to_84.sh
bash scripts/pull_from_84.sh bridge_humen_001
```

公网 SSH 见 [SSH_84_PUBLIC.md](SSH_84_PUBLIC.md)。

## 代码入口

| 路径 | 作用 |
|------|------|
| `scripts/fly_continuous_complete.py` | v6 整桥扫描 |
| `scripts/run_complete_v6.sh` | 扫描、过滤、质检、网格 |
| `scripts/finalize_full_bridge.py` | 中线走廊过滤和侧视图 |
| `scripts/airsim_pose_utils.py` | 位姿与 look-at |
| `scripts/mesh_bridge_from_cloud.py` | 从点云出网格 |
| `aerial_inspect/survey/` | 立面航迹、样条、视点（任务规划，不是这次深度扫描本身） |

更早的补洞脚本（`fly_continuous_cable_*.py`、`fly_deck_*.py`、`run_complete_v3.sh`）留在仓库里作记录。归档结果只用 v6。

## 还没做的

相对 [PRD.md](PRD.md) 的 M1–M3：语义搜索、抵近、环绕影像、COLMAP 曾跑过，但旧配置没有严格沿主航道走（中心钉在西塔、桥轴 +26°、立面长度只有约 264 m），稀疏重建只有几百点。规划已改成 midspan + 桥轴 −30.7° + 整跨约 838 m。下一步应用新航点重跑环绕与 COLMAP；深度扫描仍沿 `configs/sim/humen_centerline.json`。
