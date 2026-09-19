# 84 公网 SSH 连接说明（aerial-inspect 用）

> **给 aerial-inspect 对话 / 脚本维护者**：Mac 校外默认用本文；局域网用 `cursor-84-lan`。
> 最后核对：2026-09-16

## 主机信息

| 项 | 值 |
|----|-----|
| 内网 IP | `10.229.20.84` |
| 用户 | `ubantu` |
| GPU | RTX 4090 D |
| 仓库路径 | `~/Projects/aerial-inspect` |
| AirSim RPC（OpenFly） | `10.229.20.84:41451` |
| AirSim RPC（虎门） | `10.229.20.84:41463` |

## 公网架构

```
Mac
  → cloudflared Access SSH
  → ssh-84.david-x.com
  → Cloudflare Named Tunnel（跑在 10.229.20.125）
  → 10.229.20.84:22 (ubantu)
```

备用路径（经 125 跳转，不依赖 ssh-84 Access 应用）：

```
Mac → ssh-125.david-x.com (Cloudflare) → 125 → 10.229.20.84:22
```

## Mac 连接方式

`~/.ssh/config` 应包含（由 `cursor-web-bridge` 维护）：

```sshconfig
Host cursor-84-public
  HostName ssh-84.david-x.com
  User ubantu
  IdentityFile ~/.ssh/cursor_webbridge_125
  IdentitiesOnly yes
  ProxyCommand ~/Projects/cursor-web-bridge/bin/cloudflared access ssh --hostname %h

Host cursor-84-via-125
  HostName 10.229.20.84
  User ubantu
  ProxyJump cursor-125-public

Host cursor-84-lan
  HostName 10.229.20.84
  User ubantu
```

| 场景 | 命令 |
|------|------|
| **校外公网（推荐）** | `ssh cursor-84-public` |
| **公网备用** | `ssh cursor-84-via-125` |
| **公司局域网** | `ssh cursor-84-lan` |

### 首次公网直连

Cloudflare Zero Trust 需手动建 Access 应用（API token 无写权限）：

1. https://one.dash.cloudflare.com/ → **Access** → **Applications** → **Self-hosted**
2. Application name: `ssh-84`
3. Public hostname: `ssh-84.david-x.com`
4. Session: 24 hours
5. Policy **Allow**：仅加入需要登录的账号（不要把个人邮箱或手机号写进公开仓库）

本机登录：

```bash
~/Projects/cursor-web-bridge/bin/cloudflared access login ssh-84.david-x.com
ssh cursor-84-public
```

## 隧道配置核对（2026-09-16）

`cursor-web-bridge/config/cloudflared.yml` **已含 84 ingress**：

```yaml
- hostname: ssh-84.david-x.com
  service: ssh://10.229.20.84:22
```

维护脚本（在能连 125 时于 Mac 执行）：

```bash
cd ~/Projects/cursor-web-bridge
./scripts/setup-ssh-84-access.sh
```

若 125 SSH 不可达但有 `CLOUDFLARE_API_TOKEN`：

```bash
INCLUDE_SSH=1 INCLUDE_SSH110=1 INCLUDE_SSH84=1 ./scripts/push-tunnel-config-api.sh
```

## aerial-inspect 脚本用法

```bash
cd ~/Projects/aerial-inspect
source scripts/remote_84.env    # 默认 AERIAL_SSH_HOST=cursor-84-public

# 同步代码到 84
bash scripts/sync_to_84.sh

# 在 84 跑任务（例：虎门采图）
bash scripts/run_humen_capture_on_84.sh

# 拉回产物
bash scripts/pull_from_84.sh bridge_humen_001
```

`remote_84.env` 核心变量：

| 变量 | 默认 | 说明 |
|------|------|------|
| `AERIAL_SSH_HOST` | `cursor-84-public` | 公网 SSH（`~/.ssh/config` + cloudflared Access） |
| `AERIAL_RSYNC_REMOTE` | 同 `AERIAL_SSH_HOST` | rsync 远端别名 |
| `AERIAL_RSYNC_SSH` | `ssh` | rsync 传输（走 SSH config ProxyCommand） |
| `AERIAL_SSH_TARGET` | `ubantu@10.229.20.84` | 仅局域网直连时用 |
| `AIRSIM_PUBLIC_HOST` | `10.229.20.84` | Mac 直连 AirSim |

备用（经 125 跳转，不依赖 ssh-84 Access）：

```bash
export AERIAL_SSH_HOST=cursor-84-via-125
export AERIAL_RSYNC_REMOTE=cursor-84-via-125
```

## 连通性自检

```bash
# DNS
dig +short ssh-84.david-x.com

# Access token（应返回 JWT，而非 "failed to find Access application"）
~/Projects/cursor-web-bridge/bin/cloudflared access token -app=ssh-84.david-x.com

# SSH
ssh -o ConnectTimeout=20 cursor-84-public 'hostname && nvidia-smi -L'
```

## 已知问题（2026-09-16）

| 现象 | 原因 | 处理 |
|------|------|------|
| `failed to find Access application` | Zero Trust 未建 `ssh-84` 应用 | 按上文建 Access + `cloudflared access login` |
| `cursor-84-via-125` 超时 | `ssh-125.david-x.com` 网络不稳 | 重试或换网络；局域网用 `cursor-84-lan` |
| `cursor-84-lan` 超时 | 不在公司网 | 用公网方式 |
| AirSim 连不上 | renderer 未起或未绑 `0.0.0.0` | 84 上 `bash scripts/start_renderer_84.sh` |

## 相关文档

- 仿真任务总览：[SIM_84.md](SIM_84.md)
- 客户端维护：`~/Projects/cursor-web-bridge/docs/ssh-84-home-client.md`
