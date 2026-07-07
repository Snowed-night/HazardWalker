# HazardWalker SSH 连接指南

更新时间：2026-06-16  
适用对象：HazardWalker 全队成员  
目标机器：`hxbl`

> 本文只整理 SSH 终端连接、免密登录、SSH 配置模板和公网 RDP 隧道。主力机环境信息请见《HazardWalker主力机环境介绍.md》。

## 1. 连接方式速查

推荐优先使用 Tailscale 私网；无法使用 Tailscale 时，使用公网反向 SSH 入口。

```text
Tailscale SSH：ssh <你的账号>@100.102.91.31
公网 SSH：     ssh -p 6002 <你的账号>@47.98.141.33
```

示例：

```bash
ssh hazard_platform@100.102.91.31
ssh -p 6002 hazard_platform@47.98.141.33
```

账号使用本组 Linux 账号：

| 组别 | Linux 账号 |
| --- | --- |
| 平台组 | `hazard_platform` |
| 导航组 | `hazard_nav` |
| 感知组 | `hazard_perception` |
| 决策组 | `hazard_decision` |
| 测试组 | `hazard_test` |
| 文档组 | `hazard_docs` |

账号初始密码和后续变更以队内最新通知为准。不要共用他人账号。

## 2. 什么时候用哪种连接

| 场景 | 推荐方式 |
| --- | --- |
| 日常终端开发 | Tailscale SSH |
| 本机没有 Tailscale | 公网 SSH |
| 长时间运行测试或脚本 | SSH + `tmux` |
| RViz / Gazebo GUI | 远程桌面 |
| 公网远程桌面 | 先用 SSH 建立本地端口转发 |

无图形界面需求时优先使用 SSH。远程桌面适合 GUI 调试，不适合承载长期任务。

## 3. 首次 SSH 登录

Tailscale 路径：

```bash
ssh <你的账号>@100.102.91.31
```

公网路径：

```bash
ssh -p 6002 <你的账号>@47.98.141.33
```

首次登录后检查：

```bash
whoami
hostname
echo $ROS_DOMAIN_ID
ros2 --help | head -1
```

如果出现主机指纹确认提示，确认连接地址无误后输入 `yes`。

## 4. 配置 SSH 公钥免密登录

建议所有成员尽快配置 SSH 公钥。后续如关闭 SSH 密码登录，将只能通过 SSH key 连接。

### 4.1 在自己电脑生成密钥

如果本机还没有 SSH key，执行：

```bash
ssh-keygen -t ed25519 -C "你的昵称或邮箱"
```

一路回车即可，也可以自行设置 passphrase。

查看公钥：

```bash
cat ~/.ssh/id_ed25519.pub
```

Windows PowerShell 中通常也可使用：

```powershell
type $env:USERPROFILE\.ssh\id_ed25519.pub
```

### 4.2 提交公钥

将 `.pub` 文件中的整行公钥发给集成组或环境负责人，并注明：

- 姓名或昵称
- 所属组别
- 对应 Linux 账号
- 公钥用途：主力机 SSH 登录

公钥应写入主力机对应账号的：

```text
/home/<username>/.ssh/authorized_keys
```

权限要求：

```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys
```

### 4.3 验证免密登录

```bash
ssh <你的账号>@100.102.91.31
ssh -p 6002 <你的账号>@47.98.141.33
```

若设置了 passphrase，本机可能会要求输入密钥口令；这不是主力机账号密码。

## 5. 本机 SSH 配置模板

可将以下内容写入自己电脑的 SSH 配置文件。

| 系统 | 配置文件路径 |
| --- | --- |
| Linux / macOS | `~/.ssh/config` |
| Windows | `C:\Users\<你的用户名>\.ssh\config` |

把 `User` 改成本组账号：

```sshconfig
Host hxbl-ts
    HostName 100.102.91.31
    User hazard_platform
    # IdentityFile ~/.ssh/id_ed25519

Host hxbl
    HostName 47.98.141.33
    Port 6002
    User hazard_platform
    # IdentityFile ~/.ssh/id_ed25519

Host hxbl-rdp-tunnel
    HostName 47.98.141.33
    Port 6002
    User hazard_platform
    LocalForward 13389 127.0.0.1:3389
    # IdentityFile ~/.ssh/id_ed25519
```

配置后可直接使用：

```bash
ssh hxbl-ts
ssh hxbl
ssh hxbl-rdp-tunnel
```

说明：

- `hxbl-ts`：Tailscale 私网 SSH
- `hxbl`：公网 SSH
- `hxbl-rdp-tunnel`：公网远程桌面隧道
- 模板写在自己的电脑上，不是写在主力机上

## 6. 用 SSH 建立公网远程桌面隧道

如果没有 Tailscale，但需要通过公网使用远程桌面，需要先在自己电脑上建立 SSH 隧道。

```bash
ssh -p 6002 <你的账号>@47.98.141.33 -L 13389:127.0.0.1:3389
```

保持该终端窗口不关闭，然后在 RDP 客户端连接：

```text
127.0.0.1:13389
```

注意：

- 隧道命令必须在自己的电脑上执行
- 不要先 SSH 到主力机后再执行 `ssh -L`
- 如果 SSH 密码登录已关闭，建立隧道时也需要使用 SSH 公钥
- RDP 登录密码与 SSH 是否禁用密码登录是两套机制

如果已配置第 5 节模板，可直接运行：

```bash
ssh hxbl-rdp-tunnel
```

然后 RDP 连接：

```text
127.0.0.1:13389
```

## 7. SSH 安全策略

推荐目标状态：

```text
PubkeyAuthentication yes
PasswordAuthentication no
PermitRootLogin no
KbdInteractiveAuthentication no
```

实施顺序：

1. 第一阶段：建账号、收集并登记 SSH 公钥，临时允许密码登录。
2. 第二阶段：确认每个成员都能用 SSH key 登录后，关闭 SSH 密码登录。

关闭密码登录前必须保留一个已登录的管理员终端，避免配置错误后锁在机器外。

后续如需更严格控制，可使用 `AllowGroups` 限制允许 SSH 登录的用户组，但建议等账号和公钥稳定后再启用。

## 8. 长时间任务建议使用 tmux

SSH 会话断开后，普通前台进程可能中断。长时间测试、构建或运行脚本建议使用 `tmux`。

新建会话：

```bash
tmux new -s hw
```

断开但保留任务：

```text
Ctrl+b，然后按 d
```

重新进入：

```bash
tmux attach -t hw
```

查看会话：

```bash
tmux ls
```

## 9. 常见问题

**SSH 连接不上？**  
先确认账号、地址、端口是否正确。Tailscale 不通时改用公网 `ssh -p 6002 <账号>@47.98.141.33`。

**提示 Permission denied？**  
确认使用的是本组账号；如果已关闭密码登录，确认公钥已添加到对应账号。

**配置了公钥仍要求密码？**  
检查本机是否使用了正确私钥；也可在 SSH 配置中显式设置 `IdentityFile ~/.ssh/id_ed25519`。

**Windows 找不到 ssh 命令？**  
在 Windows 设置中安装 OpenSSH Client，或使用 Git Bash / Windows Terminal。

**连 `127.0.0.1:13389` 失败？**  
确认 `ssh -L` 隧道在自己电脑上运行且窗口未关闭。

**禁用 SSH 密码后远程桌面还能用吗？**  
可以。禁用 SSH 密码只影响 `ssh` 登录和 `ssh -L` 建隧道，RDP 网关与 Linux 桌面登录仍按远程桌面文档执行。

**登录后 `ros2` 找不到？**  
确认使用默认 zsh 登录，或执行 `source /opt/ros/jazzy/setup.zsh`。

## 10. 速查

```text
Tailscale SSH：
ssh <hazard_账号>@100.102.91.31

公网 SSH：
ssh -p 6002 <hazard_账号>@47.98.141.33

公网 RDP 隧道：
ssh -p 6002 <hazard_账号>@47.98.141.33 -L 13389:127.0.0.1:3389

RDP 客户端连接：
127.0.0.1:13389
```
