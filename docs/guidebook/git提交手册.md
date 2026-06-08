# Git 提交手册

本文档用于说明 HazardWalker 团队最常用的 Git 操作。新成员先按这里做，不需要一开始学习复杂命令。

## 1. 分支规则

```text
main                         稳定版本，只放可演示、可提交版本
dev                          日常集成版本，各功能分支最终先合并到这里
feature/offline-algorithm-tests 当前最小 demo 和离线算法测试分支
feature/platform             平台与仿真开发
feature/nav                  导航探索开发
feature/perception           感知定位开发
feature/decision             决策状态机开发
feature/test                 测试脚本和指标统计开发
docs/report                  文档、报告、答辩材料开发
fix/xxx                      Bug 修复
```

当前最小 demo 相关修改先提交到 `feature/offline-algorithm-tests`，后续通过 PR 合并到 `dev`。当 `dev` 上形成稳定可演示版本后，再由负责人合并到 `main`。

## 2. 每天开始前先同步

每天写代码前先拉取更新，避免在过期代码上开发。

```powershell
git fetch origin
git switch dev
git pull origin dev
```

如果你正在自己的功能分支开发，再把最新 `dev` 合进来：

```powershell
git switch 你的分支名
git merge dev
```

如果你做的是当前最小 demo：

```powershell
git switch feature/offline-algorithm-tests
git pull origin feature/offline-algorithm-tests
git merge dev
```

## 3. 开始一个新任务

从 `dev` 创建自己的分支：

```powershell
git switch dev
git pull origin dev
git switch -c feature/你的任务名
```

分支名建议：

```text
feature/platform-gazebo-adapter
feature/nav-frontier
feature/perception-localize-hazard
feature/decision-state-machine
feature/test-metrics
docs/update-team-plan
fix/nav-return-state
```

## 4. 提交前检查

提交前先确认自己在正确分支，不要在 `main` 或 `dev` 上直接改。

```powershell
git branch
git status
git diff
```

至少运行离线测试：

```powershell
python scripts/run_offline_tests.py
```

如果改了 ROS 2 节点，并且本机有 ROS 2 环境，再运行：

```powershell
./scripts/build.sh
```

同时检查 README 和代码可读性：

- 修改项目总体结构、运行方式、公共约定或文档入口时，同步更新根目录 `README.md`。
- 修改某个 ROS 2 包、脚本、配置、测试或文档目录时，同步更新对应目录的 `README.md`。
- 如果本次修改不需要改 README，在提交说明或 PR 中写明原因。
- 新增代码文件时，在文件首部写明本文件作用，例如所属小组、文件职责、当前实现边界和验证方式。
- 重要模块、重要类和重要函数需要有简明注释或 docstring，说明输入、输出、关键逻辑和后续扩展点。

## 5. 提交和推送

只添加本次任务相关文件：

```powershell
git add 文件路径
git commit -m "type(scope): 简短说明"
git push -u origin 当前分支名
```

常用提交类型：

```text
feat      新功能
fix       修复问题
docs      文档修改
test      测试相关
chore     脚本、配置、项目结构
refactor  重构
```

示例：

```powershell
git commit -m "feat(nav): add frontier detector"
git commit -m "docs: update team task plan"
git commit -m "test: add result metric checks"
```

不要使用：

```text
修改
update
最终版
111
```

## 6. 提交 PR

推送后在 GitHub 创建 Pull Request。

常规合并方向：

```text
feature/offline-algorithm-tests -> dev
feature/* -> dev
docs/* -> dev
fix/* -> dev
dev -> main
```

PR 合并到 `dev` 时优先使用 `Squash and merge`，减少零散提交记录。`main` 和 `dev` 不允许随便强推。

## 7. 处理冲突

如果 GitHub 提示有冲突，先在本地同步 `dev`：

```powershell
git fetch origin
git switch 你的分支名
git merge origin/dev
```

手动处理冲突文件后：

```powershell
git add 冲突文件路径
git commit
git push
```

如果不确定怎么处理冲突，先不要乱删内容，截图或复制 `git status` 给负责人。

## 8. 不要提交的文件

不要提交大型文件、缓存和构建产物：

```text
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
__pycache__/
*.bag
*.db3
*.mcap
*.mp4
*.avi
*.zip
*.pt
*.onnx
*.engine
.vscode/settings.json
```

模型权重、rosbag、视频、训练数据放网盘或其他存储，只在文档中写下载方式。

## 9. 常用补救命令

取消暂存，但保留文件修改：

```powershell
git restore --staged 文件路径
```

丢弃某个文件的本地修改：

```powershell
git restore 文件路径
```

临时保存当前修改，方便切分支：

```powershell
git stash push -m "临时说明"
git stash pop
```

撤销已经进入公共分支的提交，优先用：

```powershell
git revert 提交ID
```

不要在 `main` 或 `dev` 上随便使用 `reset --hard` 或 `push --force`。
