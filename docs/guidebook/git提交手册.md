# HazardWalker Git 提交手册

<<<<<<< Updated upstream
本文档用于规范 HazardWalker 项目的 Git 协作流程。目标是让新成员能快速上手，同时保证代码、文档和实验材料可追踪、可审核、可复现。

## 目录

1. [最快上手流程](#1-最快上手流程)
2. [基本原则和分支说明](#2-基本原则和分支说明)
3. [首次配置和克隆仓库](#3-首次配置和克隆仓库)
4. [日常开发流程](#4-日常开发流程)
5. [提交信息规范](#5-提交信息规范)
6. [Pull Request 提交和审核](#6-pull-request-提交和审核)
7. [小改动如何处理](#7-小改动如何处理)
8. [常见撤回和修改操作](#8-常见撤回和修改操作)
9. [分支管理](#9-分支管理)
10. [合并冲突处理](#10-合并冲突处理)
11. [临时保存工作区](#11-临时保存工作区)
12. [不要提交的文件](#12-不要提交的文件)
13. [已有本地项目如何绑定远程仓库](#13-已有本地项目如何绑定远程仓库)

## 1. 最快上手流程

新成员第一次参与项目，只需要先掌握这一节。

### 1.1 克隆仓库

```powershell
git clone https://github.com/组织名或用户名/HazardWalker.git  # 克隆远程仓库到本地
cd HazardWalker                                               # 进入项目目录
code .                                                        # 用 VS Code 打开项目
```

### 1.2 配置个人身份

首先创建自己的github账号

```powershell
git config --global user.name "你的名字"                       # 设置提交时显示的名字
git config --global user.email "你的邮箱"                       # 设置提交时显示的邮箱
git config --global --list                                     # 查看当前 Git 全局配置
```

### 1.3 创建自己的开发分支

```powershell
git switch dev                                                 # 切到日常集成分支 dev
git pull origin dev                                            # 拉取远程 dev 最新内容
git switch -c feature/你的任务名                               # 从 dev 创建自己的功能分支
```

当前已经存在分支名：

```text
main                 稳定版本，只放可演示、可提交版本
dev                  日常集成版本
feature/platform     平台与仿真开发
feature/nav          导航探索开发
feature/perception   感知定位开发
feature/test         测试脚本开发
docs/report          文档材料开发
```

### 1.4 提交代码或文档

```powershell
git status                                                     # 查看当前修改了哪些文件
git add 文件路径                                               # 暂存本次要提交的文件
git commit -m "docs: update git workflow guide"                # 创建本地提交
git push -u origin feature/你的任务名                          # 第一次推送当前分支到远程
```

如果不是第一次推送当前分支，直接：

```powershell
git push                                                       # 推送当前分支到远程
```

注意：
你应该拉取dev分支，这是日常集成版本
但是你应该在 feature/platform 上修改和提交相关模块修改
不是直接提交到 dev
更不是提交到 main

### 1.5 创建 Pull Request

推送后，在 GitHub/Gitee 网页创建合并申请：

```text
feature/你的任务名 -> dev
```

填写仓库自动生成的 PR 模板，等待审核通过后再合并。

## 2. 基本原则和分支说明

### 2.1 基本原则
=======
本文档用于规范 HazardWalker 项目的 Git 协作流程。目标是让新成员能快速上手，同时保证代码、文档、实验结果的版本管理清晰可追溯。

## 目录

1. [先记住这几条](#1-先记住这几条)
2. [新成员快速上手](#2-新成员快速上手)
3. [日常开发流程](#3-日常开发流程)
4. [提交信息规范](#4-提交信息规范)
5. [小改动如何处理](#5-小改动如何处理)
6. [取消暂存、撤回修改和删除文件](#6-取消暂存撤回修改和删除文件)
7. [分支管理](#7-分支管理)
8. [Pull Request 使用与审核](#8-pull-request-使用与审核)
9. [合并冲突处理](#9-合并冲突处理)
10. [临时保存工作区](#10-临时保存工作区)
11. [首次创建或绑定仓库](#11-首次创建或绑定仓库)
12. [不要提交的文件](#12-不要提交的文件)

## 1. 先记住这几条
>>>>>>> Stashed changes

- 不要直接在 `main` 分支上开发。
- 日常开发从 `dev` 创建自己的功能分支。
- 真正保存版本的是 `commit`，不是 `add`。
<<<<<<< Updated upstream
- 所有功能分支通过 Pull Request 合并到 `dev`。
- 只有稳定、可演示、可提交的版本才能从 `dev` 合并到 `main`。
- 公共分支不要随意使用 `reset --hard`、`push --force` 等危险操作。

### 2.2 分支结构
=======
- 提交前先 `git status`，确认当前分支和文件列表。
- 功能分支通过 Pull Request 合并到 `dev`。
- 稳定、可演示、可提交的版本才从 `dev` 合并到 `main`。
- 公共分支不要随便 `reset --hard`，不要随便 `push --force`。

当前分支结构：
>>>>>>> Stashed changes

```text
main                 稳定版本，只放可演示、可提交版本
dev                  日常集成版本
feature/platform     平台与仿真开发
feature/nav          导航探索开发
feature/perception   感知定位开发
feature/test         测试脚本开发
docs/report          文档材料开发
```

<<<<<<< Updated upstream
### 2.3 查看当前分支

```powershell
git branch                                                     # 查看本地分支，带 * 的是当前分支
git branch -r                                                  # 查看远程分支
git branch -a                                                  # 查看本地和远程全部分支
git status                                                     # 查看当前分支和工作区状态
```

## 3. 首次配置和克隆仓库

### 3.1 HTTPS 克隆
=======
## 2. 新成员快速上手

### 2.1 克隆仓库

先在 PowerShell 进入你想放项目的位置，例如桌面：
>>>>>>> Stashed changes

```powershell
cd C:\Users\你的用户名\OneDrive\Desktop                         # 进入桌面目录
git clone https://github.com/组织名或用户名/HazardWalker.git     # 克隆远程仓库到本地
cd HazardWalker                                                # 进入项目目录
code .                                                         # 用 VS Code 打开项目
```

### 3.2 SSH 克隆

如果已经配置 SSH key，可以使用：

```powershell
git clone git@github.com:组织名或用户名/HazardWalker.git        # 使用 SSH 克隆仓库
cd HazardWalker                                                # 进入项目目录
code .                                                         # 用 VS Code 打开项目
```

<<<<<<< Updated upstream
### 3.3 修改 Git 身份
=======
### 2.2 配置个人身份

首次使用 Git 时配置姓名和邮箱：

```powershell
git config --global user.name "你的名字"                        # 设置提交时显示的名字
git config --global user.email "你的邮箱"                        # 设置提交时显示的邮箱
git config --global --list                                      # 查看当前 Git 全局配置
```

如果写错了，可以重新设置：
>>>>>>> Stashed changes

```powershell
git config --global user.name "新名字"                          # 修改提交姓名
git config --global user.email "新邮箱"                          # 修改提交邮箱
```

<<<<<<< Updated upstream
## 4. 日常开发流程

### 4.1 每天开始开发前

每次开始开发前，先同步最新 `dev`，再回到自己的分支。

```powershell
git switch dev                                                 # 切换到 dev
git pull origin dev                                            # 拉取远程 dev 最新内容
git switch feature/你的分支名                                  # 切回自己的开发分支
git merge dev                                                  # 将最新 dev 合并到自己的分支
```

如果还没有自己的分支：

```powershell
git switch dev                                                 # 先切到 dev
git pull origin dev                                            # 同步最新 dev
git switch -c feature/你的任务名                               # 从 dev 创建自己的功能分支
```

### 4.2 开发完成后

```powershell
git status                                                     # 查看修改文件
git add 文件路径                                               # 暂存本次真正要提交的文件
git commit -m "feat(nav): add frontier exploration"            # 创建本地提交
git push                                                       # 推送当前分支到远程
```

第一次推送新分支时：

```powershell
git push -u origin feature/你的分支名                          # 第一次推送并建立跟踪关系
```

### 4.3 什么时候可以用 `git add .`

如果你确认当前所有修改都属于同一个提交主题，可以使用：

```powershell
git add .                                                      # 暂存当前目录下所有修改
```

提交前建议先检查：

```powershell
git status                                                     # 确认没有误提交大文件或无关文件
```

如果发现加错文件：

```powershell
git restore --staged 文件路径                                  # 从暂存区移除某个文件
git restore --staged .                                         # 取消所有暂存
```

## 5. 提交信息规范

### 5.1 格式

```text
type(scope): message
```

常用类型：

```text
feat      新功能
fix       修复 Bug
docs      文档修改
test      测试相关
refactor  重构
chore     环境、脚本、配置
perf      性能优化
```

### 5.2 示例

```powershell
git commit -m "feat(perception): add HSV red ball detector"     # 新增红球检测功能
git commit -m "fix(platform): correct lidar frame transform"    # 修复雷达坐标系变换错误
git commit -m "docs: update interface specification"            # 更新接口文档
git commit -m "test: add exploration metric logger"             # 新增探索指标记录脚本
```

不要使用：

```text
修改
update
最终版
111
```

## 6. Pull Request 提交和审核

### 6.1 PR 合并方向

所有功能分支都通过 Pull Request 合并：

```text
feature/* -> dev
docs/* -> dev
fix/* -> dev
dev -> main
```

### 6.2 PR 模板

本仓库提供 PR 模板：

```text
.github/pull_request_template.md
```

在 GitHub 创建 Pull Request 时，描述框会自动填充模板。提交人需要填写：

```text
修改人
所属小组
修改概述
实现方式
影响范围
是否修改接口
验证方式
风险与待办
审核建议
```

### 6.3 PR 标题示例

```text
chore: initialize project structure
docs: update git workflow guide
feat(perception): add HSV red ball detector
fix(nav): handle unreachable frontier goal
```

### 6.4 PR 内容示例

```text
修改人：
- 江晨

所属小组：
- 文档答辩组

修改概述：
- 更新 Git 提交流程说明
- 补充小改动如何合并进已有提交
- 补充 PR 模板使用方式

实现方式：
- 在 docs/guidebook/git提交手册.md 中重排手册结构
- 将快速上手流程放在文档前部
- 保持 PowerShell 命令后带中文注释

影响范围：
- 文档 / 报告 / PPT

是否修改接口：
- 否

验证方式：
- 已本地检查 Markdown 内容和命令说明

风险与待办：
- 后续可根据团队实际操作补充截图

审核建议：
- 文档答辩组
- 技术总负责人
```

### 6.5 审核规则

- `main` 禁止直接 push。
- `dev` 尽量禁止直接 push。
- 功能分支合并到 `dev` 前，至少 1 人审核。
- 改接口、话题、坐标系、消息格式时，必须由技术负责人审核。
- 改启动脚本、测试脚本、依赖配置时，必须由系统集成负责人审核。

按修改类型选择审核人：

```text
平台、仿真、话题、TF、控制接口：平台与仿真组 + 技术总负责人
导航、SLAM、探索、返航：导航探索组 + 技术总负责人
视觉、点云、识别、定位：感知定位组 + 技术总负责人
启动脚本、配置、测试统计：系统集成测试组
报告、PPT、手册、会议纪要：文档答辩组
```

### 6.6 合并前检查

文档类 PR 至少检查：

```text
标题层级是否清晰
命令是否能看懂
是否有明显错别字
是否误提交无关文件
```

代码类 PR 至少检查：

```text
是否能编译
是否能运行最小 demo
是否改了接口文档
是否影响其他模块
是否引入大文件或本地配置
```

本地检查命令：

```powershell
git status                                                     # 确认工作区状态
git pull origin dev                                            # 确认基于最新 dev
colcon build                                                   # ROS 项目构建检查，有 ROS 环境时执行
```

### 6.7 推荐合并方式

```text
feature/* -> dev：Squash and merge
dev -> main：Merge commit 或 Squash and merge，按团队约定执行
```

`Squash and merge` 可以把一个 PR 里的多个零散提交压成一个提交，让 `dev` 分支历史更干净。

## 7. 小改动如何处理

小改动包括：

```text
文档里补几句话
修正错别字
调整 README 的一两处说明
补充注释
修改很小的配置说明
```

### 7.1 还没有提交

直接和本次相关修改一起提交。

```powershell
git status                                                     # 查看当前修改
git add docs/guidebook/git提交手册.md                          # 暂存文档修改
git commit -m "docs: update git workflow guide"                # 提交为一次文档更新
```

### 7.2 已提交但还没 push

把小改动合并进上一个提交：

```powershell
git add docs/guidebook/git提交手册.md                          # 暂存补充的小改动
git commit --amend --no-edit                                   # 合并进上一个提交，不修改提交说明
```

如果想修改提交说明：

```powershell
git add docs/guidebook/git提交手册.md                          # 暂存补充的小改动
git commit --amend -m "docs: update git workflow guide"         # 合并进上一个提交，并修改提交说明
```

### 7.3 已 push 到自己的功能分支

如果该分支只有你自己使用，可以：

```powershell
git add docs/guidebook/git提交手册.md                          # 暂存补充的小改动
git commit --amend --no-edit                                   # 合并进上一个提交
git push --force-with-lease                                    # 安全强推自己的功能分支
```

不要对 `main` 或 `dev` 这样做。

### 7.4 已经有多个零散小提交

新手优先在 PR 页面使用：

```text
Squash and merge
```

熟练成员可以本地整理提交：

```powershell
git log --oneline                                              # 查看最近提交
git rebase -i HEAD~3                                           # 整理最近 3 个提交，谨慎使用
```

### 7.5 已合并到公共分支

不要重写公共分支历史。重新开小分支修正：

```powershell
git switch dev                                                 # 切到 dev
git pull origin dev                                            # 同步最新 dev
git switch -c docs/fix-typo                                    # 新建文档修正分支
git add docs/guidebook/git提交手册.md                          # 暂存修改
git commit -m "docs: fix typo in git guide"                    # 提交文档修正
git push -u origin docs/fix-typo                               # 推送分支并提交 PR
```

## 8. 常见撤回和修改操作

### 8.1 取消暂存

已经 `git add`，但不想放进本次提交：

```powershell
git restore --staged 文件路径                                  # 取消某个文件的暂存，保留文件修改
git restore --staged .                                         # 取消所有已暂存文件
```

### 8.2 丢弃本地修改

危险：会丢掉未提交修改。

```powershell
git restore 文件路径                                           # 丢弃某个文件的本地修改
git restore .                                                  # 丢弃所有已跟踪文件的本地修改
```

### 8.3 删除未跟踪文件

先预览：

```powershell
git clean -n                                                   # 预览将要删除的未跟踪文件
```

确认后删除：

```powershell
git clean -f                                                   # 删除未跟踪文件
git clean -fd                                                  # 删除未跟踪文件和文件夹
```

### 8.4 修改最近一次提交文字

未 push 时：

```powershell
git commit --amend -m "新的提交说明"                           # 修改最近一次 commit 的提交信息
```

已 push 到个人分支时：

```powershell
git commit --amend -m "新的提交说明"                           # 修改最近一次 commit 的提交信息
git push --force-with-lease                                    # 安全强推自己的功能分支
```

### 8.5 撤回最近一次提交

```powershell
git reset --soft HEAD~1                                        # 撤销最近一次 commit，保留暂存状态
git reset --mixed HEAD~1                                       # 撤销最近一次 commit，保留文件修改但取消暂存
git reset --hard HEAD~1                                        # 撤销最近一次 commit，并彻底丢弃修改，谨慎使用
```

### 8.6 撤销已经推送的公共提交

公共分支推荐用 `revert`：

```powershell
git log --oneline                                              # 查看提交历史和提交 ID
git revert 提交ID                                              # 生成反向提交，撤销指定提交
git push                                                       # 推送撤销结果
```

撤销最近一次提交：

```powershell
git revert HEAD                                                # 撤销最近一次提交，适合公共分支
```

## 9. 分支管理

### 9.1 创建和推送分支

```powershell
git switch -c feature/perception-red-ball                      # 基于当前分支创建并切换到新分支
git push -u origin feature/perception-red-ball                 # 第一次推送新分支并建立跟踪关系
```

### 9.2 删除本地分支

删除前先切到其他分支：

```powershell
git switch dev                                                 # 切到 dev，避免删除当前分支
git branch -d feature/旧分支名                                 # 删除已经合并的本地分支
git branch -D feature/旧分支名                                 # 强制删除未合并的本地分支，谨慎使用
```

### 9.3 删除远程分支

```powershell
git push origin --delete feature/旧分支名                      # 删除远程分支
```

### 9.4 清理失效远程分支记录

```powershell
git fetch --prune                                              # 清理本地已失效的远程分支引用
```

### 9.5 重命名当前分支

```powershell
git branch -m 新分支名                                         # 将当前分支重命名
git push -u origin 新分支名                                    # 推送新分支名到远程
git push origin --delete 旧分支名                              # 删除远程旧分支
```

## 10. 合并冲突处理

### 10.1 合并最新 dev

```powershell
git switch feature/你的分支名                                  # 切到自己的功能分支
git fetch origin                                               # 获取远程最新信息
git merge origin/dev                                           # 将远程 dev 合并到当前分支
```

### 10.2 处理冲突

查看冲突文件：

```powershell
git status                                                     # 查看哪些文件发生冲突
```

冲突文件通常包含：

```text
<<<<<<< HEAD
你当前分支的代码
=======
被合并分支的代码
>>>>>>> origin/dev
```

手动处理冲突后：

```powershell
git add 冲突文件路径                                           # 标记冲突已解决
git commit                                                     # 完成合并提交
git push                                                       # 推送解决冲突后的分支
```

### 10.3 放弃本次合并

```powershell
git merge --abort                                              # 放弃本次 merge，回到合并前状态
```

## 11. 临时保存工作区

如果代码改到一半，但需要切换分支：

```powershell
git stash push -m "临时保存导航调参"                           # 临时保存当前未提交修改
git stash list                                                 # 查看所有临时保存记录
git stash pop                                                  # 恢复最近一次 stash，并从列表中删除
git stash apply                                                # 恢复最近一次 stash，但保留在列表中
git stash drop stash@{0}                                       # 删除指定 stash
git stash clear                                                # 清空所有 stash，谨慎使用
```

## 12. 不要提交的文件

以下文件不要直接提交到 Git：

```text
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
*.bag
*.db3
*.mp4
*.avi
*.zip
*.pt
*.onnx
*.engine
__pycache__/
.vscode/settings.json
```

大文件如 rosbag、视频、模型权重应放网盘或 Git LFS，并在文档中记录下载方式。

## 13. 已有本地项目如何绑定远程仓库

如果本地已经有项目文件，但还不是 Git 仓库：
=======
### 2.3 第一次创建自己的开发分支
>>>>>>> Stashed changes

```powershell
git switch dev                                                  # 切换到 dev 分支
git pull origin dev                                             # 拉取远程 dev 最新代码
git switch -c feature/你的任务名                                # 从 dev 创建自己的功能分支
git push -u origin feature/你的任务名                           # 第一次推送分支并建立跟踪关系
```

分支命名示例：

```text
feature/platform-gazebo-scene
feature/nav-frontier
feature/perception-red-ball
feature/test-metrics
docs/git-guide
fix/tf-frame-error
```

## 3. 日常开发流程

### 3.1 每天开始开发前

先同步最新 `dev`，再回到自己的分支。

```powershell
git switch dev                                                  # 切换到日常集成分支 dev
git pull origin dev                                             # 拉取远程 dev 的最新内容
git switch feature/你的分支名                                   # 切回自己的开发分支
git merge dev                                                   # 将最新 dev 合并到自己的分支
```

### 3.2 开发完成后提交

```powershell
git status                                                      # 查看当前分支和修改文件
git add 文件路径                                                # 只暂存本次真正要提交的文件
git commit -m "feat(nav): add frontier exploration"             # 创建一次本地提交
git push                                                        # 推送当前分支到远程
```
<<<<<<< Updated upstream
=======

如果本次修改都是同一件事，也可以使用：

```powershell
git add .                                                       # 暂存当前目录下所有修改，确认无无关文件后再用
git commit -m "chore: initialize project structure"             # 提交本次修改
git push                                                        # 推送到远程
```

### 3.3 推送后创建 PR

在 GitHub 或 Gitee 页面创建 Pull Request：

```text
feature/你的分支名 -> dev
```

如果是阶段稳定版本，再由负责人创建：

```text
dev -> main
```

## 4. 提交信息规范

提交信息格式：

```text
type(scope): message
```

常用类型：

```text
feat      新功能
fix       修复 Bug
docs      文档修改
test      测试相关
refactor  重构
chore     环境、脚本、配置、项目结构
perf      性能优化
```

示例：

```powershell
git commit -m "feat(perception): add HSV red ball detector"     # 新增红球检测功能
git commit -m "fix(platform): correct lidar frame transform"    # 修复雷达坐标系变换错误
git commit -m "docs: update interface specification"            # 更新接口文档
git commit -m "test: add exploration metric logger"             # 新增探索指标记录脚本
git commit -m "chore: initialize project structure"             # 初始化项目结构
```

不要使用以下提交信息：

```text
修改
update
最终版
111
```

## 5. 小改动如何处理

小改动包括：

```text
文档里补几句话
修正错别字
调整 README 的一两处说明
补充注释
修改很小的配置说明
```

### 5.1 还没有提交

直接和本次相关修改一起提交。

```powershell
git status                                                      # 查看当前修改
git add docs/guidebook/git提交手册.md                           # 暂存这个文档修改
git commit -m "docs: update git workflow guide"                 # 提交为一次文档更新
```

### 5.2 已经提交，但还没有 push

用 `--amend` 把小改动合并进上一个提交。

```powershell
git add docs/guidebook/git提交手册.md                           # 暂存补充的小改动
git commit --amend --no-edit                                    # 合并进上一个提交，不修改提交说明
```

如果想顺便改提交说明：

```powershell
git add docs/guidebook/git提交手册.md                           # 暂存补充的小改动
git commit --amend -m "docs: update git workflow guide"         # 合并进上一个提交，并修改提交说明
```

### 5.3 已经 push 到自己的功能分支

如果这个分支只有你自己在用，可以继续 `amend`，再安全强推自己的功能分支。

```powershell
git add docs/guidebook/git提交手册.md                           # 暂存补充的小改动
git commit --amend --no-edit                                    # 合并进上一个提交
git push --force-with-lease                                     # 安全强推自己的功能分支
```

不要在 `main` 或 `dev` 上这样做。

### 5.4 已经有多个零散小提交

优先在 PR 合并时使用：

```text
Squash and merge
```

这样可以把多个小提交压成一个干净提交。

熟练成员也可以本地整理提交：

```powershell
git log --oneline                                               # 查看最近提交
git rebase -i HEAD~3                                            # 整理最近 3 个提交，谨慎使用
```

新手优先使用 PR 页面里的 `Squash and merge`。

### 5.5 已经合并到 dev 或 main

不要为了整理历史去重写公共分支。重新开一个小 PR 修正即可。

```powershell
git switch dev                                                  # 切到 dev
git pull origin dev                                             # 同步最新 dev
git switch -c docs/fix-typo                                     # 新建文档修正分支
git add docs/guidebook/git提交手册.md                           # 暂存修改
git commit -m "docs: fix typo in git guide"                     # 提交文档修正
git push -u origin docs/fix-typo                                # 推送分支并提交 PR
```

## 6. 取消暂存、撤回修改和删除文件

### 6.1 取消暂存

已经执行 `git add`，但不想把该文件放进本次提交：

```powershell
git restore --staged 文件路径                                   # 取消某个文件的暂存，但保留文件内容修改
git restore --staged .                                          # 取消所有已暂存文件
```

### 6.2 丢弃本地修改

危险：会丢掉未提交的修改。

```powershell
git restore 文件路径                                            # 丢弃某个文件的本地修改
git restore .                                                   # 丢弃当前目录下所有已跟踪文件的本地修改
```

### 6.3 删除未跟踪的新文件

先预览：

```powershell
git clean -n                                                    # 预览将要删除的未跟踪文件
```

确认后删除：

```powershell
git clean -f                                                    # 删除未跟踪文件
git clean -fd                                                   # 删除未跟踪文件和文件夹
```

### 6.4 撤回最近一次提交

保留代码修改：

```powershell
git reset --soft HEAD~1                                         # 撤销最近一次 commit，保留暂存状态
git reset --mixed HEAD~1                                        # 撤销最近一次 commit，保留文件修改但取消暂存
```

彻底丢弃代码修改：

```powershell
git reset --hard HEAD~1                                         # 撤销最近一次 commit，并丢弃对应修改，谨慎使用
```

### 6.5 撤销已经进入公共分支的提交

如果提交已经进入 `dev` 或 `main`，推荐用 `revert`。

```powershell
git log --oneline                                               # 查看提交历史和提交 ID
git revert 提交ID                                               # 生成一个反向提交，用于撤销指定提交
git push                                                        # 推送撤销结果
```

撤销最近一次提交：

```powershell
git revert HEAD                                                 # 撤销最近一次提交，适合公共分支
```

## 7. 分支管理

### 7.1 查看分支

```powershell
git branch                                                      # 查看本地分支，带 * 的是当前分支
git branch -r                                                   # 查看远程分支
git branch -a                                                   # 查看本地和远程全部分支
```

### 7.2 创建和推送分支

```powershell
git switch dev                                                  # 切换到 dev 分支
git pull origin dev                                             # 同步最新 dev
git switch -c feature/perception-red-ball                       # 基于 dev 创建并切换到新分支
git push -u origin feature/perception-red-ball                  # 第一次推送新分支并建立跟踪关系
```

### 7.3 删除本地分支

删除前先切到其他分支：

```powershell
git switch dev                                                  # 先切换到 dev，避免删除当前分支
git branch -d feature/旧分支名                                  # 删除已经合并的本地分支
git branch -D feature/旧分支名                                  # 强制删除未合并的本地分支，谨慎使用
```

### 7.4 删除远程分支

```powershell
git push origin --delete feature/旧分支名                       # 删除远程分支
```

### 7.5 清理失效远程分支记录

```powershell
git fetch --prune                                               # 清理本地已失效的远程分支引用
```

### 7.6 重命名当前分支

```powershell
git branch -m 新分支名                                          # 将当前分支重命名
git push -u origin 新分支名                                     # 推送新分支名到远程
git push origin --delete 旧分支名                               # 删除远程旧分支
```

## 8. Pull Request 使用与审核

本仓库已提供 PR 模板：

```text
.github/pull_request_template.md
```

在 GitHub 创建 Pull Request 时，描述框会自动填充模板。提交人按模板填写，审核人按模板检查。

### 8.1 PR 标题

PR 标题建议沿用提交规范：

```text
chore: initialize project structure
docs: update git workflow guide
feat(perception): add HSV red ball detector
fix(nav): handle unreachable frontier goal
```

### 8.2 PR 内容填写示例

```text
修改人：
- 江晨

所属小组：
- 文档答辩组

修改概述：
- 更新 Git 提交流程说明
- 补充小改动如何合并进已有提交
- 补充 PR 模板使用方式

实现方式：
- 在 docs/guidebook/git提交手册.md 中新增“小改动处理”和“PR 模板使用”章节
- 保持 PowerShell 命令后带中文注释，方便新手阅读

影响范围：
- 文档 / 报告 / PPT

是否修改接口：
- 否

验证方式：
- 已本地检查 Markdown 内容和命令说明

风险与待办：
- 后续根据团队实际 GitHub 操作再补充截图或示例

审核建议：
- 文档答辩组
- 技术总负责人
```

### 8.3 审核人选择

```text
平台、仿真、话题、TF、控制接口：平台与仿真组 + 技术总负责人
导航、SLAM、探索、返航：导航探索组 + 技术总负责人
视觉、点云、识别、定位：感知定位组 + 技术总负责人
启动脚本、配置、测试统计：系统集成测试组
报告、PPT、手册、会议纪要：文档答辩组
```

### 8.4 合并前检查

```powershell
git status                                                      # 确认没有遗漏文件
git pull origin dev                                             # 确认基于最新 dev
```

文档类 PR 至少检查：

```text
标题层级是否清晰
命令是否能看懂
是否有明显错别字
是否误提交了无关文件
```

代码类 PR 至少检查：

```text
是否能编译
是否能运行最小 demo
是否改了接口文档
是否影响其他模块
是否引入大文件或本地配置
```

如果是 ROS 项目，至少确认能构建：

```powershell
colcon build                                                    # 构建 ROS 工作空间，确认没有编译错误
```

### 8.5 合并方式

推荐：

```text
feature/* -> dev：Squash and merge
dev -> main：由负责人按阶段合并
```

`Squash and merge` 可以把功能分支上的多个小提交压成一个提交，保持 `dev` 的提交树清晰。

## 9. 合并冲突处理

### 9.1 将最新 dev 合并到当前分支

```powershell
git switch feature/你的分支名                                   # 切换到自己的功能分支
git fetch origin                                                # 获取远程最新信息
git merge origin/dev                                            # 将远程 dev 合并到当前分支
```

### 9.2 处理冲突

查看冲突文件：

```powershell
git status                                                      # 查看哪些文件发生冲突
```

冲突文件中通常会出现：

```text
<<<<<<< HEAD
你当前分支的代码
=======
被合并分支的代码
>>>>>>> origin/dev
```

手动处理冲突后：

```powershell
git add 冲突文件路径                                            # 标记冲突已解决
git commit                                                      # 完成合并提交
git push                                                        # 推送解决冲突后的分支
```

### 9.3 放弃本次合并

如果冲突太乱，不想继续合并：

```powershell
git merge --abort                                               # 放弃本次 merge，回到合并前状态
```

## 10. 临时保存工作区

如果代码改到一半，但需要切换分支，可以用 `stash` 临时保存。

```powershell
git stash push -m "临时保存导航调参"                            # 临时保存当前未提交修改
git stash list                                                  # 查看所有临时保存记录
git stash pop                                                   # 恢复最近一次 stash，并从列表中删除
git stash apply                                                 # 恢复最近一次 stash，但保留在列表中
git stash drop stash@{0}                                        # 删除指定 stash
git stash clear                                                 # 清空所有 stash，谨慎使用
```

## 11. 首次创建或绑定仓库

如果本地已经有项目文件，但还不是 Git 仓库，可以这样初始化。

```powershell
git init                                                        # 初始化当前目录为 Git 仓库
git branch -M main                                              # 将当前主分支命名为 main
git remote add origin https://github.com/组织名或用户名/HazardWalker.git  # 添加远程仓库地址
git remote -v                                                   # 检查远程仓库地址是否正确
git add .                                                       # 暂存当前所有文件
git commit -m "chore: initialize HazardWalker repository"        # 创建第一次提交
git push -u origin main                                         # 第一次推送 main，并建立跟踪关系
```

如果远程仓库已有 README，首次推送被拒绝：

```powershell
git pull origin main --allow-unrelated-histories                # 拉取远程已有内容，并允许合并两边历史
git push -u origin main                                         # 合并后再次推送
```

只有确认远程仓库没有重要内容时，才允许：

```powershell
git push -u origin main --force-with-lease                      # 用本地 main 覆盖远程 main，谨慎使用
```

## 12. 不要提交的文件

以下文件不要直接提交到 Git：

```text
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
*.bag
*.db3
*.mcap
*.mp4
*.avi
*.zip
*.pt
*.onnx
*.engine
__pycache__/
.vscode/settings.json
```

建议在 `.gitignore` 中排除这些内容。大文件如 rosbag、视频、模型权重应放网盘或 Git LFS，并在文档中记录下载方式。
>>>>>>> Stashed changes
