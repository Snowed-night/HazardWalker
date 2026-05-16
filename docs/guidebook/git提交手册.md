# Git 提交手册

本文档用于规范 HazardWalker 项目的 Git 协作流程。目标是让新成员能快速上手，同时保证代码、文档、实验结果都能稳定追踪、审核和回滚。

## 目录

1. [一些注意点](#1-一些注意点)
2. [5 分钟快速上手](#2-5-分钟快速上手)
3. [分支规则](#3-分支规则)
4. [日常开发流程](#4-日常开发流程)
5. [提交信息规范](#5-提交信息规范)
6. [Pull Request 提交流程](#6-pull-request-提交流程)
7. [PR 模板填写规范](#7-pr-模板填写规范)
8. [小改动怎么提交](#8-小改动怎么提交)
9. [撤回、修改和删除常用操作](#9-撤回修改和删除常用操作)
10. [分支管理](#10-分支管理)
11. [合并冲突处理](#11-合并冲突处理)
12. [临时保存工作区](#12-临时保存工作区)
13. [不要提交的文件](#13-不要提交的文件)
14. [首次建仓或绑定远程仓库](#14-首次建仓或绑定远程仓库)
15. [进阶整理提交](#15-进阶整理提交)

## 1. 一些注意点

- 不要直接在 `main` 上开发。
- 日常开发从 `dev` 拉新分支。
- 改完代码后按 `status -> add -> commit -> push -> PR` 走。
- 真正保存版本的是 `commit`，不是 `add`。
- 功能分支通过 Pull Request 合并到 `dev`。
- 稳定可演示版本才从 `dev` 合并到 `main`。
- 公共分支不要随便 `reset --hard`，不要随便 `push --force`。
- 同一件事的小改动尽量合并成一个提交，PR 合并时优先 `Squash and merge`。

## 2. 5 分钟快速上手

### 2.1 第一次下载项目

在桌面或你希望放项目的位置打开 PowerShell：

```powershell
git clone https://github.com/组织名或用户名/HazardWalker.git  # 克隆远程仓库到本地
cd HazardWalker                                               # 进入项目目录
code .                                                        # 用 VS Code 打开项目
```

如果使用 SSH：

```powershell
git clone git@github.com:组织名或用户名/HazardWalker.git       # 使用 SSH 克隆仓库
cd HazardWalker                                               # 进入项目目录
code .                                                        # 用 VS Code 打开项目
```

### 2.2 第一次配置身份

```powershell
git config --global user.name "你的名字"                       # 设置提交时显示的名字
git config --global user.email "你的邮箱"                       # 设置提交时显示的邮箱
git config --global --list                                     # 查看当前 Git 配置
```

如果写错了：

```powershell
git config --global user.name "新名字"                         # 修改提交姓名
git config --global user.email "新邮箱"                         # 修改提交邮箱
```

### 2.3 第一次创建自己的开发分支

```powershell
git switch dev                                                 # 切到日常集成分支
git pull origin dev                                            # 拉取最新 dev
git switch -c feature/你的任务名                               # 从 dev 创建自己的功能分支
```

示例：

```powershell
git switch -c docs/update-git-guide                            # 创建文档更新分支
git switch -c feature/perception-red-ball                      # 创建红球检测功能分支
```

### 2.4 改完后提交

```powershell
git status                                                     # 查看修改了哪些文件
git add 文件路径                                               # 暂存本次要提交的文件
git commit -m "docs: update git workflow guide"                # 提交修改
git push -u origin 当前分支名                                  # 第一次推送当前分支
```

后续同一个分支再次推送：

```powershell
git push                                                       # 推送当前分支的新提交
```

推送后，在 GitHub 上创建 PR：

```text
当前功能分支 -> dev
```

## 3. 分支规则

当前分支结构：

```text
main                    稳定版本，只放可演示、可提交版本
dev                     日常集成版本
feature/platform-xxx    平台与仿真开发
feature/nav-xxx         导航探索开发
feature/perception-xxx  感知定位开发
feature/test-xxx        测试脚本开发
docs/xxx                文档材料开发
fix/xxx                 Bug 修复
```

常用命令：

```powershell
git branch                                                     # 查看本地分支，带 * 的是当前分支
git branch -r                                                  # 查看远程分支
git branch -a                                                  # 查看本地和远程全部分支
git switch dev                                                 # 切换到 dev 分支
git switch -c feature/新分支名                                 # 基于当前分支创建并切换到新分支
git push -u origin feature/新分支名                            # 第一次推送新分支并建立跟踪关系
```

## 4. 日常开发流程

每次开始开发前，先同步最新 `dev`。

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
git switch -c feature/你的任务名                               # 从 dev 创建功能分支
```

开发完成后：

```powershell
git status                                                     # 检查当前修改
git add 文件路径                                               # 暂存本次真正要提交的文件
git commit -m "feat(nav): add frontier exploration"            # 创建一次本地提交
git push                                                       # 推送当前分支
```

如果这次修改都属于同一个主题，可以使用：

```powershell
git add .                                                      # 暂存当前目录下所有改动，提交前务必先看 git status
git commit -m "chore: initialize project structure"            # 提交项目结构初始化
git push                                                       # 推送当前分支
```

## 5. 提交信息规范

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

不要使用：

```text
修改
update
最终版
111
```

## 6. Pull Request 提交流程

所有功能分支都通过 Pull Request 合并：

```text
feature/* -> dev
docs/* -> dev
fix/* -> dev
dev -> main
```

推荐流程：

```powershell
git status                                                     # 确认本地修改都已提交
git push                                                       # 推送当前分支到远程
```

然后在 GitHub 页面创建 PR：

```text
base: dev
compare: 你的功能分支
```

合并规则：

- `main` 禁止直接 push。
- `dev` 尽量禁止直接 push。
- 功能分支合并到 `dev` 前，至少 1 人审核。
- 修改接口、话题、坐标系、消息格式时，必须由技术负责人审核。
- 修改启动脚本、测试脚本、依赖配置时，必须由系统集成负责人审核。
- 合并功能分支到 `dev` 时，推荐使用 `Squash and merge`，保持提交树干净。

## 7. PR 模板填写规范

仓库已提供 PR 模板：

```text
.github/pull_request_template.md
```

创建 PR 时，GitHub 会自动填充模板。提交人按模板填写即可。

PR 标题建议沿用提交规范：

```text
chore: initialize project structure
docs: update git workflow guide
feat(perception): add HSV red ball detector
fix(nav): handle unreachable frontier goal
```

PR 内容示例：

```text
修改人：
- **

所属小组：
- 文档答辩组

修改概述：
- 更新 Git 提交流程说明
- 补充小改动如何合并进已有提交
- 补充 PR 模板使用方式

实现方式：
- 在 docs/guidebook/git提交手册.md 中新增相关章节
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

按修改类型选择审核人：

```text
平台、仿真、话题、TF、控制接口：平台与仿真组 + 技术总负责人
导航、SLAM、探索、返航：导航探索组 + 技术总负责人
视觉、点云、识别、定位：感知定位组 + 技术总负责人
启动脚本、配置、测试统计：系统集成测试组
报告、PPT、手册、会议纪要：文档答辩组
```

## 8. 小改动怎么提交

小改动包括：

```text
文档里补几句话
修正错别字
调整 README 的一两处说明
补充注释
修改很小的配置说明
```

### 8.1 还没有提交

直接和本次相关修改一起提交。

```powershell
git status                                                     # 查看当前修改
git add docs/guidebook/git提交手册.md                          # 暂存这个文档修改
git commit -m "docs: update git workflow guide"                # 提交为一次文档更新
```

### 8.2 已提交，但还没有 push

把小改动合并进上一个提交。

```powershell
git add docs/guidebook/git提交手册.md                          # 暂存补充的小改动
git commit --amend --no-edit                                   # 合并进上一个提交，不修改提交说明
```

如果想顺便改提交说明：

```powershell
git add docs/guidebook/git提交手册.md                          # 暂存补充的小改动
git commit --amend -m "docs: update git workflow guide"         # 合并进上一个提交，并修改提交说明
```

### 8.3 已 push 到自己的功能分支

如果这个分支只有你自己在用，可以继续 `amend`，然后安全强推自己的分支。

```powershell
git add docs/guidebook/git提交手册.md                          # 暂存补充的小改动
git commit --amend --no-edit                                   # 合并进上一个提交
git push --force-with-lease                                    # 安全强推自己的功能分支
```

不要对 `main` 或 `dev` 这样做。

### 8.4 已经进了公共分支

如果小改动已经合并进 `dev` 或 `main`，不要重写公共历史。重新开一个修正分支即可。

```powershell
git switch dev                                                 # 切到 dev
git pull origin dev                                            # 同步最新 dev
git switch -c docs/fix-typo                                    # 新建文档修正分支
git add docs/guidebook/git提交手册.md                          # 暂存修改
git commit -m "docs: fix typo in git guide"                    # 提交文档修正
git push -u origin docs/fix-typo                               # 推送分支并提交 PR
```

## 9. 撤回、修改和删除常用操作

### 9.1 取消暂存

```powershell
git restore --staged 文件路径                                  # 取消某个文件的暂存，但保留文件修改
git restore --staged .                                         # 取消所有已暂存文件
```

### 9.2 丢弃本地修改

危险：会丢掉未提交的修改。

```powershell
git restore 文件路径                                           # 丢弃某个文件的本地修改
git restore .                                                  # 丢弃当前目录下所有已跟踪文件的本地修改
```

### 9.3 删除未跟踪的新文件

先预览：

```powershell
git clean -n                                                   # 预览将要删除的未跟踪文件
```

确认后删除：

```powershell
git clean -f                                                   # 删除未跟踪文件
git clean -fd                                                  # 删除未跟踪文件和文件夹
```

### 9.4 修改最近一次提交文字

如果还没有推送：

```powershell
git commit --amend -m "新的提交说明"                           # 修改最近一次 commit 的提交信息
```

如果已经推送到自己的功能分支：

```powershell
git commit --amend -m "新的提交说明"                           # 修改最近一次 commit 的提交信息
git push --force-with-lease                                    # 安全强推自己的功能分支
```

### 9.5 撤回最近一次提交

```powershell
git reset --soft HEAD~1                                        # 撤销最近一次 commit，保留暂存状态
git reset --mixed HEAD~1                                       # 撤销最近一次 commit，保留文件修改但取消暂存
git reset --hard HEAD~1                                        # 撤销最近一次 commit，并彻底丢弃对应修改，谨慎使用
```

### 9.6 撤销已经推送的公共提交

如果提交已经进入公共分支，推荐用 `revert`，不要用 `reset`。

```powershell
git log --oneline                                              # 查看提交历史和提交 ID
git revert 提交ID                                              # 生成一个反向提交，用于撤销指定提交
git push                                                       # 推送撤销结果
```

撤销最近一次提交：

```powershell
git revert HEAD                                                # 撤销最近一次提交，适合公共分支
```

## 10. 分支管理

### 10.1 删除本地分支

删除前先切到其他分支：

```powershell
git switch dev                                                 # 切换到 dev，避免删除当前分支
git branch -d feature/旧分支名                                 # 删除已经合并的本地分支
git branch -D feature/旧分支名                                 # 强制删除未合并的本地分支，谨慎使用
```

### 10.2 删除远程分支

```powershell
git push origin --delete feature/旧分支名                      # 删除远程分支
```

### 10.3 清理失效远程分支记录

```powershell
git fetch --prune                                              # 清理本地已失效的远程分支引用
```

### 10.4 重命名当前分支

```powershell
git branch -m 新分支名                                         # 将当前分支重命名
git push -u origin 新分支名                                    # 推送新分支名到远程
git push origin --delete 旧分支名                              # 删除远程旧分支
```

## 11. 合并冲突处理

### 11.1 将最新 dev 合并到当前分支

```powershell
git switch feature/你的分支名                                  # 切换到自己的功能分支
git fetch origin                                               # 获取远程最新信息
git merge origin/dev                                           # 将远程 dev 合并到当前分支
```

### 11.2 处理冲突

查看冲突文件：

```powershell
git status                                                     # 查看哪些文件发生冲突
```

冲突文件中通常会出现：

```text
你当前分支的代码
```

手动处理冲突后：

```powershell
git add 冲突文件路径                                           # 标记冲突已解决
git commit                                                     # 完成合并提交
git push                                                       # 推送解决冲突后的分支
```

### 11.3 放弃本次合并

```powershell
git merge --abort                                              # 放弃本次 merge，回到合并前状态
```

## 12. 临时保存工作区

如果代码改到一半，但需要切换分支，可以用 `stash` 临时保存。

```powershell
git stash push -m "临时保存导航调参"                           # 临时保存当前未提交修改
git stash list                                                 # 查看所有临时保存记录
git stash pop                                                  # 恢复最近一次 stash，并从列表中删除
git stash apply                                                # 恢复最近一次 stash，但保留在列表中
git stash drop stash@{0}                                       # 删除指定 stash
git stash clear                                                # 清空所有 stash，谨慎使用
```

## 13. 不要提交的文件

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

大文件如 rosbag、视频、模型权重应放网盘或 Git LFS，并在文档中记录下载方式。

## 14. 首次建仓或绑定远程仓库

如果本地已经有项目文件，但还不是 Git 仓库，可以这样初始化。

```powershell
git init                                                       # 初始化当前目录为 Git 仓库
git branch -M main                                             # 将当前主分支命名为 main
git remote add origin https://github.com/组织名或用户名/HazardWalker.git  # 添加远程仓库地址
git remote -v                                                  # 检查远程仓库地址是否正确
git add .                                                      # 暂存当前所有文件
git commit -m "chore: initialize HazardWalker repository"       # 创建第一次提交
git push -u origin main                                        # 第一次推送 main，并建立跟踪关系
```

如果远程仓库已有 README，首次推送被拒绝：

```powershell
git pull origin main --allow-unrelated-histories               # 拉取远程已有内容，并允许合并两边历史
git push -u origin main                                        # 合并后再次推送
```

只有确认远程仓库没有重要内容时，才允许：

```powershell
git push -u origin main --force-with-lease                     # 用本地 main 覆盖远程 main，谨慎使用
```

## 15. 进阶整理提交

如果本地已经有多个零散小提交，还没有合并到公共分支，可以在 PR 合并时使用 `Squash and merge`，把多个提交压成一个提交。

如果你熟悉 Git，也可以在本地交互式整理提交：

```powershell
git log --oneline                                              # 查看最近提交
git rebase -i HEAD~3                                           # 整理最近 3 个提交，谨慎使用
```

新手优先使用 PR 页面里的 `Squash and merge`。交互式 rebase 适合熟练成员使用，不要对 `main` 或 `dev` 这类公共分支随意使用。
