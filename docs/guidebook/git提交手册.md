# HazardWalker 团队 Git 工作流程

本文档用于规范 HazardWalker 项目的代码协作流程。所有成员都应按本文档操作，避免代码丢失、分支混乱和版本不可复现。

## 1. 基本原则

- 不要直接在 `main` 分支上开发。
- 日常开发基于 `dev` 分支创建自己的功能分支。
- 真正保存版本的是 `commit`，不是 `add`。
- 所有功能分支通过 Pull Request 合并到 `dev`。
- 只有稳定、可演示、可提交的版本才能从 `dev` 合并到 `main`。
- 公共分支不要使用 `reset --hard`、`push --force` 等危险操作。

分支结构：

```text
main                 稳定版本，只放可演示、可提交版本
dev                  日常集成版本
feature/platform     平台与仿真开发
feature/nav          导航探索开发
feature/perception   感知定位开发
feature/test         测试脚本开发
docs/report          文档材料开发
fix/xxx              Bug 修复
```

## 2. 首次克隆仓库

仓库已经由负责人创建好，其他成员直接克隆即可。

cd到桌面
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

## 3. 配置个人 Git 身份

首次使用 Git 时，需要配置自己的姓名和邮箱。

```powershell
git config --global user.name "你的名字"                       # 设置提交时显示的名字
git config --global user.email "你的邮箱"                       # 设置提交时显示的邮箱
git config --global --list                                     # 查看当前 Git 全局配置
```

如果写错了，可以重新设置：

```powershell
git config --global user.name "新名字"                         # 修改提交姓名
git config --global user.email "新邮箱"                         # 修改提交邮箱
```

## 4. 已有本地项目时绑定远程仓库

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

## 5. 查看和切换分支

```powershell
git branch                                                     # 查看本地分支，带 * 的是当前分支
git branch -r                                                  # 查看远程分支
git branch -a                                                  # 查看本地和远程全部分支
git switch dev                                                 # 切换到 dev 分支
```

创建并切换新分支：

```powershell
git switch -c feature/perception-red-ball                      # 基于当前分支创建并切换到新分支
```

第一次推送新分支：

```powershell
git push -u origin feature/perception-red-ball                 # 推送新分支到远程，并建立跟踪关系
```

## 6. 日常开发标准流程

每次开始开发前，先同步最新 `dev`。

```powershell
git switch dev                                                 # 切换到日常集成分支 dev
git pull origin dev                                            # 拉取远程 dev 的最新内容
git switch feature/你的分支名                                  # 切回自己的开发分支
git merge dev                                                  # 将最新 dev 合并到自己的分支
```

如果你还没有自己的分支：

```powershell
git switch dev                                                 # 先切到 dev
git pull origin dev                                            # 同步最新 dev
git switch -c feature/你的任务名                               # 从 dev 创建自己的功能分支
```

开发完成后：

```powershell
git status                                                     # 查看当前修改了哪些文件
git add 文件路径                                               # 只暂存本次真正要提交的文件
git commit -m "feat(nav): add frontier exploration"            # 创建一次本地提交
git push                                                       # 推送当前分支到远程
```

如果是第一次推送当前分支：

```powershell
git push -u origin feature/你的分支名                          # 第一次推送并建立跟踪关系
```

推送后，在 GitHub 或 Gitee 上创建 Pull Request：

```text
feature/你的分支名 -> dev
```

## 7. 提交信息规范

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
chore     环境、脚本、配置
perf      性能优化
```

示例：

```powershell
git commit -m "feat(perception): add HSV red ball detector"     # 新增红球检测功能
git commit -m "fix(platform): correct lidar frame transform"    # 修复雷达坐标系变换错误
git commit -m "docs: update interface specification"            # 更新接口文档
git commit -m "test: add exploration metric logger"             # 新增探索指标记录脚本
```

不要使用以下提交信息：

```text
修改
update
最终版
111
```

## 8. 取消暂存、撤回修改和删除文件

### 8.1 取消暂存

已经执行 `git add`，但不想把该文件放进本次提交：

```powershell
git restore --staged 文件路径                                  # 取消某个文件的暂存，但保留文件内容修改
git restore --staged .                                         # 取消所有已暂存文件
```

### 8.2 丢弃本地修改

危险：会丢掉未提交的修改。

```powershell
git restore 文件路径                                           # 丢弃某个文件的本地修改
git restore .                                                  # 丢弃当前目录下所有已跟踪文件的本地修改
```

### 8.3 删除未跟踪的新文件

先预览：

```powershell
git clean -n                                                   # 预览将要删除的未跟踪文件
```

确认后删除：

```powershell
git clean -f                                                   # 删除未跟踪文件
git clean -fd                                                  # 删除未跟踪文件和文件夹
```

## 9. 修改、撤回和删除提交

### 9.1 修改最近一次提交文字

如果还没有推送：

```powershell
git commit --amend -m "新的提交说明"                           # 修改最近一次 commit 的提交信息
```

如果已经推送到自己的功能分支：

```powershell
git commit --amend -m "新的提交说明"                           # 修改最近一次 commit 的提交信息
git push --force-with-lease                                    # 安全强推自己的功能分支
```

不要在 `main` 或 `dev` 上随意这样做。

### 9.2 撤回最近一次提交，但保留修改

```powershell
git reset --soft HEAD~1                                        # 撤销最近一次 commit，保留暂存状态
git reset --mixed HEAD~1                                       # 撤销最近一次 commit，保留文件修改但取消暂存
```

### 9.3 撤回最近一次提交，并丢弃修改

危险：会删除代码修改。

```powershell
git reset --hard HEAD~1                                        # 撤销最近一次 commit，并彻底丢弃对应修改
```

### 9.4 撤销已经推送的提交

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

## 10. 删除和重命名分支

### 10.1 删除本地分支

删除前先切到其他分支：

```powershell
git switch dev                                                 # 先切换到 dev，避免删除当前分支
git branch -d feature/旧分支名                                 # 删除已经合并的本地分支
git branch -D feature/旧分支名                                 # 强制删除未合并的本地分支，谨慎使用
```

### 10.2 删除远程分支

```powershell
git push origin --delete feature/旧分支名                      # 删除远程分支
```

### 10.3 清理本地失效的远程分支记录

```powershell
git fetch --prune                                              # 清理本地已失效的远程分支引用
```

### 10.4 重命名当前分支

```powershell
git branch -m 新分支名                                         # 将当前分支重命名
git push -u origin 新分支名                                    # 推送新分支名到远程
git push origin --delete 旧分支名                              # 删除远程旧分支
```

## 11. 合并、冲突和放弃合并

### 11.1 将最新 dev 合并到当前分支

```powershell
git switch feature/你的分支名                                  # 切换到自己的功能分支
git fetch origin                                               # 获取远程最新信息
git merge origin/dev                                           # 将远程 dev 合并到当前分支
```

### 11.2 处理合并冲突

查看冲突文件：

```powershell
git status                                                     # 查看哪些文件发生冲突
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
git add 冲突文件路径                                           # 标记冲突已解决
git commit                                                     # 完成合并提交
git push                                                       # 推送解决冲突后的分支
```

### 11.3 放弃本次合并

如果冲突太乱，不想继续合并：

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

## 13. Pull Request 审核和合并规范

所有功能分支都通过 Pull Request 合并：

```text
feature/* -> dev
dev -> main
```

建议审核规则：

- `main` 禁止直接 push。
- `dev` 尽量禁止直接 push。
- 功能分支合并到 `dev` 前，至少 1 人审核。
- 改接口、话题、坐标系、消息格式时，必须由技术负责人审核。
- 改启动脚本、测试脚本、依赖配置时，必须由系统集成负责人审核。

PR 模板：

```text
本次修改：
- 

验证方式：
- 

影响范围：
- 

是否修改接口：
- 是/否

风险：
- 
```

合并前检查：

```powershell
git status                                                     # 确认工作区干净
git pull origin dev                                            # 同步最新 dev
```

如果是 ROS 项目，至少确认能构建：

```powershell
colcon build                                                   # 构建 ROS 工作空间，确认没有编译错误
```

## 14. 不要提交的大文件和临时文件

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

建议在 `.gitignore` 中排除这些内容。大文件如 rosbag、视频、模型权重应放网盘或 Git LFS，并在文档中记录下载方式。

## 15. 开发最常用流程

每天开始开发：

```powershell
git switch dev                                                 # 切到 dev
git pull origin dev                                            # 更新 dev
git switch feature/你的分支名                                  # 切回自己的分支
git merge dev                                                  # 合并最新 dev
```

开发完成后：

```powershell
git status                                                     # 检查修改
git add 文件路径                                               # 暂存需要提交的文件
git commit -m "feat(scope): 简要说明"                          # 提交修改
git push                                                       # 推送到远程
```

然后创建 Pull Request：

```text
feature/你的分支名 -> dev
```

## 16. 一些注意点

- 平时开发：先更新 `dev`，再回自己分支 `merge dev`。
- 改完代码：`status` -> `add` -> `commit` -> `push` -> 提 PR。
- 稳定版本：只有经过测试的 `dev` 才能合并到 `main`。
- 不要直接在 `main` 上开发。
- 不要把大文件、构建产物、日志文件提交到 Git。
- 公共分支不要强推，不要随便重写历史。
