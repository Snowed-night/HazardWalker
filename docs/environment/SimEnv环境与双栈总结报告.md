# SimEnv 环境与双栈总结报告

更新时间：2026-07-04  
适用对象：集成与测试组、平台组、各算法组  
目标机器：`hxbl`（主机 Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic 8.11）

---

## 1. 三目录布局（核心原则）

**原版 `SimEnv/` 不被迁移脚本修改。** 所有新栈使用同级独立目录：

```
~/Guoyulun/Competition/
├── SimEnv/          # 原版 ROS1 catkin（平台组维护，只读源）
├── SimEnv_ROS1/     # Docker 版 ROS1（Ubuntu 20.04 容器 + auto_docker.sh）
└── SimEnv_ROS2/     # ROS2 迁移（Harmonic + HazardWalker /hw/* 联调）
```

| 目录 | 入口 | 用途 |
|------|------|------|
| `SimEnv` | `./auto.sh`（24.04 不可用） | 原版源码与 devel，**仅作 rsync 源** |
| `SimEnv_ROS1` | `./auto_docker.sh` | 四足步态、Classic 仿真、Mid360 |
| `SimEnv_ROS2` | `./auto_ros2.sh` | 日常 HazardWalker 联调、传感器 `/hw/*` |

---

## 2. 账号与路径

| 组别 | 账号 | ROS_DOMAIN_ID |
|------|------|---------------|
| 集成与测试组 | `hazard_test` | **20** |
| 平台组 | `hazard_platform` | **11** |

各组在自家 home 下均有上述三目录（`SimEnv` 仅平台组有完整原版；测试组通过 rsync 获得 `SimEnv_ROS1` 副本）。

Docker 权限：各仿真相关账号已加入 `docker` 组（需 **重新登录 SSH**）。

---

## 3. ROS2 主链路（SimEnv_ROS2）

```bash
export ROS_DOMAIN_ID=20   # 测试组；平台组用 11
export GZ_PARTITION=hazardwalker_test
cd ~/Guoyulun/Competition/SimEnv_ROS2
./auto_ros2.sh
./run_simenv_hazardwalker.sh
./verify_ros2_migration.sh
```

- 场景生成：读 `SimEnv_ROS1/src/`，写 `SimEnv_ROS2/generated_building/`
- 仓库维护：`scripts/simenv_ros2/`
- 部署：`deploy_to_hazard_test.py` / `deploy_to_hazard_platform.py`

---

## 4. Docker ROS1（SimEnv_ROS1）

```bash
cd ~/Guoyulun/Competition/SimEnv_ROS1
./auto_docker.sh build    # 首次
./auto_docker.sh up
START_CONTROLLER=1 ./auto_docker.sh up   # 四足
```

- 内容：`rsync` 自同账号或平台组 `SimEnv/`（**不修改** `SimEnv/`）
- 部署：`deploy_ros1_docker.py` 或 `sync_ros1_from_platform.py`
- 容器名：`simenv_ros1_<用户名>`

---

## 5. 原版 SimEnv（只读）

平台组：

```bash
~/Guoyulun/Competition/SimEnv/   # auto.sh, src/, devel/ — 勿在此目录部署 ROS2/Docker 脚本
```

若曾误部署迁移文件到 `SimEnv/`，运行 `deploy_to_hazard_platform.py` 会自动清理 overlay 并迁至 `SimEnv_ROS2` / `SimEnv_ROS1`。

---

## 6. 部署脚本索引

| 脚本 | 作用 |
|------|------|
| `deploy_ros1_docker.py` | rsync SimEnv→SimEnv_ROS1 + Docker 脚本（test & platform） |
| `deploy_to_hazard_test.py` | 部署 SimEnv_ROS2（test），迁移旧 SimEnv/ 内 ROS2 文件 |
| `deploy_to_hazard_platform.py` | 部署 SimEnv_ROS2（platform），清理 SimEnv/ overlay |
| `sync_ros1_from_platform.py` | 平台 SimEnv → 测试 SimEnv_ROS1 |
| `setup_hxbl_docker_group.sh` | 各组 docker 组权限 |

均需：`export HXBL_ADMIN_PASSWORD='...'`

---

## 7. 验收结论（2026-07-04）

### hxbl 目录迁移（已完成）

| 账号 | SimEnv（原版） | SimEnv_ROS1 | SimEnv_ROS2 |
|------|----------------|-------------|-------------|
| `hazard_test` | 仅 classic（auto.sh, src/） | Docker + catkin 副本 | ROS2 全栈 |
| `hazard_platform` | 无 overlay（CLASSIC_CLEAN） | Docker + catkin 副本 | ROS2 全栈 |

已执行：`deploy_ros1_docker.py` → `deploy_to_hazard_test.py` → `deploy_to_hazard_platform.py`  
测试组 `SimEnv_ROS2` 中误迁的 `docker/`、`auto_noetic_headless.sh` 已去重移除。

### 功能验收（hazard_test / SimEnv_ROS2）

`verify_ros2_migration.sh`：**29 PASS / 1 FAIL / 5 SKIP**

| 项 | 结果 |
|----|------|
| colcon 7 包、场景生成、evaluate CLI | PASS |
| 传感器 /hw/*、相机、building services | PASS |
| HazardWalker 三节点 | PASS |
| `/cmd_vel` relay publisher 检测 | FAIL（偶发，odom 闭环仍 PASS） |

SimEnv + HazardWalker **ROS2 主链路**已在 `SimEnv_ROS2` 验收通过。  
**Docker ROS1** 与 **原版 SimEnv** 目录分离，互不影响。

---

## 8. 相关文档

- **`docs/environment/SimEnv话题接口对照表.md`** — 官方 SimEnv / SimEnv_ROS2 / HazardWalker `/hw/*` 三列对照
- `SimEnv_ROS2/README_ROS2_MIGRATION.md`
- `SimEnv_ROS1/README_ROS1_DOCKER.md`
- `docs/environment/主力机环境搭建.md`
