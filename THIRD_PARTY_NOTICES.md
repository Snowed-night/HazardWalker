# 第三方来源与运行资源

HazardWalker自有代码沿用根目录Apache-2.0许可。ROS、Gazebo、Cartographer、Unitree及其它第三方目录保留各自作者、版权和许可证，根目录许可不替代它们。

平台包`src`包含官方仿真、楼宇生成器、宇树A1描述与控制源码及第三方依赖。复用时保留原LICENSE和版权头。

宇树运动策略`.pt`和SDK预编译`.so/.a`按外部运行资源处理，不进入本Git分支。路径与哈希见`reproduction/runtime-assets.json`，从具备使用权限的官方SimEnv或原交付环境取得，再执行导入脚本。

源包中的Windows电机ID维护工具不属于仿真入口，本次源码导出省略。模型网格和激光扫描模式文件保留原路径。

`reports/perception`的历史图像和JSON用于原版离线检查，部分记录显式标为内部或历史，不代表本次新取得的成绩。
