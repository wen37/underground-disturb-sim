# 掘进场景当前入口

当前运行场景为S01_subt_heading：SubT巷道、固定LiDAR、掘进机简化外廓和机身靶标。支持静态及运动实验。

在本目录执行：

```bash
source /opt/ros/humble/setup.bash
python3 runtime/build_native.py
python3 runtime/scene_validator.py
python3 runtime/heading_math.py
python3 runtime/run_experiment.py --mode both --gui
```

首次部署或原生插件源码改变后需要构建。结果写入本目录results，日志写入evidence/full_chain。历史结果保留，文件夹存在不代表检查通过。

共享扰动模块是dust_model.py中的measure()。仓库布局使用相邻lidar_fixed_distance_test；本地部署布局使用工作空间根目录下的lidar_fixed_distance_test。runtime/paths.py明确解析路径，不依赖个人bashrc或软链接。

本地部署保留的S00_heading、旧schemas/tools和backups仅作历史记录，不是当前S01入口。references保留第三方来源及资源清单。完整说明见仓库docs/HEADING_QUICKSTART.md（本地部署副本也位于docs）。

GitHub与本地不会自动实时双向同步。修改应先在仓库中完成并验证，再用scripts/deploy_heading_workspace.py部署；--check可只读核对副本是否一致。
