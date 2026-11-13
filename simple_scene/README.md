# 早期场景成果

从原 coal_sim_ws 整理保留的源码、配置、资源及工具；不含旧工作区构建物和Git元数据。
COLCON_IGNORE 防止与当前掘进场景的同名包自动混合构建。这里保留早期成果，不代表其全部历史脚本已经重新运行验收；当前已验收入口是 src/coal_heading_scene。
资源来源及许可见 THIRD_PARTY.md 与包内清单。工具路径已按脚本位置解析。

## 洞穴评估包 cave_eval

`src/cave_eval` 保存早期洞穴评估包。原始洞穴资源仍位于工作区根目录的 `upstream/gazebo_cave_world`，无需复制或移动；许可证随资源及包内 `LICENSE.upstream` 保留。

从工作区根目录执行以下命令，显式指定包目录，以独立构建本包并保留本目录的 `COLCON_IGNORE`：

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths simple_scene/src/cave_eval --packages-select cave_eval
source install/setup.bash
ros2 launch cave_eval cave.launch.py gui:=false original:=false
```

`original:=true` 加载原始洞穴；`original:=false` 加载带设备、激光雷达、运动控制和位姿真值的场景。需要图形窗口时使用 `gui:=true`，RViz 可通过 `rviz:=true` 启动。
安装环境加载后，可从其他目录运行启动命令。启动文件按安装包位置向上查找工作区资源，也可用 `CAVE_WORLD_PATH` 显式指定原始洞穴资源目录。
