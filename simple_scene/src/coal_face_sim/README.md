# SubT 直段场景与采煤机往复运动

环境：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11.10.2。
本工程复用 OpenRobotics 的 Tunnel Tile 5 网格和碰撞、Tunnel Tile 1 的贴图。
原始下载和版本信息保存在 `~/coal_sim_resources/upstream`，适配副本在本包 `models/`。

## 启动

首次构建或更新后：

```bash
source /opt/ros/humble/setup.bash
cd ~/coal_sim_ws
colcon build --symlink-install --packages-select coal_face_sim
source install/setup.bash
```

关闭已有 Gazebo 实例后，一条命令启动场景与运动：

```bash
ros2 launch coal_face_sim coal_face.launch.py
```

无界面运行或调整完整往返周期：

```bash
ros2 launch coal_face_sim coal_face.launch.py gui:=false period:=40.0
```

启动文件自动设置模型搜索路径，无需手动 export。Ctrl+C 停止整个启动组。
旧的 `shearer_test.world` 保留作静态落体验证，新入口使用 `coal_face_motion.world`。

## 运动与接口

- `config/motion.yaml` 配置位置范围、周期、控制频率与实体名，重启后生效。
- 默认沿世界 Y 轴在 -4 至 +4 m 往复，X=0、Z=0.4005 m。
- 余弦轨迹 `y = center - amplitude*cos(2*pi*t/period)`，端点速度连续变为零。
- 默认完整往返 40 s，单程 20 s，最大速度约 0.6283 m/s。
- `/set_entity_state`：Gazebo 状态设置服务；控制节点等待服务和真值就绪后运动。
- `/shearer/reference`：期望位姿与速度，nav_msgs/msg/Odometry，不是真值。
- `/ground_truth`：p3d 插件从实际 base_link 状态读取的独立真值，nav_msgs/msg/Odometry，50 Hz。
- 真值参考系 world，参考点 base_link 中心，无额外位置或姿态偏移。
- `/clock` 由 Gazebo 以 100 Hz 发布；控制节点 use_sim_time=true。
- 暂停仿真时运动时钟停止，恢复后继续。仿真时间回退时控制节点重新起算轨迹。

## 模型限制

采煤机为 0.8 × 2.0 × 0.8 m、100 kg 的开发占位模型，不代表实际设备参数。
运动通过设置实体状态实现，关闭该模型的重力以固定高度；属于运动学简化。
碰撞几何仍在，但控制并不依赖碰撞约束轨迹，不能用来验证真实接触力、打滑或可信 IMU 动力学。
默认轨迹限制在已检查的单拼块中央；控制器拒绝超出保守范围的参数。
当前尚未提供 TF、激光/IMU/UWB、作业工况、扰动注入与算法评估模块。
本次轨迹误差只是指令跟踪误差，不是项目“测距误差小于 0.1 m”的考核结论。

## 检查

```bash
ros2 topic echo /ground_truth --once
ros2 topic hz /ground_truth
ros2 topic echo /shearer/reference --once
```

Windows 工作目录 `D:\work1\coal_face_sim_update\validation` 保存自动验证报告、实际轨迹 CSV 和参考轨迹 CSV。
自动验证在独立 ROS_DOMAIN_ID=71、Gazebo master 端口 11371 中运行，检查至少两个往返周期、端点、实际轨迹误差、消息时间戳连续性、暂停恢复以及 Gazebo 状态查询交叉验证。

模型来源：

- https://github.com/osrf/subt/blob/master/subt_ign/worlds/tunnel_circuit_practice_01.sdf
- https://fuel.gazebosim.org/1.0/OpenRobotics/models/Tunnel%20Tile%205
- https://fuel.gazebosim.org/1.0/OpenRobotics/models/Tunnel%20Tile%201

包内原有 license TODO 尚未替用户选定，发布前需确认代码许可证并核对各项资源许可。
