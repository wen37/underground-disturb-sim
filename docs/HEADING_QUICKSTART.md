# SubT掘进场景与Mock联调

本入口复用仓库SubT模型，运行固定测站二维雷达、EBZ200H外廓和机身靶板。与原采煤工作面入口并存，不修改原公共接口。

## 环境与资源

Ubuntu22.04、ROS2 Humble、Gazebo Classic11。已安装ROS时补充：

```bash
sudo apt install build-essential cmake libgazebo-dev ros-humble-gazebo-dev python3-colcon-common-extensions python3-yaml python3-jsonschema python3-openpyxl ros-humble-gazebo-ros-pkgs ros-humble-tf2-ros ros-humble-nav-msgs
source /opt/ros/humble/setup.bash
cd underground-disturb-sim
python3 scripts/restore_assets.py
```

已有完整本地模型归档时，可用 `python3 scripts/restore_assets.py --from-local /path/to/models`；该目录必须对应仓库完整资产清单。恢复器按哈希检查，不把模型复制到新场景目录。只构建已有coal_face_sim包内的运动适配器，不需要构建UWB模块；不依赖个人bashrc或旧install目录。

```bash
colcon --log-base src/coal_scene_tools/experiments/heading/evidence/native_log build --base-paths src/coal_face_sim --packages-select coal_face_sim --build-base src/coal_scene_tools/experiments/heading/evidence/native_build --install-base src/coal_scene_tools/experiments/heading/evidence/native_install --cmake-args -DBUILD_TESTING=OFF
```

上述命令编译运行所需插件；验证脚本另行执行，关闭构建期lint不代表测试通过。运行器会设置该插件目录，无须source旧工作空间。

## 执行顺序

```bash
python3 src/coal_scene_tools/experiments/lidar_fixed_distance_test/run_fixed_distance.py
python3 src/coal_scene_tools/experiments/lidar_fixed_distance_test/run_fixed_distance.py
python3 src/coal_scene_tools/experiments/heading/runtime/scene_validator.py
python3 src/coal_scene_tools/experiments/heading/tests/test_contracts.py
python3 src/coal_scene_tools/experiments/heading/runtime/run_experiment.py --mode both
```

`--mode static`只运行静态，`--mode motion`只运行运动；追加`--gui`显示窗口。实验结束后自动关闭自己的Gazebo进程。默认隔离为ROS_DOMAIN_ID=137和Gazebo端口11357；占用时拒绝启动，不杀其他实例。

Gazebo使用`--lockstep`，配置实时倍率0.5，为处理留出余量；雷达仍为仿真时间40Hz，不能用墙钟时间直接比较。正式窗口为静态3s、运动15.5s；边界时间戳允许分别120–121帧和620–621帧，接收、处理及独立观察器配对数必须一致，时间对齐丢弃必须为0，原始扫描不得出现≥50ms间隔。24–26ms之外的间隔波动单独报告。

完整流程现在有4个返回码：静态程序、静态独立观察器、运动程序、运动独立观察器。必须全部为0且passed=true。出现异常查看输出的log_dir，不能把文件生成当作验收通过。程序等待扫描/位姿/时钟稳定2s后才开始正式采样，预热诊断单独记录，不计作正式实验成功。

## 坐标和参数

map为右手系，+Y前进、X横向、+Z向上。map→world恒等；map→station_link→lidar_link固定；map→roadheader_base_link为Gazebo位姿；机身→target_link固定。机身xyz与rpy为m/rad；消息姿态为四元数。

| 参数 | 值与来源 |
|---|---|
| 巷道 | 4个原尺寸20m SubT Tile5，名义80m；不规则原网格，不是固定矩形净宽 |
| EBZ200H | 长11.6m、履带宽2.7m、选用铲板宽3.2m、机身高2.0m、水平截割头最高2.43m，[官方手册](https://sanyglobal-img.sany.com.cn/product/picture_album/20220517/EBZ200H-164955.pdf) |
| UTM-30LX | 0.1–30m（指定白色目标）、270°、0.25°、25ms/扫描，[官方规格](https://www.hokuyo-aut.jp/search/single.php?serial=169) |
| 仿真扫描 | 40Hz、1081束（含两端，由角度计算）；0.001m为SDF分辨设置，不是实机精度 |
| 初始机身中心 | map中[0,68,1.349]m，姿态0；外廓[3.2,11.6,2.43]m |
| 固定测站 | map中[0,57.1,0.134]m，姿态0 |
| 雷达安装 | 相对测站[0,0,1.8]m，yaw=π/2rad；map中[0,57.1,1.934]m |
| 靶板 | 相对机身[0,-5.9,0.585]m；尺寸[0.5,0.02,0.5]m；几何辅助靶板，不是棱镜 |
| 中心距离 | 初始到靶板中心5.00m、首个表面4.99m |
| 路线 | 0–2s停止；2–12s以0.2m/s前进2m；12–15s模拟截割；运动采样至15.5s |
| 呼吸性粉尘 | 后方Y0–55m为2mg/m³；司机区Y55–70m为10mg/m³；迎头Y70–80m为20mg/m³；背景0 |

安装位姿、靶板尺寸和浓度属于实验设计值，不冒充厂家参数。当前专用校验器限制固定布局；改布局需同步配置、SDF、解析真值和校验规则。详情见[集成说明](HEADING_INTEGRATION.md)。

## 数据与验收

原始扫描为 `/heading/scan_raw`；基线与Mock为 `/heading/scan_baseline`、`/heading/scan_mock`。三者均为LaserScan，header.frame_id=lidar_link。机身原生位姿为 `/heading/body_odometry`（100Hz配置）；对齐的机身/靶标真值为 `/heading/body_truth` 和 `/heading/target_truth`。

机身真值是实际Gazebo中心位姿，靶标真值由实际机身位姿及固定安装关系计算；逐束真值是无噪声ray首次碰撞距离，不是靶标中心距离。扫描匹配最近原生位姿，最大时间差25ms，超差明确失败。中心束另作解析核对，误差须小于1cm。

有效光束沿有限路径计算 C平均=Σ(C区域×穿过长度)/总长度，含背景段，遇首个障碍停止。矩形区域重叠拒绝。关闭扰动与raw完全相同；当前Mock加0.02m，ROS float32验收容差1e-6m。无回波不送入Mock。

场景结果在 `src/coal_scene_tools/experiments/heading/results/北京时间/`，含summary.json、config_snapshot.yaml、beam_detail.csv和场景实验报表.xlsx。固定距离输出在相邻实验目录的results。CSV以10Hz保存整帧1081束；40Hz所有正式帧参加在线验收和Excel分区工况统计。字段包含zone_id、work_mode、路径平均浓度mg/m³、机身xyz/rpy、靶标xyz和对齐差。不得把CSV保存频率误认为扫描频率。

`heading_math.position_error`接受map下的stamp_s与xyz_m，检查有限数值和25ms对齐，返回dx/dy/dz及三维距离误差。当前仅实现单对位姿的误差计算与输入校验，尚未实现批量定位评价报告（配对数、拒绝数、平均误差、RMSE和P95）。场景Excel中的RMSE是Mock测距误差，不是定位误差。本阶段不实现定位算法。

## 已知局限

- 10mg/m³按任务指定为司机工况测试值，2/10/20mg/m³均无现场标定依据。
- Mock不使用浓度预测状态，没有随机噪声或真实粉尘光学模型。
- 迎头高浓度区配置及数学测试存在，但当前遮挡布局没有该区实际扫描样本。
- 机身为整体BOX；运动由Gazebo更新回调按仿真时间执行配置路线，实验专用`/heading/trajectory_start`接收Float64起始仿真秒；该起始信号和路线指令都不是机身真值。底部保守抬升0.134m，无接地动力学、截割头旋转或流体仿真。
- 原S00矩形场景不迁入；当前隧道用SubT网格，粉尘区仍是矩形体积近似。
- 不自动对接公共SimHeader、QueryRay或九张公共CSV；实验话题与报告明确隔离，不声称完成公共接口迁移。
- Gazebo Classic沿用现有环境。有限网格净空检查与运行通过不等于真实矿用设备适用性认证。

验收记录见[HEADING_VALIDATION.md](HEADING_VALIDATION.md)。未来替换Mock时保持接口与单位，明确新模型版本，并经审核更新保留校验清单；不要通过删除校验绕过有意的模型变更。
