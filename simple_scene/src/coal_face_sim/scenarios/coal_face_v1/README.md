# SubT 正式场景资源归档

本目录包含已归档的 SubT 拼块和正式采煤工作面 `worlds/coal_face.sdf`。下文资源核查记录保留归档时的结果。

正式场景在 Ubuntu22 的新终端执行 `coal-face` 启动。当前往复轨迹由采煤机模型内的 `libcoal_face_trajectory.so` 在每个物理步直接推进，不再运行外部异步位姿服务控制。参数位于 `coal_face.sdf` 中 `coal_face_trajectory` 插件配置：Y=5～195 m，周期 1200 s，余弦速度，峰值约 0.4974 m/s；参考话题 50 Hz。`/ground_truth` 仍由独立 p3d 插件读取 Gazebo 模型状态。

控制实现见 `runtime_sources/coal_face_trajectory.cpp`；控制修复与回归证据见 `validation/control_fix_20261029/`。旧 `coal_face_motion.py` 已停用，以防同时运行两个控制器。正式 launch 安装目录只安装运行所需模型和世界，工具、测试记录及备份不再作为启动入口安装。

| 模型目录 | 用途 | Fuel 版本 |
|---|---|---|
| models/subt_tunnel_tile_1 | 四岔口及共享贴图来源 | 5 |
| models/subt_tunnel_tile_2 | 90 度弯段 | 16 |
| models/subt_tunnel_tile_5 | 直段 | 14 |
| models/subt_tunnel_tile_blocker | 封堵块 | 12 |

- 源仓库提交：ba15f9ff35ccfb43f44937d6d84e047615d174ff。
- 原始文件独立保存在 D:/work1/work1-codex/subt-asset-archive/upstream/，原 Fuel 缓存未修改。
- 正式副本补齐 11 项共享贴图，将 COLLADA 在线图片路径改为本地相对路径；移除已被本地文件取代的 model.config 在线依赖声明。
- 为 Classic 保留 OGRE 材质、移除 PBR 配置；材质名称按模型加前缀避免冲突。移除岔口中的新 Gazebo 比赛检测插件，因为本项目只复用静态场景资源。网格几何未改变。
- asset_manifest.json 保存来源、版本、修改内容和 117 个模型文件的 SHA-256。来源元数据中的网页地址仅作追溯，不是运行资源依赖；这些记录不代表完成额外许可授权。

验证结果：86 处资源引用全部可解析，没有活动在线依赖、缺失文件或双份归档校验差异。在只有回环接口、无外网路由、空 HOME/Fuel 缓存的临时网络命名空间中，四类拼块加载通过；四个独立激光探针各收到 75 帧、每帧 360 点，频率约 5 Hz，仿真推进 15 秒。图形界面中直段、弯段、岔口材质可见；封堵块沿用原始外观。该测试探针配置与原采煤机的 720 点/10 Hz 配置独立。

报告：validation/static_report.json、validation/offline_runtime_report.json。
离线日志仍可能出现默认 Fuel 服务域名解析失败提示；测试已禁止外网，模型照常加载。该结果证明资源无需联网可用，不表示 Gazebo 不尝试访问其默认服务。

正式 Linux 路径：/home/ll/coal_sim_ws/src/coal_face_sim/scenarios/coal_face_v1/。
Windows 镜像：D:/work1/work1-codex/scenarios/coal_face_v1/。

其他场景引用资源时，将本目录 models 加入 GAZEBO_MODEL_PATH，再用上述本地 model:// 名称引用。worlds/offline_resource_check.sdf 仅为四拼块分开展示的验证世界；正式入口是 coal_face.sdf。

静态复核：python3 /mnt/d/work1/work1-codex/subt-asset-archive/verify_archive.py。
离线复核：unshare -Urn -- bash /mnt/d/work1/work1-codex/subt-asset-archive/offline_test.sh。
