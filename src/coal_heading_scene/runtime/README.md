# 运行程序

从上一级场景目录执行；完整顺序见上一级README.md。

- build_native.py：编译原生运动插件；首次部署或插件源码变化时执行。
- scene_validator.py：检查配置、SDF、资源及固定距离接口保留清单。
- heading_math.py：路径浓度与单对位姿误差自检。
- run_experiment.py --mode both [--gui]：启动Gazebo并执行静态、运动实验。
- run_heading.py：接收真实扫描与位姿，调用共享dust_model.measure，输出CSV和Excel。
- observe_topics.py：独立订阅检查三路扫描。
- audit_reports.py：核算已保存报表。

扰动算法不在此目录内；具体共享模块位置由paths.FIXED指定。当前Mock固定+0.02m，不是真实粉尘模型。
