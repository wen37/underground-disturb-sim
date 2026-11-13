# 当前掘进场景运行步骤

唯一开发目录为本工作区，不再从另一个仓库部署副本。
在本场景目录执行：

```bash
source /opt/ros/humble/setup.bash
python3 runtime/build_native.py
python3 runtime/scene_validator.py
python3 tests/test_contracts.py
python3 runtime/run_experiment.py --mode both
```

需要界面时加 `--gui`。构建输出位于工作区根目录的 build/heading_native、install/heading_native 和 log/heading_native；实验数据位于本场景 results，诊断日志位于 evidence/full_chain。生成物不上传 GitHub。

共享扰动模块位于工作区根目录 lidar_fixed_distance_test。当前为固定 +0.02 m Mock，浓度不参与计算。保留输入哈希检查；修改物理模型属于另一项任务。

原生路线、坐标、传感器和工况约束见 HEADING_INTEGRATION.md；第三方来源见 ../references/THIRD_PARTY.md。清理输出时不要删除工作区 install 下仍在使用的插件。
