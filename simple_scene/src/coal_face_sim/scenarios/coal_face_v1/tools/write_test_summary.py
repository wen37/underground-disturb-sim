#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Generate a compact, evidence-linked Chinese acceptance note after shutdown."""
import hashlib,json
from pathlib import Path
root=Path(__file__).resolve().parents[1]
out=root/'validation/full_scale'
result=json.loads((out/'acceptance_results.json').read_text())
motion=result['runtime']['motion'];perf=result['performance'];rates=result['runtime']['rates'];scan=result['scan_geometry'];gates=result['runtime_gates']
geometry=json.loads((root/'design_v02/geometric_checks.json').read_text())
first=root/'validation/full_scale_attempt1'
floor=json.loads((first/'live_layout_floor_report.json').read_text());offline=json.loads((first/'offline_full_report.json').read_text())
previous=json.loads((first/'acceptance_results.json').read_text())
same_world=previous['source_sha256']['worlds/coal_face.sdf']==result['source_sha256']['worlds/coal_face.sdf']
baseline=first/'baseline'
pkg=root.parents[1]
baseline_ok={name:hashlib.sha256((pkg/name).read_bytes()).hexdigest()==hashlib.sha256((baseline/name.replace('/','__')).read_bytes()).hexdigest() for name in ['scripts/shearer_motion.py','worlds/coal_face_motion.world']}
verdict=all(gates.values()) and geometry['passed'] and floor['passed'] and offline['passed'] and same_world
report={'platform_scene_passed':verdict,'complete_v1_closed_loop_passed':False,'runtime_gates':gates,'geometry_passed':geometry['passed'],'floor_probes_passed':floor['passed'],'offline_passed':offline['passed'],'offline_and_floor_source_world_unchanged':same_world,'original_single_tile_files_unchanged':baseline_ok,'outstanding':['/uwb_ranges_raw and teammate disturbance modules are absent.','No base-station scene YAML is configured.','No NLOS occupancy/obstacle-data API has been agreed or exercised; collision SDF geometry exists.','No disturbance-to-error-statistics closed-loop acceptance.'],'provenance':{'static_geometry':'../../design_v02/geometric_checks.json','floor':'../full_scale_attempt1/live_layout_floor_report.json','offline':'../full_scale_attempt1/offline_full_report.json','attempt1_failure':'Read-only query of live SQLite triggered writer lock; original evidence retained. Second attempt analyzed only after recorder exit.'}}
report['precision_review_required']={'motion_max_error_m':motion['ideal_position_max_error_m'],'scan_max_error_m':scan['max_error_m'],'scan_beams_over_5cm':scan['beams_over_5cm'],'note':'Functional gates passed; maximum-error tolerances are not agreed. Do not claim complete precision acceptance.'}
(out/'verdict.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
tf=result['runtime']['tf']
turns='；'.join(f"{t['expected_time_s']:.0f} s 时 Y≈{t['measured_y_m']:.4f} m" for t in result['turning_points'])
text=f'''# 采煤工作面场景测试记录（2026-10-28）

**结论：{'基础场景功能检查通过' if verdict else '存在未通过项，见 verdict.json'}；精度验收有保留，完整 V1.0 扰动闭环尚未验收。**

| 项目 | 实测结果 |
|---|---|
| 布局与几何 | 两巷直段各 60 m、工作面 200 m；192 个模型全部加载。18 个隧道实例、17 处接口，设备未检出几何干涉；支架顶部采样最小净空 {geometry['support_top_roof_gap_min_m']*100:.2f} cm。 |
| 地板与离线 | 4 处地板落体探针通过。空 HOME、仅回环网络、无外网路由下，完整世界加载并输出传感器 20 s；与本轮世界文件哈希相同。 |
| 运动 | 完整运行 {result['runtime']['duration_sim_s']:.2f} 仿真秒；{turns}；最高速度 {motion['peak_speed_ms']:.4f} m/s。 |
| 真值与传感器 | 真值 {rates['truth']['hz_sim']:.2f} Hz、IMU {rates['imu']['hz_sim']:.2f} Hz、激光 {rates['scan']['hz_sim']:.3f} Hz，每帧 720 点；全程未见超过 1.5 倍标称周期的时间戳缺口。 |
| TF 与几何对齐 | {tf['resolved']} 帧扫描已查到对应时刻的 world→laser_link；停机尾部待处理 {tf['pending_at_shutdown']} 帧。{scan['scan_frames']} 帧、{scan['finite_beams']} 条有效射线独立核对，95% 误差≤{scan['p95_error_m']*1000:.3f} mm；{scan['beams_over_5cm']} 条偏差超过 5 cm，最大 {scan['max_error_m']*1000:.2f} mm，需继续核查。RViz 叠加状态正常，截图另附。 |
| 资源 | 无 Gazebo GUI 长测（软件渲染），实时因子中位数 {perf['rtf_median']:.3f}，gzserver 峰值内存约 {perf['gzserver_peak_rss_mb']:.0f} MB；非所有地图/设备的性能结论。 |
| V1.0 闭环 | 未通过完整闭环验收：尚无 UWB 原始话题、基站 YAML，以及扰动注入—误差统计联测。UWB/粉尘算法由对应师弟负责。现有 collision 几何可供后续遮挡接口接入。 |

**需要保留的限制：**采煤机采用运动学状态设置，不代表真实接触/牵引动力学；IMU 是理想运动学输出；激光是二维 LaserScan。控制参考话题最长间隔 {rates['reference']['max_dt_s']:.3f} s，轨迹位置误差 RMS {motion['ideal_position_rmse_m']*1000:.2f} mm、最大 {motion['ideal_position_max_error_m']*100:.2f} cm，不能写成“零误差”。支架参数仅为示意。

**本次改动原因：**原 200 m 场景只有静态预览，原运动脚本限定单块隧道范围，因此增加独立 `coal_face_full.launch.py` 与 `coal_face.sdf` 进行全程实测；原小场景世界及运动脚本哈希保持一致。测试使用 ROS 域 90 / Gazebo 端口 11990，结束后关闭本次进程。

**测试过程说明：**首轮在读取录制中的 SQLite 时发生读写锁冲突，保留为 `full_scale_attempt1`；第二轮完整重跑，结束录包后才分析数据库。录包与独立观察者在共同时间窗内的时间戳一致性见 `bag_consistency.json`。

**成果位置：**本目录 `acceptance_results.json`、`verdict.json`、`motion_verification.png`、`rviz_scan_overlay.png`；原始录包位于同级 `rosbag/`。静态与离线证据路径见 `verdict.json`。

启动正式大场景（Ubuntu22 终端）：

```bash
source /opt/ros/humble/setup.bash
source ~/coal_sim_ws/install/setup.bash
ROS_DOMAIN_ID=90 GAZEBO_MASTER_URI=http://127.0.0.1:11990 LIBGL_ALWAYS_SOFTWARE=1 ros2 launch coal_face_sim coal_face_full.launch.py gui:=true rviz:=true
```
'''
(out/'测试结果-精简版.md').write_text(text,encoding='utf-8')
print(json.dumps(report,indent=2,ensure_ascii=False))
