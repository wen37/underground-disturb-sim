#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Observe the platform interface; do not fabricate teammates' UWB data."""
import json,os,time
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='90'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu,LaserScan
from nav_msgs.msg import Odometry
out=Path(__file__).resolve().parents[1]/'validation/full_scale'
rclpy.init();node=Node('platform_interface_snapshot');seen={};subs=[]
def callback(key):
    def receive(m):
        entry={'frame_id':m.header.frame_id}
        if isinstance(m,Odometry):entry['child_frame_id']=m.child_frame_id
        if isinstance(m,LaserScan):entry.update(samples=len(m.ranges),range_min_m=m.range_min,range_max_m=m.range_max)
        if isinstance(m,Imu):entry.update(orientation_covariance=list(m.orientation_covariance),acceleration_covariance=list(m.linear_acceleration_covariance))
        seen[key]=entry
    return receive
for key,typ in [('/ground_truth',Odometry),('/scan_raw',LaserScan),('/imu_raw',Imu)]:subs.append(node.create_subscription(typ,key,callback(key),qos_profile_sensor_data))
end=time.monotonic()+8
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
topics=dict(node.get_topic_names_and_types())
report={'nodes':node.get_node_names_and_namespaces(),'topics':topics,'observed_messages':seen,'not_present':['/uwb_ranges_raw'] if '/uwb_ranges_raw' not in topics else [],'scene_yaml_files':[str(f) for f in out.parents[1].rglob('*.yaml')],'scope':'Interface presence only. Existing collision SDF can supply geometry; NLOS/dust algorithms and error-statistics closed loop are not exercised.'}
(out/'interface_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));node.destroy_node();rclpy.shutdown()
