#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Transient RViz overlay of collision sections, for inspection only."""
import json,os,signal,subprocess,time
from pathlib import Path
os.environ.update(ROS_DOMAIN_ID='90',GAZEBO_MASTER_URI='http://127.0.0.1:11990',LIBGL_ALWAYS_SOFTWARE='1')
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from analyze_full_scale import collision_segments
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'validation/full_scale'
cfg=json.loads((ROOT/'design_v02/layout_parameters.json').read_text())
segments,z=collision_segments(cfg)
import yaml
config=yaml.safe_load((ROOT.parents[1]/'config/coal_face_lidar.rviz').read_text())
config['Visualization Manager']['Displays'].append({'Class':'rviz_default_plugins/Marker','Name':'Collision section at laser height','Enabled':True,'Topic':{'Value':'/platform_test/collision_sections','Reliability Policy':'Reliable','Durability Policy':'Transient Local','Depth':1},'Queue Size':10})
config['Visualization Manager']['Views']['Current'].update({'Target Frame':'base_link','Scale':50})
(OUT/'scan_overlay.rviz').write_text(yaml.safe_dump(config,sort_keys=False))
rclpy.init();node=Node('collision_section_overlay')
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
pub=node.create_publisher(Marker,'/platform_test/collision_sections',qos)
m=Marker();m.header.frame_id='world';m.ns='collision_sections';m.id=0;m.type=Marker.LINE_LIST;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=.015;m.color.r=.1;m.color.g=.85;m.color.b=1.;m.color.a=1.
for seg in segments:
    for xy in seg:
        p=Point();p.x=float(xy[0]);p.y=float(xy[1]);p.z=float(z-.005);m.points.append(p)
node.create_timer(1.,lambda:pub.publish(m));pub.publish(m)
log=open(OUT/'rviz.log','w');proc=subprocess.Popen(['rviz2','-d',str(OUT/'scan_overlay.rviz'),'--ros-args','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
    end=time.monotonic()+90
    while proc.poll() is None and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
finally:
    if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=10)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM)
    log.close();node.destroy_node();rclpy.shutdown()
