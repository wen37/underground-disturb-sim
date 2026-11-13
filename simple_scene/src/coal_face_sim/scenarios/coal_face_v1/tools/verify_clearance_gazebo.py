#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Independent Gazebo/ODE ray checks at representative measured sections."""
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

ROOT=Path(__file__).resolve().parents[1]
OUT=Path('/mnt/d/work1/work1-codex/scenarios/coal_face_v1/measurements/inner_clearance_dense')
os.environ['ROS_DOMAIN_ID']='88';os.environ['GAZEBO_MASTER_URI']='http://127.0.0.1:11888'
os.environ['GAZEBO_MODEL_PATH']=str(ROOT/'models');os.environ['GAZEBO_MODEL_DATABASE_URI']=''

def build():
    rows=list(csv.DictReader((OUT/'inner_clearance_samples.csv').open(encoding='utf-8-sig')))
    records=[]
    sdf=ET.Element('sdf',{'version':'1.6'});world=ET.SubElement(sdf,'world',{'name':'clearance_checks'})
    for model_id,offset in [('subt_tunnel_tile_5',0),('subt_tunnel_tile_2',40),('subt_tunnel_tile_1',80)]:
        own=[r for r in rows if r['model']==model_id and float(r['height_above_floor_m'])==1.]
        selected=[own[len(own)//2],min(own,key=lambda r:float(r['headroom_centerline_m']))]
        indices=set()
        include=ET.SubElement(world,'include');ET.SubElement(include,'uri').text='model://'+model_id
        ET.SubElement(include,'pose').text=f'{offset} 0 0 0 0 0'
        for row in selected:
            if row['station'] in indices:continue
            indices.add(row['station']);x=float(row['x_m']);y=float(row['y_m']);floor=float(row['floor_z_m'])
            if model_id.endswith('_5'):nx,ny=1.,0.
            elif model_id.endswith('_2'):
                vec=np.array([x-10,y+10]);nx,ny=vec/np.linalg.norm(vec)
            elif row['branch'] in ['north','south']:nx,ny=1.,0.
            else:nx,ny=0.,1.
            specs=[('floor',1.,0.,math.pi/2,1.-floor),
                   ('roof',floor+.01,0.,-math.pi/2,float(row['headroom_centerline_m'])-.01)]
            for height in [1.,2.8]:
                hrow=next(r for r in rows if r['model']==model_id and r['station']==row['station'] and float(r['height_above_floor_m'])==height)
                yaw=math.atan2(ny,nx)
                specs.extend([(f'left_{height}',floor+height,yaw,0.,float(hrow['clearance_left_m'])),
                              (f'right_{height}',floor+height,yaw+math.pi,0.,float(hrow['clearance_right_m']))])
            for name,z,yaw,pitch,expected in specs:
                key=f'probe_{len(records)}';topic='/clearance_check/'+key
                probe=ET.SubElement(world,'model',{'name':key});ET.SubElement(probe,'static').text='true'
                ET.SubElement(probe,'pose').text=f'{x+offset} {y} {z} 0 {pitch} {yaw}'
                link=ET.SubElement(probe,'link',{'name':'probe'})
                sensor=ET.SubElement(link,'sensor',{'name':'ray','type':'ray'});ET.SubElement(sensor,'always_on').text='true';ET.SubElement(sensor,'update_rate').text='2'
                ray=ET.SubElement(sensor,'ray');h=ET.SubElement(ET.SubElement(ray,'scan'),'horizontal')
                for tag,val in [('samples','1'),('resolution','1'),('min_angle','0'),('max_angle','0')]:ET.SubElement(h,tag).text=val
                r=ET.SubElement(ray,'range')
                for tag,val in [('min','.001'),('max','10'),('resolution','.0001')]:ET.SubElement(r,tag).text=val
                plugin=ET.SubElement(sensor,'plugin',{'name':key,'filename':'libgazebo_ros_ray_sensor.so'})
                ET.SubElement(ET.SubElement(plugin,'ros'),'remapping').text='~/out:='+topic
                ET.SubElement(plugin,'output_type').text='sensor_msgs/Range';ET.SubElement(plugin,'frame_name').text=key
                records.append({'topic':topic,'model':model_id,'station':int(row['station']),'measurement':name,'expected_range_m':expected})
    ET.indent(sdf,space='  ');ET.ElementTree(sdf).write(OUT/'gazebo_check.sdf',encoding='utf-8',xml_declaration=True)
    return records

def main():
    records=build();samples={r['topic']:[] for r in records}
    with (OUT/'gazebo_check.log').open('w') as log:
        proc=subprocess.Popen(['gzserver',str(OUT/'gazebo_check.sdf'),'-s','libgazebo_ros_init.so'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            rclpy.init();node=Node('independent_clearance_check')
            subs=[node.create_subscription(Range,r['topic'],lambda m,key=r['topic']:samples[key].append(m.range),100) for r in records]
            deadline=time.monotonic()+45
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
                if all(len(v)>=3 for v in samples.values()):break
                if proc.poll() is not None:raise RuntimeError('Gazebo exited')
            errors=[]
            for r in records:
                data=samples[r['topic']]
                r['samples']=len(data)
                r['actual_range_m']=float(np.median(data)) if data else None
                r['error_m']=r['actual_range_m']-r['expected_range_m'] if data else None
                if data:errors.append(abs(r['error_m']))
            report={'method':'Independent gzserver ODE CPU ray sensors, static representative stations, floor/roof and both sides at 1.0 and 2.8 m.',
                'rays':records,'max_abs_error_m':max(errors) if errors else None,
                'passed':all(r['samples']>=3 and r['error_m'] is not None and abs(r['error_m'])<.001 for r in records)}
            (OUT/'gazebo_crosscheck.json').write_text(json.dumps(report,indent=2))
            print(json.dumps(report,indent=2),flush=True)
            node.destroy_node();rclpy.shutdown()
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)

if __name__=='__main__':main()
