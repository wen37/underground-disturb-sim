#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Independent model-state layout and floor-contact probes in test domain 90."""
import json, math, os, time
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='90'
os.environ['GAZEBO_MASTER_URI']='http://127.0.0.1:11990'
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetEntityState

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'validation/full_scale'
cfg=json.loads((ROOT/'design_v02/layout_parameters.json').read_text())
def call(node,client,req,timeout=15):
    if not client.wait_for_service(timeout_sec=timeout):raise RuntimeError('Service missing '+client.srv_name)
    fut=client.call_async(req);rclpy.spin_until_future_complete(node,fut,timeout_sec=timeout)
    if not fut.done():raise RuntimeError('Service timeout '+client.srv_name)
    result=fut.result()
    if not result.success:raise RuntimeError(str(result))
    return result

def main():
    rclpy.init();node=Node('full_scale_geometry_probes');states=[]
    sub=node.create_subscription(ModelStates,'/model_states',states.append,10)
    end=time.monotonic()+20
    while not states and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    if not states:raise RuntimeError('No Gazebo model states')
    last=states[-1];poses=dict(zip(last.name,last.pose))
    expected={t['name']:t['position_xyz_m'] for t in cfg['tile_instances']}
    expected.update({o['name']:o['center_xyz_m'] for o in cfg['equipment']})
    errors={}
    for name,pos in expected.items():
        if name not in poses:errors[name]='missing';continue
        p=poses[name].position;e=max(abs(a-b) for a,b in zip([p.x,p.y,p.z],pos))
        if e>1e-6:errors[name]=e
    report={'loaded_model_count':len(last.name),'expected_model_count':len(expected)+1,'missing_or_misplaced':errors,'shearer_present':'shearer' in poses,'floor_probes':[]}
    spawn=node.create_client(SpawnEntity,'/spawn_entity');delete=node.create_client(DeleteEntity,'/delete_entity');get=node.create_client(GetEntityState,'/get_entity_state')
    owned=[]
    try:
        for i,(x,y) in enumerate([(-.2,20),(-.2,100),(-.2,180),(-10.1,-10)]):
            name=f'acceptance_floor_probe_{i}'
            req=SpawnEntity.Request();req.name=name;req.reference_frame='world';req.initial_pose.position.x=float(x);req.initial_pose.position.y=float(y);req.initial_pose.position.z=1.0;req.initial_pose.orientation.w=1.0
            req.xml='<sdf version="1.6"><model name="probe"><link name="body"><inertial><mass>1</mass><inertia><ixx>.0066667</ixx><iyy>.0066667</iyy><izz>.0066667</izz></inertia></inertial><collision name="box"><geometry><box><size>.2 .2 .2</size></box></geometry></collision></link></model></sdf>'
            call(node,spawn,req);owned.append(name)
        until=time.monotonic()+6
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        for name in owned:
            req=GetEntityState.Request();req.name=name;req.reference_frame='world';state=call(node,get,req).state
            p=state.pose.position;v=state.twist.linear
            report['floor_probes'].append({'name':name,'position_xyz_m':[p.x,p.y,p.z],'velocity_xyz_ms':[v.x,v.y,v.z],'expected_center_z_m':.1005,'passed':abs(p.z-.1005)<.005 and abs(v.z)<.01})
    finally:
        for name in owned:
            req=DeleteEntity.Request();req.name=name
            try:call(node,delete,req)
            except Exception as exc:report.setdefault('cleanup_errors',[]).append(str(exc))
        report['passed']=not errors and report['shearer_present'] and len(report['floor_probes'])==4 and all(p['passed'] for p in report['floor_probes']) and not report.get('cleanup_errors')
        (OUT/'live_layout_floor_report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
