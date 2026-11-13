import json
import os
from pathlib import Path
import time
os.environ['ROS_DOMAIN_ID']='89'
os.environ['GAZEBO_MASTER_URI']='http://127.0.0.1:11889'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

BASE=Path(__file__).resolve().parents[1]
cfg=json.loads((BASE/'design_v02/layout_parameters.json').read_text())
expected={t['name'] for t in cfg['tile_instances']}|{o['name'] for o in cfg['equipment']}|{'shearer_design'}
rclpy.init();node=Node('design_preview_check')
states=[];clocks=[]
subs=[node.create_subscription(ModelStates,'/design_preview/model_states',states.append,10),
      node.create_subscription(Clock,'/clock',clocks.append,qos_profile_sensor_data)]
end=time.monotonic()+30
while time.monotonic()<end:
    rclpy.spin_once(node,timeout_sec=.05)
    if len(states)>=2 and len(clocks)>=20:break
names=set(states[-1].name) if states else set()
stamps=[m.clock.sec+m.clock.nanosec*1e-9 for m in clocks]
report={'expected_model_count':len(expected),'loaded_model_count':len(names),'missing_models':sorted(expected-names),
        'clock_samples':len(clocks),'clock_duration_s':stamps[-1]-stamps[0] if len(stamps)>1 else 0,
        'static_preview_only':True,'passed':not (expected-names) and len(states)>=2 and len(stamps)>1 and stamps[-1]>stamps[0]}
(BASE/'design_v02/preview_runtime_report.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
