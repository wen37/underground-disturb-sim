#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Load the full world in a fresh network namespace with only loopback."""
import json, os, signal, subprocess, sys, tempfile, time
from pathlib import Path

def main():
    if '--inside' not in sys.argv:
        root=Path(__file__).resolve().parents[1]
        return subprocess.call(['unshare','-Urn',sys.executable,__file__,'--inside',str(root)])
    root=Path(sys.argv[-1]);out=root/'validation/full_scale'
    subprocess.run(['ip','link','set','lo','up'],check=True)
    links=subprocess.check_output(['ip','-j','link'],text=True)
    routes=subprocess.check_output(['ip','route'],text=True)
    with tempfile.TemporaryDirectory(prefix='coal_full_offline_') as fresh:
        env=os.environ.copy();env.update(HOME=fresh,ROS_LOG_DIR=fresh+'/roslog',ROS_DOMAIN_ID='91',GAZEBO_MASTER_URI='http://127.0.0.1:11991',GAZEBO_MODEL_PATH=str(root/'models'),GAZEBO_MODEL_DATABASE_URI='',LIBGL_ALWAYS_SOFTWARE='1')
        os.environ.update(env)
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from gazebo_msgs.msg import ModelStates
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        rclpy.init();node=Node('offline_full_acceptance');names=[];stamps=[];counts={'scan':0,'truth':0}
        def states(m):names[:] = m.name
        def clock(m):stamps.append(m.clock.sec+m.clock.nanosec*1e-9)
        def inc(k):
            def callback(m):counts[k]+=1
            return callback
        subs=[node.create_subscription(ModelStates,'/model_states',states,10),node.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data),node.create_subscription(LaserScan,'/scan_raw',inc('scan'),qos_profile_sensor_data),node.create_subscription(Odometry,'/ground_truth',inc('truth'),qos_profile_sensor_data)]
        start=time.monotonic();log=open(out/'offline_gzserver.log','w')
        proc=subprocess.Popen(['gzserver','--verbose',str(root/'worlds/coal_face.sdf'),'-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while proc.poll() is None and time.monotonic()-start<100:
                rclpy.spin_once(node,timeout_sec=.05)
                if stamps and stamps[-1]-stamps[0]>=20:break
            duration=stamps[-1]-stamps[0] if len(stamps)>1 else 0
            report={'network_links':json.loads(links),'routes':routes,'fresh_home':fresh,'model_path':str(root/'models'),'model_count':len(names),'sim_duration_s':duration,'scan_count':counts['scan'],'truth_count':counts['truth'],'wall_s':time.monotonic()-start,'passed':len(names)==192 and duration>=20 and counts['scan']>=190 and counts['truth']>=950 and not routes.strip(),'scope':'Full world load and raw sensor publication offline; full motion cycle tested separately.'}
            (out/'offline_full_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
        finally:
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM)
            log.close();node.destroy_node();rclpy.shutdown()
    return 0

if __name__=='__main__':raise SystemExit(main())
