#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Run an isolated 1200 s full-cycle test, collecting timestamp-level evidence."""
import bisect, json, math, os, signal, subprocess, time, traceback
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from tf2_ros import Buffer, TransformListener

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'validation/control_fix_20261029/run_02'
OUT.mkdir(parents=True, exist_ok=True)
def stamp(m): return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def write(name,data):
    temp=OUT/(name+'.tmp');temp.write_text(json.dumps(data,indent=2,ensure_ascii=False));temp.replace(OUT/name)

class Monitor(Node):
    def __init__(self):
        super().__init__('full_scale_acceptance')
        self.rows={k:[] for k in ['truth','reference','imu','scan']}
        self.clock=None;self.epoch=None;self.started=time.monotonic();self.samples=[];self.lastsample=-100
        self.pending=[];self.tf_ok=0;self.tf_error=[];self.subs=[]
        qos=QoSProfile(depth=5000,reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic,typ,key in [('/ground_truth',Odometry,'truth'),('/shearer/reference',Odometry,'reference'),('/imu_raw',Imu,'imu'),('/scan_raw',LaserScan,'scan')]:
            self.subs.append(self.create_subscription(typ,topic,lambda m,k=key:self.receive(k,m),qos))
        self.subs.append(self.create_subscription(Clock,'/clock',self.onclock,qos))
        self.buffer=Buffer(cache_time=Duration(seconds=60))
        self.listener=TransformListener(self.buffer,self)
    def onclock(self,m):self.clock=m.clock.sec+m.clock.nanosec*1e-9
    def receive(self,key,m):
        t=stamp(m)
        if key in ['truth','reference']:
            p=m.pose.pose.position;v=m.twist.twist.linear;q=m.pose.pose.orientation
            self.rows[key].append([t,p.x,p.y,p.z,v.x,v.y,v.z,q.x,q.y,q.z,q.w])
            if key=='reference' and self.epoch is None:
                angle=math.atan2(v.y/(95*2*math.pi/1200),(100-p.y)/95)
                self.epoch=t-angle/(2*math.pi/1200)
        elif key=='imu':
            a=m.linear_acceleration;v=m.angular_velocity
            self.rows[key].append([t,a.x,a.y,a.z,v.x,v.y,v.z])
        else:
            ranges=np.array(m.ranges)
            self.rows[key].append([t,len(ranges),int(np.isfinite(ranges).sum())])
            self.pending.append(t)
            if t-self.lastsample>=50:
                self.samples.append({'stamp':t,'angle_min':m.angle_min,'angle_increment':m.angle_increment,'range_min':m.range_min,'range_max':m.range_max,'ranges':[float(x) if math.isfinite(x) else None for x in ranges]})
                self.lastsample=t
    def check_tf(self):
        pending=[]
        for t in self.pending:
            try:
                tf=self.buffer.lookup_transform('world','laser_link',Time(nanoseconds=round(t*1e9)))
                p=tf.transform.translation
                self.tf_ok+=1
                truth=self.rows['truth']
                # Translation expected from ground-truth body plus fixed .55 m mount.
                if truth:
                    index=bisect.bisect_left(truth,t,key=lambda row:row[0])
                    j=min([max(0,index-1),min(index,len(truth)-1)],key=lambda j:abs(truth[j][0]-t))
                    row=truth[j]
                    self.tf_error.append([t,abs(row[0]-t),abs(p.x-row[1]),abs(p.y-row[2]),abs(p.z-row[3]-.55)])
            except Exception:
                pending.append(t)
        self.pending=pending

def rate_report(rows):
    a=np.array(rows)
    if len(a)<2:return {'count':len(a),'pass':False}
    dt=np.diff(a[:,0]);typ=float(np.median(dt));gaps=dt>typ*1.51
    return {'count':len(a),'first_s':float(a[0,0]),'last_s':float(a[-1,0]),'hz_sim':float((len(a)-1)/(a[-1,0]-a[0,0])),
        'median_dt_s':typ,'max_dt_s':float(dt.max()),'gaps_over_1_5_nominal':int(gaps.sum()),'duplicate_or_reversed_stamps':int((dt<=0).sum())}

def main():
    os.environ.update(ROS_DOMAIN_ID='93',GAZEBO_MASTER_URI='http://127.0.0.1:11993',LIBGL_ALWAYS_SOFTWARE='1',GAZEBO_MODEL_DATABASE_URI='')
    if (OUT/'rosbag').exists():raise RuntimeError('Existing evidence bag; use a fresh output directory before rerunning')
    rclpy.init();node=Monitor();processes=[];files=[];error=None
    try:
        for name,cmd in [('bag',['ros2','bag','record','-o',str(OUT/'rosbag'),'--use-sim-time','--max-cache-size','100000000','/clock','/ground_truth','/shearer/reference','/scan_raw','/imu_raw','/tf','/tf_static']),('launch',['ros2','launch','coal_face_sim','coal_face_full.launch.py','gui:=false','rviz:=false'])]:
            f=open(OUT/(name+'.log'),'w');files.append(f)
            processes.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,env=os.environ.copy()))
        last=0;history=[];done=False;accelerated=False
        while time.monotonic()-node.started<3000:
            rclpy.spin_once(node,timeout_sec=.02)
            now=time.monotonic()
            if node.clock is not None and node.clock>=30 and not accelerated:
                subprocess.run(['gz','physics','-w','coal_face_v1','-u','10000'],env=os.environ.copy(),check=True,timeout=15)
                accelerated=True
            if now-last>=5:
                node.check_tf()
                elapsed=now-node.started;phase=None if node.epoch is None or node.clock is None else node.clock-node.epoch
                status={'wall_elapsed_s':elapsed,'sim_time_s':node.clock,'trajectory_elapsed_s':phase,'counts':{k:len(v) for k,v in node.rows.items()},'tf_resolved':node.tf_ok,'tf_pending':len(node.pending),'process_alive':[p.poll() is None for p in processes]}
                history.append(status);write('progress.json',status)
                print(json.dumps(status),flush=True);last=now
                if not all(status['process_alive']):raise RuntimeError('A test subprocess exited')
                if elapsed>120 and node.epoch is None:raise RuntimeError('No trajectory after 120 wall seconds')
                if phase is not None and phase>=1205:done=True;break
        if not done:raise RuntimeError('Wall-time deadline before completing full cycle')
    except Exception as exc:
        error=str(exc);traceback.print_exc()
    finally:
        # Let TF callbacks catch up before ending simulation.
        for _ in range(20):rclpy.spin_once(node,timeout_sec=.01)
        node.check_tf()
        for p in reversed(processes):
            # Let ROS launch relay SIGINT once to its children.
            if p.poll() is None:p.send_signal(signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)
        for f in files:f.close()
    arrays={k:np.array(v) for k,v in node.rows.items()};np.savez_compressed(OUT/'samples.npz',**arrays,tf_checks=np.array(node.tf_error))
    write('scan_samples.json',node.samples)
    rates={k:rate_report(v) for k,v in node.rows.items()}
    report={'error':error,'epoch_sim_s':node.epoch,'duration_sim_s':None if node.epoch is None else node.clock-node.epoch,'wall_elapsed_s':time.monotonic()-node.started,'rates':rates,'tf':{'resolved':node.tf_ok,'pending_at_shutdown':len(node.pending)},'history':history,
        'limits':['IMU is ideal kinematic differentiation, not a vibration/contact model.','LaserScan is planar, not a 3D point cloud.','UWB and disturbance injectors are not supplied by this platform test.','Physics-step kinematic plugin drives a non-gravity body; this does not validate traction/contact dynamics. First 30 sim seconds at 1x, remainder target 10x; physics step remains 1 ms.']}
    if len(arrays['truth']) and node.epoch is not None:
        a=arrays['truth'];phase=a[:,0]-node.epoch;mask=phase>=2;a=a[mask];phase=phase[mask]
        expected=100-95*np.cos(2*np.pi*phase/1200)
        report['motion']={'y_min_m':float(a[:,2].min()),'y_max_m':float(a[:,2].max()),'last_y_m':float(a[-1,2]),'peak_speed_ms':float(np.abs(a[:,5]).max()),'ideal_position_rmse_m':float(np.sqrt(np.mean((a[:,2]-expected)**2))),'ideal_position_max_error_m':float(np.abs(a[:,2]-expected).max()),'x_max_deviation_m':float(np.abs(a[:,1]-.6).max()),'z_max_deviation_m':float(np.abs(a[:,3]-.6605).max())}
    write('runtime_report.json',report)
    node.destroy_node();rclpy.shutdown()
    print('REPORT',str(OUT/'runtime_report.json'),flush=True)
    if error:raise SystemExit(1)

if __name__=='__main__':main()
