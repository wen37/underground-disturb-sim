#!/usr/bin/env python3
"""ROS2 adapter for the preserved dust interface; no localization algorithm."""
import argparse,csv,json,math,sys,time,copy
from collections import deque,defaultdict
from datetime import datetime
from pathlib import Path
from random import Random
from zoneinfo import ZoneInfo
import yaml,rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped,PoseStamped
from std_msgs.msg import String,Float64
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster,Buffer,TransformListener
from rclpy.time import Time
from openpyxl import Workbook
from heading_math import path_concentration,route_pose,validate_zones
from paths import ROOT, FIXED
sys.path.insert(0,str(FIXED))
import dust_model

def rpy(q):
 return [math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))),math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
def quat(yaw):return (0.,0.,math.sin(yaw/2),math.cos(yaw/2))
def xyz(p):return [p.position.x,p.position.y,p.position.z]

class Experiment(Node):
 def __init__(self,args):
  super().__init__('heading_experiment',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.args=args;self.scene=ROOT/'scenes/S01_subt_heading'
  self.cfg=yaml.safe_load((self.scene/'scenario.yaml').read_text());self.sem=yaml.safe_load((self.scene/'scene_semantics.yaml').read_text());self.route=yaml.safe_load((self.scene/'demo_route.yaml').read_text())
  self.bounds=validate_zones(self.sem['zones'])
  self.sensor_qos=QoSProfile(depth=256,reliability=ReliabilityPolicy.BEST_EFFORT)
  self.phase_qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.phase_pub=self.create_publisher(String,'/heading/experiment_phase',self.phase_qos)
  self.first_scan=None;self.last_scan=None;self.stable_since=None;self.preheat_count=0;self.preheat_gaps=[];self.received=0;self.formal_gaps=[];self.last_formal=None;self.last_received=None;self.processing_times=[];self.max_pending=0;self.failure_reason=None
  self.out=ROOT/'results'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%dT%H%M%S')
  self.out.mkdir(parents=True,exist_ok=False)
  self.file=(self.out/'beam_detail.csv').open('w',newline='');self.writer=csv.writer(self.file)
  self.columns=['frame_id','stamp_s','elapsed_s','beam_index','angle_rad','raw_range_m','truth_range_m','baseline_range_m','mock_range_m','valid','state','zone_id','path_average_concentration_mg_m3','work_mode','body_x_m','body_y_m','body_z_m','body_roll_rad','body_pitch_rad','body_yaw_rad','target_x_m','target_y_m','target_z_m','pose_time_offset_s','center_theory_m','center_error_m']
  self.writer.writerow(self.columns);self.rows=0;self.frames=[];self.history=deque(maxlen=2000);self.pending=deque();self.start=None;self.last_control=-1;self.future=None;self.frame_count=0;self.dropped=0;self.done=False;self.last_record=-1;self.fixed_seen=[]
  self.baseline_fail=0;self.mock_fail=0;self.valid_beams=0;self.invalid_beams=0;self.sums=defaultdict(lambda:[0,0.,0.]);self.rng=Random(42);self.command_fail=[]
  self.tf=TransformBroadcaster(self);self.static_tf=StaticTransformBroadcaster(self);self.buffer=Buffer();self.listener=TransformListener(self.buffer,self);self.tf_checks={}
  self.pub_base=self.create_publisher(LaserScan,'/heading/scan_baseline',10);self.pub_mock=self.create_publisher(LaserScan,'/heading/scan_mock',10)
  self.pub_truth=self.create_publisher(PoseStamped,'/heading/body_truth',10);self.pub_target=self.create_publisher(PoseStamped,'/heading/target_truth',10);self.pub_mode=self.create_publisher(String,'/heading/work_mode',10)
  self.command_pub=self.create_publisher(Float64,'/heading/trajectory_start',1)
  self.create_subscription(ModelStates,'/heading/model_states',self.states,self.sensor_qos)
  self.create_subscription(Odometry,'/heading/body_odometry',self.odometry,self.sensor_qos)
  self.create_subscription(LaserScan,'/heading/scan_raw',self.scan,self.sensor_qos)
  self.timer=self.create_timer(.01,self.tick)
  st=self.cfg['station'];li=self.cfg['sensors']['lidar'];ta=self.cfg['target']
  self.static_tf.sendTransform([self.transform('map','world',[0,0,0],0),self.transform('map','station_link',st['xyz_m'],0),self.transform('station_link','lidar_link',li['xyz_m'],li['rpy_rad'][2]),self.transform('roadheader_base_link','target_link',ta['xyz_m'],0)])
 def seconds(self):return self.get_clock().now().nanoseconds/1e9
 def transform(self,parent,child,pos,yaw,stamp=None):
  t=TransformStamped();t.header.stamp=stamp or self.get_clock().now().to_msg();t.header.frame_id=parent;t.child_frame_id=child
  t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,pos)
  t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=quat(yaw);return t
 def states(self,m):
  if 'robot_01' not in m.name:return
  t=self.seconds();p=m.pose[m.name.index('robot_01')]
  if t<=0:return

  for name in ['station_01']:
   if name in m.name:self.fixed_seen.append(xyz(m.pose[m.name.index(name)]))
 def odometry(self,m):
  t=m.header.stamp.sec+m.header.stamp.nanosec/1e9;p=m.pose.pose
  self.history.append((t,copy.deepcopy(p)))
  self.tf.sendTransform(self.transform('map','roadheader_base_link',xyz(p),rpy(p.orientation)[2],m.header.stamp))
 def phase(self,status):
  self.phase_pub.publish(String(data=json.dumps({'status':status,'run_id':self.out.name,'mode':self.args.mode,'start_s':self.start,'end_s':None if self.start is None else self.start+self.args.duration,'duration_s':self.args.duration})))
 def scan(self,m):
  ts=m.header.stamp.sec+m.header.stamp.nanosec/1e9
  if self.start is None:
   self.preheat_count+=1
   if self.first_scan is None:self.first_scan=ts;self.stable_since=ts
   if self.last_scan is not None and not .024-1e-9<=ts-self.last_scan<=.026+1e-9:
    self.preheat_gaps.append({'stamp_s':ts,'delta_s':ts-self.last_scan});self.stable_since=ts
   self.last_scan=ts
   ready=self.history and abs(self.history[-1][0]-ts)<=.025 and abs(self.seconds()-ts)<.05 and self.fixed_seen and self.command_pub.get_subscription_count()>0
   if not ready:self.stable_since=ts
   if not ready or ts-self.stable_since<2.:return
   self.start=ts;self.phase('MEASURING')
   if self.args.mode=='motion':self.command_pub.publish(Float64(data=ts))
  if ts>self.start+self.args.duration+1e-6:return
  if self.last_formal is not None and not .024-1e-9<=ts-self.last_formal<=.026+1e-9:self.formal_gaps.append({'stamp_s':ts,'delta_s':ts-self.last_formal})
  self.last_formal=ts;self.received+=1;self.last_received=time.monotonic()
  self.pending.append((m,time.monotonic()));self.max_pending=max(self.max_pending,len(self.pending))
 def tick(self):
  if self.start is None:return
  now=self.seconds();elapsed=now-self.start
  # One frame per callback: odometry, scan reception and clock must get turns.
  if self.pending and self.history:
   m,arrival=self.pending[0];ts=m.header.stamp.sec+m.header.stamp.nanosec/1e9
   if self.history[-1][0]>=ts or time.monotonic()-arrival>2.:
    self.pending.popleft();tp,p=min(self.history,key=lambda a:abs(a[0]-ts))
    if abs(tp-ts)>.025:self.dropped+=1
    else:
     began=time.perf_counter();self.process(m,ts,p,abs(tp-ts));self.processing_times.append(time.perf_counter()-began)
  for frame in ['world','station_link','lidar_link','roadheader_base_link','target_link']:
   try:
    t=self.buffer.lookup_transform('map',frame,Time());self.tf_checks[frame]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
   except Exception:pass
  if elapsed>=self.args.duration+.2 and not self.pending:self.done=True
 def process(self,m,ts,p,offset):
  elapsed=ts-self.start;mode=route_pose(elapsed,self.route)[1] if self.args.mode=='motion' else 'IDLE'
  self.frame_count+=1;body=xyz(p);angles=rpy(p.orientation);target=[body[0],body[1]-5.9,body[2]+.585]
  lidar=[0.,57.1,1.934];theory=target[1]-.01-lidar[1]
  center=len(m.ranges)//2;err=abs(m.ranges[center]-theory)
  self.frames.append([ts,elapsed,mode,len(m.ranges),m.ranges[center],theory,err,offset,*body,*target])
  for pub,pos in [(self.pub_truth,body),(self.pub_target,target)]:
   pm=PoseStamped();pm.header.stamp=m.header.stamp;pm.header.frame_id='map';pm.pose.position.x,pm.pose.position.y,pm.pose.position.z=pos;pm.pose.orientation=p.orientation;pub.publish(pm)
  self.pub_mode.publish(String(data=mode))
  baseline=copy.deepcopy(m);mock=copy.deepcopy(m);record=elapsed-self.last_record>=.099
  if record:self.last_record=elapsed
  for i,raw in enumerate(m.ranges):
   valid=math.isfinite(raw) and m.range_min<=raw<=m.range_max;angle=m.angle_min+i*m.angle_increment
   if valid:
    endpoint=[lidar[0]-raw*math.sin(angle),lidar[1]+raw*math.cos(angle),lidar[2]]
    data=path_concentration(lidar,endpoint,self.sem['zones'],self.sem['background_concentration_mg_m3'],prepared_bounds=self.bounds);c=data['path_average_concentration_mg_m3'];zone=data['zone_id']
    b=dust_model.measure(raw_range_m=float(raw),concentration_mg_m3=c,enabled=False,rng=self.rng);v=dust_model.measure(raw_range_m=float(raw),concentration_mg_m3=c,enabled=True,rng=self.rng)
    baseline.ranges[i]=b.observed_range_m;mock.ranges[i]=v.observed_range_m;state=v.state;self.valid_beams+=1
    self.baseline_fail+=b.observed_range_m!=raw;self.mock_fail+=abs(v.observed_range_m-raw-.02)>1e-9
    key=(mode,zone);self.sums[key][0]+=1;self.sums[key][1]+=v.observed_range_m-raw;self.sums[key][2]+=(v.observed_range_m-raw)**2
   else:
    c=None;zone='NO_RETURN';state='NO_RETURN';self.invalid_beams+=1
   if record:
    self.writer.writerow([self.frame_count,ts,elapsed,i,angle,raw,raw,baseline.ranges[i],mock.ranges[i],valid,state,zone,c,mode,*body,*angles,*target,offset,theory if i==center else '',err if i==center else '']);self.rows+=1
  self.pub_base.publish(baseline);self.pub_mock.publish(mock)
 def finish(self):
  self.file.close();hz=(len(self.frames)-1)/(self.frames[-1][0]-self.frames[0][0]) if len(self.frames)>1 else 0
  mode_set=sorted(set(f[2] for f in self.frames));maxerr=max((f[6] for f in self.frames),default=math.inf)
  summary={'mode':self.args.mode,'frames':len(self.frames),'scan_hz_sim_time':hz,'all_beam_count_1081':all(f[3]==1081 for f in self.frames),'max_center_error_m':maxerr,'max_pose_time_offset_s':max((f[7] for f in self.frames),default=None),'dropped_unaligned_frames':self.dropped,'valid_beams_checked':self.valid_beams,'invalid_beams_preserved':self.invalid_beams,'baseline_failures':self.baseline_fail,'mock_failures':self.mock_fail,'csv_rows':self.rows,'csv_record_hz':10,'processed_scan_hz':40,'work_modes':mode_set,'body_start_y_m':self.frames[0][9] if self.frames else None,'body_end_y_m':self.frames[-1][9] if self.frames else None,'station_fixed':all(math.dist(v,[0,57.1,.134])<1e-9 for v in self.fixed_seen) and bool(self.fixed_seen),'tf_map_lookup':self.tf_checks,'command_failures':self.command_fail,'truth_definition':'Noise-free Gazebo ray first-return is per-beam truth; center ray independently checked analytically; Gazebo P3D native simulation timestamp, nearest pose sample within25ms; map and world identity.','dust_basis':self.sem['concentration_basis']}
  summary.update(warmup={'frames':self.preheat_count,'first_stamp_s':self.first_scan,'formal_start_s':self.start,'gaps':self.preheat_gaps,'stable_required_sim_s':2.0},formal_received_frames=self.received,formal_interval_jitter_count=len(self.formal_gaps),formal_interval_jitter=self.formal_gaps,max_pending_frames=self.max_pending,processing_wall_ms_max=max(self.processing_times,default=0)*1000,processing_wall_ms_mean=sum(self.processing_times)/max(1,len(self.processing_times))*1000,expected_frames=round(self.args.duration*40)+1,failure_reason=self.failure_reason)
  summary['passed']=len(self.frames)>=80 and abs(hz-40)<1 and maxerr<.01 and self.baseline_fail==0 and self.mock_fail==0 and len(self.tf_checks)==5 and summary['station_fixed'] and not self.command_fail
  summary['expected_frame_count_range']=[summary['expected_frames']-1,summary['expected_frames']]
  summary['source_long_gap_count']=sum(g['delta_s']>=.05-1e-9 for g in self.formal_gaps)
  summary['missing_expected_frames']=max(0,summary['expected_frames']-1-self.received)
  summary['passed'] &= self.dropped==0 and self.received==len(self.frames) and summary['expected_frames']-1<=self.received<=summary['expected_frames'] and summary['source_long_gap_count']==0 and self.failure_reason is None
  if self.args.mode=='motion':summary['passed'] &= set(mode_set)=={'IDLE','TRAVEL','CUTTING_SIMULATED'} and abs(summary['body_end_y_m']-70)<.01
  failures=[]
  checks={'frequency_40hz':abs(hz-40)<1,'center_error_under_1cm':maxerr<.01,'beam_count_1081':summary['all_beam_count_1081'],'baseline_values':self.baseline_fail==0,'mock_values':self.mock_fail==0,'tf_frames':len(self.tf_checks)==5,'station_fixed':summary['station_fixed'],'motion_commands':not self.command_fail,'zero_alignment_drops':self.dropped==0,'all_received_processed':self.received==len(self.frames),'window_frame_count':summary['expected_frames']-1<=self.received<=summary['expected_frames'],'no_source_long_gap':summary['source_long_gap_count']==0,'no_runtime_error':self.failure_reason is None}
  if self.args.mode=='motion':checks.update(work_modes=set(mode_set)=={'IDLE','TRAVEL','CUTTING_SIMULATED'},final_position=summary['body_end_y_m'] is not None and abs(summary['body_end_y_m']-70)<.01)
  summary['failed_checks']=[name for name,ok in checks.items() if not ok]
  summary['passed']=summary['passed'] and not summary['failed_checks']
  if summary['failed_checks'] and not summary['failure_reason']:summary['failure_reason']=', '.join(summary['failed_checks'])
  self.phase('COMPLETE' if summary['passed'] else 'FAILED')
  (self.out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));(self.out/'config_snapshot.yaml').write_text(yaml.safe_dump({'scenario':self.cfg,'semantics':self.sem,'route':self.route},allow_unicode=True))
  wb=Workbook();ws=wb.active;ws.title='验收摘要';ws.append(['项目','值'])
  for k,v in summary.items():ws.append([k,json.dumps(v,ensure_ascii=False) if isinstance(v,(dict,list)) else v])
  ws=wb.create_sheet('逐帧真值');ws.append(['stamp_s','elapsed_s','work_mode','beam_count','center_raw_m','center_theory_m','center_error_m','pose_time_offset_s','body_x_m','body_y_m','body_z_m','target_x_m','target_y_m','target_z_m'])
  for f in self.frames:ws.append(f)
  ws=wb.create_sheet('分区工况统计');ws.append(['work_mode','zone_id','valid_beam_count','mean_bias_m','RMSE_m'])
  for (mode,zone),(n,total,ss) in sorted(self.sums.items()):ws.append([mode,zone,n,total/n,math.sqrt(ss/n)])
  for ws in wb:
   ws.freeze_panes='A2';ws.auto_filter.ref=ws.dimensions
   for col in ws.columns:ws.column_dimensions[col[0].column_letter].width=24
  wb.save(self.out/'场景实验报表.xlsx')
  print(json.dumps(summary,ensure_ascii=False,indent=2));print('RESULT',self.out);return summary['passed']

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--mode',choices=['static','motion'],default='static');parser.add_argument('--duration',type=float,default=3);args=parser.parse_args()
 rclpy.init();node=Experiment(args);deadline=time.monotonic()+180
 try:
  while not node.done and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
  if not node.done:node.failure_reason="wall-clock deadline exceeded; inspect warmup and received counts"
  ok=node.finish()
  until=time.monotonic()+.3
  while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
 finally:node.destroy_node();rclpy.shutdown()
 return 0 if ok else 1
if __name__=='__main__':raise SystemExit(main())
