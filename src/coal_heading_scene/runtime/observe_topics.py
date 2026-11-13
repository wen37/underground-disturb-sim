"""Independent ROS subscriber: check every formal frame, including missing frames."""
import json,math,sys,time
from pathlib import Path
from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time

def main():
    rclpy.init();node=Node('heading_independent_observer')
    qos=QoSProfile(depth=256,reliability=ReliabilityPolicy.BEST_EFFORT)
    phase_qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    samples=defaultdict(dict);stamps=defaultdict(list);phase={};lookups={}
    buffer=Buffer();listener=TransformListener(buffer,node)
    def scan(m,key):
        ts=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        samples[ts][key]=list(m.ranges);stamps[key].append(ts)
    def state(m):phase.update(json.loads(m.data))
    for key in ['raw','baseline','mock']:
        node.create_subscription(LaserScan,'/heading/scan_'+key,lambda m,k=key:scan(m,k),qos)
    node.create_subscription(String,'/heading/experiment_phase',state,phase_qos)
    deadline=time.monotonic()+190;done_at=None;last_tf=0
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.02)
        if time.monotonic()-last_tf>.1:
            last_tf=time.monotonic()
            for frame in ['world','station_link','lidar_link','roadheader_base_link','target_link']:
                try:
                    t=buffer.lookup_transform('map',frame,Time());lookups[frame]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
                except Exception:pass
        if phase.get('status') in ['COMPLETE','FAILED']:
            if done_at is None:done_at=time.monotonic()
            if time.monotonic()-done_at>1.:break
    start=phase.get('start_s');end=phase.get('end_s')
    formal={t:v for t,v in samples.items() if start is not None and start-1e-6<=t<=end+1e-6}
    rates={};gaps={};counts={};long_gaps={}
    for key in ['raw','baseline','mock']:
        ts=[t for t in stamps[key] if start is not None and start-1e-6<=t<=end+1e-6]
        counts[key]=len(ts);rates[key]=(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 else 0
        long_gaps[key]=sum(b-a>=.05-1e-9 for a,b in zip(ts,ts[1:]))
        gaps[key]=sum(not .024-1e-9<=b-a<=.026+1e-9 for a,b in zip(ts,ts[1:]))
    paired=[v for v in formal.values() if len(v)==3];bf=mf=valid=length_fail=0;maxerr=0.
    for f in paired:
        if not len(f['raw'])==len(f['baseline'])==len(f['mock'])==1081:length_fail+=1;continue
        for raw,base,mock in zip(f['raw'],f['baseline'],f['mock']):
            if math.isfinite(raw):
                valid+=1;bf+=raw!=base;error=abs(mock-raw-.02);maxerr=max(maxerr,error);mf+=not math.isfinite(error) or error>1e-6
            else:bf+=math.isfinite(base);mf+=math.isfinite(mock)
    expected=round(phase.get('duration_s',0)*40)+1
    result={'phase':phase,'expected_frame_count_range':[expected-1,expected],'source_long_gap_counts':long_gaps,'topic_frame_counts':counts,'paired_frames':len(paired),'topic_rates_sim_hz':rates,'topic_interval_jitter_counts':gaps,'valid_beams':valid,'baseline_failures':bf,'mock_failures':mf,'length_failures':length_fail,'max_float32_mock_residual_m':maxerr,'tf_map_lookup':lookups}
    result['passed']=phase.get('status')=='COMPLETE' and expected-1<=counts['raw']<=expected and all(v==counts['raw'] for v in counts.values()) and len(paired)==counts['raw'] and not any(long_gaps.values()) and all(abs(h-40)<1 for h in rates.values()) and bf==mf==length_fail==0 and len(lookups)==5
    Path(sys.argv[1]).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    node.destroy_node();rclpy.shutdown();return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
