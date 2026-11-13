#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Assess live evidence, source collision geometry and recorded messages."""
import hashlib, json, math, sqlite3, os
from pathlib import Path
import numpy as np
from measure_clearance import collision_triangles, section
from measure_tiles import rotation, transform

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'validation'/os.environ.get('COAL_TEST_RUN','full_scale')
def read(name):return json.loads((OUT/name).read_text())
def write(name,obj):(OUT/name).write_text(json.dumps(obj,indent=2,ensure_ascii=False))

def collision_segments(cfg):
    # Intersect the actual collision meshes with the horizontal laser plane.
    z=cfg['motion']['base_z_m']+.55;segments=[];templates={}
    for tile in cfg['tile_instances']:
        kind=tile['model']
        if kind not in templates:templates[kind]=section(collision_triangles(ROOT/'models'/kind,ROOT/'models'),np.array([0,0,1]),z)
        local=templates[kind];matrix=rotation([0,0,1],tile['yaw_rad']);matrix[:3,3]=tile['position_xyz_m']
        segments.extend(transform(local.reshape(-1,3),matrix).reshape(-1,2,3)[:,:,:2])
    for obj in cfg['equipment']:
        c=np.array(obj['center_xyz_m']);h=np.array(obj['size_xyz_m'])/2
        if not c[2]-h[2]<z<c[2]+h[2]:continue
        p=np.array([[c[0]-h[0],c[1]-h[1]],[c[0]+h[0],c[1]-h[1]],[c[0]+h[0],c[1]+h[1]],[c[0]-h[0],c[1]+h[1]]])
        segments.extend([[p[i],p[(i+1)%4]] for i in range(4)])
    return np.array(segments),z

def scan_check(cfg,data):
    seg,z=collision_segments(cfg);a=seg[:,0];edge=seg[:,1]-a
    def cross(a,b):return a[...,0]*b[...,1]-a[...,1]*b[...,0]
    reports=[];all_errors=[]
    truth=data['truth']
    for scan in read('scan_samples.json'):
        t=scan['stamp']
        if not truth[0,0]<=t<=truth[-1,0]:continue
        position=np.array([np.interp(t,truth[:,0],truth[:,1]),np.interp(t,truth[:,0],truth[:,2])])
        angle=scan['angle_min']+np.arange(len(scan['ranges']))*scan['angle_increment']
        direction=np.c_[np.cos(angle),np.sin(angle)];delta=a-position
        det=cross(direction[:,None,:],edge[None,:,:])
        with np.errstate(divide='ignore',invalid='ignore'):
            distance=cross(delta,edge)[None,:]/det
            along=cross(delta[None,:,:],direction[:,None,:])/det
        valid=(np.abs(det)>1e-12)&(distance>=scan['range_min'])&(distance<=scan['range_max'])&(along>=-1e-8)&(along<=1+1e-8)
        expected=np.where(valid,distance,np.inf).min(axis=1)
        observed=np.array([np.inf if r is None else r for r in scan['ranges']])
        both=np.isfinite(expected)&np.isfinite(observed);errors=np.abs(observed[both]-expected[both])
        mismatched=int((np.isfinite(expected)!=np.isfinite(observed)).sum())
        outliers=[]
        for beam in np.where(both & (np.abs(np.where(both,observed,0)-np.where(both,expected,0))>.05))[0]:
            hit=int(np.argmin(np.where(valid[beam],distance[beam],np.inf)))
            outliers.append({'beam':int(beam),'angle_rad':float(angle[beam]),'observed_m':float(observed[beam]),'expected_m':float(expected[beam]),'distance_to_expected_segment_endpoint_m':float(min(along[beam,hit],1-along[beam,hit])*np.linalg.norm(edge[hit]))})
        all_errors.extend(errors)
        reports.append({'stamp':t,'body_xy_m':position.tolist(),'matched_finite_beams':int(both.sum()),'finite_state_mismatches':mismatched,'p95_abs_error_m':float(np.percentile(errors,95)),'max_abs_error_m':float(errors.max()),'beams_over_5cm':int((errors>.05).sum()),'outliers_over_5cm':outliers})
    e=np.array(all_errors)
    report={'method':'Independent ray/segment intersections of the world collision mesh at laser height, plus equipment collision boxes; sensor pose interpolated from measured Gazebo truth. All beams kept, including edge outliers.',
        'laser_height_m':z,'scan_frames':len(reports),'finite_beams':len(e),'median_error_m':float(np.median(e)),'p95_error_m':float(np.percentile(e,95)),'max_error_m':float(e.max()),'beams_over_5cm':int((e>.05).sum()),'finite_state_mismatches':sum(r['finite_state_mismatches'] for r in reports),'frames':reports,
        'criterion':'At least 20 route samples; 95th percentile <= 1 cm; report all >5 cm edge outliers separately.'}
    report['passed']=len(reports)>=20 and report['p95_error_m']<=.01 and report['finite_state_mismatches']==0
    write('scan_geometry_report.json',report)
    return report

def bag_check(data):
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu,LaserScan
    from tf2_msgs.msg import TFMessage
    db=next((OUT/'rosbag').glob('*.db3'));con=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
    topic_ids={n:i for i,n in con.execute('SELECT id,name FROM topics')};result={}
    for key,name,typ in [('truth','/ground_truth',Odometry),('reference','/shearer/reference',Odometry),('imu','/imu_raw',Imu),('scan','/scan_raw',LaserScan)]:
        times=[]
        for (blob,) in con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(topic_ids[name],)):
            m=deserialize_message(blob,typ);times.append(m.header.stamp.sec*1000000000+m.header.stamp.nanosec)
        monitor=np.rint(data[key][:,0]*1e9).astype(np.int64);record=np.array(times,dtype=np.int64)
        lo=max(monitor[0],record[0]);hi=min(monitor[-1],record[-1]);s1=set(monitor[(monitor>=lo)&(monitor<=hi)]);s2=set(record[(record>=lo)&(record<=hi)])
        result[name]={'bag_count':len(record),'monitor_count':len(monitor),'common_window_s':[lo/1e9,hi/1e9],'missing_from_bag':len(s1-s2),'missing_from_monitor':len(s2-s1),'same_stamps_in_common_window':s1==s2}
    truth=data['truth'];truth_times=np.rint(truth[:,0]*1e9).astype(np.int64);errors=[];tf_times=[];static=[]
    for (blob,) in con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(topic_ids['/tf'],)):
        for tf in deserialize_message(blob,TFMessage).transforms:
            if tf.header.frame_id!='world' or tf.child_frame_id!='base_link':continue
            t=tf.header.stamp.sec*1000000000+tf.header.stamp.nanosec;i=int(np.searchsorted(truth_times,t))
            if i==len(truth) or truth_times[i]!=t:continue
            p=tf.transform.translation;q=tf.transform.rotation
            errors.append(np.abs(np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w])-np.r_[truth[i,1:4],truth[i,7:11]]));tf_times.append(t)
    for (blob,) in con.execute('SELECT data FROM messages WHERE topic_id=?',(topic_ids['/tf_static'],)):
        for tf in deserialize_message(blob,TFMessage).transforms:
            p=tf.transform.translation
            static.append({'parent':tf.header.frame_id,'child':tf.child_frame_id,'translation_m':[p.x,p.y,p.z]})
    tf_errors=np.array(errors)
    tf_report={'dynamic_compared':len(errors),'max_xyz_quaternion_abs_error':tf_errors.max(axis=0).tolist(),'static_transforms':static,'passed':bool(tf_errors.max()<1e-8 and any(x['parent']=='base_link' and x['child']=='laser_link' and np.allclose(x['translation_m'],[0,0,.55]) for x in static)),'method':'Compare each recorded world-to-base_link transform to the independent truth message at the exact same timestamp; separately verify the laser static mount.'}
    con.close();write('bag_consistency.json',result);write('tf_exact_comparison.json',tf_report);return result,tf_report

def main():
    cfg=json.loads((ROOT/'design_v02/layout_parameters.json').read_text());runtime=read('runtime_report.json');data=np.load(OUT/'samples.npz')
    geometry=scan_check(cfg,data);bag,tf_report=bag_check(data)
    first300={}
    for key in ['truth','reference','imu','scan']:
        a=data[key];a=a[(a[:,0]>=runtime['epoch_sim_s']+1)&(a[:,0]<runtime['epoch_sim_s']+301)]
        dt=np.diff(a[:,0]);nominal=.1 if key=='scan' else .02
        first300[key]={'count':len(a),'frequency_hz':float((len(a)-1)/(a[-1,0]-a[0,0])),'max_gap_s':float(dt.max()),'gaps_over_1_5_nominal':int((dt>1.5*nominal).sum()),'reversed_or_duplicate_stamps':int((dt<=0).sum())}
    history=runtime['history'];intervals=[]
    for a,b in zip(history,history[1:]):
        if a['sim_time_s'] is not None and b['sim_time_s'] is not None and a['sim_time_s']>10:intervals.append((b['sim_time_s']-a['sim_time_s'])/(b['wall_elapsed_s']-a['wall_elapsed_s']))
    resources=read('resource_samples.json')['samples']
    sources={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in [ROOT/'worlds/coal_face.sdf',*list((ROOT/'models').rglob('model.sdf'))]}
    summary={'five_minute_window':first300,'runtime':{k:v for k,v in runtime.items() if k!='history'},'performance':{'mode':'headless, software rendering, another user tile viewer also running','rtf_median':float(np.median(intervals)),'rtf_min_5s_interval':float(min(intervals)),'gzserver_peak_rss_mb':max(r['rss_mb'] for r in resources),'gzserver_median_cpu_percent_one_core':float(np.median([r['cpu_percent_one_core'] for r in resources if r['cpu_percent_one_core'] is not None]))},'scan_geometry':{k:v for k,v in geometry.items() if k!='frames'},'bag_consistency':bag,'source_sha256':sources}
    a=data['truth'];phase=a[:,0]-runtime['epoch_sim_s']
    turns=[]
    for expected_t,expected_y in [(600.,195.),(1200.,5.)]:
        region=a[np.abs(phase-expected_t)<=1.]
        if len(region):
            row=region[np.argmin(np.abs(region[:,2]-expected_y))]
            turns.append({'expected_time_s':expected_t,'nearest_measured_time_s':float(row[0]-runtime['epoch_sim_s']),'expected_y_m':expected_y,'measured_y_m':float(row[2]),'error_m':float(abs(row[2]-expected_y))})
    summary['turning_points']=turns
    imu=data['imu'];ip=imu[:,0]-runtime['epoch_sim_s'];expected_ay=95*(2*np.pi/1200)**2*np.cos(ip*2*np.pi/1200)
    summary['ideal_imu_diagnostics']={'max_gravity_z_error_ms2':float(np.abs(imu[:,3]-9.81).max()),'y_acceleration_rmse_vs_ideal_ms2':float(np.sqrt(np.mean((imu[:,2]-expected_ay)**2))),'all_values_finite':bool(np.isfinite(imu).all()),'scope':'Diagnostics of the ideal kinematic IMU only; control/timestamp irregularity is included, no vibration/noise calibration.'}
    summary['tf_exact_comparison']=tf_report
    summary['runtime_gates']={'finished_full_cycle':runtime['error'] is None and runtime['duration_sim_s']>=1200,
        'truth_50hz_without_large_gaps':all(first300[k]['gaps_over_1_5_nominal']==0 for k in ['truth','imu','scan']) and all(runtime['rates'][k]['gaps_over_1_5_nominal']==0 for k in ['truth','imu','scan']),
        'two_turning_points_within_1cm':len(turns)==2 and all(t['error_m']<.01 for t in turns),
        'peak_speed_not_over_0_5ms':runtime['motion']['peak_speed_ms']<=.5,
        'all_recorded_scans_have_720_beams':bool(np.all(data['scan'][:,1]==720)),
        'bag_matches_observer_in_common_window':all(v['same_stamps_in_common_window'] for v in bag.values()),
        'scan_collision_alignment':geometry['passed'],
        'tf_matches_truth_at_exact_stamp':tf_report['passed'],
        'truth_reference_imu_all_finite':bool(all(np.isfinite(data[k]).all() for k in ['truth','reference','imu'])),
        'fixed_body_xz_within_1mm':bool(runtime['motion']['x_max_deviation_m']<.001 and runtime['motion']['z_max_deviation_m']<.001)}
    write('acceptance_results.json',summary)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    a=data['truth'];t=a[:,0]-runtime['epoch_sim_s'];ideal=100-95*np.cos(t*2*np.pi/1200)
    fig,axes=plt.subplots(3,1,figsize=(10,8),constrained_layout=True)
    axes[0].plot(t,ideal,label='cosine reference',lw=2);axes[0].plot(t[::20],a[::20,2],label='Gazebo truth',lw=.8);axes[0].set_ylabel('Y (m)');axes[0].legend();axes[0].grid()
    axes[1].plot(t,a[:,2]-ideal,lw=.5);axes[1].set_ylabel('Position error (m)');axes[1].grid()
    axes[2].plot(t,a[:,5],lw=.8);axes[2].axhline(.5,color='red',ls='--');axes[2].axhline(-.5,color='red',ls='--');axes[2].set_ylabel('Y speed (m/s)');axes[2].set_xlabel('Trajectory elapsed simulation time (s)');axes[2].grid()
    fig.savefig(OUT/'motion_verification.png',dpi=160);plt.close(fig)
    print(json.dumps({k:v for k,v in summary.items() if k not in ['source_sha256','runtime']},indent=2))

if __name__=='__main__':main()
