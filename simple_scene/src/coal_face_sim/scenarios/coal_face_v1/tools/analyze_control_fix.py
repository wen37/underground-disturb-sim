#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Independent truth / analytic-trajectory regression; only read a closed bag."""
import json,sys,hashlib
from pathlib import Path
import numpy as np
import analyze_full_scale as base
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'validation/control_fix_20261029/run_02'
base.OUT=OUT
runtime=json.loads((OUT/'runtime_report.json').read_text());assert runtime['error'] is None
data=np.load(OUT/'samples.npz');truth=data['truth'];epoch=runtime['epoch_sim_s']
t=truth[:,0]-epoch;expected=100-95*np.cos(t*2*np.pi/1200);error=truth[:,2]-expected
increment_error=np.diff(truth[:,2])-np.diff(expected)
bag,tf=base.bag_check(data)
scan=base.scan_check(json.loads((ROOT/'design_v02/layout_parameters.json').read_text()),data)
turns=[]
for when,y in [(600.,195.),(1200.,5.)]:
 i=np.argmin(abs(t-when));turns.append(dict(expected_sim_elapsed_s=when,measured_sim_elapsed_s=float(t[i]),actual_y_m=float(truth[i,2]),error_m=float(abs(truth[i,2]-y))))
checks={
 'complete_full_cycle':runtime['duration_sim_s']>=1200,
 'max_trajectory_error_below_1mm':bool(abs(error).max()<.001),
 'no_unexplained_position_jump_over_1mm':bool(abs(increment_error).max()<.001),
 'both_endpoints_within_1mm':all(p['error_m']<.001 for p in turns),
 'peak_speed_below_0_5_ms':bool(abs(truth[:,5]).max()<=.5),
 'truth_reference_imu_scan_no_message_gaps':all(runtime['rates'][key]['gaps_over_1_5_nominal']==0 for key in ['truth','reference','imu','scan']),
 'truth_reference_imu_finite':bool(all(np.isfinite(data[key]).all() for key in ['truth','reference','imu'])),
 'scan_has_720_beams':bool(np.all(data['scan'][:,1]==720)),
 'bag_matches_monitor':all(item['same_stamps_in_common_window'] for item in bag.values()),
 'tf_matches_independent_truth':tf['passed'],
 'scan_p95_collision_alignment':scan['passed'],
}
summary=dict(passed=all(checks.values()),checks=checks,duration_sim_s=runtime['duration_sim_s'],wall_elapsed_s=runtime['wall_elapsed_s'],trajectory_epoch_sim_s=epoch,max_trajectory_error_m=float(abs(error).max()),trajectory_rmse_m=float(np.sqrt(np.mean(error**2))),max_unexplained_increment_m=float(abs(increment_error).max()),turns=turns,rates=runtime['rates'],scan_geometry={k:v for k,v in scan.items() if k!='frames'},tf=tf,bag=bag,
 scope='Prescribed kinematic trajectory and ideal raw chains. First 30 sim seconds at 1x, then target 10x with unchanged 1ms step. Does not validate traction dynamics or disturbance models. Single-pose scan maxima retain moving-edge timing sensitivity diagnosed in root_cause_20261029.',
 source_sha256={str(p.relative_to(ROOT.parent.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'worlds/coal_face.sdf',ROOT.parent.parent/'src/coal_face_trajectory.cpp',ROOT.parent.parent/'launch/coal_face_full.launch.py']})
(OUT/'fix_acceptance.json').write_text(json.dumps(summary,indent=2))
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(3,1,figsize=(9,8),constrained_layout=True)
axes[0].plot(t,expected,label='analytic reference');axes[0].plot(t[::10],truth[::10,2],label='independent Gazebo truth',lw=.7);axes[0].set_ylabel('Y (m)');axes[0].legend()
axes[1].plot(t,error*1000);axes[1].set_ylabel('Position difference (mm)')
axes[2].plot(t,truth[:,5]);axes[2].set_ylabel('Y velocity (m/s)');axes[2].set_xlabel('Elapsed simulation time (s)')
for ax in axes:ax.grid()
fig.savefig(OUT/'motion_after_fix.png',dpi=150);plt.close(fig)
print(json.dumps(summary,indent=2))
if not summary['passed']:sys.exit(1)
