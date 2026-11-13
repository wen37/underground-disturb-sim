#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Analyze only a FINISHED recording: even read-only SQLite can block a writer."""
import json, sqlite3
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,LaserScan
out=Path(__file__).resolve().parents[1]/'validation/full_scale'
if not (out/'runtime_report.json').exists():
    raise SystemExit('Refusing to open a live recording. Wait for runtime_report.json after test shutdown.')
epoch=json.loads((out/'runtime_report.json').read_text())['epoch_sim_s']
db=next((out/'rosbag').glob('*.db3'));con=sqlite3.connect('file:'+str(db)+'?mode=ro',uri=True)
topics={n:i for i,n in con.execute('SELECT id,name FROM topics')};report={}
for name,typ,nominal in [('/ground_truth',Odometry,.02),('/imu_raw',Imu,.02),('/scan_raw',LaserScan,.1),('/shearer/reference',Odometry,.02)]:
    ts=[]
    for (blob,) in con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(topics[name],)):
        m=deserialize_message(blob,typ);t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if epoch+1<=t<epoch+301:ts.append(t)
    a=np.array(ts);dt=np.diff(a)
    report[name]={'count':len(a),'duration_s':float(a[-1]-a[0]),'hz_sim':float((len(a)-1)/(a[-1]-a[0])),'max_gap_s':float(dt.max()),'gaps_over_1_5_nominal':int((dt>1.5*nominal).sum()),'reversed_or_duplicate':int((dt<=0).sum())}
con.close();(out/'five_minute_checkpoint.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
