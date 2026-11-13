#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Read-only sampling of the tested Gazebo process; do not include other viewers."""
import json, os, sys, time
from pathlib import Path
pid=int(sys.argv[1]);proc=Path('/proc')/str(pid)
assert b'coal_face.sdf' in (proc/'cmdline').read_bytes(), 'PID is not the tested world'
root=Path(__file__).resolve().parents[1]/'validation/full_scale'
rows=[];previous=None;hz=os.sysconf('SC_CLK_TCK');start=time.monotonic()
while proc.exists() and not (root/'runtime_report.json').exists() and time.monotonic()-start<3000:
    now=time.monotonic();fields=(proc/'stat').read_text().split();ticks=int(fields[13])+int(fields[14])
    stat=dict(line.split(':',1) for line in (proc/'status').read_text().splitlines() if ':' in line)
    mem=dict(line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines() if ':' in line)
    rows.append({'wall_s':now-start,'rss_mb':int(stat['VmRSS'].split()[0])/1024,'cpu_percent_one_core':None if previous is None else (ticks-previous[1])/hz/(now-previous[0])*100,'system_available_mb':int(mem['MemAvailable'].split()[0])/1024})
    previous=(now,ticks)
    temp=root/'resource_samples.tmp';temp.write_text(json.dumps({'pid':pid,'samples':rows},indent=2));temp.replace(root/'resource_samples.json')
    time.sleep(5)
