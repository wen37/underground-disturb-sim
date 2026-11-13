#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Brief GUI smoke check; close only the client created by this script."""
import json,os,signal,subprocess,time
from pathlib import Path
out=Path(__file__).resolve().parents[1]/'validation/full_scale'
env=os.environ.copy();env.update(ROS_DOMAIN_ID='90',GAZEBO_MASTER_URI='http://127.0.0.1:11990',LIBGL_ALWAYS_SOFTWARE='1',GAZEBO_MODEL_PATH=str(out.parents[1]/'models'))
log=open(out/'gzclient_smoke.log','w');p=subprocess.Popen(['gzclient','--verbose'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
start=time.monotonic()
try:
    while p.poll() is None and time.monotonic()-start<45:time.sleep(.2)
    report={'pid':p.pid,'wall_duration_s':time.monotonic()-start,'alive_until_smoke_end':p.poll() is None,'scope':'GUI smoke test only, not a full-duration GUI frame-rate benchmark.'}
    (out/'gui_smoke_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
finally:
    if p.poll() is None:p.send_signal(signal.SIGINT)
    try:p.wait(timeout=10)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM)
    log.close()
