#!/usr/bin/env python3
import os
from pathlib import Path
import signal
import subprocess
import time

BASE=Path(__file__).resolve().parents[1]
os.environ['GAZEBO_MASTER_URI']='http://127.0.0.1:11889'
os.environ['ROS_DOMAIN_ID']='89'
os.environ['GAZEBO_MODEL_PATH']=str(BASE/'models')
os.environ['GAZEBO_MODEL_DATABASE_URI']=''
os.environ['LIBGL_ALWAYS_SOFTWARE']='1';os.environ['QT_X11_NO_MITSHM']='1';os.environ['QT_OPENGL']='software'
logs=BASE/'design_v02';logs.mkdir(exist_ok=True)
with (logs/'preview_server.log').open('w') as sl,(logs/'preview_gui.log').open('w') as gl:
    server=subprocess.Popen(['gzserver',str(BASE/'worlds/coal_face_design_preview.sdf'),'-s','libgazebo_ros_init.so'],stdout=sl,stderr=subprocess.STDOUT,start_new_session=True)
    gui=None
    try:
        time.sleep(2)
        if server.poll() is not None:raise RuntimeError('See preview_server.log')
        gui=subprocess.Popen(['gzclient'],stdout=gl,stderr=subprocess.STDOUT,start_new_session=True)
        print('Static coal-face design preview. Robot is stationary; close GUI or Ctrl+C to stop.',flush=True)
        gui.wait()
    except KeyboardInterrupt:pass
    finally:
        for process in [gui,server]:
            if process is not None and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
