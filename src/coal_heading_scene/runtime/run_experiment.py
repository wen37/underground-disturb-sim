"""Run an isolated Gazebo instance and keep failures as inspectable evidence."""
import argparse,json,os,signal,socket,subprocess,time
from datetime import datetime
from zoneinfo import ZoneInfo
from paths import ROOT,MODELS,NATIVE_INSTALL

def stop(process):
    """停止本次子进程；入参为进程对象；无返回值。"""
    if process is not None and process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)

def main():
    """执行静态或运动实验；参数来自命令行；返回退出码。"""
    parser=argparse.ArgumentParser()
    parser.add_argument('--mode',choices=['static','motion','both'],default='both')
    parser.add_argument('--gui',action='store_true');args=parser.parse_args()
    subprocess.run(['python3',str(ROOT/'runtime/scene_validator.py')],check=True)
    if not (NATIVE_INSTALL/'coal_face_sim/lib/libcoal_heading_trajectory.so').exists():raise SystemExit('Run python3 runtime/build_native.py first')
    with socket.socket() as sock:
        try:sock.bind(('127.0.0.1',11357))
        except OSError:raise SystemExit('Port11357 occupied; this tool will not stop another simulator.')
    env=dict(os.environ,ROS_DOMAIN_ID='137',GAZEBO_MASTER_URI='http://127.0.0.1:11357',GAZEBO_MODEL_DATABASE_URI='',GAZEBO_MODEL_PATH=str(MODELS),GAZEBO_PLUGIN_PATH=str(NATIVE_INSTALL/'coal_face_sim/lib')+os.pathsep+os.environ.get('GAZEBO_PLUGIN_PATH',''))
    stamp=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%dT%H%M%S_%f')
    out=ROOT/'evidence/full_chain'/('run_'+stamp);out.mkdir(parents=True)
    codes=[];runs=[];error=None;server=gui=observer=experiment=None;gui_alive=None
    command=['gzserver','--lockstep','--verbose','-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so',str(ROOT/'scenes/S01_subt_heading/world/scene.sdf'),'--ros-args','--params-file',str(ROOT/'runtime/gazebo_params.yaml')]
    with (out/'gazebo.log').open('w') as log:
        try:
            server=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(3)
            if server.poll() is not None:raise RuntimeError('Gazebo exited during startup')
            if args.gui:gui=subprocess.Popen(['gzclient'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            for mode in (['static','motion'] if args.mode=='both' else [args.mode]):
                observer=subprocess.Popen(['python3',str(ROOT/'runtime/observe_topics.py'),str(out/(mode+'_observer.json'))],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                with (out/(mode+'.log')).open('w') as f:
                    experiment=subprocess.Popen(['python3',str(ROOT/'runtime/run_heading.py'),'--mode',mode,'--duration','3' if mode=='static' else '15.5'],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
                    rc=experiment.wait(timeout=200)
                codes.append(rc)
                if rc:
                    try:codes.append(observer.wait(timeout=5))
                    except subprocess.TimeoutExpired:stop(observer)
                    break
                codes.append(observer.wait(timeout=15))
                if codes[-1]:break
                text=(out/(mode+'.log')).read_text()
                runs.append({'mode':mode,'result_dir':next(line[7:] for line in text.splitlines() if line.startswith('RESULT '))})
            if gui is not None:gui_alive=gui.poll() is None
        except Exception as exc:error=f'{type(exc).__name__}: {exc}'
        finally:
            for process in [experiment,observer,gui,server]:stop(process)
    expected=4 if args.mode=='both' else 2
    result={'mode':args.mode,'return_codes':codes,'passed':error is None and len(codes)==expected and all(c==0 for c in codes) and gui_alive is not False,'error':error,'gui_requested':args.gui,'gui_alive_at_end':gui_alive,'runs':runs,'log_dir':str(out)}
    (out/'orchestration.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
