#!/usr/bin/env python3
"""Validate the additive SubT runtime without rewriting the protected S00 checker."""
import json,math,hashlib,subprocess,os
from pathlib import Path
import xml.etree.ElementTree as E
import yaml,sys
from heading_math import validate_zones
from paths import ROOT, MODELS, FIXED
sys.path.insert(0,str(ROOT.parents[1]/'tools'))
from validate_scene import UniqueLoader,finite
def read_yaml(path):
 data=yaml.load(path.read_text(),Loader=UniqueLoader);finite(data);return data
SCENE=ROOT/'scenes/S01_subt_heading'
def validate():
 s=read_yaml(SCENE/'scenario.yaml');sem=read_yaml(SCENE/'scene_semantics.yaml');route=read_yaml(SCENE/'demo_route.yaml');world=E.parse(SCENE/'world/scene.sdf').find('world')
 def check(ok,msg):
  if not ok:raise ValueError(msg)
 def numbers(parent,path,want):
  node=parent.find(path);check(node is not None,'missing '+path)
  actual=list(map(float,node.text.split()));check(len(actual)==len(want) and all(math.isfinite(a) and abs(a-b)<1e-9 for a,b in zip(actual,want)),'mismatch '+path)
 check(all(d['schema_version']=='heading-runtime-1' and d['scene_version']==s['scene_version'] for d in [s,sem,route]),'heading profile version mismatch')
 check(s['global_frame']=='map' and s['world_frame']=='world','global frames')
 check(s['robot']['initial_pose']=={'xyz_m':[0,68,1.349], 'rpy_rad':[0,0,0]},'runtime profile requires initial body [0,68,1.349] and no rotation')
 check(s['station']['xyz_m']==[0,57.1,.134] and s['station']['rpy_rad']==[0,0,0],'runtime profile requires fixed station pose')
 check(s['target']['xyz_m']==[0,-5.9,.585] and s['target']['rpy_rad']==[0,0,0],'runtime profile requires fixed target offset')
 check(s['sensors']['lidar']['xyz_m']==[0,0,1.8] and s['sensors']['lidar']['rpy_rad']==[0,0,math.pi/2],'runtime profile requires fixed lidar pose')

 check(s['map_to_world']=={'xyz_m':[0,0,0],'rpy_rad':[0,0,0]},'map/world must be identity')
 check(world.get('name')==s['scene_id']==sem['scene_id']==route['scene_id'],'scene ids')
 check(s['robot']['parent_frame']=='map' and s['station']['parent_frame']=='map','top-level parents')
 check(s['sensors']['lidar']['parent_frame']=='station_link' and s['target']['parent_frame']=='roadheader_base_link','mount parents')
 robot=world.find("model[@name='robot_01']");station=world.find("model[@name='station_01']")
 numbers(robot,'pose',s['robot']['initial_pose']['xyz_m']+s['robot']['initial_pose']['rpy_rad'])
 numbers(station,'pose',s['station']['xyz_m']+s['station']['rpy_rad'])
 numbers(robot,"link[@name='roadheader_base_link']/collision[@name='ebz200h_envelope_collision']/geometry/box/size",s['robot']['footprint_size_m'])
 numbers(robot,"link[@name='target_link']/pose",s['target']['xyz_m']+s['target']['rpy_rad'])
 li=s['sensors']['lidar'];numbers(station,"link[@name='lidar_link']/pose",li['xyz_m']+li['rpy_rad'])
 numbers(robot,"link[@name='target_link']/collision/geometry/box/size",[.5,.02,.5])
 sensor=station.find('.//sensor');check(sensor.get('type')=='ray','ray sensor')
 for path,value in [('update_rate',li['rate_hz']),('ray/scan/horizontal/samples',li['beam_count']),('ray/scan/horizontal/min_angle',li['angle_min_rad']),('ray/scan/horizontal/max_angle',li['angle_max_rad']),('ray/range/min',li['range_min_m']),('ray/range/max',li['range_max_m'])]:numbers(sensor,path,[value])
 check(li['beam_count']==1081 and abs((li['angle_max_rad']-li['angle_min_rad'])/(li['beam_count']-1)-math.radians(.25))<1e-12,'angular increment')
 for model,name,parent,child in [(robot,'target_mount','roadheader_base_link','target_link'),(station,'lidar_mount','station_link','lidar_link')]:
  j=model.find("joint[@name='%s']"%name);check(j is not None and j.get('type')=='fixed' and j.findtext('parent')==parent and j.findtext('child')==child,'fixed mounting')
 includes=world.findall('include');check(len(includes)==5,'four tiles plus blocker')
 for i,y in enumerate([10,30,50,70]):
  inc=next(x for x in includes if x.findtext('name')==f'tunnel_{i}');check(inc.findtext('uri')=='model://subt_tunnel_tile_5','original SubT tile');numbers(inc,'pose',[0,y,0,0,0,0])
 # Resolve every SDF mesh/material resource locally, including directory URIs.
 for path in [SCENE/'world/scene.sdf',*list((MODELS).glob('*/model.sdf'))]:
  for uri in E.parse(path).findall('.//uri'):
   value=uri.text
   if value.startswith('model://'):check((MODELS/value[8:]).exists(),'missing model resource '+value)
 validate_zones(sem['zones']);check(sem['dust_kind']=='respirable' and sem['concentration_unit']=='mg/m3','dust convention')
 trajectory=robot.find("plugin[@name='heading_trajectory']");check(trajectory is not None,'native trajectory plugin')
 numbers(trajectory,'speed_m_s',[route['speed_m_s']]);numbers(trajectory,'travel_start_s',[route['segments'][1]['start_s']]);numbers(trajectory,'travel_end_s',[route['segments'][1]['end_s']])
 segments=route['segments'];check([x['work_mode'] for x in segments]==['IDLE','TRAVEL','CUTTING_SIMULATED'],'route modes')
 check(segments[0]['start_s']==0 and segments[0]['start_y_m']==s['robot']['initial_pose']['xyz_m'][1],'route origin')
 for a,b in zip(segments,segments[1:]):check(a['end_s']==b['start_s'] and a['end_y_m']==b['start_y_m'],'route continuity')
 for a in segments:
  check(a['end_s']>a['start_s'],'route time');speed=(a['end_y_m']-a['start_y_m'])/(a['end_s']-a['start_s']);check(abs(speed-(route['speed_m_s'] if a['work_mode']=='TRAVEL' else 0))<1e-12,'route speed')
 for p,h in json.loads((FIXED/'preserved_sha256.json').read_text()).items():check(hashlib.sha256((FIXED/p).read_bytes()).hexdigest()==h,'protected fixed-distance input changed '+p)
 env=dict(os.environ,GAZEBO_MODEL_PATH=str(MODELS));proc=subprocess.run(['gz','sdf','-k',str(SCENE/'world/scene.sdf')],capture_output=True,text=True,env=env);check(proc.returncode==0,'SDF parser: '+proc.stderr)
 return {'passed':True,'ERROR':0,'scene_id':s['scene_id'],'checks':['protected baseline','frame tree declarations','SDF-config parity','sensor parameters','fixed mount joints','four original SubT tiles','offline resource resolution','route continuity and speed','nonoverlapping respirable zones','SDF parser'],'limitations':['mesh envelope is not net clearance','runtime and TF gate require separate live experiment evidence']}
if __name__=='__main__':
 try:result=validate()
 except Exception as e:result={'passed':False,'ERROR':1,'error':str(e)}
 (ROOT/'evidence/full_chain').mkdir(parents=True,exist_ok=True)
 (ROOT/'evidence/full_chain/scene_validator.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2));raise SystemExit(0 if result['passed'] else 1)
