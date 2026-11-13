#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Generate a parameterized design preview and validate geometric placement."""
import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from measure_tiles import transform,rotation
from measure_clearance import collision_triangles,Rays

BASE=Path(__file__).resolve().parents[1]
ROOT=Path(__file__).resolve().parents[1]/'models'
OUT=BASE/'design_v02'
OUT.mkdir(exist_ok=True)
(BASE/'worlds').mkdir(exist_ok=True)
FLOOR=.0005

def port(point,inward,yaw,origin):
    R=rotation([0,0,1],yaw)[:3,:3]
    return R@np.array([*point,0.])+origin,R@np.array([*inward,0.])

def triangle_box_intersections(triangles,center,size):
    half=np.asarray(size)/2.;center=np.asarray(center)
    mask=np.all(triangles.max(axis=1)>=center-half-1e-9,axis=1)&np.all(triangles.min(axis=1)<=center+half+1e-9,axis=1)
    tri=triangles[mask]-center
    if not len(tri):return 0
    active=np.ones(len(tri),dtype=bool)
    edges=np.stack([tri[:,1]-tri[:,0],tri[:,2]-tri[:,1],tri[:,0]-tri[:,2]],axis=1)
    axes=[np.cross(edges[:,0],edges[:,1])]
    for edge in range(3):
        for unit in np.eye(3):axes.append(np.cross(edges[:,edge],unit))
    for axis in axes:
        projection=np.einsum('tvi,ti->tv',tri,axis)
        radius=np.abs(axis)@half
        separated=(projection.min(axis=1)>radius+1e-9)|(projection.max(axis=1)<-radius-1e-9)
        active &= ~separated
    return int(active.sum())

def main():
    tile_instances=[]
    def tile(name,kind,x,y,yaw):
        tile_instances.append({'name':name,'model':kind,'position_xyz_m':[x,y,0.], 'yaw_rad':yaw})
    for i in range(10):tile(f'face_{i:02}','subt_tunnel_tile_5',0.,10+20*i,0.)
    for label,y in [('transport',-10.),('return',210.)]:
        for i,x in enumerate([-60.,-40.,-20.]):tile(f'{label}_{i:02}','subt_tunnel_tile_5',x,y,math.pi/2)
    tile('transport_turn','subt_tunnel_tile_2',0.,-10.,math.pi)
    tile('return_turn','subt_tunnel_tile_2',0.,210.,-math.pi/2)
    templates={kind:collision_triangles(ROOT/kind,ROOT) for kind in ['subt_tunnel_tile_5','subt_tunnel_tile_2']}
    world_triangles=[];ports=[]
    local_ports={'subt_tunnel_tile_5':[([0,-10],[0,1]),([0,10],[0,-1])],
                 'subt_tunnel_tile_2':[([0,-10],[0,1]),([10,0],[-1,0])]}
    for instance in tile_instances:
        T=rotation([0,0,1],instance['yaw_rad']);T[:3,3]=instance['position_xyz_m']
        tri=templates[instance['model']]
        world_triangles.append(transform(tri.reshape(-1,3),T).reshape(-1,3,3))
        for number,(p,direction) in enumerate(local_ports[instance['model']]):
            pos,d=port(p,direction,instance['yaw_rad'],np.array(instance['position_xyz_m']))
            ports.append({'instance':instance['name'],'port':number,'position':pos,'inward':d,'local_point':p,'local_inward':direction,'model':instance['model'],'yaw':instance['yaw_rad']})
    world_triangles=np.concatenate(world_triangles)
    connections=[];unmatched=[];used=set();ray_sources={k:Rays(t) for k,t in templates.items()}
    angles=np.linspace(0,2*math.pi,360,endpoint=False)
    for i,p in enumerate(ports):
        if i in used:continue
        candidates=[(j,q) for j,q in enumerate(ports) if j!=i and j not in used and np.linalg.norm(p['position']-q['position'])<1e-6 and np.dot(p['inward'],q['inward'])<-.999999]
        if not candidates:
            unmatched.append({'instance':p['instance'],'port':p['port'],'position_xyz_m':p['position'].tolist()});continue
        assert len(candidates)==1
        j,q=candidates[0];used.update([i,j])
        n=np.cross(p['inward'],[0,0,1.]);world_dirs=np.cos(angles)[:,None]*n+np.sin(angles)[:,None]*[0,0,1.]
        profiles=[]
        for endpoint in [p,q]:
            origin=np.array([*endpoint['local_point'],1.])+np.array([*endpoint['local_inward'],0.])*.01
            R=rotation([0,0,1],endpoint['yaw'])[:3,:3]
            local_dirs=world_dirs@R
            profiles.append(ray_sources[endpoint['model']].cast(origin,local_dirs,max_distance=8))
        finite=np.isfinite(profiles[0])&np.isfinite(profiles[1])
        residual=np.abs(profiles[0][finite]-profiles[1][finite])
        connections.append({'a':p['instance'],'a_port':p['port'],'b':q['instance'],'b_port':q['port'],
            'position_xyz_m':p['position'].tolist(),'profile_valid_rays':int(finite.sum()),
            'profile_p95_difference_m':float(np.percentile(residual,95)), 'profile_max_difference_m':float(residual.max())})

    objects=[]
    def box(name,role,center,size,color):objects.append({'name':name,'role':role,'center_xyz_m':center,'size_xyz_m':size,'rgba':color})
    for i in range(133):box(f'support_{i:03}','support',[-1.05,.75+1.5*i,FLOOR+1.4],[1.2,1.4,2.8],[.30,.48,.66,1])
    for i in range(20):box(f'conveyor_{i:02}','conveyor',[.6,5+10*i,FLOOR+.125],[1.,10.,.25],[.16,.49,.46,1])
    for i in range(20):box(f'coal_boundary_{i:02}','coal_boundary',[1.9,5+10*i,FLOOR+1.2],[.1,10.,2.4],[.12,.11,.10,1])
    shearer={'name':'shearer_design','role':'shearer','center_xyz_m':[.6,100.,.6605],'size_xyz_m':[.8,2.,.8],'rgba':[.95,.68,.12,1]}
    # Fixed-orientation motion along a straight line has an exact AABB swept volume.
    swept={'name':'shearer_swept_volume','role':'swept_volume','center_xyz_m':[.6,100.,.6605],'size_xyz_m':[.8,192.,.8]}
    collision_checks=[]
    for obj in [*objects,swept]:
        center=np.array(obj['center_xyz_m']);size=np.array(obj['size_xyz_m'])
        # Equipment is allowed to rest on the floor. Inspect its interior above contact.
        if obj['role']!='swept_volume':center[2]+=.001;size[2]-=.002
        count=triangle_box_intersections(world_triangles,center,size)
        collision_checks.append({'name':obj['name'],'role':obj['role'],'intersecting_tunnel_triangles':count})
    overlaps=[]
    collision_boxes=[*objects,swept]
    for i,a in enumerate(collision_boxes):
        ca=np.array(a['center_xyz_m']);ha=np.array(a['size_xyz_m'])/2
        for b in collision_boxes[i+1:]:
            cb=np.array(b['center_xyz_m']);hb=np.array(b['size_xyz_m'])/2
            if np.all(ha+hb-np.abs(ca-cb)>1e-7):overlaps.append([a['name'],b['name']])
    # Sample the roof above each support top footprint (not only over path center).
    samples=[]
    for obj in objects:
        if obj['role']!='support':continue
        c=np.array(obj['center_xyz_m']);h=np.array(obj['size_xyz_m'])/2
        for x in np.linspace(c[0]-h[0],c[0]+h[0],7):
            for y in np.linspace(c[1]-h[1],c[1]+h[1],7):samples.append([x,y,FLOOR+2.8+.0001])
    # Each point maps to the straight tile containing it; all support footprints
    # are within the 200 m face, so this avoids casting against the entire world.
    local_points=np.array(samples)
    local_points[:,1]-=20*np.floor(local_points[:,1]/20)+10
    roof_gaps=ray_sources['subt_tunnel_tile_5'].cast(local_points,[0,0,1])+.0001
    bad=[c for c in collision_checks if c['intersecting_tunnel_triangles']]
    config={'status':'design preview, pending real mine/equipment parameters and dynamic full-route validation',
        'face_length_m':200.,'roadway_straight_length_m':60.,'face_endpoints_world_m':[[0,0,0],[0,200,0]],
        'roadway_axes_y_m':{'transport':-10.,'return':210.},'roadway_x_range_m':[-70.,-10.],
        'reference_turn_radius_m':10.,'floor_z_m':FLOOR,
        'motion':{'x_m':.6,'y_min_m':5.,'y_max_m':195.,'base_z_m':.6605,'period_s':1200.,'maximum_speed_ms':math.pi*190/1200},
        'support':{'count':133,'pitch_m':1.5,'center_x_m':-1.05,'size_xyz_m':[1.2,1.4,2.8]},
        'conveyor':{'count':20,'segment_length_m':10.,'center_x_m':.6,'size_xyz_m':[1.,10.,.25]},
        'coal_boundary':{'count':20,'center_x_m':1.9,'size_xyz_m':[.1,10.,2.4]},
        'tile_instances':tile_instances,'equipment':objects,'shearer_preview':shearer}
    report={'method':'Geometric connection-frame matching and 360-ray port profile comparison at 1 cm inside ends; triangle/AABB separating-axis tests above floor contacts; exact fixed-orientation swept AABB; sampled support roof gap.',
        'connections':connections,'open_ports':unmatched,'equipment_tunnel_intersections':bad,'equipment_box_overlaps':overlaps,
        'support_top_roof_gap_min_m':float(roof_gaps.min()),'support_top_roof_gap_samples':len(roof_gaps),
        'support_roof_worst_sample_world_xyz_m':samples[int(np.argmin(roof_gaps))],
        'checked_static_boxes':len(objects),'swept_body_checked':True,
        'scope_limit':'Not real mine design; no dynamic or floor-bearing test, no UWB/dust models, no rounded equipment shape or mechanical behavior.',
        'passed':bool(not bad and not overlaps and len(unmatched)==2 and len(connections)==17 and np.isfinite(roof_gaps).all() and roof_gaps.min()>.05)}
    report['source_sha256']={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest()
        for kind in templates for f in [ROOT/kind/'model.sdf',*list((ROOT/kind/'meshes').glob('*.dae'))]}
    report['checks']={'connections_matched':len(connections)==17 and len(unmatched)==2,
        'port_profiles_within_1cm':all(c['profile_valid_rays']==360 and c['profile_max_difference_m']<.01 for c in connections),
        'equipment_clear_of_tunnel_mesh':not bad,'equipment_boxes_do_not_overlap':not overlaps,
        'support_sampled_roof_gap_over_5cm':bool(np.isfinite(roof_gaps).all() and roof_gaps.min()>.05)}
    report['passed']=all(report['checks'].values())
    (OUT/'layout_parameters.json').write_text(json.dumps(config,indent=2))
    (OUT/'geometric_checks.json').write_text(json.dumps(report,indent=2))
    with (OUT/'model_instances.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.writer(f);writer.writerow(['name','model','x_m','y_m','z_m','yaw_rad'])
        for t in tile_instances:writer.writerow([t['name'],t['model'],*t['position_xyz_m'],t['yaw_rad']])
    sdf=ET.Element('sdf',{'version':'1.6'});world=ET.SubElement(sdf,'world',{'name':'coal_face_design_preview'})
    scene=ET.SubElement(world,'scene');ET.SubElement(scene,'ambient').text='.8 .8 .8 1';ET.SubElement(scene,'shadows').text='false'
    light=ET.SubElement(world,'light',{'name':'sun','type':'directional'});ET.SubElement(light,'direction').text='-.5 .2 -1';ET.SubElement(light,'diffuse').text='.8 .8 .8 1'
    for t in tile_instances:
        inc=ET.SubElement(world,'include');ET.SubElement(inc,'uri').text='model://'+t['model'];ET.SubElement(inc,'name').text=t['name'];ET.SubElement(inc,'pose').text=' '.join(map(str,[*t['position_xyz_m'],0,0,t['yaw_rad']]))
    for obj in [*objects,shearer]:
        model=ET.SubElement(world,'model',{'name':obj['name']});ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text=' '.join(map(str,[*obj['center_xyz_m'],0,0,0]))
        link=ET.SubElement(model,'link',{'name':'link'})
        for kind in ['collision','visual']:
            part=ET.SubElement(link,kind,{'name':kind});ET.SubElement(ET.SubElement(ET.SubElement(part,'geometry'),'box'),'size').text=' '.join(map(str,obj['size_xyz_m']))
            if kind=='visual':
                mat=ET.SubElement(part,'material')
                for tag in ['ambient','diffuse']:ET.SubElement(mat,tag).text=' '.join(map(str,obj['rgba']))
    state_plugin=ET.SubElement(world,'plugin',{'name':'design_preview_states','filename':'libgazebo_ros_state.so'})
    ET.SubElement(ET.SubElement(state_plugin,'ros'),'namespace').text='/design_preview'
    ET.SubElement(state_plugin,'update_rate').text='1.0'
    camera=ET.SubElement(ET.SubElement(world,'gui'),'camera',{'name':'user_camera'});ET.SubElement(camera,'pose').text='100 -80 180 0 .65 2.1'
    ET.indent(sdf,space='  ');ET.ElementTree(sdf).write(BASE/'worlds/coal_face_design_preview.sdf',encoding='utf-8',xml_declaration=True)
    print(json.dumps({'passed':report['passed'],'tiles':len(tile_instances),'connections':len(connections),'open_ports':unmatched,
        'tunnel_intersections':bad,'equipment_overlaps':overlaps,'support_min_roof_gap_m':float(roof_gaps.min()),
        'port_profile_max_difference_m':max(c['profile_max_difference_m'] for c in connections)},indent=2),flush=True)

if __name__=='__main__':main()
