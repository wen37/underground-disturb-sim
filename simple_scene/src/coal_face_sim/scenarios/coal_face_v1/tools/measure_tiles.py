#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Read-only bounding-box measurement for the archived SubT COLLADA tiles.

Results are in model coordinates and meters; bounding boxes are NOT clearance.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import numpy as np

NS={'c':'http://www.collada.org/2005/11/COLLADASchema'}

def transform(points,matrix):
    return (np.column_stack((points,np.ones(len(points))))@matrix.T)[:,:3]

def rotation(axis,angle):
    axis=np.asarray(axis,dtype=float)
    length=np.linalg.norm(axis)
    if length<1e-15:raise ValueError('Zero rotation axis')
    x,y,z=axis/length;c=math.cos(angle);s=math.sin(angle);v=1-c
    matrix=np.eye(4)
    matrix[:3,:3]=[[x*x*v+c,x*y*v-z*s,x*z*v+y*s],
                   [y*x*v+z*s,y*y*v+c,y*z*v-x*s],
                   [z*x*v-y*s,z*y*v+x*s,z*z*v+c]]
    return matrix

def pose(element):
    matrix=np.eye(4)
    if element is None:return matrix
    if element.get('relative_to'):raise ValueError('Named pose frames require explicit resolution')
    values=np.fromstring(element.text or '',sep=' ')
    if len(values)!=6:raise ValueError('Expected xyz/rpy six-value pose')
    x,y,z,r,p,yaw=values
    matrix=rotation([0,0,1],yaw)@rotation([0,1,0],p)@rotation([1,0,0],r)
    matrix[:3,3]=[x,y,z]
    return matrix

def read_dae(path):
    root=ET.parse(path).getroot()
    unit=root.find('c:asset/c:unit',NS)
    meter=float(unit.get('meter','1')) if unit is not None else 1.
    up=root.findtext('c:asset/c:up_axis',default='Y_UP',namespaces=NS)
    if up!='Z_UP':raise ValueError(f'{path}: only the current Z_UP assets are supported')
    geometries={}
    for g in root.findall('c:library_geometries/c:geometry',NS):
        mesh=g.find('c:mesh',NS)
        sources={}
        for src in mesh.findall('c:source',NS):
            accessor=src.find('c:technique_common/c:accessor',NS)
            if accessor is None:continue
            array_id=accessor.get('source','')[1:]
            array=src.find(f"c:float_array[@id='{array_id}']",NS)
            if array is None:continue
            stride=int(accessor.get('stride','1'));offset=int(accessor.get('offset','0'));count=int(accessor.get('count'))
            sources[src.get('id')]=np.fromstring(array.text,sep=' ')[offset:offset+count*stride].reshape(count,stride)
        vertices={}
        for vertex in mesh.findall('c:vertices',NS):
            pos=vertex.find("c:input[@semantic='POSITION']",NS)
            if pos is not None:vertices[vertex.get('id')]=pos.get('source')[1:]
        chunks=[]
        for primitive in list(mesh):
            kind=primitive.tag.split('}')[-1]
            if kind in ['source','vertices','extra']:continue
            if kind not in ['triangles','polylist','polygons','tristrips','trifans','lines','linestrips']:
                raise ValueError('Unsupported COLLADA primitive '+kind)
            inputs=primitive.findall('c:input',NS)
            vertex=next((e for e in inputs if e.get('semantic')=='VERTEX'),None)
            if vertex is None:continue
            width=max(int(e.get('offset','0')) for e in inputs)+1
            offset=int(vertex.get('offset','0'))
            points=sources[vertices[vertex.get('source')[1:]]][:,:3]
            for p in primitive.findall('c:p',NS):
                indices=np.fromstring(p.text or '',sep=' ',dtype=int).reshape(-1,width)[:,offset]
                chunks.append(points[np.unique(indices)])
        if chunks:geometries[g.get('id')]=np.concatenate(chunks)
    scene_ref=root.find('c:scene/c:instance_visual_scene',NS)
    if scene_ref is None:raise ValueError('No active COLLADA scene')
    scene=root.find("c:library_visual_scenes/c:visual_scene[@id='%s']"%scene_ref.get('url')[1:],NS)
    instances=[];node_transforms=[]
    def visit(node,parent):
        local=np.eye(4)
        for child in node:
            tag=child.tag.split('}')[-1]
            if tag in ['matrix','translate','rotate','scale']:
                vals=np.fromstring(child.text or '',sep=' ')
                part=np.eye(4)
                if tag=='matrix':part=vals.reshape(4,4).T
                elif tag=='translate':part[:3,3]=vals
                elif tag=='scale':part[:3,:3]=np.diag(vals)
                elif tag=='rotate':part=rotation(vals[:3],math.radians(vals[3]))
                local=local@part
            elif tag in ['lookat','skew','instance_node']:
                raise ValueError('Unsupported scene transform '+tag)
        full=parent@local
        node_transforms.append({'name':node.get('name'),'matrix':local.tolist()})
        for inst in node.findall('c:instance_geometry',NS):
            key=inst.get('url')[1:]
            if key in geometries:
                pts=transform(geometries[key],full)*meter
                instances.append({'name':node.get('name'), 'id':node.get('id'), 'geometry_id':key,'points':pts})
        for child in node.findall('c:node',NS):visit(child,full)
    for node in scene.findall('c:node',NS):visit(node,np.eye(4))
    if not instances:raise ValueError('No referenced mesh positions')
    return instances,{'unit_meter':meter,'unit_label':unit.get('name') if unit is not None else None,
                     'up_axis':up,'node_transforms':node_transforms}

def corners(size):
    size=np.asarray(size)
    return np.array([[x,y,z] for x in [-.5,.5] for y in [-.5,.5] for z in [-.5,.5]])*size

def bounds(points):
    lo=points.min(axis=0);hi=points.max(axis=0)
    return {'min_xyz_m':lo.tolist(),'max_xyz_m':hi.tolist(),'size_xyz_m':(hi-lo).tolist()}

def measure(model_path,models_root):
    model=ET.parse(model_path/'model.sdf').getroot().find('model')
    # Exclude model placement: measure in the model's own coordinates.
    result={'model':model_path.name,'coordinate_frame':'model-local XYZ, meters; model world pose excluded',
            'mesh_metadata':{},'collision_parts':[],'visual_parts':[]}
    geometry_cache={}
    totals={'collision':[],'visual':[]}
    for link in model.findall('link'):
        link_transform=pose(link.find('pose'))
        for kind in totals:
            for part in link.findall(kind):
                geom=part.find('geometry');mesh=geom.find('mesh')
                if mesh is not None:
                    uri=mesh.findtext('uri')
                    if uri.startswith('model://'):path=models_root/uri[len('model://'):]
                    else:path=model_path/uri
                    if path.suffix.lower()!='.dae':raise ValueError('Expected COLLADA mesh')
                    if path not in geometry_cache:geometry_cache[path]=read_dae(path)
                    instances,metadata=geometry_cache[path]
                    result['mesh_metadata'][path.name]=metadata
                    selected=instances
                    submesh=mesh.find('submesh')
                    if submesh is not None:
                        name=submesh.findtext('name')
                        selected=[i for i in instances if name in [i['name'],i['id'],i['geometry_id']]]
                        if not selected:raise ValueError('Submesh not found: '+name)
                    points=np.concatenate([i['points'] for i in selected]).copy()
                    if submesh is not None and submesh.findtext('center','false').lower()=='true':
                        points-=(points.min(axis=0)+points.max(axis=0))*.5
                    scale=np.fromstring(mesh.findtext('scale','1 1 1'),sep=' ')
                    points*=scale
                    info={'name':part.get('name'),'type':'mesh','mesh':path.name,'sdf_mesh_scale':scale.tolist()}
                elif geom.find('box') is not None:
                    size=np.fromstring(geom.findtext('box/size'),sep=' ')
                    points=corners(size)
                    info={'name':part.get('name'),'type':'box','box_size_xyz_m':size.tolist()}
                else:raise ValueError('Unsupported SDF geometry; do not silently omit it')
                points=transform(points,link_transform@pose(part.find('pose')))
                info.update(bounds(points));result[kind+'_parts'].append(info);totals[kind].append(points)
    for kind,chunks in totals.items():
        result[kind+'_bounds']=bounds(np.concatenate(chunks)) if chunks else None
    return result

def self_test():
    # Nontrivial unit, scene scale and column-major translation are essential.
    xml='''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema"><asset><unit meter=".01" name="centimeter"/><up_axis>Z_UP</up_axis></asset>
    <library_geometries><geometry id="g"><mesh><source id="pos"><float_array id="a">0 0 0 2 0 0 0 3 0</float_array><technique_common><accessor source="#a" count="3" stride="3"/></technique_common></source>
    <vertices id="v"><input semantic="POSITION" source="#pos"/></vertices><triangles count="1"><input semantic="VERTEX" source="#v" offset="0"/><p>0 1 2</p></triangles></mesh></geometry></library_geometries>
    <library_visual_scenes><visual_scene id="s"><node name="test"><matrix>100 0 0 0 0 100 0 0 0 0 100 0 5 0 0 1</matrix><instance_geometry url="#g"/></node></visual_scene></library_visual_scenes><scene><instance_visual_scene url="#s"/></scene></COLLADA>'''
    with tempfile.TemporaryDirectory() as folder:
        f=Path(folder)/'test.dae';f.write_text(xml)
        nodes,_=read_dae(f)
        assert np.allclose(nodes[0]['points'].min(axis=0),[.05,0,0])
        assert np.allclose(np.ptp(nodes[0]['points'],axis=0),[2,3,0])
    assert np.allclose(transform(corners([2,4,6]),pose(ET.fromstring('<pose>1 2 3 0 0 1.5707963267948966</pose>'))).ptp(axis=0),[4,2,6])
    print('Unit / node transform / SDF pose checks passed.')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models-root',type=Path,default=Path(__file__).resolve().parents[1]/'models')
    parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--self-test',action='store_true')
    args=parser.parse_args()
    if args.self_test:self_test()
    root=args.models_root.resolve()
    output=args.output_dir or root.parent/'measurements'
    output.mkdir(parents=True,exist_ok=True)
    models=[measure(p,root) for p in sorted(root.iterdir()) if (p/'model.sdf').is_file()]
    lines=['SubT model dimensions (meters; model-local X/Y/Z)','These are OUTER axis-aligned bounding boxes, NOT inner clearance.','']
    lines.append(f'{"Model":28} {"Collision X":>12} {"Y":>12} {"Z":>12}')
    for model in models:
        size=model['collision_bounds']['size_xyz_m']
        lines.append(f'{model["model"]:28} {size[0]:12.4f} {size[1]:12.4f} {size[2]:12.4f}')
        for p in model['collision_parts']:
            if p['type']=='box':lines.append('  '+p['name']+' box XYZ: '+', '.join(f'{v:.4f}' for v in p['box_size_xyz_m']))
    lines+=['','Visual bounding dimensions (includes visual-only scaling):']
    for model in models:
        lines.append(model['model']+': '+', '.join(f'{v:.4f}' for v in model['visual_bounds']['size_xyz_m']))
    lines+=['','JSON contains min/max coordinates, per-part bounds, units and node transforms.',
            'Length between connection planes and usable cross-section require separate measurements.']
    report={'method':'Referenced COLLADA vertices, active scene node transforms, unit meter, SDF mesh scale and link/part pose; model-local axis-aligned bounds.',
            'not_measured':['inner clearance','connector positions','effective assembly pitch'], 'models':models}
    (output/'tile_dimensions.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    (output/'tile_dimensions.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    with (output/'tile_dimensions.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.writer(f);writer.writerow(['model','geometry','X_m','Y_m','Z_m'])
        for m in models:
            for kind in ['collision','visual']:writer.writerow([m['model'],kind,*m[kind+'_bounds']['size_xyz_m']])
    print('\n'.join(lines));print('\nReports:',output)

if __name__=='__main__':main()
