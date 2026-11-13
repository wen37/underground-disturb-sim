#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Sample internal collision clearance; never confuse outer bounds with bore size."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from measure_tiles import NS,pose,rotation,transform,corners,measure

def dae_triangles(path):
    root=ET.parse(path).getroot()
    meter=float(root.find('c:asset/c:unit',NS).get('meter','1'))
    assert root.findtext('c:asset/c:up_axis',namespaces=NS)=='Z_UP'
    geometries={}
    for geometry in root.findall('c:library_geometries/c:geometry',NS):
        mesh=geometry.find('c:mesh',NS);sources={};vertices={};triangles=[]
        for source in mesh.findall('c:source',NS):
            accessor=source.find('c:technique_common/c:accessor',NS)
            if accessor is None:continue
            array=source.find("c:float_array[@id='%s']"%accessor.get('source')[1:],NS)
            if array is None:continue
            stride=int(accessor.get('stride','1'));offset=int(accessor.get('offset','0'));count=int(accessor.get('count'))
            sources[source.get('id')]=np.fromstring(array.text or '',sep=' ')[offset:offset+stride*count].reshape(count,stride)
        for v in mesh.findall('c:vertices',NS):
            position=v.find("c:input[@semantic='POSITION']",NS)
            if position is not None:vertices[v.get('id')]=position.get('source')[1:]
        for primitive in list(mesh):
            kind=primitive.tag.split('}')[-1]
            if kind in ['source','vertices','extra']:continue
            if kind not in ['triangles','polylist','polygons']:raise ValueError('Unsupported primitive '+kind)
            inputs=primitive.findall('c:input',NS)
            vertex=next((e for e in inputs if e.get('semantic')=='VERTEX'),None)
            if vertex is None:continue
            width=max(int(e.get('offset','0')) for e in inputs)+1
            offset=int(vertex.get('offset','0'))
            points=sources[vertices[vertex.get('source')[1:]]][:,:3]
            for p in primitive.findall('c:p',NS):
                ids=np.fromstring(p.text or '',sep=' ',dtype=int).reshape(-1,width)[:,offset]
                if not len(ids):continue
                if kind=='triangles':triangles.append(points[ids.reshape(-1,3)])
                else:
                    counts=np.fromstring(primitive.findtext('c:vcount',namespaces=NS),sep=' ',dtype=int) if kind=='polylist' else [len(ids)]
                    start=0
                    for count in counts:
                        face=ids[start:start+count];start+=count
                        for j in range(1,count-1):triangles.append(points[[face[0],face[j],face[j+1]]][None,:,:])
        if triangles:geometries[geometry.get('id')]=np.concatenate(triangles)
    reference=root.find('c:scene/c:instance_visual_scene',NS).get('url')[1:]
    scene=root.find("c:library_visual_scenes/c:visual_scene[@id='%s']"%reference,NS)
    instances=[]
    def visit(node,parent):
        local=np.eye(4)
        for child in node:
            tag=child.tag.split('}')[-1]
            if tag in ['matrix','translate','scale','rotate']:
                vals=np.fromstring(child.text or '',sep=' ');part=np.eye(4)
                if tag=='matrix':part=vals.reshape(4,4).T
                elif tag=='translate':part[:3,3]=vals
                elif tag=='scale':part[:3,:3]=np.diag(vals)
                else:part=rotation(vals[:3],math.radians(vals[3]))
                local=local@part
            elif tag in ['lookat','skew','instance_node']:raise ValueError('Unsupported transform '+tag)
        full=parent@local
        for instance in node.findall('c:instance_geometry',NS):
            key=instance.get('url')[1:]
            if key in geometries:
                tri=geometries[key]
                tri=(transform(tri.reshape(-1,3),full)*meter).reshape(-1,3,3)
                instances.append((node.get('name'),node.get('id'),key,tri))
        for child in node.findall('c:node',NS):visit(child,full)
    for node in scene.findall('c:node',NS):visit(node,np.eye(4))
    return instances

def collision_triangles(folder,root):
    model=ET.parse(folder/'model.sdf').getroot().find('model');chunks=[]
    faces=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                    [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    for link in model.findall('link'):
        lp=pose(link.find('pose'))
        for collision in link.findall('collision'):
            geometry=collision.find('geometry');mesh=geometry.find('mesh')
            if mesh is not None:
                uri=mesh.findtext('uri');path=root/uri[len('model://'):] if uri.startswith('model://') else folder/uri
                instances=dae_triangles(path);sub=mesh.find('submesh')
                if sub is not None:
                    name=sub.findtext('name');instances=[i for i in instances if name in i[:3]]
                    if sub.findtext('center','false')=='true':raise ValueError('Centered collision submesh not supported')
                tri=np.concatenate([i[3] for i in instances])
                tri*=np.fromstring(mesh.findtext('scale','1 1 1'),sep=' ')
            elif geometry.find('box') is not None:
                tri=corners(np.fromstring(geometry.findtext('box/size'),sep=' '))[faces]
            else:raise ValueError('Unsupported collision geometry')
            tri=transform(tri.reshape(-1,3),lp@pose(collision.find('pose'))).reshape(-1,3,3)
            chunks.append(tri)
    result=np.concatenate(chunks)
    reference=measure(folder,root)['collision_bounds']
    assert np.allclose(result.min(axis=(0,1)),reference['min_xyz_m'])
    assert np.allclose(result.max(axis=(0,1)),reference['max_xyz_m'])
    return result

class Rays:
    def __init__(self,triangles):
        self.triangles=triangles
        self.a=triangles[:,0,:];self.e1=triangles[:,1,:]-self.a;self.e2=triangles[:,2,:]-self.a
    def cast(self,origins,directions,max_distance=35):
        origins=np.atleast_2d(origins);directions=np.atleast_2d(directions)
        if len(directions)==1:directions=np.repeat(directions,len(origins),axis=0)
        if len(origins)==1:origins=np.repeat(origins,len(directions),axis=0)
        out=[]
        for start in range(0,len(origins),24):
            origin=origins[start:start+24,None,:];direction=directions[start:start+24,None,:]
            h=np.cross(direction,self.e2);det=np.sum(self.e1*h,axis=2)
            s=origin-self.a;u_num=np.sum(s*h,axis=2)
            q=np.cross(s,self.e1);v_num=np.sum(direction*q,axis=2);t_num=np.sum(self.e2*q,axis=2)
            with np.errstate(divide='ignore',invalid='ignore'):
                u=u_num/det;v=v_num/det;t=t_num/det
            with np.errstate(invalid='ignore'):
                valid=(np.abs(det)>1e-10)&(u>=-1e-8)&(v>=-1e-8)&(u+v<=1+1e-8)&(t>1e-6)&(t<=max_distance)
            out.extend(np.where(valid,t,np.inf).min(axis=1).tolist())
        return np.asarray(out)

def station_set(identifier,step):
    if identifier=='subt_tunnel_tile_5':
        s=np.arange(-9.75,9.751,step)
        return np.c_[np.zeros(len(s)),s],np.tile([1.,0.],(len(s),1)),['straight']*len(s)
    if identifier=='subt_tunnel_tile_1':
        # Arms outside the central branch opening; a single central width is undefined.
        s=np.arange(4.,9.751,step);p=[];n=[];labels=[]
        for label,axis,normal in [('north',[0,1],[1,0]),('south',[0,-1],[1,0]),('east',[1,0],[0,1]),('west',[-1,0],[0,1])]:
            for v in s:p.append(np.array(axis)*v);n.append(normal);labels.append(label)
        return np.asarray(p),np.asarray(n),labels
    if identifier=='subt_tunnel_tile_2':
        # Nominal connection axes from the archived geometry; validate clear space below.
        length=math.pi*10/2
        angles=np.arange(.25,length-.249,step)/10
        p=np.c_[10-10*np.cos(angles),-10+10*np.sin(angles)]
        normals=np.c_[-np.cos(angles),np.sin(angles)]
        return p,normals,['bend']*len(p)
    raise ValueError(identifier)

def interval(values):
    arr=np.asarray(values);finite=arr[np.isfinite(arr)]
    return {'min_m':float(finite.min()),'max_m':float(finite.max()),'valid':len(finite),'total':len(arr)} if len(finite) else {'valid':0,'total':len(arr)}

def section(triangles,normal,offset):
    signed=triangles@normal-offset
    candidates=triangles[(signed.min(axis=1)<-1e-9)&(signed.max(axis=1)>1e-9)]
    segments=[]
    for tri in candidates:
        d=tri@normal-offset;points=[]
        for i,j in [(0,1),(1,2),(2,0)]:
            if (d[i]<=0<d[j]) or (d[j]<=0<d[i]):points.append(tri[i]+(tri[j]-tri[i])*(-d[i])/(d[j]-d[i]))
        if len(points)==2:segments.append(points)
    return np.asarray(segments)

def run(root,output,step):
    output.mkdir(parents=True,exist_ok=True)
    results=[];allrows=[];plot_data=[]
    heights=[.25,.5,1.,1.5,2.,2.5,2.8,3.]
    for identifier in ['subt_tunnel_tile_5','subt_tunnel_tile_2','subt_tunnel_tile_1']:
        print('Measuring',identifier,flush=True)
        triangles=collision_triangles(root/identifier,root);rays=Rays(triangles)
        xy,normals,branches=station_set(identifier,step)
        origin=np.c_[xy,np.ones(len(xy))]
        floor=1.-rays.cast(origin,[0,0,-1])
        roof_origins=np.c_[xy,floor+.01]
        roof=floor+.01+rays.cast(roof_origins,[0,0,1])
        ceiling=roof-floor
        height_widths={}
        for height in heights:
            origins=np.c_[xy,floor+height]
            nd=np.c_[normals,np.zeros(len(xy))]
            left=rays.cast(origins,nd);right=rays.cast(origins,-nd)
            # Above the local ceiling is not valid free-space measurement.
            width=np.where(ceiling>height,left+right,np.nan)
            height_widths[str(height)]=interval(width)
            for i in range(len(xy)):
                allrows.append({'model':identifier,'branch':branches[i],'station':i,'x_m':float(xy[i,0]),'y_m':float(xy[i,1]),
                    'floor_z_m':float(floor[i]),'headroom_centerline_m':float(ceiling[i]),'height_above_floor_m':height,
                    'clearance_left_m':float(left[i]),'clearance_right_m':float(right[i]),'clear_width_m':float(width[i])})
        valid_path=np.isfinite(floor)&np.isfinite(ceiling)&(floor>-.2)&(floor<.9)&(ceiling>1.5)
        result={'model':identifier,'stations':len(xy),'sampling_step_approx_m':step,
            'path_valid':bool(valid_path.all()),'invalid_station_indices':np.flatnonzero(~valid_path).tolist(),
            'centerline_floor_z_m':interval(floor),'centerline_headroom_m':interval(ceiling),'width_by_height':height_widths,
            'measurement_region': 'Straight Y=-9.75..9.75, X=0' if identifier.endswith('_5') else
                'Quarter-circle R=10, center=(10,-10), inward 0.25 m from openings; radial cross sections' if identifier.endswith('_2') else
                'Four arms: 4..9.75 m from junction center; central open crossing excluded from width statistics',
            'collision_triangles':len(triangles)}
        if identifier.endswith('_1'):
            center=np.array([[0,0,1.]])
            floor0=1-rays.cast(center,[0,0,-1])[0]
            roof0=floor0+.01+rays.cast([[0,0,floor0+.01]],[0,0,1])[0]
            result['junction_center_headroom_m']=float(roof0-floor0)
        results.append(result)
        mid=len(xy)//2 if identifier!='subt_tunnel_tile_1' else 12
        # Representative cross section plus horizontal plan for method verification.
        normal=normals[mid];tangent=np.array([-normal[1],normal[0],0.])
        # Avoid slicing exactly along coplanar mesh seams (notably straight Y=0).
        section_xy=xy.copy();section_xy[mid]+=tangent[:2]*.013
        p=np.r_[section_xy[mid],0.]
        cross=section(triangles,tangent,float(p@tangent))
        plan=section(triangles,np.array([0.,0.,1.]),1.)
        plot_data.append((identifier,xy,normals,cross,plan,mid,floor[mid],ceiling[mid],section_xy[mid]))
    blocker=collision_triangles(root/'subt_tunnel_tile_blocker',root)
    results.append({'model':'subt_tunnel_tile_blocker','inner_clearance':'not applicable: closure component, not a tunnel passage',
        'outer_size_xyz_m':np.ptp(blocker,axis=(0,1)).tolist()})
    source_hashes={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest()
                   for folder in sorted(root.iterdir()) if folder.is_dir()
                   for f in [folder/'model.sdf',*folder.glob('meshes/*.dae')] if f.is_file()}
    report={'units':'meters','source_sha256':source_hashes,'method':'Double-sided nearest ray/triangle intersection against all SDF collision meshes and ground boxes; local COLLADA unit and scene transforms included.',
        'width_definition':'At each sampled centerline station, sum distances to first obstacles on both sides, in a plane normal to path; heights are measured above the centerline floor.',
        'height_definition':'First overhead collision hit directly above sampled centerline floor.',
        'limits':['Discrete samples do not prove global minimum clearance.','Widths at fixed heights are not a guarantee that a rectangular equipment envelope fits.',
            'Junction central crossing has no single closed transverse width.','No real mine dimensions or safety clearance are implied.'],
        'sample_step_m':step,'models':results}
    (output/'inner_clearance.json').write_text(json.dumps(report,indent=2,allow_nan=True))
    with (output/'inner_clearance_samples.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=list(allrows[0]));writer.writeheader();writer.writerows(allrows)
    make_plots(plot_data,output)
    print(json.dumps(results,indent=2),flush=True)
    return report

def make_plots(data,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig,axes=plt.subplots(2,3,figsize=(15,9))
    for col,(name,xy,normals,cross,plan,mid,floor,headroom,section_xy) in enumerate(data):
        ax=axes[0,col]
        if len(plan):ax.add_collection(LineCollection(plan[:,:,:2],colors='#526778',linewidths=.55))
        ax.plot(xy[:,0],xy[:,1],'.',color='#d44a38',markersize=2,label='sampled path')
        center=section_xy;n=normals[mid]
        ax.plot([center[0]-4*n[0],center[0]+4*n[0]],[center[1]-4*n[1],center[1]+4*n[1]],color='#198c85',lw=2,label='section below')
        ax.autoscale();ax.set_aspect('equal');ax.set_title(name+' / plan at Z=1 m');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.legend(fontsize=8)
        ax=axes[1,col]
        if len(cross):
            relative=cross-np.r_[center,0.]
            section_xy=np.stack([relative@np.r_[n,0.],cross[:,:,2]-floor],axis=-1)
            ax.add_collection(LineCollection(section_xy,colors='#526778',linewidths=.75))
        for h in [1.,2.,2.8]:ax.axhline(h,color='#459a9a',lw=.5,ls='--')
        ax.plot(0,0,'o',color='#d44a38',markersize=4)
        ax.set_xlim(-4,4);ax.set_ylim(-.2,5.5);ax.set_aspect('equal');ax.grid(alpha=.2)
        ax.set_title('Representative section at X=%.2f, Y=%.2f'%tuple(center))
        ax.set_xlabel('Transverse position (m)');ax.set_ylabel('Height above centerline floor (m)')
    fig.suptitle('Collision-based sampled tunnel clearance (not a guaranteed equipment envelope)',fontsize=14)
    fig.tight_layout(rect=[0,0,1,.95]);fig.savefig(output/'inner_clearance_sections.png',dpi=150);plt.close(fig)

def self_test():
    # A known rectangular corridor verifies dimensions, floor box and double-sided hits.
    vertices=corners([6,20,4]);vertices[:,2]+=2
    faces=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    rays=Rays(vertices[faces])
    assert np.allclose(rays.cast([[0,0,1]],[[0,0,-1]]),[1.])
    assert np.allclose(rays.cast([[0,0,1]],[[0,0,1]]),[3.])
    assert np.allclose(rays.cast([[0,0,1]],[[1,0,0]])+rays.cast([[0,0,1]],[[-1,0,0]]),[6.])
    print('Known-box floor/ceiling/width ray tests passed.',flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--models-root',type=Path,default=Path(__file__).resolve().parents[1]/'models')
    parser.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parents[1]/'measurements/inner_clearance_dense')
    parser.add_argument('--step',type=float,default=.05)
    args=parser.parse_args()
    self_test();run(args.models_root,args.output_dir,args.step)
