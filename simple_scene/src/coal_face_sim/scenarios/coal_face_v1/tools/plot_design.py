# 资源路径由当前脚本位置解析，便于迁移工作区。
from pathlib import Path
import json
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle
from measure_clearance import collision_triangles,section
from measure_tiles import rotation,transform

BASE=Path(__file__).resolve().parents[1]
ROOT=Path(__file__).resolve().parents[1]/'models'
OUT=BASE/'design_v02'
cfg=json.loads((OUT/'layout_parameters.json').read_text());check=json.loads((OUT/'geometric_checks.json').read_text())
font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/msyh.ttc')
plt.rcParams['font.family']=font_manager.FontProperties(fname='/mnt/c/Windows/Fonts/msyh.ttc').get_name()
plt.rcParams['axes.unicode_minus']=False
fig,(plan,sec)=plt.subplots(1,2,figsize=(12,11),gridspec_kw={'width_ratios':[1,1.15]})
colors={'support':'#527da6','conveyor':'#22938b','coal_boundary':'#514c45','shearer':'#eeb533'}
for t in cfg['tile_instances']:
    x,y,_=t['position_xyz_m'];yaw=t['yaw_rad']
    if t['model'].endswith('_5'):
        p=np.array([[0,-10,0],[0,10,0]])@rotation([0,0,1],yaw)[:3,:3].T+[x,y,0]
        plan.plot(p[:,0],p[:,1],color='#a8b9cb',linewidth=9,solid_capstyle='butt')
    else:
        a=np.linspace(0,math.pi/2,80)
        p=np.c_[10-10*np.cos(a),-10+10*np.sin(a),np.zeros(len(a))]@rotation([0,0,1],yaw)[:3,:3].T+[x,y,0]
        plan.plot(p[:,0],p[:,1],color='#a8b9cb',linewidth=9,solid_capstyle='butt')
for obj in cfg['equipment']:
    c=obj['center_xyz_m'];s=obj['size_xyz_m']
    plan.add_patch(Rectangle((c[0]-s[0]/2,c[1]-s[1]/2),s[0],s[1],color=colors[obj['role']]))
plan.plot([.6,.6],[5,195],color='#cb8d0b',linewidth=1.2)
plan.scatter([0,0],[0,200],color='#193a57',s=12)
plan.text(-68,217,'回风巷：60 m 直段',fontsize=10)
plan.text(-68,-20,'运输巷：60 m 直段',fontsize=10)
plan.text(6,100,'200 m 工作面\n（不含弯段）',rotation=90,va='center',fontsize=10)
plan.text(-68,90,'16 个直段\n2 个弯段\n133 个简化支架\n20 段输送机',fontsize=10,color='#29435c')
plan.set_xlim(-78,20);plan.set_ylim(-28,228);plan.set_aspect('equal');plan.set_xlabel('世界 X / m');plan.set_ylabel('世界 Y / m')
plan.set_title('按坐标绘制的平面布置\n浅蓝线表示通道轴线及走向',fontsize=12);plan.grid(alpha=.15)
tri=collision_triangles(ROOT/'subt_tunnel_tile_5',ROOT)
worst=check['support_roof_worst_sample_world_xyz_m'];y=worst[1]%20-10
segments=section(tri,np.array([0.,1.,0.]),y)
for line in segments:sec.plot(line[:,0],line[:,2]-.0005,color='#637486',lw=.8)
example=next(o for o in cfg['equipment'] if o['role']=='support')
for obj,label in [(example,'简化支架'),(next(o for o in cfg['equipment'] if o['role']=='conveyor'),'输送机'),
                   (next(o for o in cfg['equipment'] if o['role']=='coal_boundary'),'煤壁边界'),(cfg['shearer_preview'],'采煤机运动包络')]:
    c=obj['center_xyz_m'];s=obj['size_xyz_m']
    sec.add_patch(Rectangle((c[0]-s[0]/2,c[2]-.0005-s[2]/2),s[0],s[2],facecolor=colors[obj['role']],alpha=.8,label=label))
sec.axhline(0,color='#7a8b9c',lw=.6)
sec.annotate('支架顶面至拱顶\n采样最小间隙约 %.3f m'%check['support_top_roof_gap_min_m'],xy=(worst[0],2.8),xytext=(-3.3,4.35),
             arrowprops={'arrowstyle':'->','color':'#244964'},fontsize=10,color='#244964')
sec.set_xlim(-3.6,3.6);sec.set_ylim(-.25,4.8);sec.set_aspect('equal');sec.grid(alpha=.15)
sec.set_xlabel('工作面横向 X / m');sec.set_ylabel('相对地板高度 / m')
sec.set_title('设备断面布置\n采煤机用全行程包络截面表示',fontsize=12)
sec.legend(loc='lower center',bbox_to_anchor=(.5,-.31),ncol=2,fontsize=9)
fig.suptitle('采煤工作面场景设计 V0.2｜几何设计与静态预览',fontsize=16,y=.97)
fig.text(.52,.13,'尺寸为开发占位值，待实际设备资料确认。\n几何检查通过不等于完整运动或工程验收。',fontsize=10,color='#596c7b')
fig.tight_layout(rect=[0,.05,1,.94]);fig.savefig(OUT/'场景设计图V0.2.png',dpi=160);plt.close(fig)
print(OUT/'场景设计图V0.2.png')
