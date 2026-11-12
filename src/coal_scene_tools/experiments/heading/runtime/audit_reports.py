"""Independent saved-row checks; no ROS, production integration function not imported."""
import argparse,csv,json,math
from collections import Counter
from pathlib import Path
from openpyxl import load_workbook
import yaml

def audit(folder):
    folder=Path(folder);summary=json.loads((folder/'summary.json').read_text())
    cfg=yaml.safe_load((folder/'config_snapshot.yaml').read_text());zones=cfg['semantics']['zones']
    origin=[a+b for a,b in zip(cfg['scenario']['station']['xyz_m'],cfg['scenario']['sensors']['lidar']['xyz_m'])]
    rows=0;observed=set();modes=set();frame_ids=set();previous_y=None
    with (folder/'beam_detail.csv').open() as f:
        for r in csv.DictReader(f):
            rows+=1;raw=float(r['raw_range_m']);angle=float(r['angle_rad']);modes.add(r['work_mode']);frame_ids.add(r['frame_id'])
            if r['valid']=='False':
                assert r['zone_id']=='NO_RETURN';continue
            assert float(r['baseline_range_m'])==raw
            assert abs(float(r['mock_range_m'])-raw-.02)<1e-6
            delta=[-raw*math.sin(angle),raw*math.cos(angle),0.]
            boxes=[];cuts={0.,1.}
            for z in zones:
                lo=[min(p[0] for p in z['polygon_xy_m']),min(p[1] for p in z['polygon_xy_m']),z['z_min_m']]
                hi=[max(p[0] for p in z['polygon_xy_m']),max(p[1] for p in z['polygon_xy_m']),z['z_max_m']]
                boxes.append((lo,hi,z['respirable_concentration_mg_m3'],z['zone_id']))
                for a,b,o,d in zip(lo,hi,origin,delta):
                    if abs(d)>1e-12:
                        for edge in [a,b]:
                            t=(edge-o)/d
                            if 0<t<1:cuts.add(t)
            total=0.;hit=set();cuts=sorted(cuts)
            for a,b in zip(cuts,cuts[1:]):
                mid=[o+(a+b)/2*d for o,d in zip(origin,delta)];c=cfg['semantics']['background_concentration_mg_m3']
                for lo,hi,value,name in boxes:
                    if all(l<=v<h for l,v,h in zip(lo,mid,hi)):c=value;hit.add(name)
                total+=(b-a)*c
            assert abs(total-float(r['path_average_concentration_mg_m3']))<1e-8
            assert set(r['zone_id'].split('|'))==(hit or {'NONE'});observed.update(hit)
            for axis,offset in zip('xyz',cfg['scenario']['target']['xyz_m']):
                assert abs(float(r[f'target_{axis}_m'])-float(r[f'body_{axis}_m'])-offset)<1e-9
            assert float(r['pose_time_offset_s'])<=.025
            if r['beam_index']=='540':
                assert float(r['center_error_m'])<.01
                y=float(r['body_y_m'])
                if previous_y is not None:assert y>=previous_y-1e-9
                previous_y=y
    wb=load_workbook(folder/'场景实验报表.xlsx',read_only=True,data_only=True)
    assert wb['逐帧真值'].max_row==summary['frames']+1
    stats=list(wb['分区工况统计'].values)[1:]
    assert sum(row[2] for row in stats)==summary['valid_beams_checked']
    for row in stats:assert abs(row[3]-.02)<1e-9 and abs(row[4]-.02)<1e-9
    wb.close();assert rows==summary['csv_rows']
    result={'run':folder.name,'passed':True,'csv_rows_audited':rows,'saved_frames':len(frame_ids),'observed_zones':sorted(observed),'unsampled_zones':sorted(set(z['zone_id'] for z in zones)-observed),'work_modes':sorted(modes),'excel_counts_match':True}
    (folder/'independent_audit.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('results',type=Path,nargs='+');args=p.parse_args()
    print(json.dumps([audit(folder) for folder in args.results],indent=2))
