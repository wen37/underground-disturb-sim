#!/usr/bin/env python3
"""Verify installed scene tools with a relocated template, not a fixed source path."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from ament_index_python.packages import get_package_share_directory


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=Path('scene_tools_verification.json'))
    parser.add_argument('--csv-root',type=Path)
    args=parser.parse_args()
    checks=[]
    def run(command):
        result=subprocess.run(command,capture_output=True,text=True)
        checks.append({'command':command,'returncode':result.returncode,'stdout':result.stdout,'stderr':result.stderr})
        if result.returncode:raise RuntimeError(result.stderr or result.stdout)
        return json.loads(result.stdout)
    try:
        share=Path(get_package_share_directory('coal_scene_tools'))
        with tempfile.TemporaryDirectory(prefix='coal_scene_install_') as folder:
            root=Path(folder)/'scenes'
            root.mkdir()
            scene=root/'S01_straight'
            shutil.copytree(share/'scenes/S01_straight',scene)
            assert run(['ros2','run','coal_scene_tools','validate_scene',str(scene)])['passed']
            plan=run(['ros2','run','coal_scene_tools','scene_loader','--scenes-root',str(root),
                      '--scene-id','S01_straight','--allow-skeleton'])
            assert plan['passed'] and plan['mode']=='plan_only' and Path(plan['world']).is_relative_to(scene)
        csv_count=0
        if args.csv_root:
            schema=json.loads((args.csv_root/'tables.schema.json').read_text())
            assert schema['table_schema_version']=='0.1.0' and len(schema['tables'])==11
            for name,table in schema['tables'].items():
                rows=list(csv.reader((args.csv_root/(name+'.csv')).open(encoding='utf-8-sig')))
                assert len(rows)==1
                assert rows[0]==[r['column'] for r in table['columns']]
                assert len(rows[0])==len(set(rows[0]))
                assert {'scene_id','scene_version','run_id','schema_version','table_schema_version'}<=set(rows[0])
                csv_count+=1
        report={'passed':True,'relocated_s01_checked':True,'installed_share':str(share),'csv_tables_checked':csv_count,
                'scope':'D preflight and CSV headers; not dynamic scene loading or export implementations','checks':checks}
    except BaseException as exc:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps({'passed':False,'error':str(exc),'checks':checks},ensure_ascii=False,indent=2))
        raise
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='checks'},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
