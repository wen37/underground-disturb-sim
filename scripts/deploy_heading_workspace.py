"""Deploy identical heading sources to an existing cave_eval_ws without symlinks.

Results, evidence and S00 are retained. Replaced files are backed up first.
"""
import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

REPO = Path(__file__).resolve().parents[1]
HEADING = REPO / 'src/coal_scene_tools/experiments/heading'
FIXED = HEADING.parent / 'lidar_fixed_distance_test'

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def deploy(workspace, check=False):
    workspace = workspace.expanduser().resolve()
    target = workspace / 'src/coal_heading_scene'
    if not (workspace / 'src').is_dir():
        raise ValueError('Expected an existing workspace with a src directory')
    if target.resolve() == HEADING.resolve():
        raise ValueError('Deployment destination must differ from repository sources')
    pairs = []
    def add(source, dest):
        if not dest.resolve().is_relative_to(workspace) or dest.is_symlink():
            raise ValueError('Destination escapes workspace or is a symlink: '+str(dest))
        pairs.append((source, dest))
    for folder in ['runtime', 'tests', 'scenes/S01_subt_heading']:
        for source in sorted((HEADING/folder).rglob('*')):
            if source.is_file() and '__pycache__' not in source.parts:
                add(source, target/source.relative_to(HEADING))
    add(HEADING/'README.md',target/'README.md')
    for source in sorted(FIXED.iterdir()):
        if source.is_file(): add(source,workspace/'lidar_fixed_distance_test'/source.name)
    for source in sorted((REPO/'docs').glob('HEADING_*')):
        add(source,target/'docs'/source.name)
    native = REPO/'src/coal_face_sim'
    for source in sorted(native.rglob('*')):
        if source.is_file() and '__pycache__' not in source.parts:
            add(source,target/'support/coal_face_sim'/source.relative_to(native))
    add(REPO/'src/coal_scene_tools/tools/validate_scene.py',target/'support/scene_tools/validate_scene.py')
    marker=target/'deployment.json'
    expected_runtime={dest.name for _,dest in pairs if dest.parent==target/'runtime'}
    obsolete=[target/'references'/name for name in ['template_README.md','loader_constraints.md','assets_README.md']]
    obsolete += [target/'CONFIGURATION.md']
    obsolete += [p for p in (target/'runtime').glob('*') if p.is_file() and p.name not in expected_runtime]
    if check:
        missing=[str(dest.relative_to(workspace)) for source,dest in pairs if not dest.is_file() or digest(source)!=digest(dest)]
        stale=[str(p.relative_to(workspace)) for p in obsolete if p.exists()]
        if not marker.exists():missing.append(str(marker.relative_to(workspace)))
        else:
            data=json.loads(marker.read_text())
            if data.get('layout_version')!=1:missing.append('deployment layout version')
        result={'passed':not missing and not stale,'checked_files':len(pairs),'different_or_missing':missing,'obsolete_files':stale}
        print(json.dumps(result,ensure_ascii=False,indent=2));return result['passed']
    # Require the restored mesh before changing a working deployment.
    if not (native/'scenarios/coal_face_v1/models/subt_tunnel_tile_5/meshes/tunnel_tile_5.dae').is_file():
        raise ValueError('Restore repository assets before deployment: python3 scripts/restore_assets.py')
    stamp=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%dT%H%M%S_%f')
    backup=target/'backups'/('workspace_sync_'+stamp)
    def save(path):
        if not path.resolve().is_relative_to(workspace) or path.is_symlink():
            raise ValueError('Refusing to modify path outside workspace: '+str(path))
        if path.exists():
            saved=backup/path.relative_to(workspace);saved.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,saved)
    changed=0
    for source,dest in pairs:
        if dest.is_file() and digest(source)==digest(dest):continue
        save(dest);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest);changed+=1
    removed=[]
    for path in obsolete:
        if path.exists():save(path);path.unlink();removed.append(str(path.relative_to(workspace)))
    save(marker)
    commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip()
    marker.write_text(json.dumps({'layout_version':1,'source_commit':commit,'source_dirty':bool(subprocess.check_output(['git','status','--porcelain'],cwd=REPO,text=True).strip()),'deployed_at':stamp,'files':{str(dest.relative_to(workspace)):digest(source) for source,dest in pairs}},ensure_ascii=False,indent=2)+'\n')
    (target/'support/COLCON_IGNORE').touch()
    print(json.dumps({'changed_files':changed,'archived_obsolete':removed,'backup':str(backup),'source_commit':commit},ensure_ascii=False,indent=2))
    return deploy(workspace,check=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace',type=Path,required=True)
    p.add_argument('--check',action='store_true',help='Read-only byte comparison; no deployment')
    args=p.parse_args();raise SystemExit(0 if deploy(args.workspace,args.check) else 1)
