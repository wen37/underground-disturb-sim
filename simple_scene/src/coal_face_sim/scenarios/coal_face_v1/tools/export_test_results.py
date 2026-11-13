#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Archive completed evidence to the Windows formal platform directory."""
import hashlib,json,shutil
from pathlib import Path
root=Path(__file__).resolve().parents[1]
dest=Path('/mnt/d/work1/work1-codex/scenarios/coal_face_v1')
assert (root/'validation/full_scale/verdict.json').exists(), 'Do not export incomplete final results'
for name in ['full_scale','full_scale_attempt1']:
    shutil.copytree(root/'validation'/name,dest/'validation'/name,dirs_exist_ok=True)
shutil.copy2(root/'worlds/coal_face.sdf',dest/'worlds/coal_face.sdf')
runtime=dest/'runtime_sources';runtime.mkdir(exist_ok=True)
pkg=root.parents[1]
for name in ['scripts/coal_face_motion.py','launch/coal_face_full.launch.py','CMakeLists.txt']:
    shutil.copy2(pkg/name,runtime/Path(name).name)
shutil.copytree(dest/'tools',root/'tools',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
shutil.copytree(dest/'design_v02',root/'design_v02',dirs_exist_ok=True)
if (dest/'validation/full_scale/rviz_scan_overlay.png').exists():
    shutil.copy2(dest/'validation/full_scale/rviz_scan_overlay.png',root/'validation/full_scale/rviz_scan_overlay.png')
manifest={}
for name in ['full_scale','full_scale_attempt1']:
    for f in (root/'validation'/name).rglob('*'):
        if not f.is_file():continue
        relative=f.relative_to(root);target=dest/relative
        digest=hashlib.sha256(f.read_bytes()).hexdigest()
        assert target.exists() and hashlib.sha256(target.read_bytes()).hexdigest()==digest, str(relative)
        manifest[str(relative)]={'size':f.stat().st_size,'sha256':digest}
(dest/'validation/full_scale/export_manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps({'copied_and_verified_files':len(manifest),'total_bytes':sum(x['size'] for x in manifest.values()),'destination':str(dest)},indent=2))
