#!/usr/bin/env python3
# 资源路径由当前脚本位置解析，便于迁移工作区。
"""Add a separate full-scale entry; preserve the original single-tile demo."""
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

pkg = Path(__file__).resolve().parents[3]
scene = pkg/'scenarios/coal_face_v1'
backup = scene/'validation/full_scale/baseline'
backup.mkdir(parents=True, exist_ok=True)
for name in ['CMakeLists.txt', 'scripts/shearer_motion.py', 'worlds/coal_face_motion.world']:
    dest = backup/name.replace('/', '__')
    if not dest.exists(): shutil.copy2(pkg/name, dest)
tree = ET.parse(scene/'worlds/coal_face_design_preview.sdf')
world = tree.getroot().find('world')
world.set('name', 'coal_face_v1')
world.remove(world.find("model[@name='shearer_design']"))
for old in world.findall('plugin'): world.remove(old)
original = ET.parse(pkg/'worlds/coal_face_motion.world').getroot().find('world')
body = original.find("model[@name='shearer']")
body.find('pose').text = '0.6 5 0.6605 0 0 0'
# Preserve the configured embedded trajectory when rebuilding scene geometry.
current = ET.parse(scene/'worlds/coal_face.sdf').find('.//model[@name="shearer"]/plugin[@name="coal_face_trajectory"]')
if current is None:
    raise RuntimeError('Missing embedded trajectory configuration; run the control installation first')
body.insert(next((i for i,e in enumerate(body) if e.tag=='plugin'),len(body)),current)
world.append(body)
world.append(original.find("plugin[@name='gazebo_ros_state']"))
physics = ET.SubElement(world, 'physics', name='default', type='ode')
ET.SubElement(physics, 'max_step_size').text = '0.001'
ET.SubElement(physics, 'real_time_update_rate').text = '1000'
ET.indent(tree, space='  ')
tree.write(scene/'worlds/coal_face.sdf', encoding='utf-8', xml_declaration=True)
print('Prepared full-scale scene with embedded physics-step trajectory; existing scripts and ROS 2 launch preserved.')
