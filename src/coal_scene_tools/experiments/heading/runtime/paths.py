"""Paths for repository checkout or an explicit local workspace deployment."""
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
DEPLOYED = (ROOT / 'deployment.json').is_file()
if DEPLOYED:
    if json.loads((ROOT/'deployment.json').read_text()).get('layout_version') != 1:
        raise ValueError('Unsupported heading deployment layout')
    NATIVE_SOURCE = ROOT / 'support/coal_face_sim'
    VALIDATOR_TOOLS = ROOT / 'support/scene_tools'
    FIXED = ROOT.parents[1] / 'lidar_fixed_distance_test'
else:
    REPO = ROOT.parents[3]
    NATIVE_SOURCE = REPO / 'src/coal_face_sim'
    VALIDATOR_TOOLS = REPO / 'src/coal_scene_tools/tools'
    FIXED = ROOT.parent / 'lidar_fixed_distance_test'
MODELS = NATIVE_SOURCE / 'scenarios/coal_face_v1/models'
