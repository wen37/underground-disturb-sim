from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[3]
MODELS = REPO / 'src/coal_face_sim/scenarios/coal_face_v1/models'
FIXED = ROOT.parent / 'lidar_fixed_distance_test'
