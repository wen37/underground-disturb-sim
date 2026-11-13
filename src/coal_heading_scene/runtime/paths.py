"""根据唯一工作区布局解析路径，无需个人主目录或软链接。"""
from pathlib import Path
# 源码与共享输入均从当前文件位置解析。
ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]
NATIVE_SOURCE = ROOT / 'support/coal_face_sim'
VALIDATOR_TOOLS = ROOT / 'support/scene_tools'
FIXED = WORKSPACE / 'lidar_fixed_distance_test'
MODELS = NATIVE_SOURCE / 'scenarios/coal_face_v1/models'
# 构建产物与验收输出分开存放，不纳入源码备份。
NATIVE_BUILD = WORKSPACE / 'build/heading_native'
NATIVE_INSTALL = WORKSPACE / 'install/heading_native'
NATIVE_LOG = WORKSPACE / 'log/heading_native'
