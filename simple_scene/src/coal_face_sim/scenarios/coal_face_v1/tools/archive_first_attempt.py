# 资源路径由当前脚本位置解析，便于迁移工作区。
from pathlib import Path
p=Path(__file__).resolve().parents[1]/'validation'
a=p/'full_scale';b=p/'full_scale_attempt1'
assert a.is_dir() and not b.exists()
a.rename(b)
a.mkdir()
print('Preserved attempt 1; new output directory is empty.')
