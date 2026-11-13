# 固定距离测试实验脚本
import json
import math
from pathlib import Path

# 本文件只准备输入，不实现粉尘算法。
case = {
    "experiment_id": "fixed_distance_001",
    "data_source": "MOCK_ONLY",
    "truth_range_m": 10.0,
    "raw_range_m": 10.0,
    "dust_concentration": 10.0,
    "dust_concentration_unit": "mg/m3",
    "sample_count": 100,
    "sample_interval_s": 0.1,
    "seed": 42,
    "note": "Synthetic interface test; concentration is not calibrated."
}

# 检查测试输入，避免明显错误进入后续函数。
for name in ("truth_range_m", "raw_range_m"):
    value = case[name]
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} 必须是有限正数")

concentration = case["dust_concentration"]
if not math.isfinite(concentration) or concentration < 0:
    raise ValueError("粉尘浓度必须是有限非负数")

if case["raw_range_m"] != case["truth_range_m"]:
    raise ValueError("本案例为理想输入，raw 必须等于 truth")

if case["sample_count"] <= 0:
    raise ValueError("采样次数必须大于零")

output = Path(__file__).resolve().parent / "fixed_case.json"
output.write_text(
    json.dumps(case, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8"
)

print("输入检查通过")
print(f"真值距离：{case['truth_range_m']} m")
print(f"原始距离：{case['raw_range_m']} m")
print(
    f"固定浓度：{case['dust_concentration']} "
    f"{case['dust_concentration_unit']}"
)
print(f"计划采样次数：{case['sample_count']}")
print(f"输入文件：{output}")
