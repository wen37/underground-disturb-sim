"""粉尘模块接口。当前仅固定偏差 Mock，不具有粉尘物理意义。"""
from dataclasses import dataclass
import math
from random import Random

MODEL_NAME = 'fixed_bias_mock'
MODEL_VERSION = 'mock-fixed-bias-v1'
DATA_SOURCE = 'MOCK_ONLY'
MOCK_BIAS_M = 0.02  # 单位 m；只在此处设置 Mock 偏差。
MODEL_NOTE = '浓度仅接收、不参与计算；不预测光束状态；无随机噪声。'


@dataclass(frozen=True)
class Measurement:
    observed_range_m: float | None
    valid: bool
    state: str


def measure(*, raw_range_m: float, concentration_mg_m3: float,
            enabled: bool, rng: Random) -> Measurement:
    """输入：原始距离(m)、浓度(mg/m³)、开关、整次运行共享的随机发生器。

    输出：最终观测距离(m)、有效性、状态。不是只返回误差量。
    失效约定：Measurement(None, False, 'NO_RETURN')，不要以0表示丢点。
    真值不进入本接口；浓度与rng当前未参与Mock计算。
    后续真实模型在enabled分支内接入；关闭时必须原样返回。
    """
    if not math.isfinite(raw_range_m) or raw_range_m <= 0:
        raise ValueError('raw_range_m 必须为有限正数')
    if not math.isfinite(concentration_mg_m3) or concentration_mg_m3 < 0:
        raise ValueError('浓度必须为有限非负数，单位mg/m³')
    if not enabled:
        return Measurement(raw_range_m, True, 'BYPASS')

    # 后续替换这里；真实函数的单位、输出含义和失效语义应在本文件内适配。
    # 若真实函数返回距离，直接使用；若返回有符号误差，则raw+error。
    observed = raw_range_m + MOCK_BIAS_M
    if not math.isfinite(observed) or observed <= 0:
        raise ValueError('Mock偏差导致非法观测距离')
    return Measurement(observed, True, 'MOCK_FIXED_BIAS')


def model_parameters() -> dict:
    """导出实际使用的模型参数。替换真实模型时同步更新。"""
    return {'mock_bias_m': MOCK_BIAS_M, 'note': MODEL_NOTE}
