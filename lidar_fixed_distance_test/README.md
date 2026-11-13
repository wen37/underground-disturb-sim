# 固定距离实验与共享扰动接口

dust_model.py是两套实验共用的模块，不是仅用于单测的文件。run_fixed_distance.py和掘进场景runtime/run_heading.py都会调用measure()。

```bash
python3 prepare_input.py
python3 run_fixed_distance.py
```

prepare_input.py只准备fixed_case.json；run_fixed_distance.py执行基线和Mock实验并输出results中的CSV、Excel。

真实扰动模型在dust_model.measure的enabled分支内适配；返回Measurement（观测距离、有效性、状态）。目前固定偏差0.02m，浓度未参与计算。替换模型时还须同步处理场景调用方的无回波语义、固定偏差验收和版本/校验清单。

preserved_sha256.json记录当前审核认可的文件版本。prepare_input.py的两处注释/终端提示删减由用户于2026-11-13明确要求保留，模型和数据格式未改变。
