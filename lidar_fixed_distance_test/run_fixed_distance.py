"""读取fixed_case.json，调用独立粉尘模块，保存CSV、Excel及追溯信息。"""
import csv
import hashlib
import json
import math
from pathlib import Path
import random
from datetime import datetime
from zoneinfo import ZoneInfo

from openpyxl import Workbook, load_workbook
from openpyxl.styles import Alignment, Font, PatternFill
from openpyxl.utils import get_column_letter
import dust_model

ROOT = Path(__file__).resolve().parent


def validate(c):
    for name in ['truth_range_m', 'raw_range_m', 'dust_concentration', 'sample_interval_s']:
        value = c[name]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f'{name}必须为有限数值')
    if min(c['truth_range_m'], c['raw_range_m'], c['sample_interval_s']) <= 0:
        raise ValueError('距离和时间间隔必须大于零')
    if c['dust_concentration'] < 0 or c['dust_concentration_unit'] != 'mg/m3':
        raise ValueError('浓度必须非负且单位必须为mg/m3；其他单位需显式换算')
    if c['raw_range_m'] != c['truth_range_m']:
        raise ValueError('本固定距离理想输入实验要求raw等于truth')
    if type(c['sample_count']) is not int or not 1 <= c['sample_count'] <= 10000:
        raise ValueError('sample_count必须为1至10000的整数，表示每种模式采样次数')
    if type(c['seed']) is not int:
        raise ValueError('seed必须为整数')
    if not isinstance(c['experiment_id'], str) or not c['experiment_id'].strip():
        raise ValueError('experiment_id必须非空')


def generate_records(c, run_id):
    rows = []
    for enabled in (False, True):
        mode = 'MODEL_ENABLED' if enabled else 'BASELINE'
        rng = random.Random(c['seed'])  # 每种模式初始化一次，不逐样本重置。
        for i in range(c['sample_count']):
            result = dust_model.measure(raw_range_m=c['raw_range_m'],
                concentration_mg_m3=c['dust_concentration'], enabled=enabled, rng=rng)
            value = result.observed_range_m
            if type(result.valid) is not bool or not isinstance(result.state, str) or not result.state:
                raise ValueError('模型必须返回布尔valid与非空state')
            if result.valid:
                if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                    raise ValueError('有效观测必须为有限正距离')
            elif value is not None:
                raise ValueError('无效观测必须返回None；不能伪装成0误差')
            if not enabled and (not result.valid or value != c['raw_range_m']):
                raise ValueError('关闭扰动时模型必须直通原始距离')
            obs = f'{mode}_{i+1:06d}'
            rows.append(dict(stamp=f"{i*c['sample_interval_s']:.9f}", frame_id='mock_laser',
                experiment_id=c['experiment_id'], run_id=run_id, scene_id='fixed_distance_mock',
                scene_version='1.0', observation_id=obs, decision_id='decision_'+obs, beam_index=0,
                truth_range_m=c['truth_range_m'], raw_range_m=c['raw_range_m'], observed_range_m=value,
                delta_range_m=value-c['raw_range_m'] if result.valid else None,
                beam_state=result.state, observed_valid=result.valid,
                error_m=value-c['truth_range_m'] if result.valid else None,
                dust_concentration_mg_m3=c['dust_concentration'], mode=mode, sample_index=i+1,
                data_source=dust_model.DATA_SOURCE, model_name=dust_model.MODEL_NAME,
                params_version=dust_model.MODEL_VERSION, seed=c['seed']))
    return rows


def excel(rows, metadata, target):
    wb = Workbook()
    ws = wb.active
    ws.title = '结果'
    keys = ['mode','sample_index','stamp','truth_range_m','raw_range_m','observed_range_m',
            'error_m','dust_concentration_mg_m3','beam_state','observed_valid','data_source']
    ws.append(['模式','样本序号','模拟时间(s)','真值(m)','原始距离(m)','观测距离(m)',
               '真值误差(m)','浓度(mg/m³)','状态','有效','数据来源'])
    for r in rows:
        ws.append([float(r[k]) if k == 'stamp' else r[k] for k in keys])
    for cells in ws.iter_rows(min_row=2, min_col=3, max_col=8):
        for cell in cells:
            cell.number_format = '0.0000'
    info = wb.create_sheet('参数与说明')
    info.append(['字段','值'])
    for section, data in metadata.items():
        if isinstance(data, dict):
            for key, value in data.items():
                info.append([f'{section}.{key}', value if isinstance(value,(str,int,float,bool)) else json.dumps(value,ensure_ascii=False)])
        else:
            info.append([section, data])
    info.append(['时间说明','stamp为合成仿真秒，每种模式从0开始，不实际等待；run_id使用北京时间，精确到分钟；重名追加编号'])
    info.append(['有效性说明','无效观测和误差在Excel留空，CSV写NaN，不作为0误差'])
    info.append(['状态说明','当前BYPASS / MOCK_FIXED_BIAS为本地测试标签，不是正式ROS状态枚举'])
    info.append(['刷新说明','修改输入或模型后重新运行；Excel为静态结果快照'])
    for sheet in wb:
        sheet.freeze_panes='A2'
        sheet.sheet_view.showGridLines=False
        sheet.auto_filter.ref=sheet.dimensions
        for cell in sheet[1]:
            cell.fill=PatternFill('solid',fgColor='173F53')
            cell.font=Font(color='FFFFFF',bold=True)
        for row in sheet:
            for cell in row:
                cell.alignment=Alignment(vertical='center',wrap_text=True)
        for i in range(1,sheet.max_column+1):
            sheet.column_dimensions[get_column_letter(i)].width=22
        sheet.row_dimensions[1].height=32
    info.column_dimensions['A'].width=40
    info.column_dimensions['B'].width=100
    for i in range(2, info.max_row+1):
        info.row_dimensions[i].height=32
    wb.save(target)
    check=load_workbook(target,read_only=True,data_only=True)
    assert check['结果'].max_row == len(rows)+1
    for actual, expected in zip(check['结果'].iter_rows(min_row=2,values_only=True), rows):
        if expected['observed_valid']:
            assert math.isclose(actual[5],expected['observed_range_m'],rel_tol=0,abs_tol=1e-12)
        else:
            assert actual[5] is None and actual[6] is None
    check.close()


def main():
    c=json.loads((ROOT/'fixed_case.json').read_text(encoding='utf-8'))
    validate(c)
    # 北京时间精确到分钟；同一分钟重复运行加编号，保留全部结果。
    base_id=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%dT%H%M')
    suffix=1
    while True:
        run_id=base_id if suffix == 1 else f'{base_id}_{suffix:02d}'
        out=ROOT/'results'/run_id
        try:
            out.mkdir(parents=True,exist_ok=False)
            break
        except FileExistsError:
            suffix += 1
    rows=generate_records(c,run_id)
    metadata={'config':c,'model':{'name':dust_model.MODEL_NAME,'version':dust_model.MODEL_VERSION,
              'data_source':dust_model.DATA_SOURCE,**dust_model.model_parameters()},'run_id':run_id}
    for name in ['dust_model.py','run_fixed_distance.py']:
        content=(ROOT/name).read_bytes()
        (out/name).write_bytes(content)
        metadata[name+'_sha256']=hashlib.sha256(content).hexdigest()
    (out/'config_snapshot.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    with (out/'lidar_beam_detail.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows({k:('NaN' if v is None else v) for k,v in r.items()} for r in rows)
    excel(rows,metadata,out/'固定距离实验.xlsx')
    print(f'PASS：每种模式{c["sample_count"]}条，共{len(rows)}条；CSV、Excel及回读检查完成。')
    print(f'模型：{dust_model.MODEL_NAME} / {dust_model.DATA_SOURCE}')
    print(dust_model.MODEL_NOTE)
    print(f'结果目录：{out}')


if __name__=='__main__':
    main()
