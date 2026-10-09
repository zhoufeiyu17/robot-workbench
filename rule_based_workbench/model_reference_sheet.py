"""Compose labelled engineering reference sheets from actual Blender renders."""
from pathlib import Path
import hashlib
import json
import shutil
from PIL import Image,ImageDraw,ImageFont


def build(evidence,assets):
    evidence,assets=Path(evidence),Path(assets)
    out=assets/'references/ac_one_baseline_v1';out.mkdir(parents=True,exist_ok=True)
    def font(size):return ImageFont.truetype('/System/Library/Fonts/STHeiti Light.ttc',size)
    def sheet(name,title,panels,footer):
        im=Image.new('RGB',(1800,1010),'#f2f5f9');d=ImageDraw.Draw(im)
        d.text((45,22),title,font=font(38),fill='#18283b')
        for i,(label,path) in enumerate(panels):
            x=30+900*i;d.text((x+10,91),label,font=font(27),fill='#253d54')
            pic=Image.open(path).convert('RGB');pic.thumbnail((865,740))
            im.paste(pic,(x+(865-pic.width)//2,145+(740-pic.height)//2))
        d.text((45,923),footer,font=font(24),fill='#364d63');im.save(out/name)
    sheet('baseline-A1.png','AC-One 模型参考｜相同示意姿态，最大金属净距 83 mm',[
        ('Baseline · 保留手柄',evidence/'ac-one-reference-baseline/overview.png'),
        ('A1 结构版 · 去手柄，待重新标定',evidence/'ac-one-reference-A1/overview.png')],
        'AC-One 原生连杆与关节参数；1.5 mm × 2 胶垫；胶面净距 80 mm。无旧 X5 标定参数。')
    sheet('sticker-placement.png','标定贴纸推荐安装位置｜只用于演示，正式模型不含贴纸',[
        ('左腕：ID 24，固定在刚性相机壳顶面',evidence/'ac-one-reference-A1/sticker_left.png'),
        ('右腕：ID 30，固定在刚性相机壳顶面',evidence/'ac-one-reference-A1/sticker_right.png')],
        'DICT_4X4_50：黑框 15 × 15 mm，纸 20 × 20 mm。图中数字仅标位置，不是可打印的 ArUco 图案。')
    shutil.copy2(evidence/'ac-one-reference-baseline/mast.png',out/'mast-preserved-seat.png')
    for kind in ['baseline','A1']:
        for name in ['overview.png','render-info.json']:
            shutil.copy2(evidence/f'ac-one-reference-{kind}'/name,out/f'{kind}-{name}')
    baseline='ac_one_baseline_v1_r2' if (assets/'ac_one_baseline_v1_r2/SHA256SUMS.json').exists() else 'ac_one_baseline_v1'
    a1='ac_one_A1_pending_calibration_r2' if baseline.endswith('_r2') else 'ac_one_A1_pending_calibration'
    catalog=dict(active_baseline=baseline,a1_geometry=a1,
        a1_calibration_status='not_started_on_AC_One',
        baseline_seal_sha256=hashlib.sha256((assets/baseline/'SHA256SUMS.json').read_bytes()).hexdigest(),
        a1_geometry_seal_sha256=hashlib.sha256((assets/a1/'SHA256SUMS.json').read_bytes()).hexdigest(),
        superseded_candidates=['ac_one_x5_baseline_v1','ac_one_x5_A1']+(['ac_one_baseline_v1','ac_one_A1_pending_calibration'] if baseline.endswith('_r2') else []),
        superseded_reason='User chose native AC-One links and new calibration; do not use X5 offsets',
        handoff='../HANDOFF.md',references='references/ac_one_baseline_v1',deployment='none',
        hardware_access='calibration task owns window')
    (assets/'catalog.json').write_text(json.dumps(catalog,indent=2)+'\n')
    return out


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--evidence',required=True);p.add_argument('--assets',required=True)
    a=p.parse_args();print(build(a.evidence,a.assets))
