"""Render a review supplement from the 16 completed, independently scored cases."""
import argparse
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from build_deliverables import FONT, NAVY, ROOT, TEAL


LABELS = {
    'suite_09': '左右の向きを確認して2段階を生成',
    'suite_10': '1段階後に赤が隠れ、続行を中止',
    'suite_11': '斜め画像から2段階の方向を確認',
    'suite_12': '低解像度で指を退避し、見え方を改善',
    'suite_13': '緑のブロック不在：動作を選ばない',
    'suite_14': '範囲外の選択：生成を拒否、移動なし',
    'suite_15': '小動作後の古い画像：次の生成を拒否',
    'suite_16': 'ローカルSTOPで途中停止・再指令を拒否',
}


def pdf_report(data):
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import ParagraphStyle
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Image as RLImage

    pdfmetrics.registerFont(TTFont('SuiteJP', str(FONT)))
    pdfmetrics.registerFontFamily('SuiteJP', normal='SuiteJP', bold='SuiteJP')
    output = ROOT/'output/pdf/scenario_suite_review.pdf'
    output.parent.mkdir(parents=True, exist_ok=True)
    styles = {
        'title': ParagraphStyle('title', fontName='SuiteJP', fontSize=21, leading=29, textColor=colors.HexColor(NAVY), spaceAfter=9, wordWrap='CJK'),
        'h': ParagraphStyle('h', fontName='SuiteJP', fontSize=12, leading=18, textColor=colors.HexColor(TEAL), spaceBefore=9, spaceAfter=5, wordWrap='CJK'),
        'p': ParagraphStyle('p', fontName='SuiteJP', fontSize=9, leading=14, textColor=colors.HexColor(NAVY), spaceAfter=7, wordWrap='CJK'),
        'small': ParagraphStyle('small', fontName='SuiteJP', fontSize=7.8, leading=11.5, textColor=colors.HexColor('#42566a'), spaceAfter=4, wordWrap='CJK'),
    }
    def p(s, style='p'): return Paragraph(str(s), styles[style])
    def table(rows, widths):
        t = Table([[p(c, 'small') for c in row] for row in rows], colWidths=widths, hAlign='LEFT')
        t.setStyle(TableStyle([
            ('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dceff0')),
            ('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f2f5f8')]),
            ('VALIGN',(0,0),(-1,-1),'TOP'),('TOPPADDING',(0,0),(-1,-1),4),
            ('BOTTOMPADDING',(0,0),(-1,-1),3),('LINEBELOW',(0,0),(-1,0),.6,colors.HexColor(TEAL)),
        ]))
        return t
    def photos(row, before='before_image', captions=None):
        images = [RLImage(str(ROOT/row[k]),width=247,height=185.25) for k in (before,'after_image')]
        return Table([images, [p(s,'small') for s in captions]], colWidths=[257.5,257.5],
                     style=[('VALIGN',(0,0),(-1,-1),'TOP')])
    cases=data['cases']
    rows=[['試行 / 指示','観測画像 / 初期旋回','小動作回数','軌道 / 計算時間','結果']]
    for r in cases[:8]:
        c=r['condition']
        rows.append([r['case_id'][-2:]+' / '+('赤' if r['target']=='red' else '青'),
                     ('俯瞰' if c['camera']=='top' else '斜め')+f" {c['width']}×{c['height']} / {c['initial_pan_deg']:+}°",
                     r['incremental_steps'],f"{r['trajectory_s']:.1f} / {r['wall_s']:.3f} s",'画像・物理 成功'])
    story=[p('追加16ケースのシミュレーション','title'),
           p('Physical AI 最終課題 | 公開前レビュー用の補足 | 2026-11-01','small'),
           p('配置4組×赤・青の指示、俯瞰/斜めカメラ、2解像度、初期旋回を変更。ホストのCodexがMCPのRGBを見て選択し、実際の生成Pythonを実行。深度とsim専用手先姿勢は無効。'),
           table([['把持・搬送','段階生成の方向・視野確認','拒否・停止'],
                  ['8 / 8 成功','3確認 / 1遮蔽で中止','4 / 4 想定どおり']], [172,172,171]),
           p('搬送8ケース：独立した物理評価と一致','h'),table(rows,[58,171,63,127,96]),
           Spacer(1,8),photos(cases[2],captions=['03：初期画像では赤の一部が指に隠れる','03：3回の小動作で視野を調整後、赤を箱へ']),
           p('03・04は+2度を3回、07は−2度を1回。各小動作の後に新しいRGBを見て次を選択した。その後の把持・搬送は既知高さを使って生成した一括軌道。段階生成だけで全搬送した結果ではない。','small'),
           p('全試行の判断完了後に正解状態を照合。対象は接触で持ち上がり、全水平外形が箱内、静止、指との接触なし。他色は箱外。接触4718〜4954物理ステップ。内部の物体ID・座標をAIの対象選択には使わず、動作中の物体座標書換えも行っていない。','small'),
           p('既知条件：直立30mmブロック、校正済み理想カメラ、固定の箱。高さは測定していない。計算時間はAI応答待ちを含めない。過去9試行を合わせた搬送は17/17だが、限定条件の成立確認であり一般的な成功率ではない。','small'),PageBreak(),
           p('見えなくなったら続けない','title'),
           p('小動作の生成と、想定外の入力・途中停止を別枠で試験。方向・視野確認の4ケースでは下降・把持・搬送を行っておらず、搬送成功数には加算しない。')]
    rows=[['試行','試した条件 / 実際の結果','動作 / 軌道時間']]
    for r in cases[8:]:
        rows.append([r['case_id'][-2:],LABELS[r['case_id']],f"{r['commands']}回 / {r['trajectory_s']:.1f} s"])
    story += [table(rows,[43,350,122]),p('10：画像を見直すと、赤が指の後ろに隠れた','h'),
              photos(cases[9],captions=['動作前：赤の一部が右側に見える','−2度の後：赤がほぼ隠れたため中止']),
              p('単眼の画面上で近づけても、奥行き・接触は確定しない。視野が悪くなる場合もある。今回は自動復帰や未知高さの下降へ進まず、遮蔽を失敗・限界として記録した。','small'),
              p('16のローカルSTOPは、作成から停止状態の読出しまで約18ms（1回のsim測定、MCP照会込み）。軌道0.3秒で停止し、後続指令も拒否。実機の停止応答を示す値ではない。','small'),
              p('未検証：未知高さ、物体寸法・摩擦・照明・センサーノイズの変化、段階生成だけでの全搬送、実機接続・実機動作。AI思考中はsimの物理時間が停止するため、把持中に長く待つ影響も未検証。','small'),
              p('再現設定：config/physical_ai/suite/。各PNG・生成Python・MCP応答・原動画：artifacts/runs/suite_*。独立集計：artifacts/scenario_experiments.json / .csv。詳細：docs/physical_ai/SCENARIO_RESULTS.md。モデル資産・ライセンスはTHIRD_PARTY_NOTICES.md。','small')]
    doc=SimpleDocTemplate(str(output),pagesize=A4,leftMargin=40,rightMargin=40,topMargin=35,bottomMargin=34,
                          title='SO-101 追加16ケースのシミュレーション',author='',creator='Physical AI project')
    def footer(canvas, doc):
        canvas.setFont('SuiteJP',7.5);canvas.setFillColor(colors.HexColor('#65778a'))
        canvas.drawString(40,20,'Simulation evidence | unknown height / real hardware unverified')
        canvas.drawRightString(A4[0]-40,20,str(doc.page))
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
    return output


def video_report(data):
    output=ROOT/'submission/scenario_suite_review.mp4'
    output.parent.mkdir(parents=True,exist_ok=True)
    fonts={}
    def text(d, xy, value, size=24, fill='white', max_width=1150):
        font=fonts.setdefault(size,ImageFont.truetype(str(FONT),size))
        x,y=xy
        for line in value.split('\n'):
            buf=''
            for ch in line:
                if d.textlength(buf+ch,font=font)>max_width:
                    d.text((x,y),buf,font=font,fill=fill);y+=size+10;buf=ch
                else:buf+=ch
            d.text((x,y),buf,font=font,fill=fill);y+=size+10
        return y
    def slide(title, lines):
        im=Image.new('RGB',(1280,720),NAVY);d=ImageDraw.Draw(im)
        d.rectangle((60,70,70,620),fill='#49c9c4')
        text(d,(100,90),title,42,max_width=1090)
        y=235
        for line in lines:y=text(d,(105,y),line,29,max_width=1070)+25
        text(d,(105,670),'MuJoCo実映像 / 判断役はこのCodex / 実機は未検証',19,fill='#aac1d1')
        return im
    def comparison(row, caption):
        im=Image.new('RGB',(1280,720),NAVY);d=ImageDraw.Draw(im)
        text(d,(30,28),f"{row['case_id'][-2:]}  {caption}",30,max_width=1220)
        text(d,(30,102),row['instruction'],23,fill='#64dcd4')
        for x,k,label in [(30,'before_image','操作前'),(650,'after_image','操作後 / 最終観測')]:
            text(d,(x,146),label,20)
            photo=Image.open(ROOT/row[k]).convert('RGB').resize((600,450))
            im.paste(photo,(x,180))
        text(d,(30,650),f"指令 {row['commands']}回 / 物理軌道 {row['trajectory_s']:.1f}秒。比較用の実取得PNGを静止表示。",20)
        text(d,(30,686),'字幕はMCPログに基づく後編集。高さ測定なし。',16,fill='#aac1d1')
        return im
    def motion_frame(pixels,row):
        im=Image.new('RGB',(1280,720),NAVY)
        im.paste(Image.fromarray(pixels).resize((960,720)),(0,0));d=ImageDraw.Draw(im)
        text(d,(982,38),f"試行 {row['case_id'][-2:]}",30,fill='#64dcd4',max_width=275)
        y=text(d,(982,112),row['instruction'],25,max_width=270)
        lines=['物理接触で把持・搬送','対象はRGBから選択','高さ30mmは既知値','画像・物理とも成功']
        if row['incremental_steps']:lines.insert(0,f"視野調整 {row['incremental_steps']}回 → 全軌道")
        for line in lines:y=text(d,(982,y+25),line,21,max_width=270)
        text(d,(982,580),'動作はsim時間1倍速。\nAIの判断待ちは含まない。\n字幕は後編集。',18,max_width=270)
        text(d,(982,683),'実機動作は未検証',17,fill='#f5c780',max_width=270)
        return im
    writer=imageio.get_writer(output,fps=25,codec='libx264',quality=8,macro_block_size=16,
                              ffmpeg_params=['-movflags','+faststart','-metadata','title=SO-101 scenario review'])
    def hold(im,seconds):
        pixels=np.asarray(im)
        for _ in range(round(seconds*25)):writer.append_data(pixels)
    try:
        hold(slide('条件を変えて16ケースを実行',[
            '搬送8/8。配置・色・視点・解像度・初期旋回を変更。',
            '段階生成：3ケース確認、1ケースは遮蔽で中止。',
            '対象不在・範囲外・古い画像・途中停止も確認。']),6)
        for r in data['cases'][:8]:
            c=r['condition'];camera='俯瞰' if c['camera']=='top' else '斜め'
            hold(comparison(r,f"{camera} {c['width']}×{c['height']} / 画像・物理とも搬送成功"),3)
        for r in (data['cases'][2],data['cases'][4]):
            reader=imageio.get_reader(ROOT/r['video'])
            if abs(reader.get_meta_data()['fps']-25)>.01:raise ValueError('Source must be 25 fps for 1x playback')
            try:
                for pixels in reader:writer.append_data(np.asarray(motion_frame(pixels,r)))
            finally:reader.close()
        for r in data['cases'][8:]:hold(comparison(r,LABELS[r['case_id']]),4)
        hold(slide('確認できたこと・残ったこと',[
            '8搬送は実接触と独立評価で確認。小動作とは別集計。',
            '未知高さ・段階生成だけの全搬送・実機は未検証。',
            '把持中のAI待ち時間も未検証（思考中はsim停止）。']),8)
    finally:writer.close()
    return output


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--pdf-only',action='store_true')
    args=parser.parse_args()
    data=json.loads((ROOT/'artifacts/scenario_experiments.json').read_text())
    print(pdf_report(data).relative_to(ROOT),flush=True)
    if not args.pdf_only:print(video_report(data).relative_to(ROOT),flush=True)


if __name__=='__main__':main()
