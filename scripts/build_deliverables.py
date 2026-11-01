"""Build the review PDF/video from real recorded MCP images and physics frames."""
import argparse
import json
import os
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
FONT = Path(os.environ.get('PAI_REPORT_FONT', ROOT/'assets/fonts/NotoSansJP-Regular.ttf'))
FONT_BOLD = Path(os.environ.get('PAI_REPORT_FONT_BOLD', ROOT/'assets/fonts/NotoSansJP-Bold.ttf'))
NAVY = '#112438'
TEAL = '#087e8b'


def pdf_report(data):
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import ParagraphStyle
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Image as RLImage
    from reportlab.graphics.shapes import Drawing, Rect, String, Line
    pdfmetrics.registerFont(TTFont('JP', str(FONT)))
    pdfmetrics.registerFontFamily('JP', normal='JP', bold='JP', italic='JP', boldItalic='JP')
    output = ROOT/'output/pdf/explanation_PAI_final_review.pdf'
    output.parent.mkdir(parents=True, exist_ok=True)
    doc = SimpleDocTemplate(str(output), pagesize=A4, rightMargin=40, leftMargin=40,
                            topMargin=40, bottomMargin=38, title='視覚と言語で選ぶSO-101作業システム',
                            author='', creator='Physical AI project')
    styles = {
        'title': ParagraphStyle('title', fontName='JP', fontSize=22, leading=29, textColor=colors.HexColor(NAVY), spaceAfter=12, wordWrap='CJK'),
        'h': ParagraphStyle('h', fontName='JP', fontSize=13, leading=19, textColor=colors.HexColor(TEAL), spaceBefore=12, spaceAfter=6, wordWrap='CJK'),
        'p': ParagraphStyle('p', fontName='JP', fontSize=9.2, leading=14.5, textColor=colors.HexColor(NAVY), spaceAfter=7, wordWrap='CJK'),
        'small': ParagraphStyle('small', fontName='JP', fontSize=8, leading=11.5, textColor=colors.HexColor('#42566a'), spaceAfter=5, wordWrap='CJK')}
    def p(text, style='p'): return Paragraph(text, styles[style])
    def table(rows, widths):
        converted = [[p(str(c), 'small') for c in row] for row in rows]
        t = Table(converted, colWidths=widths, hAlign='LEFT')
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dceff0')),
                              ('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f2f5f8')]),
                              ('VALIGN',(0,0),(-1,-1),'TOP'),('TOPPADDING',(0,0),(-1,-1),7),
                              ('BOTTOMPADDING',(0,0),(-1,-1),5),('LINEBELOW',(0,0),(-1,0),.6,colors.HexColor(TEAL))]))
        return t
    def photo(path, width=245): return RLImage(str(ROOT/path), width=width, height=width*.75)
    continuous = json.loads((ROOT/'artifacts/continuous_experiments.json').read_text())
    story = [p('視覚と言語で選ぶ<br/>SO-101作業システム', 'title'),
             p('Physical AI 最終課題 | 公開前レビュー版 | 2026-11-01', 'small'),
             p('基盤モデルが「赤いブロックを箱に入れて」という指示と画像から対象を選ぶ。固定位置版に加え、画像の上面中心からXYを推定し、その位置用の関節軌道と実行Pythonを生成する版を実装。MCPで把持・搬送し、再撮影画像で結果を確認する。'),
             table([['シミュレーション','実機接続','実機動作'],['固定位置4/4、連続XY 4/4','未検証：USB未接続','未検証：動作未開始']], [172,172,171]),
             p('AIとローカル制御の役割', 'h')]
    diagram=Drawing(515,95)
    boxes=[(0,40,150,'画像 + 日本語指示'),(178,40,150,'ChatGPT / Codex'),(356,40,159,'MCP共通API')]
    for x,y,w,label in boxes:
        diagram.add(Rect(x,y,w,38,rx=6,ry=6,fillColor=colors.HexColor('#e7f3f4'),strokeColor=colors.HexColor(TEAL)))
        diagram.add(String(x+w/2,y+15,label,fontName='JP',fontSize=10,textAnchor='middle',fillColor=colors.HexColor(NAVY)))
    for x in [155,333]:
        diagram.add(Line(x,59,x+18,59,strokeColor=colors.HexColor(TEAL)))
        diagram.add(String(x+10,61,'>',fontName='JP',fontSize=11))
    diagram.add(String(255,15,'ローカルで補間・制限・停止 → MuJoCo / LeRobot',fontName='JP',fontSize=10,textAnchor='middle',fillColor=colors.HexColor(NAVY)))
    story += [diagram, p('AIは色・上面中心の選択と結果確認を担当する。ローカル処理が校正とIKで動作を生成し、制御ループが実行する。高さは既知の30mm。通常入力をRGB・関節・開度に揃え、深度を無効にした再試行も成功した（5頁）。サーバー内の別LLM呼出し、新規学習、ROSは使用しない。'),
              photo(data['trials'][0]['after_image'], 450),
              p('実取得画像：赤が箱内、青がslot_aに残る。指は退避。図の物体IDや正解座標はAIへ渡していない。', 'small'), PageBreak()]
    story += [p('画像と指示を変えて検証', 'title'),
              p('2つの配置と2色の指示を組み合わせた4条件。MCPのPNGをこのCodexが表示して選び、動作後も別のPNGで確認した。下表の独立評価は、その判断記録を残した後に物理状態から照合した。')]
    rows=[['試行','指示 / 選択','指令','画像 / 物理評価','軌道 / 実時間']]
    for i,t in enumerate(data['trials'],1):
        rows.append([f'{i:02}',('赤' if t['target']=='red' else '青')+' / '+t['selected_slot'], '完了', '成功 / 成功', f'22.0 / {t["wall_s"]:.3f} s'])
    story += [table(rows, [38,120,56,154,147]),
              p('同じ赤指定でも、置き場を交換するとslot_b → slot_aへ選択が変わる。', 'h')]
    for i in [0,1]:
        t=data['trials'][i]
        story.append(Table([[photo(t['before_image']),photo(t['after_image'])],
                            [p(f'試行{i+1:02}：操作前 / 選択 {t["selected_slot"]}', 'small'),p('操作後：赤が箱内、青は元の置き場', 'small')]],
                           colWidths=[257.5,257.5],style=[('VALIGN',(0,0),(-1,-1),'TOP')]))
    story += [p('物理接触の証拠：対象の中心高さは15mmから約65mmまで上昇。指との接触を4740〜4777物理ステップ検出し、終端では接触なし。全水平外形が箱内に収まり、静止していることを確認した。', 'small'),
              p('固定配置での成立確認であり、一般的な成功率ではない。AIの会話待ち時間は実時間欄に含めない。動作映像はシミュレーション時間1倍速。', 'small'), PageBreak()]
    story += [p('位置推定とスクリプト生成', 'title'),
              p('固定マーカーを消し、2つの連続座標配置×2色を検証。AIが960×720のPNGを見て上面中心(u,v)を指定し、校正済みカメラの視線とZ=30mm平面の交点からXYを推定した。'),
              table([['試行 / 色','選択ピクセル','推定XY [mm]','独立XY誤差']]+
                    [[f'C{i+1:02} / '+('赤' if t['target']=='red' else '青'),str([int(v) for v in t['pixel_xy']]),
                      ', '.join(f'{v*1000:.2f}' for v in t['estimated_xy_m']),f'{t["independent_xy_error_mm"]:.3f} mm']
                     for i,t in enumerate(continuous['trials'])], [90,110,205,110]),
              p('4試行すべて：生成Pythonを実行 → 指令完了 → AI画像判定成功 → 独立物理評価成功。各軌道22秒、計算実時間8.55〜8.83秒。', 'small'),
              Table([[photo(continuous['trials'][0]['preview_image']),photo(continuous['trials'][0]['after_image'])],
                     [p('AIの選択点と推定XYを重ねた確認画像','small'),p('生成スクリプト実行後：赤が箱内','small')]],colWidths=[257.5,257.5]),
              p('何を生成するか', 'h'),
              p('prepare_pickが推定XY・IKによる7段階の関節軌道JSONと、MCPのprepared_pickを呼ぶPythonを保存する。AIが対象ピクセルと根拠を選び、ローカルの決定的な雛形がコードを出力する。自由なPython本文をLLMが毎回合成する方式ではない。生成ファイルそのものを実行し、SHA-256と結果ログを保存した。'),
              p('この4試行は既知高さを使用', 'h'),
              p('既知条件：直立した30mmブロック、作業台Z=0、既知のカメラ校正と固定の箱。把持高さ19mm、持上げ70mmは設定値。未知の高さ・傾き・遮蔽は未対応。±3ピクセルは許容誤差の仮定であり、測定した認識信頼度ではない。'),
              p('誤差はAI判定後の独立評価。初期シーンを再現し、観測PNGとの完全一致を確認してから正解XYと比較した。理想カメラの4例であり、実機精度を示さない。実機の連続XY操作は未校正で未対応。', 'small'), PageBreak()]
    story += [p('共通化・工夫・限界', 'title'),
              table([['共通操作','返す情報 / ローカル動作'],
                     ['observe','通常はRGB、5関節、別項目の開度、利用可能なカメラ校正、直前結果'],
                     ['locate_on_plane','RGBで選んだ画素を既知高さの平面へ投影。高さは測定していないと明示'],
                     ['prepare_pick / prepare_step','全軌道または1回の関節相対動作のPythonを生成。生成時は動作なし'],
                     ['execute_skill','置き場または生成program_idを指定。action_idを返し、ローカルで実行'],
                     ['get_status / stop','完了・失敗・停止を確認。停止はラッチし、ローカルSTOPファイルでも操作可能'],
                     ['report_result','AIが新しい画像から判断した成功・失敗・不明と根拠を記録']], [116,399]),
              p('独自の工夫', 'h'),
              p('① 指令完了、AIの視覚判定、独立した物理評価を別々に記録。② 同じAPIでも校正・経由点はsim/realで分離。③ 古い画像からの実行を拒否し、request_idで二重実行を防止。④ 実機は読み取り・ドライランから開始し、ローカル端末のARMでのみ動作を許可する。'),
              p('関節表現と停止', 'h'),
              p('関節順：shoulder_pan、shoulder_lift、elbow_flex、wrist_flex、wrist_roll。共通単位は度。MuJoCo内部はラジアン、LeRobotはuse_degrees=True。グリッパーは共通0〜1を実機0〜100へ変換し、角度と分離する。正方向・ゼロ点の現物照合は未検証のため、実機のverifiedフラグはfalseを保持する。'),
              p('simは50Hz制御 / 500Hz物理、realは20Hz制御・初期速度10度/秒。補間、可動域、時間、追従誤差をローカルで検査。実機停止は現在位置保持で、通信断時の保持を保証しない。ソフトウェア停止は非常停止回路の代替ではない。'),
              p('検証済み / 未検証', 'h'),
              p('自動テスト56件成功：関節変換、範囲・速度、停止、実機ドライランの書込み0、接触搬送、stdio画像返却、RGB校正・歪み補正・既知平面投影、古い観測の拒否など。実機は設定ポートなしで接続前に停止。実カメラ校正・関節対応・把持・停止応答は未検証。'),
              p('固定位置・限定範囲の連続XYに対応。段階生成の方向修正は6頁。未知高さの把持、照明変動、把持失敗からの再試行、一般の衝突回避、実機連続XYは未対応。Astraを第一候補とするが、今回のホストモデルIDは未記録で、Astra固有の性能は未検証。'),
              p('再現と出典', 'h'),
              p('Python 3.12.12 / MuJoCo 3.17.0 / MCP 1.30.0 / LeRobot 0.5.1。依存はuv.lock。READMEに起動、MCP接続、校正、停止、実機への接続手順を記載。コード、ログ、原映像、PDFは公開前のレビュー用に選別し、GitHub / YouTubeへの公開は未実行。', 'small'),
              p('<link href="https://huggingface.co/lerobot/so101-tabletop-mujoco" color="#087e8b">Hugging Face SO-101 tabletop</link>：844dee6c058960c2ef42b65e2c68b47a5cf7c4f3。MuJoCo Menagerie / TheRobotStudio由来、Apache-2.0。モデル・接触形状を利用し、2個の物体、カメラ、固定置き場へ変更。LeRobot：1396b9fab7aecddd10006c33c47a487ffdcb54b4。詳細はTHIRD_PARTY_NOTICES.md。', 'small'),
              p('<link href="https://huggingface.co/docs/lerobot/main/api/robots" color="#087e8b">LeRobot API</link> / <link href="https://huggingface.co/docs/lerobot/action_representations" color="#087e8b">Action Representations</link> / <link href="https://learn.chatgpt.com/docs/extend/mcp" color="#087e8b">Codex MCP</link> / <link href="https://developers.openai.com/api/docs/guides/custom-mcp-server" color="#087e8b">ChatGPTカスタムMCP</link>（2026-10-31確認）', 'small')]
    portable = json.loads((ROOT/'artifacts/rgb_portable_experiments.json').read_text())['trials'][0]
    story += [PageBreak(), p('実RGBで用意できる入力へ', 'title'),
              p('通常入力をRGB・関節・開度・利用可能なカメラ校正に揃えた。simの仮想深度と、実機では未校正の手先姿勢は無効。過去の仮想深度診断は実機共通構成の成果に含めない。未知の高さを測定したとは扱わない。'),
              Table([[photo(portable['preview_image']), photo(portable['after_image'])],
                     [p('RGBで赤の上面(495,428)を選択','small'),p('再撮影：赤が箱内、青が元の場所','small')]], colWidths=[257.5,257.5]),
              p('深度なしの再検証：1試行 / 1成功', 'h'),
              p('既知Z=30mmの平面と画像の視線からXY=(250.421,38.672)mmを推定。生成Pythonを実行し、赤を接触で搬送した。22.0秒軌道 / 計算実時間8.483秒。AIの再撮影判定後に独立評価を行い、箱内・静止・指との非接触を確認。接触は4747物理ステップ。'),
              table([['実機へ用意した処理','状態'],
                     ['RGB内部校正・歪み補正','OpenCVの実画像校正CLIを実装。現物未校正'],
                     ['外部校正・既知平面投影','測定した校正板座標からsolvePnP。共通locate_on_planeへ接続'],
                     ['実機の最小搬送','画像で固定置き場を選び、実機で教示した関節経由点を使用'],
                     ['実機の連続XY・未知高さ','未対応。カメラ校正だけでは実機IKは完成しない']], [182,333]),
              p('現物で残っている確認', 'h'),
              p('現在はUSBシリアル0台。設定では正面RGBが有効、手首は無効。カメラ校正値は未取得。校正板を使って内部・外部校正し、独立した点で位置誤差を照合する。校正ファイルはverified=falseから開始し、解像度違いを拒否。モーター動作には別の現物照合とローカルARMが必要。'),
              p('単眼の画素と校正だけでは未知の距離は一意に決まらない。高さ30mmは物体寸法による既知条件で、locate_on_planeはheight_measured=falseを返す。未知高さには複数視点や既知形状などの追加条件が必要だが、今回未実装。'),
              p('校正板PDF・撮影/校正/独立点照合CLIを同梱。詳細はdocs/physical_ai/RGB_REAL.md。再検証ログはartifacts/runs/rgb_portable_01。旧深度診断はSENSORS.mdに履歴として明示。', 'small')]
    incremental = json.loads((ROOT/'artifacts/incremental_experiments.json').read_text())
    steps = incremental['steps']
    story += [PageBreak(), p('画像を見て、次のPythonを生成', 'title'),
              p('1回分の小動作を生成・実行し、新しい画像を見てから次を選ぶ。prepare_stepはsim/real共通の関節相対API。今回のCodexは旋回を3回に分けて判断し、予想と逆の動きを画像から確認して修正した。'),
              Table([[photo(steps[0]['before_image']), photo(steps[0]['after_image'])],
                     [p('最初のRGB：赤は手先の右側','small'),p('段階1：+2度で指先が左へ。予想と逆','small')]], colWidths=[257.5,257.5]),
              Table([[photo(steps[1]['after_image']), photo(steps[2]['after_image'])],
                     [p('段階2：新画像から-2度を生成。右へ戻る','small'),p('段階3：新画像から-1度を生成。少し右へ','small')]], colWidths=[257.5,257.5]),
              table([['確認した処理','結果'],
                     ['1回ずつ生成・実行・再撮影','3指令完了。各0.6秒、計算実時間は合計0.749秒'],
                     ['予想と実際を比較して方向を修正','最初の方向予想に1回の誤り。次の生成で修正'],
                     ['搬送・高さ測定・実機動作','この試行では未実施。搬送成功数へ加算しない']], [182,333]),
              p('既定値：関節ごと2度以下、合計4度以下、開度差分0.1以下。腕と開度は別々に変更する。古い画像や先に生成した次動作を拒否。生成Pythonは1回で終了し、AIの新しい判断を待つ。'),
              p('深度や実機IKを入力に要求しないが、関節角の差分は手先のmm移動や接触を保証しない。未知高さの把持完遂は未検証。実機は標準ドライランで、現物の方向・範囲確認とローカルARMが必要。詳細：INCREMENTAL.md。', 'small')]
    def footer(canvas, doc):
        canvas.setFont('JP', 8)
        canvas.setFillColor(colors.HexColor('#65778a'))
        canvas.drawString(40,23,'Physical AI | simulation verified / real hardware unverified')
        canvas.drawRightString(A4[0]-40,23,str(doc.page))
    doc.build(story, onFirstPage=footer, onLaterPages=footer)
    return output


def video_report(data):
    output = ROOT/'submission/physical_ai_demo.mp4'
    output.parent.mkdir(parents=True, exist_ok=True)
    fonts={n:ImageFont.truetype(str(FONT), n) for n in [17,20,21,22,24,25,30,36,38,48]}
    def text(draw, xy, value, size=22, fill='white', max_width=None, line_gap=9):
        x,y=xy
        for line in value.split('\n'):
            buf=''
            for ch in line:
                if max_width and draw.textlength(buf+ch,font=fonts[size]) > max_width:
                    draw.text((x,y),buf,font=fonts[size],fill=fill); y+=size+line_gap;buf=ch
                else: buf+=ch
            draw.text((x,y),buf,font=fonts[size],fill=fill);y+=size+line_gap
        return y
    def slide(title, lines):
        im=Image.new('RGB',(1280,720),NAVY);d=ImageDraw.Draw(im)
        d.rectangle((65,70,75,610),fill='#49c9c4')
        text(d,(105,95),title,48,max_width=1080,line_gap=14)
        y=260
        for line in lines:
            y=text(d,(110,y),line,30,fill='#e0eaf0',max_width=1060,line_gap=12)+25
        text(d,(110,656),'Physical AI 最終課題 | 公開前レビュー版',20,fill='#8eafc2')
        return im
    def frame(im, trial, index, label, phase=None):
        out=Image.new('RGB',(1280,720),NAVY);out.paste(im.resize((960,720)),(0,0));d=ImageDraw.Draw(out)
        text(d,(982,36),f'試行 {index:02} / 04',30,fill='#64dcd4')
        color='赤' if trial['target']=='red' else '青'
        y=text(d,(982,104),f'指示\n「{color}いブロックを\n箱に入れて」',25,max_width=274,line_gap=12)
        if 'pixel_xy' in trial:
            u,v=trial['pixel_xy']; x,y_m=trial['estimated_xy_m']
            selected=f'画像 ({u:.0f}, {v:.0f}) px\nXY ({x*1000:.1f}, {y_m*1000:.1f}) mm\nZ=30 mmは既知値'
        else:
            selected=f'画像からの選択\n{trial["selected_slot"]}'
        y=text(d,(982,y+22),selected,20,max_width=274,line_gap=9)
        y=text(d,(982,y+35),label,22,fill='#64dcd4',max_width=274)
        if phase:
            text(d,(982,y+12),phase,22,max_width=274)
        text(d,(982,567),'字幕は実際のMCPログに\n基づく後編集。\n判断役：このCodex。',17,fill='#b2c8d8',max_width=275,line_gap=7)
        text(d,(982,671),'実機：未接続・未検証',17,fill='#f5c780')
        return out
    writer=imageio.get_writer(output,fps=25,codec='libx264',quality=8,macro_block_size=16,
                              ffmpeg_params=['-movflags','+faststart','-metadata','title=Physical AI SO-101 review'])
    def hold(im, seconds):
        pixels=np.asarray(im)
        for _ in range(round(seconds*25)): writer.append_data(pixels)
    try:
        hold(slide('場所を推定して動かすSO-101',[
            '基盤モデルが画像から対象を選び、MCPで把持・搬送。',
            '連続座標の2配置 × 2色の指示を検証。',
            'MuJoCoの実映像 / シミュレーション時間1倍速']),7)
        hold(slide('判断と制御を分離する',[
            'ChatGPT / Codex：画像 + 指示 → 上面中心 → 結果確認',
            'ローカル：カメラ校正 → XY → IK → 関節軌道',
            '同じAPIをMuJoCoとLeRobot実機へ。新規学習なし。']),7)
        hold(slide('その場の画像からPythonを生成',[
            'prepare_pickが軌道JSON・実行Python・確認画像を保存。',
            '生成PythonからMCPを呼び、4試行を実際に実行。',
            'AIがピクセルを選び、コードは決定的な雛形で生成。']),7)
        hold(slide('この4試行は既知高さを使用',[
            'XYは画像から推定。この4試行の高さは30mmと設定。',
            '直立ブロック・校正済みカメラ・固定の箱で検証。',
            '未知の高さ・姿勢、実機での連続位置操作は未対応。']),7)
        phase_names={'approach':'把持位置の上へ','descend':'下降','close':'接触して把持','lift':'持ち上げ',
                     'transfer':'箱へ搬送','release':'開いて箱へ置く','observe_pose':'観測姿勢へ戻る'}
        for index,trial in enumerate(data['trials'],1):
            hold(frame(Image.open(ROOT/trial.get('preview_image',trial['before_image'])),trial,index,'選択点を確認（静止）'),3)
            run=ROOT/Path(trial['video']).parent
            events=[json.loads(s) for s in (run/'events.jsonl').read_text().splitlines()]
            plan=next(e for e in events if e['event']=='action_started')['trajectory']
            ends=np.cumsum([s['duration_s'] for s in plan])
            reader=imageio.get_reader(ROOT/trial['video'])
            try:
                for i,pixels in enumerate(reader):
                    phase=plan[min(int(np.searchsorted(ends,i/25)),len(plan)-1)]['phase']
                    writer.append_data(np.asarray(frame(Image.fromarray(pixels),trial,index,
                        '実行映像 / 1倍速', phase_names[phase])))
            finally: reader.close()
            hold(frame(Image.open(ROOT/trial['after_image']),trial,index,
                       'AI再撮影判定：成功','対象は箱内\n他の色は元の置き場'),3)
        hold(slide('連続XY指定の4試行で4成功',[
            '独立XY誤差0.34〜0.77mm：理想カメラ・既知高さの4例。',
            '実機：USB未接続のため、接続・搬送とも未検証。',
            'モデル資産：Hugging Face / MuJoCo Menagerie (Apache-2.0)']),10)
        portable = json.loads((ROOT/'artifacts/rgb_portable_experiments.json').read_text())['trials'][0]
        pane = Image.new('RGB', (1280,720), NAVY)
        d = ImageDraw.Draw(pane)
        text(d,(55,32),'追加検証：RGBで赤を選んで搬送',38)
        pane.paste(Image.open(ROOT/portable['preview_image']).resize((576,432)),(45,110))
        pane.paste(Image.open(ROOT/portable['after_image']).resize((576,432)),(659,110))
        text(d,(60,555),'操作前：赤の上面を選択',25)
        text(d,(674,555),'操作後：赤が箱内、青は元の場所',25)
        text(d,(60,615),'深度とsim専用の手先姿勢は無効。追加1試行も成功。',25,fill='#64dcd4')
        text(d,(60,660),'高さ30mmは既知条件。未知の高さを測った結果ではない。',20)
        hold(pane,10)
        incremental = json.loads((ROOT/'artifacts/incremental_experiments.json').read_text())
        steps = incremental['steps']
        pane = Image.new('RGB', (1280,720), NAVY)
        d = ImageDraw.Draw(pane)
        text(d,(55,32),'段階生成：画像を見て次の動きを修正',36)
        pane.paste(Image.open(ROOT/steps[0]['after_image']).resize((576,432)),(45,110))
        pane.paste(Image.open(ROOT/steps[2]['after_image']).resize((576,432)),(659,110))
        text(d,(60,555),'最初：予想と逆に左へ動いた',24)
        text(d,(674,555),'新画像から負方向へ修正し、右へ',24)
        text(d,(60,610),'1回分のPython → 実行 → 再撮影 → 次を生成。3段階を確認。',24,fill='#64dcd4')
        text(d,(60,655),'方向予想の誤り1回も記録。把持・搬送・高さ測定は未実施。',21)
        hold(pane,8)
        hold(slide('実機へ残る作業を明確にする',[
            '実RGBの校正・歪み補正・既知平面への投影を実装。',
            '実機は未接続。関節方向・範囲・視点と搬送の現物試験が必要。',
            '未知高さ・実機の連続XYは未対応。自動テスト56件成功。']),8)
    finally: writer.close()
    return output


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--pdf-only',action='store_true')
    parser.add_argument('--video-only',action='store_true')
    args=parser.parse_args()
    if not FONT.is_file(): raise SystemExit('Set PAI_REPORT_FONT to a Japanese TrueType font')
    data=json.loads((ROOT/'artifacts/experiments.json').read_text())
    if not args.video_only: print(pdf_report(data).relative_to(ROOT),flush=True)
    if not args.pdf_only:
        continuous=json.loads((ROOT/'artifacts/continuous_experiments.json').read_text())
        print(video_report(continuous).relative_to(ROOT),flush=True)


if __name__=='__main__': main()
