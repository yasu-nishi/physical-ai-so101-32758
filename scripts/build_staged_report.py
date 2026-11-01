"""Review PDF and <=3 minute video from actual, closed staged trials."""
import argparse
import json
import math
from pathlib import Path
import subprocess

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from build_deliverables import FONT, FONT_BOLD, NAVY, ROOT, TEAL


def pdf_report(data):
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import ParagraphStyle
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Image as RLImage
    pdfmetrics.registerFont(TTFont('StagedJP',str(FONT)))
    pdfmetrics.registerFont(TTFont('StagedJPBold',str(FONT_BOLD)))
    pdfmetrics.registerFontFamily('StagedJP',normal='StagedJP',bold='StagedJPBold')
    styles={
        'title':ParagraphStyle('title',fontName='StagedJPBold',fontSize=20,leading=28,textColor=colors.HexColor(NAVY),spaceAfter=10,wordWrap='CJK'),
        'h':ParagraphStyle('h',fontName='StagedJPBold',fontSize=12,leading=17,textColor=colors.HexColor(TEAL),spaceBefore=9,spaceAfter=5,wordWrap='CJK'),
        'p':ParagraphStyle('p',fontName='StagedJP',fontSize=9,leading=14,textColor=colors.HexColor(NAVY),spaceAfter=7,wordWrap='CJK'),
        'small':ParagraphStyle('small',fontName='StagedJP',fontSize=7.8,leading=11.5,textColor=colors.HexColor('#42566a'),spaceAfter=5,wordWrap='CJK')}
    def p(t,s='p'):return Paragraph(str(t),styles[s])
    def table(rows,widths):
        result=Table([[p(c,'small') for c in row] for row in rows],colWidths=widths,hAlign='LEFT')
        result.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dceff0')),
            ('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,colors.HexColor('#f2f5f8')]),
            ('VALIGN',(0,0),(-1,-1),'TOP'),('TOPPADDING',(0,0),(-1,-1),4),('BOTTOMPADDING',(0,0),(-1,-1),4)]))
        return result
    def photos(paths,captions,height=172):
        width=height*4/3
        return Table([[RLImage(str(ROOT/f),width=width,height=height) for f in paths],
                      [p(t,'small') for t in captions]],colWidths=[257.5]*2,
                     style=[('VALIGN',(0,0),(-1,-1),'TOP')])
    cases={c['case_id']:c for c in data['cases']};sort=cases['staged_05']
    recovery=json.loads((ROOT/'artifacts/real_standby_recovery_verification.json').read_text())
    supply=json.loads((ROOT/'artifacts/real_connector_press_verification.json').read_text())
    api_file=ROOT/'artifacts/api_live_verification.json'
    api=json.loads(api_file.read_text()) if api_file.exists() else None
    api_note=(f'Astra API版は{api["trial_count"]}試行中{api["success_count"]}件の搬送を確認（5頁）。'
              if api else '外部Astra APIクライアントを実装。実モデルの結果は別途検証する。')
    story=[p('画像から高さを測り、\n次の一段を生成する'.replace('\n','<br/>'),'title'),
        p('Physical AI 最終課題 | SO-101 / MuJoCo / Python / MCP / LeRobot','small'),
        p('sim：複数タスクを実行。実機：接続・観測・一部動作を確認、把持・搬送成功0件。実機の最新結果は4頁。','small'),
        p('目的：基盤モデルの視覚・言語判断とローカル制御を分離し、同じAPIをシミュレーションと実機へ接続する。既知高さの一括搬送から、実行時に未知の高さ・位置を測り直す段階生成へ拡張した。'),
        table([['接続AI：画像と指示から判断','Python：計測・生成・ローカル実行'],
            ['対象、可視の辺、次の動作、失敗と再試行を選ぶ。MCPのPNGを実際に見る。',
             'カメラ幾何、関節からのFK/IK、1段Python生成、関節補間、速度・範囲・停止を処理。']], [257.5,257.5]),
        p('observe → measure_height → prepare_cartesian_step → 生成Python実行 → 新しいRGB → 次を判断','small'),
        p('単眼RGBで未知高さを推定する条件','h'),
        p('AIが同じ垂直辺の下端・上端を画素で指定。下端の光線と既知支持面の交点を求め、その鉛直線と上端光線から高さを計算する。物体寸法・正解位置・ID・深度バッファを入力しない。上面中央が見えればXYも計算し、見えなければ高さだけを返す。'),
        p('条件は校正済み斜めカメラ、既知支持面、直立する可視の辺。任意形状や隠れた下端には対応しない。画素±2pxの区間は校正・支持面誤差を含まず、近い高さを判別できない例もあった。','small'),
        photos(['artifacts/runs/staged_05/measurement_9d1ba0a7d6d8.png',sort['after_image']],['黄色の辺から計測：青42.8mm、赤28.1mmと推定','26回の一段動作後：高い青は箱、低い赤は机']),
        p('独自の工夫：1回分のPythonと根拠・予想変化を保存し、試験持上げと解放後の画像で確認。把持中は実測開度に戻さず閉指令を維持。AI待機中もsimの接触物理を進め、滑り・落下も記録する。','small'),
        p(api_note+' 同じMCPでRGB→一段生成→実行→再観測を行う。次頁の7 runは会話中のCodexによるもの（厳密なモデルIDは未記録）で、API版と区別する。サーバー内LLM呼出し・新規学習なし。','small'),PageBreak(),
        p('成功・復旧・停止を分けて記録','title'),
        p('7 run・5初期シーン。開発中の修正を含む成立確認であり、一般的な成功率ではない。全AI判断を終了してから独立物理評価を行った。99指令中98指令完了を、タスク成功数に置き換えない。')]
    rows=[['run','指令完了','動作 s','結果']]
    for r in data['cases']:
        rows.append([r['case_id'].replace('staged_',''),f"{r['commands_completed']}/{r['commands']}",f"{r['trajectory_s']:.2f}",r['summary_ja']])
    story += [table(rows,[35,58,54,368]),
        p('完全タスクを落下なしで達成：1 run。落下後の最終状態達成：1。対象のみ成功・他物体転倒：1。複合タスクの部分達成：1。開発停止：2。高さ判別保留：1。04は最初から積んだ別シーンで、02の未完工程を成功扱いしていない。','small'),
        p('初期高さの独立照合（重複配置を除く9対象）','h')]
    rows=[['run / 色','RGB推定 mm','正解 mm','誤差 mm','画素±2pxのみの区間 mm']]
    for r in data['cases']:
        if r['case_id'] in ('staged_01b','staged_01c'):continue
        for h in r['initial_height_checks']:
            rows.append([r['case_id'].replace('staged_','')+' / '+('赤' if h['color']=='red' else '青'),
                f"{h['estimated_mm']:.2f}",f"{h['actual_initial_mm']:.2f}",f"{h['error_mm']:+.2f}",
                f"{h['interval_mm'][0]:.2f} - {h['interval_mm'][1]:.2f}"])
    story += [table(rows,[68,94,85,75,193]),
        p('絶対誤差平均1.49mm、最大4.17mm。初期正解高さは18.84-55.42mm。理想校正のsim結果であり、実機精度ではない。03では2つの区間が重なり、対象を決めずに停止。05は別配置を追加した試行で、03の記録を消していない。','small'),
        p('物理接触で把持・移載。初期設定以外で物体座標を書き換えない。箱内判定は全水平外形、中心高さ帯、静止、指との非接触を確認するため、背の高い部品は箱の縁から突出し得る。積上げは赤の底と青の上面、中心位置、静止、指との非接触を照合した。','small'),
        p('所要時間の詳細：動作軌道合計150.32秒。各runのAI待機を含む最後の観測sim時刻、制御計算時間、失敗理由はCSV/JSONに別記。待機中は概ね実時間、動作中は通常計算速度で進む。動画はsim時間1倍速で待機をカット。','small'),PageBreak(),
        p('見直して修正する','title'),
        photos(['artifacts/runs/staged_02/programs/67dc083602e0_after.png',
                'artifacts/runs/staged_02/programs/577456a7f3e6_after.png'],
               ['積上げ初回：赤が横へ落下。指令完了でも失敗','再把持し保持中の中心ずれを補正。積上げを確認'],height=155),
        photos(['artifacts/runs/staged_04/programs/ee67c1168be8_after.png',cases['staged_04']['after_image']],
               ['取外し：試験持上げで赤が滑って机へ落下','落下後の高さ・位置を再推定し、赤を別置き'],height=155),
        p('共通APIと残る制約','h'),
        p('LeRobot実機も同じobserve / prepare_cartesian_step / execute_skill / get_status / stopを使用。校正・経由点・速度は実機専用設定に分ける。初期状態は観測・ドライランで、操作者がローカルでARMして動作を開始する。接続・一部の短い動作まで確認したが、把持・搬送は未達（4頁）。'),
        p('手首カメラの動的校正、力制御、一般的な障害物回避、任意形状・姿勢は未実装。積上げ補正に使った保持物体の上面平面は寸法とFKからの条件付き推定で、滑り・回転で無効になる。04の台の上面中央は隠れていたため高さのみ使用。推定支持面の誤差伝播も未実装。','small'),
        p('成果物：起動手順、結果集計、sim・real設定、全試行の観測画像・生成Python・MCPログ・原動画。自動テストの件数と範囲も同梱し、模擬通信テストを実機成功と区別する。公開用は必要なファイルを選別し、氏名・所属・メール・認証情報を含めない。','small'),
        p('出典：HF lerobot/so101-tabletop-mujoco @844dee6c / MuJoCo Menagerie @0059d433（Apache-2.0）。LeRobot 0.5.1 @1396b9fa。MuJoCo 3.17.0、MCP 1.30.0。詳細は同梱の出典一覧と依存関係定義に記載。幾何の参考：Criminisi et al., Single View Metrology, IJCV（robots.ox.ac.uk/~vgg/publications/2000/Criminisi00a/）。','small'),PageBreak(),
        p('実機：接続と部分動作、搬送は未達','title'),
        p('実機の結果はシミュレーションとは別集計。操作AIは接続中のCodexで、サーバー内から別LLM APIを呼んでいない。'),
        table([['区分','確認できたこと / 未完了のこと'],
            ['接続・観測','6モーターと保存済み校正の一致、実測関節値、固定RGB画像のMCP取得を確認。実機画像は公開資料に含めない。'],
            ['部分動作','手首単独、グリッパー開口、共通XYZ APIで下降2区間・上昇2区間が指令完了。関節値からのFK変化は独立した手先位置の実測ではない。'],
            ['待機姿勢への復帰',f'肩の要求 -10度に対して実測 {recovery["measured_shoulder_delta_deg"]:.3f}度。{recovery["motion_command"]["wall_s"]:.3f}秒でID6の電圧エラーにより中断。待機姿勢は未達。'],
            ['タスク','イチゴをピンクのカップへ搬送するタスクは成功0件。指令完了や実機接続をタスク成功に数えない。']], [110,405]),
        p('動作を止めた理由と修正','h'),
        p('軌道の許容速度10度/sに対し、サーボ側の設定は約4.4度/sだった。実機設定を3度/sへ下げ、補間速度をオフラインで照合した。電圧保護値、出力上限、停止条件は変更しない。速度修正後の共通XYZ動作は未再検証。'),
        p(f'差し直し後も、全トルクOFF・書込み0の読取り診断で異常を再現。{supply["elapsed_s"]:.3f}秒・{supply["sample_count"]}標本中、手首回転ID5から2件、グリッパーID6から1件の電圧エラー。ID6は下限4.0Vに対して3.8Vとエラーを同時に返した。応答ID・長さ・チェックサムは一致した。'),
        p('3.8Vはサーボ内部センサーの報告で、外部テスターの測定値ではない。正常応答だけの4.8-4.9Vという範囲と分けて保存した。電源、分配配線、サーボ内部検出のどこに原因があるかは未特定。予備電源・電圧計がなく、単独直結による比較も未実施。最新確認は全トルクOFF、動作サーバー停止。'),
        p('限界と次の検証','h'),
        p('電源系統の比較が成立するまで実機の自動復帰・搬送を再開しない。復旧後も最新姿勢と画像から計画し直し、低速の単独動作、実手先の独立計測、対象の把持・解放後の画像確認を順に検証する。実機の一般的な位置精度・タスク成功率はまだ示せない。'),
        p('実機の部分動作、待機姿勢への復帰、速度設定の修正、電圧・通信診断の記録を同梱。生の実画像・校正・機器固有情報は公開資料に含めない。','small'),
        p('エラー仕様：Waveshare ST Servo Memory Map（docs.waveshare.com/Memory_Map_Explanation/ST_Servo_Memory_Map_Explanation）。同じエラー文字列でも原因は異なり、LeRobot issue #2387は低電圧仕様モーターへの12V給電が原因だった。今回は同一原因と断定しない。','small')]
    if api:
        final=api['trials'][-1]
        rows=[['試行','API応答 / 動作','所要秒数','結果']]
        for row in api['trials']:
            conclusion=('赤を箱へ移載。AI画像判断と独立評価が一致。' if row['task_success']
                        else '累積トークン上限で途中停止。搬送未完了。')
            rows.append([str(row['trial']),f'{row["api_responses"]} / {row["commands_completed"]}',
                         f'{row["elapsed_wall_s"]:.2f}',conclusion])
        story += [PageBreak(),p('Astra APIで一連の作業を実行','title'),
            p('指示：「赤いブロックを箱に入れて」。外部APIのモデルがRGBを見て対象・計測する辺・次の一段を選択し、同じMCPを介して実行した。モデル識別子は応答でも gpt-6-astra と確認。実機への指令は出していない。'),
            table(rows,[40,112,75,288]),
            p('同一初期配置での開発確認。初回は250,000 tokenの累積上限で中断したため、画像履歴を含む搬送に合わせて上限を1,500,000 tokenへ変更。中断を除外せず記録し、無作為な条件に対する一般的な成功率は主張しない。','small'),
            photos([final['before_image'],final['after_image']],
                   ['APIへ渡した初期RGB：赤と青は机、箱は空','解放後のRGB：赤が箱内、青は机に残る']),
            p(f'完了試行：指令{final["commands_completed"]}回、API応答{final["api_responses"]}回、全体{final["elapsed_wall_s"]:.2f}秒。動作軌道の合計は{final["trajectory_s"]:.2f}秒。実行前に却下された計画{final["rejected_tool_calls"]}件を含み、モデルが修正して続行した。所要時間はAPI待機とローカル処理を含む。'),
            p('成功の確認方法','h'),
            p('モデルは解放後の新画像から成功を報告。全AI判断の終了後、最終動作完了時の独立物理状態を照合し、赤の全水平外形が箱内、静止、指との接触なし、青は箱外と確認した。把持中の接触も記録され、物体座標の書換えで移載を演出していない。'),
            p('共通API・制御と判断役の分離','h'),
            p('APIクライアントだけがモデルを呼び、MCPサーバーは計測・生成・制限・補間・停止を担当する。モデルは準備した一段のプレビューを見て実行を選び、完了後の新RGBを受け取って次を決める。任意Pythonを実行する機能や、物体の正解状態を読むツールは与えていない。'),
            p('このAPI試験は従来の7 runへ合算しない。成立したのは今回の配置と指示でのシミュレーション搬送であり、API版の実機搬送は未検証。実機の電圧異常と未達事項は前頁のとおり。','small')]
    output=ROOT/'output/pdf/staged_unknown_height_review.pdf'
    def footer(c,doc):
        c.setFont('StagedJP',7.5);c.setFillColor(colors.HexColor('#65778a'))
        c.drawString(40,20,'Simulation + partial real motion | real transfer not completed | review before publication')
        c.drawRightString(A4[0]-40,20,str(doc.page))
    SimpleDocTemplate(str(output),pagesize=A4,leftMargin=40,rightMargin=40,topMargin=32,bottomMargin=34,
        title='未知高さと段階生成によるSO-101作業システム',author='',creator='Physical AI project').build(story,onFirstPage=footer,onLaterPages=footer)
    # The submission must contain no creation/modification timestamps, including
    # hidden PDF metadata. Keep the descriptive title and empty author only.
    from pypdf import PdfReader, PdfWriter
    reader=PdfReader(output)
    writer=PdfWriter()
    writer.append(reader)
    writer.add_metadata({'/Title':'未知高さと段階生成によるSO-101作業システム',
                         '/Author':'','/Creator':'Physical AI project'})
    writer.root_object.pop('/Metadata',None)
    temporary=output.with_suffix('.tmp.pdf')
    writer.write(temporary)
    temporary.replace(output)
    return output


def video_report(data):
    output=ROOT/'submission/staged_unknown_height_demo.mp4';output.parent.mkdir(exist_ok=True)
    font_cache={};edit=[];cursor=0
    def text(d,xy,value,size=24,fill='white',width=1150):
        font=font_cache.setdefault(size,ImageFont.truetype(str(FONT),size));x,y=xy
        for line in value.split('\n'):
            buf=''
            for ch in line:
                if d.textlength(buf+ch,font=font)>width and ch not in '、。，．!?！？)]）］」』':
                    d.text((x,y),buf,font=font,fill=fill);y+=size+9;buf=ch
                else:buf+=ch
            d.text((x,y),buf,font=font,fill=fill);y+=size+9
        return y
    def slide(title,lines):
        im=Image.new('RGB',(1280,720),NAVY);d=ImageDraw.Draw(im)
        d.rectangle((60,65,70,635),fill='#49c9c4');text(d,(100,95),title,40,width=1090)
        y=225
        for line in lines:y=text(d,(105,y),line,28,width=1070)+22
        text(d,(105,670),'実MuJoCo映像 / 接続Codexの画像判断 / 実機搬送は未達 / 字幕は後編集',19,fill='#aac1d1')
        return im
    def picture(path,title,lines):
        im=Image.new('RGB',(1280,720),NAVY);im.paste(Image.open(ROOT/path).convert('RGB').resize((960,720)),(0,0))
        sidebar(im,title,lines);return im
    def sidebar(im,title,lines):
        d=ImageDraw.Draw(im);y=text(d,(982,35),title,28,fill='#64dcd4',width=272)
        for line in lines:y=text(d,(982,y+22),line,21,width=270)
        text(d,(982,584),'sim時間1倍速\nAI待機はカット\n説明字幕は後編集\n実機搬送は未達',18,width=270,fill='#f5c780')
    writer=imageio.get_writer(output,fps=25,codec='libx264',quality=8,macro_block_size=16,ffmpeg_params=['-movflags','+faststart'])
    def hold(im,seconds,source,label):
        nonlocal cursor
        frames=round(seconds*25);pixels=np.asarray(im)
        for _ in range(frames):writer.append_data(pixels)
        edit.append({'kind':'still','output_start_s':cursor/25,'duration_s':frames/25,'source':source,'label':label});cursor+=frames
    cases={c['case_id']:c for c in data['cases']}
    def clips(cid,indices,title):
        nonlocal cursor
        run=ROOT/'artifacts/runs'/cid
        events=[json.loads(x) for x in (run/'events.jsonl').read_text().splitlines()]
        actions=[e for e in events if e['event']=='action_finished' and e['command_status']=='completed']
        plans={e['program_id']:e for e in events if e['event']=='cartesian_program_prepared'}
        for i in indices:
            action=actions[i];pid=action['args']['program_id'];plan=plans[pid]
            post=next(e for e in events if e['event']=='observation' and e['utc']>action['utc'] and e['status']['last_action']['action_id']==action['action_id'])
            start=max(0,round((post['simulation_time_s']-action['trajectory_s']-.28)*25))
            end=round((post['simulation_time_s']+.48)*25)
            count=end-start;source=str((run/'simulation.mp4').relative_to(ROOT))
            cmd=['ffmpeg','-v','error','-ss',f'{start/25:.2f}','-i',str(ROOT/source),'-frames:v',str(count),'-an','-sn','-f','rawvideo','-pix_fmt','rgb24','-']
            process=subprocess.Popen(cmd,stdout=subprocess.PIPE)
            frames=0
            try:
                while True:
                    raw=process.stdout.read(960*720*3)
                    if not raw:break
                    if len(raw)!=960*720*3:raise RuntimeError('Incomplete decoded frame')
                    im=Image.new('RGB',(1280,720),NAVY)
                    im.paste(Image.frombytes('RGB',(960,720),raw),(0,0))
                    sidebar(im,title,[f'{cid} / 一段 {i+1}','実行前の予想：\n'+plan['expected_visual_change'],'動作後の新画像と比較して次を判断'])
                    writer.append_data(np.asarray(im));frames+=1
            finally:process.stdout.close();code=process.wait()
            if code or frames!=count:raise RuntimeError(f'Clip decode mismatch {cid} {pid}: {frames}/{count}')
            edit.append({'kind':'motion','case_id':cid,'program_id':pid,'source':source,
                         'source_start_frame':start,'source_end_frame_exclusive':end,'source_fps':25,
                         'output_start_s':cursor/25,'duration_s':frames/25,'speed':1.0});cursor+=frames
    try:
        hold(slide('未知高さ × 一段ずつPython生成',[
            'RGBを見る → 可視の辺から高さを測る → 次の一段を生成',
            'MuJoCo / SO-101 / MCP。サーバー内のLLM呼出しなし。',
            '物体ID・正解位置・深度をAIの対象選択に使わない。']),6,None,'intro')
        row=cases['staged_05']
        measurement='artifacts/runs/staged_05/measurement_9d1ba0a7d6d8.png'
        hold(picture(measurement,'高さで選び分ける',[
            '指示：高い方は箱、低い方は空いた机へ。','青42.8mm / 赤28.1mm','黄色で選んだ垂直辺とカメラ校正から推定。寸法の入力なし。']),6,measurement,'height choice')
        clips('staged_05',range(26),'高さで選び分ける')
        hold(picture(row['after_image'],'解放・退避して確認',[
            '高い青が箱内。低い赤は新しい机上位置。','26指令完了。落下なしで両方の最終状態を確認。']),4,row['after_image'],'height sort completed')
        hold(picture(cases['staged_02']['before_image'],'別タスク：積み上げ',[
            '青を移し、その上に赤を積む。','初回は位置がずれて落下。次の画像から再計測する。']),3,cases['staged_02']['before_image'],'stack initial')
        clips('staged_02',[19,20,21],'初回の積上げは失敗')
        hold(picture('artifacts/runs/staged_02/programs/67dc083602e0_after.png','落下を検出',[
            '指令は完了したが、赤は青の横に落ちた。','再把持後、保持中の中心ずれを画像から補正。']),3,'artifacts/runs/staged_02/programs/67dc083602e0_after.png','stack failed')
        clips('staged_02',range(27,36),'画像を見直して修正')
        hold(picture('artifacts/runs/staged_02/programs/577456a7f3e6_after.png','積上げを確認',[
            '指を離しても赤が青の上に残った。','その後の取外しは関節限界で未着手。全工程成功とは数えない。']),4,'artifacts/runs/staged_02/programs/577456a7f3e6_after.png','stack recovered')
        hold(picture(cases['staged_04']['before_image'],'別シーン：上だけ取る',[
            '最初から重ねた配置。上の赤を空き机面へ。','前のrunの続きではなく、別試行。']),3,cases['staged_04']['before_image'],'unstack initial')
        clips('staged_04',range(5),'試験持上げで滑落')
        hold(picture('artifacts/runs/staged_04/programs/ee67c1168be8_after.png','指が空、赤が机へ',[
            '落下後の位置と現在の高さを再推定。','閉指令と接近幅を変更し、再把持する。']),3,'artifacts/runs/staged_04/programs/ee67c1168be8_after.png','drop observed')
        clips('staged_04',range(10,18),'再把持から別置きへ')
        hold(picture(cases['staged_04']['after_image'],'最終状態は達成',[
            '赤と青が別々に机上へ。','途中で落下1回。無落下成功とは数えない。']),3,cases['staged_04']['after_image'],'recovered after drop')
        hold(picture(cases['staged_03']['before_image'],'高さ差が曖昧',[
            '青24.9mm / 赤23.3mm','画素誤差の範囲が重なり、高い方を決めず動作なし。','追加視点や精度改善が必要。']),6,cases['staged_03']['before_image'],'ambiguous abstention')
        hold(picture(cases['staged_01c']['after_image'],'対象成功と全体成功',[
            '別試行：赤は箱へ入ったが、青を倒した。','対象のみ成功として記録。失敗runも保存。']),5,cases['staged_01c']['after_image'],'collateral motion')
        hold(slide('確認できた範囲を、そのまま提出',[
            '7 run・5シーン。失敗・復旧・判別保留を全件記録。',
            '単眼計測は可視垂直辺・既知支持面・校正カメラが条件。',
            '実機は接続・観測・部分動作まで確認。電圧異常で停止、搬送成功0件。']),8,None,'limitations')
    finally:writer.close()
    edl={'fps':25,'duration_s':cursor/25,'speed':'1x simulation time; waiting periods cut',
         'captions':'added from actual logs; not a recording of model reasoning','segments':edit}
    (ROOT/'submission/staged_video_edit.json').write_text(json.dumps(edl,ensure_ascii=False,indent=2)+'\n')
    (ROOT/'artifacts/staged_video_edit.json').write_text(json.dumps(edl,ensure_ascii=False,indent=2)+'\n')
    if not 60<=cursor/25<=180:raise RuntimeError(f'Video duration outside 1-3 minutes: {cursor/25}')
    print(json.dumps({'video':str(output.relative_to(ROOT)),'duration_s':cursor/25,'segments':len(edit)},ensure_ascii=False),flush=True)
    return output


def main():
    p=argparse.ArgumentParser();p.add_argument('--pdf-only',action='store_true');a=p.parse_args()
    data=json.loads((ROOT/'artifacts/staged_experiments.json').read_text())
    print(pdf_report(data).relative_to(ROOT),flush=True)
    if not a.pdf_only:video_report(data)


if __name__=='__main__':main()
