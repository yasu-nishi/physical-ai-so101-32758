"""Caption the recorded live API trial; preserve all simulation frames at 1x."""
import hashlib
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from scripts.build_deliverables import FONT, FONT_BOLD, ROOT, NAVY


def build():
    trial=json.loads((ROOT/'artifacts/api_live_verification.json').read_text())['trials'][-1]
    assert trial['task_success']
    source=ROOT/trial['video']
    events=[json.loads(x) for x in (source.parent/'events.jsonl').read_text().splitlines()]
    actions=[e for e in events if e['event']=='action_finished' and e['command_status']=='completed']
    plans={e['program_id']:e for e in events if e['event']=='cartesian_program_prepared'}
    windows=[]
    for i,action in enumerate(actions,1):
        post=next(e for e in events if e['event']=='observation' and
                  (e['status'].get('last_action') or {}).get('action_id')==action['action_id'])
        plan=plans[action['args']['program_id']]
        windows.append({'step':i,'start_s':max(0,post['simulation_time_s']-action['trajectory_s']),
                        'end_s':post['simulation_time_s'],'reason':plan['visual_reason'],
                        'expected':plan['expected_visual_change']})
    fonts={}
    def text(draw,xy,value,size=24,width=275,fill='white',bold=False):
        key=(size,bold)
        if key not in fonts:
            fonts[key]=ImageFont.truetype(str(FONT_BOLD if bold else FONT),size)
        font=fonts[key]
        x,y=xy
        for line in value.split('\n'):
            buf=''
            for char in line:
                if draw.textlength(buf+char,font=font)>width and char not in '、。，．！？)]）］」』':
                    draw.text((x,y),buf,font=font,fill=fill);y+=size+8;buf=char
                else:buf+=char
            draw.text((x,y),buf,font=font,fill=fill);y+=size+8
        return y
    def title(lines):
        im=Image.new('RGB',(1280,720),NAVY);d=ImageDraw.Draw(im)
        text(d,(75,75),'Astra APIでSO-101を動かす',42,width=1100,bold=True)
        y=205
        for line in lines:y=text(d,(80,y),line,28,width=1100)+20
        text(d,(80,665),'実MuJoCo映像 / シミュレーション時間1倍速 / 説明字幕は後編集',20,width=1120,fill='#abc7d3')
        return np.asarray(im)
    panels={}
    def panel(window):
        key=window['step'] if window else 0
        if key in panels:return panels[key]
        im=Image.new('RGB',(320,720),NAVY);d=ImageDraw.Draw(im)
        y=text(d,(20,26),'指示：赤いブロックを\n箱に入れて',27,bold=True)
        y=text(d,(20,y+25),'判断：gpt-6-astra\n操作：共通MCP\n物理：MuJoCo',21)+30
        if window:
            y=text(d,(20,y),f'一段の動作 {key} / {len(actions)}\nPythonでローカル実行',23,fill='#68ded4',bold=True)+18
            text(d,(20,y),window['expected'],20)
        else:
            y=text(d,(20,y),'画像を見て\n次の一段を判断',26,fill='#68ded4',bold=True)+20
            text(d,(20,y),'計測・計画・プレビュー確認もAPIのツール呼出しで進める。',20)
        text(d,(20,590),'sim時間1倍速\nAPI待機を含む\n字幕は後編集\nAPI版実機は未検証',18,fill='#f5cf8a')
        panels[key]=im
        return im
    output=ROOT/'submission/astra_api_demo.mp4'
    output.parent.mkdir(parents=True,exist_ok=True)
    writer=imageio.get_writer(output,fps=25,codec='libx264',quality=8,macro_block_size=16,
                              ffmpeg_params=['-movflags','+faststart'])
    intro=title(['「赤いブロックを箱に入れて」',
        'RGB画像 → Astra APIの判断 → MCP → 一段のPython動作',
        '同じカメラ画像・操作APIを使い、解放後に結果を確認する。'])
    outro=title([f'動作{trial["commands_completed"]}回・API応答{trial["api_responses"]}回・約{trial["elapsed_wall_s"]:.0f}秒で完了',
        '赤は箱内で静止し、指と非接触。画像判断と独立物理評価が一致。',
        '初回はトークン上限で中断。上限修正後の完了試行を収録。',
        '限定条件の成立確認。実機搬送と一般的な成功率は未検証。'])
    frames=0
    try:
        for _ in range(150):writer.append_data(intro)
        reader=imageio.get_reader(source)
        try:
            for n,frame in enumerate(reader):
                when=n/25
                window=next((w for w in windows if w['start_s']<=when<=w['end_s']),None)
                im=Image.new('RGB',(1280,720),NAVY)
                im.paste(Image.fromarray(frame),(0,0));im.paste(panel(window),(960,0))
                writer.append_data(np.asarray(im));frames+=1
        finally:reader.close()
        for _ in range(150):writer.append_data(outro)
    finally:writer.close()
    metadata={'path':str(output.relative_to(ROOT)),'source':trial['video'],
              'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
              'sha256':hashlib.sha256(output.read_bytes()).hexdigest(),
              'source_frames':frames,'fps':25,'duration_s':(frames+300)/25,
              'intro_s':6,'outro_s':6,'speed':'1x simulation time; all source frames retained; API waiting included',
              'caption_font':'Noto Sans JP Regular / Bold (SIL OFL 1.1)',
              'caption_scope':'Post-edited labels; movement windows inferred from logged post-action simulation time and trajectory duration',
              'action_windows':windows,'all_frames_decoded':False,'visual_review':False}
    (ROOT/'artifacts/api_video_validation.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in metadata.items() if k!='action_windows'},ensure_ascii=False,indent=2))


if __name__=='__main__':build()
