"""Review video from recorded camera trials, including both losses of grasp."""
import hashlib
import json
from pathlib import Path
import subprocess

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from build_deliverables import FONT, ROOT, NAVY


def main():
    data=json.loads((ROOT/'artifacts/camera_view_experiments.json').read_text())
    rows={r['case_id']:r for r in data['cases']}
    output=ROOT/'artifacts/camera_view_review.mp4'
    output.parent.mkdir(exist_ok=True)
    edits=[];cursor=0;fonts={}
    def text(draw,xy,value,size=21,width=270,color='white'):
        font=fonts.setdefault(size,ImageFont.truetype(str(FONT),size));x,y=xy
        for line in value.split('\n'):
            buf=''
            for c in line:
                if draw.textlength(buf+c,font=font)>width:
                    draw.text((x,y),buf,font=font,fill=color);y+=size+9;buf=c
                else:buf+=c
            draw.text((x,y),buf,font=font,fill=color);y+=size+9
        return y
    def frame(rgb,title,lines,still=False):
        image=Image.new('RGB',(1280,720),NAVY);image.paste(rgb,(0,0));draw=ImageDraw.Draw(image)
        y=text(draw,(982,32),title,27,color='#64dcd4')
        for line in lines:y=text(draw,(982,y+20),line)+4
        text(draw,(982,584),('静止画を保持\n' if still else 'sim時間1倍速\n')+'AI待機をカット\n字幕は後編集\n実機は未検証',18,color='#f5c780')
        return image
    writer=imageio.get_writer(output,fps=25,codec='libx264',quality=8,macro_block_size=16,
                              ffmpeg_params=['-movflags','+faststart'])
    def hold(path,title,lines,seconds):
        nonlocal cursor
        rgb=Image.open(ROOT/path).convert('RGB')
        pixels=np.asarray(frame(rgb,title,lines,True));count=round(seconds*25)
        for _ in range(count):writer.append_data(pixels)
        edits.append({'kind':'still','source':path,'title':title,'output_start_s':cursor/25,'duration_s':count/25})
        cursor+=count
    run=ROOT/'artifacts/runs/camera_front_01'
    events=[json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
    actions=[e for e in events if e['event']=='action_finished']
    plans={e['program_id']:e for e in events if e['event']=='cartesian_program_prepared'}
    try:
        hold(rows['camera_front_01']['before_image'],'カメラ位置を変える',
             ['実際のRGBを見て対象と辺を選ぶ。','同じAPI・別のカメラ校正。','高さ測定と搬送の結果を分けて確認。'],6)
        for name,label in [('side','横側'),('low','低い斜め視点')]:
            row=rows[f'camera_{name}_01'];checks=row['initial_height_checks']
            hold(row['before_image'],label,[f"赤 {checks[0]['estimated_mm']:.1f}mm\n青 {checks[1]['estimated_mm']:.1f}mm",
                 'AIが同じ縦辺の上下端を選んで測定。','校正は理想的なsim値。'],5)
        hold(rows['camera_top_01']['before_image'],'真上：測定を保留',
             ['腕で一部が隠れ、縦辺が十分に見えない。','高さを推測せず停止。','このケースは動作指令0件。'],6)
        source=str((run/'simulation.mp4').relative_to(ROOT))
        for i,action in enumerate(actions):
            pid=action['args']['program_id'];plan=plans[pid]
            post=next(e for e in events if e['event']=='observation' and e['utc']>action['utc'] and
                      e['status']['last_action']['action_id']==action['action_id'])
            first=max(0,round((post['simulation_time_s']-action['trajectory_s']-.28)*25))
            last=round((post['simulation_time_s']+.48)*25);count=last-first
            process=subprocess.Popen(['ffmpeg','-v','error','-ss',f'{first/25:.2f}','-i',str(ROOT/source),
                '-frames:v',str(count),'-an','-sn','-f','rawvideo','-pix_fmt','rgb24','-'],stdout=subprocess.PIPE)
            frames=0
            try:
                while True:
                    raw=process.stdout.read(960*720*3)
                    if not raw:break
                    if len(raw)!=960*720*3:raise RuntimeError('Incomplete video frame')
                    image=frame(Image.frombytes('RGB',(960,720),raw),f'一段 {i+1} / {len(actions)}',
                        ['実行前の予想：\n'+plan['expected_visual_change'],'実行後に再撮影し、次を判断する。'])
                    writer.append_data(np.asarray(image));frames+=1
            finally:
                process.stdout.close();code=process.wait()
            if code or frames!=count:raise RuntimeError('Video source range mismatch')
            edits.append({'kind':'motion','source':source,'program_id':pid,'source_start_frame':first,
                'source_end_frame_exclusive':last,'source_fps':25,'speed':1.0,'output_start_s':cursor/25,'duration_s':frames/25})
            cursor+=frames
            if i==7:
                hold(str((run/f'programs/{pid}_after.png').relative_to(ROOT)),'結果：机上へ落下',
                     ['指令は完了したが、搬送は失敗。','画像と開度から把持喪失を確認。','位置・高さを測り直して再把持。'],6)
        hold(rows['camera_front_01']['after_image'],'再試行も把持が外れた',
             ['赤は最終的に箱内にある。','箱の上で滑落した結果で、予定した下降・開放ではない。','制御された搬送成功は0/1。'],8)
        hold(rows['camera_low_01']['before_image'],'今回確認できた範囲',
             ['斜め3視点・同じ2物体を6回測定。',f"高さ誤差：平均 {data['height_mean_absolute_error_mm']:.2f}mm / 最大 {data['height_maximum_absolute_error_mm']:.2f}mm",
              '実機校正・実機動作は未検証。搬送中の保持は改善が必要。'],8)
    finally:writer.close()
    if not 60<=cursor/25<=180:raise RuntimeError('Review video must be 1..3 minutes')
    result={'video':str(output.relative_to(ROOT)),'duration_s':cursor/25,'fps':25,
        'speed':'1x simulation time; deliberation cut; captions added afterwards',
        'sha256':hashlib.sha256(output.read_bytes()).hexdigest(),'segments':edits,'scope':'simulation only, includes two uncommanded drops'}
    (ROOT/'artifacts/camera_video_edit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='segments'},ensure_ascii=False))


if __name__=='__main__':main()
