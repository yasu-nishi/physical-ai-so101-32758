"""Decode recorded videos and prepare every edit midpoint for visual inspection."""
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess

from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parents[1]


def main():
    edit=json.loads((ROOT/'artifacts/camera_video_edit.json').read_text())
    results=[]
    paths=[edit['video']]+[str(p.relative_to(ROOT)) for p in sorted((ROOT/'artifacts/runs').glob('camera_*/simulation.mp4'))]
    for relative in paths:
        path=ROOT/relative
        subprocess.run(['ffmpeg','-v','error','-i',str(path),'-f','null','-'],check=True,capture_output=True)
        result={'path':relative,'passed':True,'all_frames_decoded':True,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        if relative==edit['video']:
            probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries',
                'format=duration:stream=width,height,r_frame_rate','-of','json',str(path)]))
            stream=probe['streams'][0]
            assert (stream['width'],stream['height'],stream['r_frame_rate'])==(1280,720,'25/1')
            assert abs(float(probe['format']['duration'])-edit['duration_s'])<.05
            assert 60<=edit['duration_s']<=180
            assert result['sha256']==edit['sha256']
            result.update(duration_s=edit['duration_s'],cut_midpoints=len(edit['segments']),all_cut_midpoints_visually_checked=False)
        results.append(result)
    folder=ROOT/'tmp/camera_review.local';folder.mkdir(parents=True,exist_ok=True)
    pages=[]
    for page in range(math.ceil(len(edit['segments'])/6)):
        sheet=Image.new('RGB',(1280,1140),'#dbe4ea');draw=ImageDraw.Draw(sheet)
        for i,part in enumerate(edit['segments'][page*6:page*6+6]):
            second=part['output_start_s']+part['duration_s']/2
            raw=subprocess.check_output(['ffmpeg','-v','error','-ss',str(second),'-i',str(ROOT/edit['video']),
                '-frames:v','1','-f','image2pipe','-vcodec','png','-'])
            frame=Image.open(io.BytesIO(raw)).convert('RGB').resize((640,360))
            x=(i%2)*640;y=(i//2)*380
            sheet.paste(frame,(x,y));draw.text((x+8,y+362),f"cut {page*6+i+1} | {second:.2f}s | {part['kind']}",fill='black')
        path=folder/f'cuts_{page+1}.png';sheet.save(path);pages.append(str(path.relative_to(ROOT)))
    (ROOT/'artifacts/camera_video_validation.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps({'decoded_videos':len(results),'duration_s':edit['duration_s'],'contact_sheets':pages}))


if __name__=='__main__':main()
