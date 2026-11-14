"""Remux a continuous capture and make a clearly labelled 20x presentation copy."""
import json,subprocess,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'reports/scene2-20261111';VIDEOS=OUT/'videos'
session=json.loads((OUT/'evidence/recording-session.json').read_text());RUN=session['run']
result=json.loads((ROOT/'evidence'/RUN/'summary.json').read_text())
count=len(result['completed']);attempt=RUN.split('-')[-1]
source=VIDEOS/f'scene2-full-report-{attempt}.mkv'
full=VIDEOS/f'scene2-full-attempt{attempt}-{count}of6.mp4'
demo=VIDEOS/f'scene2-demo-attempt{attempt}-20x-{count}of6.mp4'
def probe(path):
    return json.loads(subprocess.check_output(['ffprobe','-v','error','-count_packets','-show_entries',
        'format=duration,size:stream=codec_type,codec_name,width,height,r_frame_rate,nb_read_packets','-of','json',str(path)]))
original=probe(source);duration=float(original['format']['duration'])
session=json.loads((OUT/'evidence/recording-session.json').read_text())
offset=(session['policy_launch_wall_ms']-session['video_start_wall_ms'])/1000
rows=[json.loads(line) for line in (ROOT/'evidence'/RUN/'events.jsonl').read_text().splitlines()]
chapters=[(0,'Initial scene / policy startup')]
counts=dict(A=0,B=0,C=0)
for row in rows:
    if row['event']=='completed':
        kind=row['ledger'][-1]['kind'];counts[kind]+=1
        chapters.append((row['elapsed_s']+offset,f'{kind}{counts[kind]} confirmed ({row["count"]}/6)'))
    if row['event']=='final_check' and row.get('stable')==5:
        chapters.append((row['elapsed_s']+offset,'Final five-frame all-bin check passed'))
chapters=[(t,title) for t,title in chapters if t<duration]
metadata=[';FFMETADATA1',f'title=Scene2 seed0 - complete attempt - validated {count}/6',
          'comment=New report run; not the earlier six-20261111-01 recording. Chapter times approximate.']
for i,(start,title) in enumerate(chapters):
    end=chapters[i+1][0] if i+1<len(chapters) else duration
    metadata.extend(['[CHAPTER]','TIMEBASE=1/1000',f'START={int(start*1000)}',f'END={int(end*1000)}',f'title={title}'])
meta=OUT/'evidence/video-chapters.ffmetadata';meta.write_text('\n'.join(metadata)+'\n')
subprocess.run(['ffmpeg','-hide_banner','-loglevel','warning','-i',str(source),'-i',str(meta),
                '-map','0:v:0','-map_metadata','1','-map_chapters','1','-c:v','copy','-movflags','+faststart','-y',str(full)],check=True)
remuxed=probe(full)
assert original['streams'][0]['nb_read_packets']==next(s for s in remuxed['streams'] if s['codec_type']=='video')['nb_read_packets']
assert abs(float(remuxed['format']['duration'])-duration)<.2
subprocess.run(['ffmpeg','-hide_banner','-loglevel','warning','-xerror','-i',str(full),'-map','0:v:0','-map_chapters','-1','-map_metadata','-1',
    '-vf',f"setpts=(PTS-STARTPTS)/20,fps=15,drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:text='20x SPEED  |  SCENE2 SEED0  |  VALIDATED {count}/6':fontcolor=white:fontsize=22:box=1:boxcolor=black@0.65:boxborderw=10:x=20:y=20",
    '-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart','-y',str(demo)],check=True)
accelerated=probe(demo);assert abs(float(accelerated['format']['duration'])*20-duration)<3
def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()
manifest=dict(run=RUN,scene='scene2',seed=0,
    full_video=dict(file=str(full.relative_to(OUT)),speed=1,continuous=True,untrimmed=True,sha256=sha(full),probe=remuxed),
    demo_video=dict(file=str(demo.relative_to(OUT)),speed=20,sha256=sha(demo),probe=accelerated),
    chapters=[dict(seconds=t,title=title) for t,title in chapters],
    notes=['录像为新补录，不是原成功回合 six-20261111-01 的录像。','原速版为连续场景窗口录制，无音轨；未删除等待或修正阶段。',
           '章节时间根据策略日志与采集启动时间对齐，约有数秒误差。','20 倍速版覆盖同一完整时段，仅用于展示。'])
manifest['result']=dict(completed_count=len(result['completed']),visual_success=result['visual_success'],
                        final_bin_counts=result.get('final_bin_counts'),policy_elapsed_s=result['elapsed_s'],task_success=result.get('task_success'))
manifest['failed_attempts']=[dict(file='videos/failed_attempts/scene2-attempt01-FAILED-2of6.mp4',completed_count=2,
                                summary='evidence/failed-recording-01/summary.json')]
(OUT/'媒体清单.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
def timestamp(t):
    t=int(t);return f'{t//3600:02d}:{t//60%60:02d}:{t%60:02d}'
(OUT/'录像导航.txt').write_text('录像回合 '+RUN+'（单独补录）\n原速视频章节约有数秒定位误差。\n\n'+'\n'.join(f'{timestamp(t)}  {title}' for t,title in chapters)+'\n')
print(json.dumps(dict(full_seconds=duration,demo_seconds=accelerated['format']['duration'],counts=counts),ensure_ascii=False))

