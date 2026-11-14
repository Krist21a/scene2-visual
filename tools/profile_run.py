"""Summarise observed run timing and tracking; does not infer task success."""
import argparse
import collections
import json
from pathlib import Path

ap=argparse.ArgumentParser()
ap.add_argument('folder',type=Path)
args=ap.parse_args()
rows=[json.loads(line) for line in (args.folder/'events.jsonl').read_text().splitlines()]
stages=collections.defaultdict(lambda:dict(samples=0,command_interval_s=0.,max_reference_error_rad=0.))
previous=None
for row in rows:
    if row['event']!='command':continue
    stage=stages[row['stage']];stage['samples']+=1
    # Exclude planning/camera waits and transitions; this is not stage wall time.
    if previous and previous['stage']==row['stage'] and row['elapsed_s']-previous['elapsed_s']<3:
        stage['command_interval_s']+=row['elapsed_s']-previous['elapsed_s']
    error=max(abs(row['command'][i]-row['measured'][i]) for i in (*range(7),*range(8,15)))
    stage['max_reference_error_rad']=max(stage['max_reference_error_rad'],error)
    previous=row
summary=args.folder/'summary.json'
result=dict(folder=str(args.folder),last_event_elapsed_s=rows[-1]['elapsed_s'] if rows else None,
    stages=dict(sorted(stages.items(),key=lambda pair:-pair[1]['command_interval_s'])),
    note='Command intervals exclude gaps >=3 s and stage transitions. Success comes only from saved summary.')
if summary.exists():
    saved=json.loads(summary.read_text())
    result['result']={k:saved[k] for k in ('visual_success','task_success','elapsed_s','requested_count','final_bin_counts','error') if k in saved}
    result['completed_count']=len(saved.get('completed',[]))
    if 'phase_timings' in saved:result['phase_timings']=saved['phase_timings']
print(json.dumps(result,indent=2))
