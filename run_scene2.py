#!/usr/bin/env python3
from bootstrap import ROOT
import argparse
import json
import time
import traceback
import hashlib
from pathlib import Path
import cv2
import numpy as np
import yaml
from scene2.timing import PhaseTimer
from scene2.provenance import source_manifest


def json_value(value):
    if isinstance(value,np.ndarray): return value.tolist()
    if isinstance(value,np.generic): return value.item()
    raise TypeError(type(value).__name__)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode',choices=['preflight','observe','single','run'],default='preflight')
    ap.add_argument('--kind',choices=['A','B','C'],default=None)
    ap.add_argument('--count',type=int,choices=range(1,7),default=None,help='Number of new placements for a development run; full validation uses six')
    ap.add_argument('--prepared',action='store_true',help='Continue an explicitly prepared development scene, not an independent reset')
    ap.add_argument('--resume-held',type=Path,help='Development-only C transport continuation from an events.jsonl checkpoint')
    ap.add_argument('--prepare-only',action='store_true')
    ap.add_argument('--seed',type=int,help='Provenance only; policy never uses seed for target locations')
    ap.add_argument('--simulation-label',help='Host lifecycle receipt for this scene')
    ap.add_argument('--output',type=Path,default=ROOT/'outputs'/time.strftime('%Y%m%d-%H%M%S'))
    args = ap.parse_args()
    if args.resume_held and (not args.prepared or args.mode!='single' or args.kind!='C'):
        ap.error('--resume-held requires --prepared --mode single --kind C')
    args.output.mkdir(parents=True,exist_ok=False)
    manifest=source_manifest(ROOT)
    (args.output/'source-manifest.json').write_text(json.dumps(dict(
        baseline_tag='ac-v1-speed-20261026',baseline_commit='ed5ade3aca953066897c456a9c3246c6240e697a',
        **manifest,arguments=vars(args)),default=str,indent=2))
    started = time.monotonic()
    log_file = (args.output/'events.jsonl').open('w',buffering=1)
    def log(event,**values):
        row = dict(event=event,elapsed_s=time.monotonic()-started,**values)
        log_file.write(json.dumps(row,default=json_value)+'\n')
        if event not in ('command',): print(json.dumps(row,default=json_value),flush=True)
    timing=PhaseTimer(log)
    seq = 0
    def save(label,s,parts,bins):
        nonlocal seq
        from scene2.perception import Detector
        seq += 1
        prefix=args.output/f'{seq:03d}-{label}'
        np.savez_compressed(str(prefix)+'.npz',**{k:s[k] for k in (
            'rgb','depth_m','intrinsic','camera_to_base','stamp','table_z_base_m')})
        cv2.imwrite(str(prefix)+'.png',cv2.cvtColor(Detector.annotate(s,parts,bins),cv2.COLOR_RGB2BGR))
    scene=robot=sorter=None
    result = dict(mode=args.mode,continued_scene=args.prepared,completed=[],visual_success=False,task_success=None,
                  source_sha256=manifest['source_sha256'],seed=args.seed,simulation_label=args.simulation_label)
    try:
        import rospy
        from scene1.rgbd import RgbdScene
        from scene2.perception import Detector, serialise
        rospy.init_node('scene2_visual',anonymous=True)
        if args.mode in ('single','run'):
            import rosgraph
            publishers=rosgraph.Master(rospy.get_name()).getSystemState()[0]
            for topic,nodes in publishers:
                if topic in ('/kuavo_arm_traj','/gripper/command'):
                    others=[n for n in nodes if n not in ('/arms_ik_node','/sim_leju_claw_interface')
                            and not n.startswith('/challenge_task_scene2_')]
                    if others: raise RuntimeError(f'Another command publisher on {topic}: {others}')
        config = yaml.safe_load((ROOT/'vendor/scene1/config.yaml').read_text())['cartesian_alignment']
        scene=RgbdScene(rospy,config)
        if args.mode in ('preflight','observe'):
            deadline=time.monotonic()+20
            while True:
                try:
                    s=scene.read()
                    break
                except (ValueError,RuntimeError):
                    if time.monotonic()>deadline: raise
                    time.sleep(.2)
            parts,bins=Detector().detect(s)
            save(args.mode,s,parts,bins)
            log('observation',**serialise(parts,bins))
            if args.mode=='preflight' and (len(parts)!=6 or len(bins)!=3):
                raise RuntimeError('Preflight requires six distinct parts and three bins')
            result['preflight_passed']=len(parts)==6 and len(bins)==3
        else:
            from scene2.control import Robot
            from scene2.task import Sorter
            robot=Robot(rospy,log,allow_loaded_recovery=bool(args.resume_held))
            robot.timing=timing
            if not args.prepared:
                robot.initialise()
            sorter=Sorter(robot,scene,log,save)
            sorter.timing=timing
            if args.prepare_only:
                sorter.observe('prepared')
            else:
                requested=1 if args.mode=='single' else (args.count or 6)
                result['requested_count']=requested
                if args.resume_held:sorter.resume_held(args.resume_held)
                else:sorter.run(requested,args.kind)
                result['final_bin_counts']=sorter.confirm_final()
                result['visual_success']=True
    except Exception as e:
        result['error']=str(e)
        log('stopped',error=str(e),traceback=traceback.format_exc())
    finally:
        if sorter:
            result['completed']=sorter.completed
            result['initial_bin_counts']=sorter.initial_counts
        result['elapsed_s']=time.monotonic()-started
        result['phase_timings']=timing.summary()
        (args.output/'summary.json').write_text(json.dumps(result,indent=2,default=json_value)+'\n')
        if scene: scene.close()
        if sorter: sorter.close()
        if robot: robot.close()
        log_file.close()
    return 1 if 'error' in result else 0


if __name__=='__main__':
    raise SystemExit(main())
