"""Host-side deployment and official Scene2 lifecycle; no scene modifications."""
import argparse
import datetime
import json
from pathlib import Path
import shlex
import subprocess
import sys
import time
import hashlib

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
SIM='kuavo_challenge_container_GPU_046e4301'
ALGO='kuavo-ros-noetic-s12-ready'


def call(*args,**kwargs):
    return subprocess.run(['sudo','-n','docker',*args],check=True,**kwargs)


def verify_deployed_source(prune_stale=False):
    from scene2.provenance import source_manifest
    expected=source_manifest(ROOT)
    script=('import json,sys; from scene2.provenance import verify_source_tree; '
            'print(json.dumps(verify_source_tree("/root/scene2-visual",json.loads(sys.argv[1]),'
            f'prune_stale={prune_stale!r})))')
    result=call('exec',ALGO,'bash','/root/scene2-visual/run.sh','-c',script,
                json.dumps(expected),capture_output=True,text=True)
    receipt=json.loads(result.stdout)
    if receipt!=expected:raise RuntimeError('Container source verification receipt disagrees')
    return receipt


def validate_runtime(receipt,expected,require_running=False):
    for path,digest in expected.items():
        if receipt['files'].get(path)!=digest:
            raise RuntimeError('Scene2 requires the verified A+C runtime; mismatch: '+path+
                '. Stop simulation/policy, then run sudo -n python3 vendor/scene1/tools/select_controller.py ac --apply')
    if require_running:
        for name in ('humanoid_VR_hand_control','libhumanoid_controllers.so'):
            accepted={digest for path,digest in expected.items() if path.endswith('/'+name)}
            loaded=receipt.get('loaded',{}).get(name,[])
            if not loaded or any(digest not in accepted for digest in loaded):
                raise RuntimeError('Running process does not use the verified A+C artifact: '+name)
    return receipt


def verify_runtime(require_running=False):
    manifest=json.loads((ROOT/'vendor/scene1/evidence/control-runtime-manifest.json').read_text())['files']
    artifacts={
        'devel/.private/humanoid_interface_ros/lib/humanoid_interface_ros/humanoid_VR_hand_control':'controllers/A/humanoid_VR_hand_control',
        'devel/.private/humanoid_controllers/lib/libhumanoid_controllers.so':'controllers/C/libhumanoid_controllers.so',
        'src/humanoid-control/humanoid_controllers/config/kuavo_v52/mpc/dynamic_qr.info':'controllers/baseline-config/dynamic_qr.info',
        'src/humanoid-control/humanoid_controllers/config/kuavo_v52/mpc/task.info':'controllers/baseline-task/task.info'}
    expected={'/root/kuavo_ws/'+path:manifest.get(source) or
              hashlib.sha256((ROOT/'vendor/scene1'/source).read_bytes()).hexdigest()
              for path,source in artifacts.items()}
    script='''import hashlib,json,pathlib
sha=lambda p:hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()
paths=json.loads(%r)
result={'files':{p:sha(p) for p in paths},'loaded':{}}
for process in pathlib.Path('/proc').iterdir():
 if not process.name.isdigit():continue
 try:
  exe=process/'exe'
  if exe.resolve().name=='humanoid_VR_hand_control':
   result['loaded'].setdefault('humanoid_VR_hand_control',[]).append(sha(exe))
  for line in (process/'maps').read_text().splitlines():
   if '/libhumanoid_controllers.so' not in line:continue
   path=line.split(maxsplit=5)[5]
   digest='deleted' if path.endswith(' (deleted)') else sha(path)
   values=result['loaded'].setdefault('libhumanoid_controllers.so',[])
   if digest not in values:values.append(digest)
 except (FileNotFoundError,PermissionError,ProcessLookupError):continue
print(json.dumps(result))
'''%json.dumps(list(expected))
    receipt=json.loads(call('exec',SIM,'python3','-c',script,capture_output=True,text=True).stdout)
    return validate_runtime(receipt,expected,require_running)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('action',choices=['start','stop','sync','run'])
    ap.add_argument('--seed',type=int,default=0)
    ap.add_argument('--mode',choices=['preflight','observe','single','run'],default='preflight')
    ap.add_argument('--kind',choices=['A','B','C'])
    ap.add_argument('--count',type=int,choices=range(1,7))
    ap.add_argument('--prepared',action='store_true',help='Continue a development scene; not an independent reset')
    ap.add_argument('--resume-held',help='Container checkpoint path for validated high C continuation')
    ap.add_argument('--label',default=datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    a=ap.parse_args()
    if a.prepared and (a.action!='run' or a.mode not in ('single','run')):
        ap.error('--prepared requires run --mode single/run')
    if a.resume_held and not (a.action=='run' and a.prepared and a.mode=='single' and a.kind=='C'):
        ap.error('--resume-held requires run --prepared --mode single --kind C')
    if not a.label or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in a.label):
        ap.error('Label must use letters, numbers, hyphens or underscores')
    if a.action=='sync':
        processes=call('exec',ALGO,'ps','-eo','args',capture_output=True,text=True).stdout
        if any('python' in line and 'run_scene2.py' in line for line in processes.splitlines()):
            raise RuntimeError('Policy is running; cannot replace its recorded source')
        call('exec',ALGO,'mkdir','-p','/root/scene2-visual/vendor/scene1')
        for name in ['scene2','tools','tests','assets','run_scene2.py','run.sh','bootstrap.py']:
            call('cp',str(ROOT/name),ALGO+':/root/scene2-visual/')
        for name in ['scene1','config.yaml']:
            call('cp',str(ROOT/'vendor/scene1'/name),ALGO+':/root/scene2-visual/vendor/scene1/')
        verify_deployed_source(prune_stale=True)
    elif a.action=='start':
        state=call('exec',SIM,'ps','-eo','args',capture_output=True,text=True).stdout
        if any(x in state for x in ['roslaunch','challenge_task.py','humanoid_sqp']):
            raise RuntimeError('Existing simulation detected; stop it before a new independent reset')
        receipt=verify_runtime()
        (ROOT/'evidence'/f'{a.label}-runtime.json').write_text(json.dumps(receipt,indent=2)+'\n')
        log=f'/tmp/scene2-visual-{a.label}.log'
        command=('cd /root/kuavo_ws; source installed/setup.zsh; source devel/setup.zsh; '
                 'export LD_LIBRARY_PATH=/opt/drake/lib:$LD_LIBRARY_PATH; '
                 f'exec rosrun challenge_cup_task_template challenge_task.py --scene scene2 --seed {a.seed} --no-timer-gui '
                 f'> {shlex.quote(log)} 2>&1')
        call('exec','-d','-t','-e','PYTHONUNBUFFERED=1','-e','DISPLAY=:0','-e','XAUTHORITY=/tmp/scene1-current-display.Xauthority',SIM,'zsh','-lc',command)
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            read=subprocess.run(['sudo','-n','docker','exec',SIM,'cat',log],capture_output=True,text=True)
            result=read.stdout
            if '场景实例已初始化。' in result:
                verify_runtime(require_running=True)
                (ROOT/'evidence'/f'{a.label}-simulation.log').write_text(result)
                (ROOT/'evidence'/'active-simulation.json').write_text(json.dumps(dict(seed=a.seed,label=a.label,log=log)))
                print('Official Scene2 ready:',a.label,flush=True)
                return
            if '[FATAL]' in result or 'Traceback (most recent call last)' in result:
                raise RuntimeError(result[-4000:])
            time.sleep(.5)
        raise RuntimeError('Scene2 startup timed out; inspect '+log)
    elif a.action=='stop':
        code='import rosnode; rosnode.kill_nodes([n for n in rosnode.get_node_names() if n.startswith("/challenge_task_scene2_")])'
        call('exec',SIM,'bash','-lc','source /root/kuavo_ws/devel/setup.bash; python3 -c '+shlex.quote(code))
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            state=call('exec',SIM,'ps','-eo','args',capture_output=True,text=True).stdout
            if not any(x in state for x in ['roslaunch','challenge_task.py','humanoid_sqp']):break
            time.sleep(.5)
        else:raise RuntimeError('Simulation cleanup did not finish within 30 seconds')
        active=ROOT/'evidence'/'active-simulation.json'
        if active.exists():
            previous=json.loads(active.read_text())
            call('cp',SIM+':'+previous['log'],str(ROOT/'evidence'/(previous['label']+'-final.log')))
            active.unlink()
    else:
        log=ROOT/'evidence'/f'{a.label}.log'
        if log.exists(): raise RuntimeError('Run label already exists')
        verify_deployed_source()
        if a.mode in ('single','run'):
            receipt=verify_runtime(require_running=True)
            (ROOT/'evidence'/f'{a.label}-runtime.json').write_text(json.dumps(receipt,indent=2)+'\n')
        cmd=['sudo','-n','docker','exec',ALGO,'bash','/root/scene2-visual/run.sh','run_scene2.py','--mode',a.mode,'--output','outputs/'+a.label]
        active=ROOT/'evidence'/'active-simulation.json'
        if not active.exists():raise RuntimeError('Missing active simulation receipt; start a fresh scene')
        simulation=json.loads(active.read_text())
        cmd+=['--seed',str(simulation['seed']),'--simulation-label',simulation['label']]
        if a.kind: cmd+=['--kind',a.kind]
        if a.count: cmd+=['--count',str(a.count)]
        if a.prepared:cmd+=['--prepared']
        if a.resume_held:cmd+=['--resume-held',a.resume_held]
        recording=None
        if a.mode in ('single','run'):
            from recording import Recording
            recording=Recording(ROOT/'evidence'/f'{a.label}.mkv').start()
        try:
            with log.open('x') as stream:
                if recording:recording.receipt['policy_launch_wall_s']=time.time()
                result=subprocess.run(cmd,stdout=stream,stderr=subprocess.STDOUT)
        finally:
            if recording:recording.stop()
        call('cp',ALGO+':/root/scene2-visual/outputs/'+a.label,str(ROOT/'evidence'/a.label))
        raise SystemExit(result.returncode)


if __name__=='__main__': main()
