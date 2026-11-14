"""Fresh-scene development probes and frozen-source seed 0/1/2 acceptance.

Every invocation creates a new series. A failed seed invalidates the entire
series; no prepared/resume paths and no aggregation across candidate versions.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scene2.provenance import source_manifest
from tools.workstation import ROOT, ALGO, call

EXPECTED = dict(A=2, B=2, C=2)
BASELINE_SECONDS = 3011.924


def validate_round(result, manifest, independent, video, source_sha256, seed, events):
    if result.get('source_sha256') != source_sha256 or manifest.get('source_sha256') != source_sha256:
        raise ValueError('Source changed during the acceptance series')
    if result.get('seed') != seed or manifest['arguments'].get('seed') != seed:
        raise ValueError('Seed provenance mismatch')
    if (result.get('continued_scene') or manifest['arguments'].get('prepared') or
            manifest['arguments'].get('resume_held')):
        raise ValueError('Acceptance cannot use a continued scene')
    if result.get('error') or not result.get('visual_success') or result.get('requested_count') != 6:
        raise ValueError('Production six-piece run did not succeed')
    if result.get('initial_bin_counts') != dict.fromkeys('ABC', 0):
        raise ValueError('Acceptance requires initially empty bins')
    ledger = result.get('completed', [])
    if len(ledger) != 6 or {k: sum(e['kind'] == k for e in ledger) for k in 'ABC'} != EXPECTED:
        raise ValueError('Completion ledger is not A2/B2/C2')
    if result.get('final_bin_counts') != EXPECTED:
        raise ValueError('Final production counts are not A2/B2/C2')
    final = [e for e in events if e['event'] == 'final_check'][-5:]
    if (len(final) != 5 or [e['stable'] for e in final] != [1,2,3,4,5] or
            any(e['counts'] != EXPECTED for e in final)):
        raise ValueError('Production five-frame evidence missing')
    stamps = [e['stamp'] for e in final]
    if any(b <= a for a,b in zip(stamps,stamps[1:])):
        raise ValueError('Repeated production frame')
    frames = independent.get('frames', [])
    if (not independent.get('verified') or independent.get('expected') != EXPECTED or
            len(frames) != 5 or any(f['counts'] != EXPECTED for f in frames)):
        raise ValueError('Independent five-frame check failed')
    for frame in frames:
        if frame['stamp'] <= stamps[-1]:
            raise ValueError('Independent frames must be newer than production frames')
        stamps.append(frame['stamp'])
    if (video.get('premature_exit', True) or not video.get('sha256') or
            video.get('started_wall_s', float('inf')) > video.get('policy_launch_wall_s', 0)):
        raise ValueError('Continuous recording is missing or started late')
    duration = float(video.get('probe', {}).get('format', {}).get('duration', 0))
    if duration + 2 < result['elapsed_s']:
        raise ValueError('Video is shorter than the full policy run')
    if seed == 0 and result['elapsed_s'] >= BASELINE_SECONDS:
        raise ValueError('Seed0 did not improve on the 3011.924 second baseline')


def read(path):
    return json.loads(path.read_text())


def workstation(*args):
    subprocess.run([sys.executable, str(ROOT/'tools/workstation.py'), *args], check=True)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--label', required=True)
    ap.add_argument('--phase', choices=['development','acceptance'], required=True)
    ap.add_argument('--development', type=Path, help='Optional successful A/C pair receipt on this exact source')
    a=ap.parse_args()
    if not a.label or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in a.label):
        ap.error('Invalid label')
    dest=ROOT/'evidence'/a.label
    dest.mkdir(exist_ok=False)
    manifest=source_manifest(ROOT); digest=manifest['source_sha256']
    (dest/'candidate-source.json').write_text(json.dumps(manifest,indent=2)+'\n')
    receipt=dict(phase=a.phase,source_sha256=digest,rounds=[],status='running',task_success=None)
    def save(): (dest/'series.json').write_text(json.dumps(receipt,indent=2)+'\n')
    save()
    try:
        if a.phase=='acceptance' and a.development is not None:
            development=read(a.development)
            if (development.get('phase')!='development' or development.get('status')!='passed' or
                    development.get('source_sha256')!=digest or
                    sorted(r.get('kind','') for r in development.get('rounds',[]))!=['A','C']):
                raise ValueError('Both A/C pairs must pass on this exact candidate source')
            receipt['development']=str(a.development)
        elif a.phase=='acceptance':
            # Integration failures after earlier placements need a fresh full
            # round. Do not manufacture or reuse a different-source pair
            # receipt. All three full-round checks below remain mandatory.
            receipt['development']=None
            receipt['entry']='direct_fresh_six_piece'
        workstation('sync')
        # Optional development probes exercise the currently limiting A pair
        # first. Acceptance always starts three fresh complete scenes.
        jobs=[(0,'A'),(0,'C')] if a.phase=='development' else [(0,None),(1,None),(2,None)]
        for index,(seed,kind) in enumerate(jobs):
            if source_manifest(ROOT)['source_sha256']!=digest:raise ValueError('Candidate source changed')
            label=f'{a.label}-{kind or "six"}-s{seed}'
            sim=f'{label}-sim'
            workstation('start','--label',sim,'--seed',str(seed))
            try:
                args=['run','--mode','run','--label',label]
                if kind:args+=['--kind',kind,'--count','2']
                workstation(*args)
                folder=ROOT/'evidence'/label
                result=read(folder/'summary.json'); run_manifest=read(folder/'source-manifest.json')
                events=[json.loads(line) for line in (folder/'events.jsonl').read_text().splitlines()]
                final_stamp=max(e['stamp'] for e in events if e['event']=='final_check')
                expected=EXPECTED if kind is None else {k:2 if k==kind else 0 for k in 'ABC'}
                verify_label=label+'-independent'
                command=['exec',ALGO,'bash','/root/scene2-visual/run.sh','tools/verify_bins.py',
                         '--expected',','.join(f'{k}:{v}' for k,v in expected.items()),
                         '--after-stamp',str(final_stamp),'--output','outputs/'+verify_label]
                try:
                    with (ROOT/'evidence'/f'{verify_label}.log').open('x') as stream:
                        call(*command,stdout=stream,stderr=subprocess.STDOUT)
                finally:
                    call('cp',ALGO+':/root/scene2-visual/outputs/'+verify_label,str(ROOT/'evidence'/verify_label))
                independent=read(ROOT/'evidence'/verify_label/'summary.json')
                if kind is None:
                    validate_round(result,run_manifest,independent,read(ROOT/'evidence'/f'{label}.video.json'),digest,seed,events)
                elif (not result.get('visual_success') or len(result.get('completed',[]))!=2 or
                      result.get('initial_bin_counts')!=dict.fromkeys('ABC',0) or
                      result.get('final_bin_counts')!=expected or not independent.get('verified') or
                      result.get('source_sha256')!=digest or result.get('continued_scene')):
                    raise ValueError('Fresh two-piece development gate failed')
                if source_manifest(ROOT)['source_sha256']!=digest:raise ValueError('Source changed during run')
                receipt['rounds'].append(dict(seed=seed,kind=kind,run=label,independent=verify_label,elapsed_s=result['elapsed_s']))
                save()
            finally:
                workstation('stop')
        receipt['status']='passed'
    except BaseException as ex:
        receipt.update(status='failed',error=str(ex))
        raise
    finally:
        save()


if __name__=='__main__':main()
