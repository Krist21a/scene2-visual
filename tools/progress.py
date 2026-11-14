"""Concise read-only status from a running policy's console log."""
import json,sys
from pathlib import Path

rows=[]
for line in Path(sys.argv[1]).read_text().splitlines():
    if not line.startswith('{'):continue
    try:rows.append(json.loads(line))
    except json.JSONDecodeError:continue  # A live writer may be mid-line.
ledger=next((r['ledger'] for r in reversed(rows) if r['event']=='completed'),[])
print('completed',len(ledger),'kinds',''.join(p['kind'] for p in ledger))
for r in rows[-4:]:
    print({k:r[k] for k in ('event','elapsed_s','stage','kind','stable','in_bin','reason') if k in r})
if rows and rows[-1]['event']=='stopped':print(rows[-1].get('error'))
