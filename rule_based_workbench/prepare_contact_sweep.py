"""Offline source contact samples for the fixed-pose symmetric closing sweep."""
import argparse,json,hashlib,time
from pathlib import Path
from dataclasses import asdict
from playground_geometry import load
from grasp_contact_certificate import enrich_contact_sweep


def main(args):
    args.output.mkdir(exist_ok=False);started=time.perf_counter()
    planner,obs,data=load(args.input,json.loads(args.source.read_text()),args.native_code,args.assets)
    proof=json.loads(args.contact.read_text());closing=json.loads(args.closing.read_text())
    proof=enrich_contact_sweep(planner,proof,closing)
    samples=proof['jaw_sweep_source_samples']
    proof.update(jaw_sweep_source_sha256=hashlib.sha256(args.closing.read_bytes()).hexdigest(),
                 sweep_preparation_seconds=time.perf_counter()-started)
    (args.output/'contact.json').write_text(json.dumps(proof,indent=2))
    print(json.dumps(dict(passed=True,samples=len(samples),maximum_claw_penetration_m=max(r['maximum_claw_penetration_m'] for r in samples),seconds=time.perf_counter()-started)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('input','source','native-code','assets','contact','closing','output'):p.add_argument('--'+name,type=Path,required=True)
    main(p.parse_args())
