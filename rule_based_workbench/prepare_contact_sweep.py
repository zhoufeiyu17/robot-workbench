"""Offline source contact samples for the fixed-pose symmetric closing sweep."""
import argparse,json,hashlib,time
from pathlib import Path
from dataclasses import asdict
from playground_geometry import load


def main(args):
    args.output.mkdir(exist_ok=False);started=time.perf_counter()
    planner,obs,data=load(args.input,json.loads(args.source.read_text()),args.native_code,args.assets)
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    proof=json.loads(args.contact.read_text());closing=json.loads(args.closing.read_text())
    if not proof['passed'] or not closing['passed']:raise ValueError('passed source contact required')
    samples=[]
    for row in closing['samples']:
        if not row['free']:raise ValueError('source sweep has a failed sample')
        state={**closing['terminal_state'],**row['jaws']};frames=planner.fk.forward(state)
        cup=RigidTransform(**row['world_from_cup']);maximum=0.
        tool=planner.geo.mount.compose(planner.fk.pose(frames['left_link6']))
        for band,shape in planner.geo.cup_bands:
            pose=cup.compose(RigidTransform((0,0,(band.z_min_m+band.z_max_m)/2)))
            obj=planner.geo.fcl.CollisionObject(shape,planner.geo.whole.transform(pose))
            for link,name,geometry,_,_ in planner.geo.whole.parts:
                if link not in ('left_link7','left_link8'):continue
                other=planner.geo.fcl.CollisionObject(geometry,planner.geo.whole.transform(planner.geo.mount.compose(planner.fk.pose(frames[link]))))
                _,witnesses=planner.geo.cup_pair(obj,other)
                maximum=max(maximum,max((depth for _,depth in witnesses),default=0.))
        if maximum+proof['numerical_contact_reserve_m']>=.003:raise ValueError('source sweep contact exceeds3mm')
        samples.append(dict(jaw_position_m=row['jaws']['left_joint7'],tool_from_cup=asdict(tool.inverse().compose(cup)),maximum_claw_penetration_m=maximum))
    proof.update(jaw_sweep_source_samples=samples,jaw_sweep_source_sha256=hashlib.sha256(args.closing.read_bytes()).hexdigest(),
                 sweep_preparation_seconds=time.perf_counter()-started)
    (args.output/'contact.json').write_text(json.dumps(proof,indent=2))
    print(json.dumps(dict(passed=True,samples=len(samples),maximum_claw_penetration_m=max(r['maximum_claw_penetration_m'] for r in samples),seconds=time.perf_counter()-started)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('input','source','native-code','assets','contact','closing','output'):p.add_argument('--'+name,type=Path,required=True)
    main(p.parse_args())
