"""Offline root-joint domain proof for fixed parts flagged by the GPU probe."""
import argparse,json,time,traceback,hashlib
from pathlib import Path
import xml.etree.ElementTree as ET


def main(args):
    args.output.mkdir(exist_ok=False);begin=time.perf_counter();result=dict(passed=False,hardware_access=False,gpu_used=False)
    try:
        import numpy as np,fcl,trimesh
        from playground_geometry import load
        from conservative_spheres import cover
        from joint_domain import certify_pair
        from curobo_robot_model import digest
        planner,obs,data=load(args.input,json.loads(args.source.read_text()),args.native_code,args.assets)
        from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
        urdf=ac_one_asset_spec(args.assets).urdf_path
        bounds={j.get('name'):tuple(float(j.find('limit').get(k)) for k in ('lower','upper')) for j in ET.parse(urdf).getroot().findall('joint') if j.get('type')!='fixed'}
        state={n:(lo+hi)/2 for n,(lo,hi) in bounds.items()}
        risks=json.loads((args.output.parent/'world-v1/result.json').read_text())['start_world_risks']
        self_probe=args.output.parent/'verify-v6/result.json'
        if self_probe.exists():
            risks += [dict(robot_part=a,world_part=b) for a,b in json.loads(self_probe.read_text()).get('home_proxy_pairs',[])]
        cache={};parts={p['name']:p for p in data['parts']}
        def geometry(name):
            if name not in cache:
                p=parts[name];h=trimesh.convex.convex_hull(np.asarray(p['vertices']))
                fs=np.column_stack((np.full(len(h.faces),3),h.faces)).reshape(-1)
                c,r,_=cover(p['vertices'],faces=p['faces'])
                cache[name]=(fcl.Convex(h.vertices,len(h.faces),fs),np.column_stack((c,r)).tolist())
            return cache[name]
        checks=[]
        for risk in risks:
            a,b=risk['robot_part'],risk['world_part']
            if a not in parts or b not in parts:continue
            ga,sa=geometry(a);gb,sb=geometry(b);la,lb=parts[a]['link'],parts[b]['link']
            def distance(frames):
                objects=[fcl.CollisionObject(shape,fcl.Transform(frames[link][:3,:3],frames[link][:3,3])) for shape,link in ((ga,la),(gb,lb))]
                return max(0.,float(fcl.distance(*objects)))
            proof=certify_pair(planner.fk,la,sa,lb,sb,state,bounds,max_cells=2048,distance_at=distance)
            checks.append(dict(parts=[a,b],**proof));print(b,proof['certified'],proof['reason'],proof['cells'],flush=True)
        result.update(passed=True,urdf_sha256=hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),source_parts_sha256=digest(data['parts']),checks=checks,
                      certified=[p['parts'] for p in checks if p['certified']],scope='offline joint domain only; no new runtime exclusions installed')
    except Exception as error:result.update(error=str(error),traceback=traceback.format_exc())
    result['seconds']=time.perf_counter()-begin;(args.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='checks'},indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('input','source','native-code','assets','output'):p.add_argument('--'+n,type=Path,required=True)
    raise SystemExit(main(p.parse_args()))
