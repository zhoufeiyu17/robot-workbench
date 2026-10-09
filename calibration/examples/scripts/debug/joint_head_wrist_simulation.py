"""Isolated RTX cross-wrist capture/fit smoke; never exposes a motion endpoint."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True);p.add_argument('--plan',required=True);p.add_argument('--output',required=True);a=p.parse_args()
    output=Path(a.output);output.mkdir(exist_ok=False)
    config=json.loads(Path(a.config).read_text());plan=json.loads(Path(a.plan).read_text())
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'active_gpu':0,'multi_gpu':False,'renderer':'RayTracedLighting'})
    try:
        from arx_env.joint_head_workbench_geometry import Geometry
        from arx_env.joint_head_wrist_calibration import solve_wrist_calibration
        from joint_head_workbench_live import RTXRenderer
        from scipy.spatial.transform import Rotation
        g=Geometry(config);renderer=RTXRenderer(g);observations={'left':[],'right':[]};truth={}
        for side in observations:
            X=np.asarray(plan['nominal_mounts'][side]);delta=np.eye(4);delta[:3,:3]=Rotation.from_rotvec([.012,-.015,.008]).as_matrix();delta[:3,3]=[.002,-.001,.002];truth[side]=X@delta
        (output/'simulation-truth.json').write_text(json.dumps({s:v.tolist() for s,v in truth.items()}))
        for row in plan['waypoints']:
            if row.get('capture') is False:continue
            side,target=row['observer'],row['target'];frames=g.frames(row['q']);T=frames[side]['link6']@truth[side]
            op=renderer.UsdGeom.Xformable(renderer.stage.GetPrimAtPath('/World/Head')).GetOrderedXformOps()[0]
            op.Set(renderer.Gf.Matrix4d(*(T@np.diag([1,-1,-1,1])).T.ravel().tolist()))
            _,image=renderer.render(row['q'],[83,83],subframes=4)
            renderer.cv2.imwrite(str(output/(row['id']+'.png')),renderer.cv2.cvtColor(image,renderer.cv2.COLOR_RGB2BGR))
            corners,ids,_=renderer.detector.detectMarkers(renderer.cv2.cvtColor(image,renderer.cv2.COLOR_RGB2GRAY))
            expected=30 if target=='right' else 24;found={} if ids is None else {int(i):xy.reshape(4,2).tolist() for i,xy in zip(ids.ravel(),corners)}
            if expected not in found:print(row['id'],'MISS',flush=True);continue
            observations[side].append({'id':row['id'],'split':row['split'],'T_world_observer_link6':frames[side]['link6'].tolist(),'T_world_marker':(frames[target]['link6']@g.mounts[target]).tolist(),'corners_uv':found[expected]})
            print(row['id'],'DETECTED',flush=True)
        (output/'observations.json').write_text(json.dumps(observations,indent=2))
        results={};source=hashlib.sha256(Path(config['candidate']).read_bytes()).hexdigest()
        for side,rows in observations.items():
            try:results[side]=solve_wrist_calibration(rows,{**g.k,'rectified':True},g.marker_size,plan['nominal_mounts'][side],source_calibration_id=source)
            except ValueError as exc:results[side]={'error':str(exc)}
        (output/'results.json').write_text(json.dumps(results,indent=2))
    finally:app.close()
if __name__=='__main__':main()
