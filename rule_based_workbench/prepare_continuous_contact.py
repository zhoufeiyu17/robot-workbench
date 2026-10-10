"""One-time frozen-fixture contact preparation; never an online planner step."""
import argparse,json,time,traceback,hashlib,math
from pathlib import Path
from dataclasses import asdict
p=argparse.ArgumentParser()
for n in ('input','source','native-code','assets','seed-plan','output'):p.add_argument('--'+n,type=Path,required=True)
p.add_argument('--grasp-height-offset-mm',type=float,default=0.,help='offline diagnostic candidate only; formal policy remains unchanged')
a=p.parse_args();a.output.mkdir();started=time.perf_counter();result=dict(passed=False,hardware_access=False,offline_model_preparation=True)
try:
 if not math.isfinite(a.grasp_height_offset_mm) or not 0<=a.grasp_height_offset_mm<=20:raise ValueError('bounded0–20mm diagnostic height required')
 from playground_geometry import load
 from curobo_robot_model import digest
 planner,obs,data=load(a.input,json.loads(a.source.read_text()),a.native_code,a.assets)
 from robot_sim.tasks.manager_based.coffee_make.relative_geometry import GeometryAudit
 from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
 from motiongen_targets import reference_grasp
 from grasp_contact_certificate import source_closing,source_contact_certificate
 from curobo_planning import A1Kinematics
 from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
 source=json.loads(a.source.read_text());parent=a.seed_plan
 seed=json.loads(parent.read_text())['path'][-1]
 initial=dict(planner.home_state,left_joint7=planner.open_position_m,left_joint8=planner.open_position_m)
 _,_,relative=reference_grasp(planner,data,urdf_path=ac_one_asset_spec(a.assets).urdf_path)
 raw=obs.sample.world_from_cup.compose(relative)
 raised=RigidTransform(tuple(v+(a.grasp_height_offset_mm/1000 if i==2 else 0.) for i,v in enumerate(raw.position)),raw.orientation_wxyz)
 goal=planner.geo.mount.inverse().compose(raised)
 solver=A1Kinematics(ac_one_asset_spec(a.assets).urdf_path,planner.fk.movable,initial)
 solved=solver.batch_ik([goal.position],[goal.orientation_wxyz],[seed]);assert bool(solved.success.all()), 'continuous original-height grasp IK failed'
 q=solved.solution.reshape(-1,6)[0].cpu().tolist()
 world_tool=planner.geo.mount.compose(goal)
 lift_world=RigidTransform(tuple(v+(.025 if i==2 else 0.) for i,v in enumerate(world_tool.position)),world_tool.orientation_wxyz)
 lift_goal=planner.geo.mount.inverse().compose(lift_world)
 lifted=solver.batch_ik([lift_goal.position],[lift_goal.orientation_wxyz],[q])
 assert bool(lifted.success.all()), 'local short-lift IK failed'
 lift_q=lifted.solution.reshape(-1,6)[0].cpu().tolist()
 state=dict(planner.home_state,left_joint7=planner.open_position_m,left_joint8=planner.open_position_m,**dict(zip([f'left_joint{i}' for i in range(1,7)],q)))
 print('BEGIN one-time linked-jaw contact preparation',flush=True)
 closing=source_closing(planner,state,obs.sample.world_from_cup,require_passed=False)
 (a.output/'closing.json').write_text(json.dumps(closing,indent=2))
 if not closing.get('passed'):raise ValueError(closing.get('error'))
 result.update(source_contact_certificate(planner,data,closing,obs.sample.world_from_cup))
 result.update(grasp_q_model=q,lift_q_model=lift_q,enrichment_seconds=time.perf_counter()-started,
               grasp_height_offset_mm=a.grasp_height_offset_mm,formal_policy_modified=False)
except Exception as error:result.update(passed=False,error=str(error),traceback=traceback.format_exc())
result['seconds']=time.perf_counter()-started;(a.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
