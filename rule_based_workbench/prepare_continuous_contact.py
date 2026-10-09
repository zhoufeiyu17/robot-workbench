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
 from motiongen_targets import generate_targets
 from curobo_planning import A1Kinematics
 from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
 source=json.loads(a.source.read_text());parent=a.seed_plan
 seed=json.loads(parent.read_text())['path'][-1]
 initial=dict(planner.home_state,left_joint7=planner.open_position_m,left_joint8=planner.open_position_m)
 raw=RigidTransform(**generate_targets(planner,obs,data)['grasp_candidates'][0]['world_from_tool'])
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
 original=planner.geo.audit;parts={name:(link,geometry) for link,name,geometry,_,_ in planner.geo.whole.parts}
 bands={'cup/'+band.shape_path:(band,shape) for band,shape in planner.geo.cup_bands}
 def audit(frames,cup,**options):
  report=original(frames,cup,**options);bad=[]
  for row in report.violations:
   if row[0] in bands and row[1] in parts and parts[row[1]][0] in ('left_link7','left_link8'):
    band,shape=bands[row[0]];link,geometry=parts[row[1]]
    cup_pose=cup.compose(RigidTransform((0,0,(band.z_min_m+band.z_max_m)/2)))
    objs=[planner.geo.fcl.CollisionObject(shape,planner.geo.whole.transform(cup_pose)),planner.geo.fcl.CollisionObject(geometry,planner.geo.whole.transform(planner.geo.mount.compose(frames[link])))]
    _,witnesses=planner.geo.cup_pair(*objs)
    if all(depth<=.003+1e-7 for _,depth in witnesses):continue
   bad.append(row)
  return GeometryAudit(not bad,tuple(bad))
 planner.geo.audit=audit
 print('BEGIN one-time linked-jaw contact preparation',flush=True)
 closing=planner.geo.closing_sweep(planner.fk,state,obs.sample.world_from_cup)
 (a.output/'closing.json').write_text(json.dumps(closing,indent=2))
 if not closing['passed']:raise ValueError(closing.get('error'))
 tool=planner.geo.mount.compose(planner.fk.pose(planner.fk.forward(closing['terminal_state'])['left_link6']))
 actual_cup=RigidTransform(**closing['world_from_cup'])
 result.update(passed=True,grasp_q_model=q,lift_q_model=lift_q,urdf_sha256=data['urdf_sha256'],source_parts_sha256=digest(data['parts']),cup_profile_sha256=planner.cup.profile_sha256,
   terminal_jaw_position_m=closing['terminal_state']['left_joint7'],linked_jaws=closing['terminal_state']['left_joint7']==closing['terminal_state']['left_joint8'],
   original_cup_from_tool=asdict(obs.sample.world_from_cup.inverse().compose(tool)),tool_from_held_cup=asdict(tool.inverse().compose(actual_cup)),
   original_cup_from_held_cup=asdict(obs.sample.world_from_cup.inverse().compose(actual_cup)),claw_cup_allowance_m=.003,physical_force_verified=False,samples=len(closing['samples']))
 from motiongen_plan import world_config
 from cup_contact import cup_geometry_digest
 frames={name:planner.fk.pose(value) for name,value in planner.fk.forward(closing['terminal_state']).items()}
 cup=RigidTransform(**closing['world_from_cup']);maximum=0.;contacts=[]
 for band,shape in planner.geo.cup_bands:
  pose=cup.compose(RigidTransform((0,0,(band.z_min_m+band.z_max_m)/2)))
  obj=planner.geo.fcl.CollisionObject(shape,planner.geo.whole.transform(pose))
  for link,name,geometry,_,_ in planner.geo.whole.parts:
   if link not in ('left_link7','left_link8'):continue
   other=planner.geo.fcl.CollisionObject(geometry,planner.geo.whole.transform(planner.geo.mount.compose(frames[link])))
   _,witnesses=planner.geo.cup_pair(obj,other);depth=max((d for _,d in witnesses),default=0.)
   maximum=max(maximum,depth)
   if depth>0:contacts.append(dict(part=name,band=band.shape_path,penetration_m=depth))
 world,_,_=world_config(planner,cup)
 result.update(cup_geometry_sha256=cup_geometry_digest(world),maximum_claw_penetration_m=maximum,
  cup_radius_from_origin_m=max(math.hypot(b.radius_m,max(abs(b.z_min_m),abs(b.z_max_m))) for b in planner.cup.bands),
  numerical_contact_reserve_m=1e-5,first_band_center_z_m=(planner.cup.bands[0].z_min_m+planner.cup.bands[0].z_max_m)/2,
  contact_geometry_stops_m=closing['contact_geometry_stops_m'],contacts=contacts,
  certificate_scope='source-mesh terminal contact plus runtime rigid pose/jaw displacement bound; not a force or full-path certificate',
  enrichment_seconds=time.perf_counter()-started)
 result.update(grasp_height_offset_mm=a.grasp_height_offset_mm,formal_policy_modified=False,
  maximum_body_metal_penetration_m=max((c['penetration_m'] for c in contacts if c['band'] in ('/Cup/Collisions/MiddleBody','/Cup/Collisions/UpperBody')),default=0.))
 assert maximum+result['numerical_contact_reserve_m']<.003
 distances=[]
 for band,shape in planner.geo.cup_bands:
  pose=cup.compose(RigidTransform((0,0,(band.z_min_m+band.z_max_m)/2)))
  obj=planner.geo.fcl.CollisionObject(shape,planner.geo.whole.transform(pose))
  for link,name,geometry,_,_ in planner.geo.whole.parts:
   if link!='left_link6':continue
   other=planner.geo.fcl.CollisionObject(geometry,planner.geo.whole.transform(planner.geo.mount.compose(frames[link])))
   hit=planner.geo.fcl.collide(obj,other,planner.geo.fcl.CollisionRequest(),planner.geo.fcl.CollisionResult())
   distance=float(planner.geo.fcl.distance(obj,other));distances.append(dict(part=name,band=band.shape_path,collision=bool(hit),distance_m=distance))
 result['wrist_cup_source_checks']=distances;result['minimum_wrist_clearance_m']=min(d['distance_m'] for d in distances)
 result['wrist_source_separation_proven']=bool(distances) and all(not d['collision'] and d['distance_m']>1e-5 for d in distances)
 assert result['wrist_source_separation_proven']
except Exception as error:result.update(passed=False,error=str(error),traceback=traceback.format_exc())
result['seconds']=time.perf_counter()-started;(a.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
