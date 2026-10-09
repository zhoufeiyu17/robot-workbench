"""Bounded real-GPU MotionGen API smoke; no hardware or executable plan output."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time


def run(args):
    from benchmark_planning import write_new
    from curobo_planning import robot_config, LEFT_JOINTS
    from motiongen_motion import MotionGenerator
    from playground_geometry import load
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(passed=False, scope='MotionGen API smoke with skeletal collision proxy; not path safety validation',
                  hardware_access=False, executable_plan=False)
    started = time.perf_counter()
    try:
        source = json.loads(args.source.read_text())
        planner, observation, data = load(args.input, source, args.native_code, args.assets)
        from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
        config, identity = robot_config(ac_one_asset_spec(args.assets).urdf_path,
                                        planner.fk.movable, planner.home_state)
        config['kinematics'].update(collision_link_names=['left_link3', 'left_link6'],
            collision_spheres={name:[dict(center=[0.,0.,0.],radius=.001)]
                               for name in ('left_link3','left_link6')}, collision_sphere_buffer=0.,
            self_collision_ignore={}, self_collision_buffer={'left_link3':0.,'left_link6':0.})
        # The skeletal API fixture has no right/jaw links; a real full-body
        # collision model retains those subtrees and all their locked joints.
        config['kinematics']['lock_joints'] = None
        config['kinematics']['cspace'].update(max_acceleration=2., max_jerk=60.)
        world = dict(cuboid={'far_box':dict(dims=[.1,.1,.1],pose=[10.,10.,10.,1.,0.,0.,0.])})
        engine = MotionGenerator(config, world, use_cuda_graph=not args.no_cuda_graph)
        engine.warmup()
        start = [planner.home_state[n] for n in LEFT_JOINTS]
        goal_q = start.copy(); goal_q[0] += .05; goal_q[1] += .05
        target_state = {**planner.home_state, **dict(zip(LEFT_JOINTS,goal_q))}
        target = planner.fk.pose(planner.fk.forward(target_state)['left_link6'])
        results = []
        for index in range(2):
            path, receipt = engine.plan_pose(start, [asdict(target)], name='pose-'+str(index))
            end = {**planner.home_state,**dict(zip(LEFT_JOINTS,path[-1]))}
            pose = planner.fk.pose(planner.fk.forward(end)['left_link6'])
            import math
            position_error = math.dist(pose.position,target.position)
            quaternion_error = min(math.dist(pose.orientation_wxyz,target.orientation_wxyz),
                                   math.dist(pose.orientation_wxyz,[-v for v in target.orientation_wxyz]))
            assert position_error < 1e-5 and quaternion_error < 1e-5, (position_error,quaternion_error)
            results.append(dict(**receipt,native_fk_position_error_m=position_error,
                                native_fk_quaternion_chord_error=quaternion_error))
        path, receipt = engine.plan_joint(path[-1], start, name='home')
        assert max(abs(a-b) for a,b in zip(path[-1],start)) <= .005
        results.append(receipt)
        report.update(passed=True,initialization_seconds=engine.initialization_seconds,
                      warmup_seconds=engine.warmup_seconds,queries=results,robot_identity=identity)
    except Exception as error:
        import traceback
        report.update(error=f'{type(error).__name__}: {error}',traceback=traceback.format_exc())
        if 'engine' in locals():
            report.update(queries=engine.receipts,initialization_seconds=engine.initialization_seconds,
                          warmup_seconds=engine.warmup_seconds)
    report['total_seconds'] = time.perf_counter()-started
    write_new(args.output/'result.json',report)
    print(json.dumps(report,indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('native-code','assets','input','source','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--no-cuda-graph',action='store_true')
    raise SystemExit(run(parser.parse_args()))
