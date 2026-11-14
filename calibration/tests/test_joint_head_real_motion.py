"""Deterministic callback tests; no hardware or SDK access."""
from dataclasses import replace
import math
import unittest
from arx_env.joint_head_jaw_mapping import feedback_gaps

from arx_env.joint_head_real_motion import (MotionRunner, StationaryWindow, feedback_to_command,
    make_plan, make_workbench_plans, jaw_peak_scale, moving_jaw_indices)


def limits():
    return [[0., .082] if i in (6, 13) else [-2., 2.] for i in range(14)]


def feedback(t, q, *, v=None, sdk=None):
    # Both SDK epochs deliberately differ from the host and each other.
    return dict(qpos=list(q), qvel=list([0.] * 14 if v is None else v),
                sample_time_s=t, sdk_timestamps=list([600. + t - 8500., 90. + t - 8500.] if sdk is None else sdk))


class RealMotionTest(unittest.TestCase):
    def test_motion_record_is_bounded_and_does_not_relabel_cached_native_timestamps(self):
        q=[0.]*14
        plan=make_plan('right',q,q,limits(),duration_s=20.,created_at_s=8500.)
        runner=MotionRunner(plan,begin=lambda q:q,send=lambda q:None,stop=lambda reason:None)
        state=feedback(8500.,q);state['controller_telemetry']={'sample_wall_ns':123,'arms':{}}
        runner.start(8500.,state)
        for n in range(1,1201):
            state=feedback(8500.+n*.01,q);state['controller_telemetry']={'sample_wall_ns':123,'arms':{}}
            runner.tick(8500.+n*.01,state)
        record=runner.motion_record()
        self.assertEqual(len(record['rows']),1024);self.assertTrue(record['truncated'])
        self.assertEqual(record['sample_count'],1201);self.assertEqual(len(record['native_samples']),1)
        self.assertEqual(record['native_samples'][0]['sample_wall_ns'],123)
        self.assertEqual(record['columns'][-1],'previous_command14')

    def test_disabled_jaw_keeps_other_tracking_guards_and_fault_evidence(self):
        for failed_axis in (0,13):
            q = [0.]*14
            plan = make_plan('both',q,q,limits(),kind='workbench',duration_s=1.,created_at_s=8500.)
            stopped = []
            runner = MotionRunner(plan,begin=lambda q:q,send=lambda q:None,stop=stopped.append,
                                  disabled_grippers=('left',))
            runner.start(8500.,feedback(8500.,q))
            for n in range(1,30):
                if n > 10:q[failed_axis] = .1 if failed_axis == 0 else .02
                runner.tick(8500.+n*.05,feedback(8500.+n*.05,q))
                if runner.status == 'FAULT':break
            self.assertEqual(runner.status,'FAULT')
            self.assertIn('measured arm=',runner.reason)
            self.assertTrue(stopped)
            self.assertNotIn('left',runner.target_error['jaw_mm'])
            metric = 'joint_rad' if failed_axis == 0 else 'jaw_mm'
            self.assertGreater(runner.target_error['tracking'][metric], .08 if failed_axis == 0 else 4.)

    def test_reviewed_two_hundredths_arrival_still_rejects_larger_error(self):
        for error, expected in ((.0151, 'READY'), (.0201, 'FAULT')):
            q = [0.] * 14
            plan = make_plan('both', q, q, limits(), kind='workbench', duration_s=1., created_at_s=8500.)
            runner = MotionRunner(plan, begin=lambda q: q, send=lambda q: None,
                                  stop=lambda reason: None, arm_tolerance=.02)
            runner.start(8500., feedback(8500., q))
            for n in range(1, 95):
                if n > 8:
                    q[10] = error
                runner.tick(8500.+n*.05, feedback(8500.+n*.05, q))
            self.assertEqual(runner.status, expected)

    def test_twelve_arrival_tolerances_are_independent_and_continue_per_joint(self):
        from arx_env.joint_head_real_motion import active_indices
        tolerances=[.005+.003*i for i in range(12)]
        for j,index in enumerate(active_indices('both')):
            for delta,expected in ((-.001,'READY'),(.001,'FAULT')):
                with self.subTest(joint=j,delta=delta):
                    q=[0.]*14
                    plan=make_plan('both',q,q,limits(),kind='workbench',duration_s=1.,created_at_s=8500.)
                    runner=MotionRunner(plan,begin=lambda q:q,send=lambda q:None,stop=lambda reason:None,
                                        arm_tolerances=tolerances)
                    runner.start(8500.,feedback(8500.,q))
                    for n in range(1,95):
                        if n>8:q[index]=tolerances[j]+delta
                        runner.tick(8500.+n*.05,feedback(8500.+n*.05,q))
                    self.assertEqual(runner.status,expected)
                    self.assertEqual(plan.q_target[index],0.)  # Original target is never rebased.
                    runner.continuation=True
                    if delta>0:
                        with self.assertRaisesRegex(ValueError,'start pose drifted'):runner._check_start_drift(q)
                    else:runner._check_start_drift(q)

    def test_arrival_tolerance_vector_validation_and_scalar_compatibility(self):
        from arx_env.joint_head_real_motion import arrival_tolerances
        self.assertEqual(arrival_tolerances(.02),(.02,)*12)
        self.assertEqual(arrival_tolerances(arm_tolerances=[.05]*12),(.05,)*12)
        for values in ([.01]*11,[.01]*13,*[[v]+[.01]*11 for v in (0.,-.01,.050001,float('nan'),float('inf'),True)]):
            with self.assertRaises(ValueError):arrival_tolerances(arm_tolerances=values)

    def test_measured_endpoint_tolerance_keeps_commands_bounded(self):
        from arx_env.joint_head_jaw_mapping import FEEDBACK, GAP_MM, command_sdk, interpolate
        for gap in (-2.000001,-2., -.5, 83.74, 85.019,87.,89.,89.000001):
            start = [0.] * 14
            for side, i in [('left', 6), ('right', 13)]:
                start[i] = interpolate(gap, GAP_MM[side], FEEDBACK[side])
            plan = make_workbench_plans(start, [dict(q12=[0.] * 12, speeds12=[.2] * 12,
                                                    jaw_gap_mm=[0, 0])], limits())[0]
            self.assertEqual(plan.q_start, tuple(start))
            held = feedback_to_command(start)
            for side, i in [('left', 6), ('right', 13)]:
                self.assertEqual(held[i], command_sdk(side, min(83., max(0., gap))))
        for gap in (-2.001, 89.001):
            start[6] = interpolate(gap, GAP_MM['left'], FEEDBACK['left'])
            with self.assertRaisesRegex(ValueError, 'feedback tolerance'):
                feedback_to_command(start)
        for gap in (-2.,83.1,87.,89.):
            with self.assertRaises(ValueError):
                command_sdk('left', gap)
        self.assertEqual(command_sdk('left',83.00000229259817),command_sdk('left',83.))
        self.assertEqual(command_sdk('left',-.000001),command_sdk('left',0.))

    def test_numeric_arm_boundary_preserves_raw_and_seals_projected_request(self):
        bounds = limits();bounds[8] = [0., 2.]
        raw = [0.]*14;raw[8] = -2.0976735626e-10
        command = feedback_to_command(raw, bounds)
        self.assertEqual(command[8], 0.)
        self.assertLess(raw[8], 0.)
        plan = make_workbench_plans(raw, [dict(q12=[0.]*12, speeds12=[.2]*12)], bounds, created_at_s=8500.)[0]
        self.assertEqual(plan.q_start[8], raw[8])
        self.assertEqual(plan.q_target[8], 0.)
        sent=[]
        runner=MotionRunner(plan, begin=lambda q:q, send=sent.append, stop=lambda reason:None)
        runner.start(8500., feedback(8500.,raw))
        for i in range(1,25):runner.tick(8500.+i*.05,feedback(8500.+i*.05,raw))
        self.assertEqual(runner.status,'READY')
        self.assertTrue(sent)
        self.assertTrue(all(q[8]>=0. for q in sent))
        target = raw[:];target[0] = .01
        held = make_plan('left', raw, target, bounds)
        self.assertEqual(held.q_target[8], 0.)
        MotionRunner(held, begin=lambda q:q, send=lambda q:None, stop=lambda reason:None)
        projected=make_plan('both', raw, raw, bounds, kind='workbench')
        self.assertEqual(projected.requested_q_target,tuple(raw));self.assertEqual(projected.q_target[8],0.)
        self.assertTrue(projected.position_corrections)
        inside = raw[:];inside[8] = -math.radians(1.5)
        self.assertEqual(feedback_to_command(inside,bounds)[8],0.)
        self.assertLess(inside[8],0.,'raw feedback must not be clipped')
        too_far = raw[:];too_far[8] = -math.radians(5.)-.00001
        with self.assertRaisesRegex(ValueError, 'position-boundary'):
            make_workbench_plans(too_far, [dict(q12=[0.]*12, speeds12=[.2]*12)], bounds)
        with self.assertRaisesRegex(ValueError, 'outside SDK limit'):
            make_plan('both', raw, [0.]*14, bounds, kind='workbench', q_start_is_command=True)

    def test_explicit_environment_tolerance_and_reviewed_projection_are_frozen(self):
        for environment,tolerance in (('fake',.005),('physical',.01)):
            bounds=limits();bounds[0]=[0.,2.]
            raw=[0.]*14;raw[0]=-tolerance
            requested=raw[:];requested[0]=-tolerance/2
            plan=make_plan('both',raw,requested,bounds,kind='workbench',environment=environment,created_at_s=8500.)
            self.assertEqual(plan.q_start,tuple(raw));self.assertEqual(plan.requested_q_target,tuple(requested))
            self.assertEqual(plan.q_target[0],0.)
            self.assertEqual({row['source'] for row in plan.position_corrections},{'q_start','q_target'})
            runner=MotionRunner(plan,begin=lambda q:q,send=lambda q:None,stop=lambda reason:None,environment=environment)
            self.assertEqual(runner.start(8500.,feedback(8500.,raw)),'VERIFYING')
            self.assertEqual(runner.feedback_position_corrections[0]['original_rad'],-tolerance)
            self.assertEqual(raw[0],-tolerance)
            with self.assertRaisesRegex(ValueError,'environment differs'):
                MotionRunner(plan,begin=lambda q:q,send=lambda q:None,stop=lambda reason:None,
                    environment='physical' if environment=='fake' else 'fake')
            altered=list(plan.requested_q_target);altered[0]=-tolerance/3
            with self.assertRaisesRegex(ValueError,'canonical contents'):
                MotionRunner(replace(plan,requested_q_target=tuple(altered)),begin=lambda q:q,send=lambda q:None,stop=lambda reason:None)
            beyond=raw[:];beyond[0]=-tolerance-.00001
            beyond_feedback=raw[:];beyond_feedback[0]=-(math.radians(5.)if environment=='physical'else tolerance)-.00001
            with self.assertRaisesRegex(ValueError,'position-boundary'):
                make_plan('both',beyond_feedback,requested,bounds,kind='workbench',environment=environment)
            with self.assertRaisesRegex(ValueError,'position-boundary'):
                make_plan('both',[0.]*14,beyond,bounds,kind='workbench',environment=environment)

    def test_constant_sdk_upper_limit_never_rounds_outside_interval(self):
        high=.07633374999999999
        bounds=limits();bounds[0]=[0.,high]
        raw=[0.]*14;raw[0]=high
        target=raw[:];target[1]=.001
        plan=make_plan('both',raw,target,bounds,kind='workbench',duration_s=1.,created_at_s=8500.)
        sent=[]
        def send(q):
            if q[0]!=high:raise ValueError('constant endpoint rounded outside SDK limit')
            sent.append(q)
        runner=MotionRunner(plan,begin=lambda q:q,send=send,stop=lambda reason:None)
        runner.start(8500.,feedback(8500.,raw))
        for i in range(1,10):runner.tick(8500.+i*.05,feedback(8500.+i*.05,raw))
        for i in range(1,151):
            stamp=runner.previous+.005
            runner.tick(stamp,feedback(stamp,raw))
            self.assertNotEqual(runner.status,'FAULT',runner.reason)
        self.assertTrue(sent)

    def test_quintic_endpoint_does_not_emit_a_negative_zero_limit_target(self):
        bounds=limits();bounds[8]=[0.,2.]
        raw=[0.]*14;raw[8]=1e-12
        plan=make_plan('both',raw,[0.]*14,bounds,kind='workbench',duration_s=1.,created_at_s=8500.)
        sent=[]
        def send(q):
            if not 0.<=q[8]<=raw[8]:raise ValueError('interpolation escaped approved endpoints')
            sent.append(q)
        runner=MotionRunner(plan,begin=lambda q:q,send=send,stop=lambda reason:None)
        runner.start(8500.,feedback(8500.,raw))
        for i in range(1,10):runner.tick(8500.+i*.05,feedback(8500.+i*.05,raw))
        self.assertIsNotNone(runner.started)
        while runner.previous < runner.started+.94:
            stamp=runner.previous+.05
            runner.tick(stamp,feedback(stamp,raw))
            self.assertNotEqual(runner.status,'FAULT',runner.reason)
        stamp=runner.started+.999999
        runner.tick(stamp,feedback(stamp,raw))
        self.assertNotEqual(runner.status,'FAULT',runner.reason)
        self.assertGreater(len(sent),5)
        self.assertTrue(all(0.<=q[8]<=raw[8] for q in sent))

    def test_both_jaws_share_trajectory_and_preserve_arms(self):
        start = [.01] * 14
        start[6], start[13] = .076, .077
        target = start.copy()
        target[6], target[13] = .0205, .0195
        plan = make_plan('both', start, target, limits(), kind='jaw_target')
        self.assertEqual(plan.active_arm, 'both')
        self.assertEqual(plan.q_target[6:14:7], (.0205, .0195))
        self.assertGreaterEqual(plan.duration_s, 1.875 * (.077-.0195) / .010)
        target[0] += .001
        with self.assertRaisesRegex(ValueError, 'arm joints unchanged'):
            make_plan('both', start, target, limits(), kind='jaw_target')

    def test_integrated_waypoints_no_arbitrary_caps_and_speed_hash(self):
        points = [dict(q12=[1.] * 12, speeds12=[2.] * 12, jaw_gap_mm=[83,83], jaw_speed_mm_s=40)] * 20
        plans = make_workbench_plans([0.] * 14, points, limits())
        self.assertEqual(len(plans), 20)
        self.assertEqual((plans[0].q_target[6], plans[0].q_target[13]), (.0788,.0773))
        self.assertEqual(plans[0].jaw_speed_mm_s, 40)
        self.assertLess(plans[1].duration_s, .5)
        slow = make_workbench_plans([0.] * 14, [dict(q=[1.]*12, speeds=[.0001]*12)], limits())
        self.assertGreater(slow[0].duration_s, 120)
        for bad in (0, -1, math.inf, math.nan, True):
            with self.assertRaises(ValueError):
                make_plan('both', [0.]*14, [0.]*14, limits(), kind='workbench', jaw_speed_mm_s=bad)
        with self.assertRaisesRegex(ValueError, 'hash'):
            MotionRunner(replace(plans[0], jaw_speed_mm_s=80), begin=lambda q:q, send=lambda q:None, stop=lambda why:None)
        with self.assertRaisesRegex(ValueError, 'unsupported'):
            make_plan('left', [0.]*14, [0.]*14, limits(), kind='jaw_probe')

    def test_two_mm_arrival_uses_distinct_command_and_feedback_mapping(self):
        from arx_env.joint_head_jaw_mapping import command_sdk, FEEDBACK, GAP_MM, interpolate
        start = [0.] * 14
        target = start.copy()
        target[6], target[13] = command_sdk('left',83), command_sdk('right',83)
        plan = make_plan('both', start, target, limits(), kind='jaw_target', jaw_speed_mm_s=1000, created_at_s=8500.)
        runner = MotionRunner(plan, begin=lambda q:q, send=lambda q:None, stop=lambda why:None)
        self.start_running(runner,start)
        measured = target.copy()
        for side, i in [('left',6), ('right',13)]:
            measured[i] = interpolate(81.1, GAP_MM[side], FEEDBACK[side])
        self.tick_until(runner, runner.started + plan.duration_s + 1., measured)
        self.assertEqual(runner.status,'READY')
        self.assertAlmostEqual(runner.target_error['jaw_mm']['left'], -1.9)
        window = StationaryWindow()
        for n in range(7):
            q = [0.] * 14
            q[6] = interpolate(1.9 if n%2 else 0, GAP_MM['left'], FEEDBACK['left'])
            ready = window.update(8500+n*.05,q,[0.]*14)
        self.assertTrue(ready)

    def setup_runner(self, *, delta=.01, consumed=None):
        start = [0.] * 14
        start[6], start[13] = .03, .04
        target = start.copy()
        target[7] += delta
        plan = make_plan("right", start, target, limits(), created_at_s=8500.)
        calls = []
        runner = MotionRunner(plan, begin=lambda q: calls.append(("begin", q)),
                              send=lambda q: calls.append(("send", q)),
                              stop=lambda why: calls.append(("stop", why)), consumed=consumed)
        return runner, calls, start, target

    def test_target_plan_keeps_active_arm_motion(self):
        runner, calls, start, target = self.setup_runner()
        self.start_running(runner, start)
        self.assertEqual(runner.execution_target[7], target[7])
        self.tick_until(runner, runner.started + runner.plan.duration_s, target)
        self.assertIn(runner.status, ("RUNNING", "SETTLING"))
        self.assertAlmostEqual(calls[-1][1][7], target[7])

    def start_running(self, runner, start):
        self.assertEqual(runner.start(8500., feedback(8500., start)), "VERIFYING")
        self.assertEqual(runner.tick(8500.05, feedback(8500.05, start)), "VERIFYING")
        self.assertEqual(self.tick_until(runner, 8500.31, start), "RUNNING")

    def tick_until(self, runner, end, q, *, v=None):
        count = math.ceil((end - runner.previous) / .04)
        begin = runner.previous
        for i in range(1, count + 1):
            t = begin + (end - begin) * i / count
            runner.tick(t, feedback(t, q, v=v))
        return runner.status

    def test_limits_hash_jaws_single_arm_and_peak(self):
        runner, _, start, target = self.setup_runner(delta=.15)
        plan = runner.plan
        self.assertAlmostEqual(plan.duration_s, 1.875 * .15 / .2)
        for index in (0, 6, 13):
            changed = target.copy(); changed[index] += .001
            with self.assertRaisesRegex(ValueError, "inactive arm and jaws"):
                make_plan("right", start, changed, limits())
        for value in (2.01001, math.nan, math.inf, True):
            changed = target.copy(); changed[7] = value
            with self.assertRaises(ValueError):
                make_plan("right", start, changed, limits())
        for kwargs in (dict(duration_s=.5), dict(max_velocity=0), dict(duration_s=0)):
            with self.assertRaises(ValueError):
                make_plan("right", start, target, limits(), **kwargs)
        for forged in (replace(plan, sha256="0" * 64), replace(plan, duration_s=.01),
                       replace(plan, q_target=tuple([1.] * 14))):
            with self.assertRaises(ValueError):
                MotionRunner(forged, begin=lambda q: None, send=lambda q: None, stop=lambda r: None)

    def test_home_exact_zero_single_arm_lower_speed_no_kind_bypass(self):
        start = [.3, -.36, .1, -.1, .05, .6, .04, .2, -.1, .15, .05, 0., -.1, .03]
        target = start.copy()
        target[:6] = [0.] * 6
        target[6] = .0788
        home = make_plan("left", start, target, limits(), kind="home", created_at_s=8500.)
        self.assertEqual(home.max_velocity, .2)
        self.assertAlmostEqual(home.duration_s, max(1.875 * .6 / .2,
            1.875 * (home.q_target[6] - feedback_to_command(start)[6]) * jaw_peak_scale("left") / 10))
        self.assertEqual(home.q_target[7:], tuple(start[7:]))
        self.assertEqual(home.q_target[6], .0788)
        self.assertNotEqual(home.sha256, make_plan("left", start, target, limits(),
                            kind="home", max_velocity=.01, created_at_s=8500.).sha256)
        make_plan("left", start, target[:6]+[start[6]]+target[7:], limits())
        make_plan("left", start, target, limits(), kind="home", max_velocity=2.)
        nonzero = target.copy(); nonzero[1] = 1e-12
        with self.assertRaisesRegex(ValueError, "exactly zero"):
            make_plan("left", start, nonzero, limits(), kind="home")
        for index in (7, 13):
            changed = target.copy(); changed[index] += .001
            with self.assertRaisesRegex(ValueError, "inactive arm and jaws"):
                make_plan("left", start, changed, limits(), kind="home")
        too_far = start.copy(); too_far[0] = .601
        make_plan("left", too_far, target, limits(), kind="home")
        restrictive = limits(); restrictive[0] = [.01, 2.]
        with self.assertRaisesRegex(ValueError, "exactly zero"):
            make_plan("left", start, target, restrictive, kind="home")
        forged = replace(home, kind="target")
        with self.assertRaises(ValueError):
            MotionRunner(forged, begin=lambda q: None, send=lambda q: None, stop=lambda r: None)

    def test_independent_sdk_epochs_and_no_command_before_verified(self):
        runner, calls, start, _ = self.setup_runner()
        self.assertEqual(runner.start(8500., feedback(8500., start)), "VERIFYING")
        self.assertEqual(calls, [])
        self.assertEqual(runner.tick(8500.05, feedback(8500.05, start)), "VERIFYING")
        self.assertEqual(calls, [])
        self.assertEqual(self.tick_until(runner, 8500.31, start), "RUNNING")
        self.assertEqual([x[0] for x in calls], ["begin", "send"])
        self.assertEqual((calls[0][1][6], calls[0][1][13]), (.03, .04))
        self.assertEqual((calls[1][1][6], calls[1][1][13]), feedback_to_command(start)[6::7])

    def test_measured_start_has_no_jump_and_rejects_unreviewed_drift(self):
        runner, calls, start, _ = self.setup_runner()
        measured = start.copy(); measured[7] += .0005; measured[13] += .0001
        self.start_running(runner, measured)
        self.assertEqual(calls[0][1], tuple(measured))
        self.assertEqual(calls[1][1], feedback_to_command(measured))
        runner, calls, start, _ = self.setup_runner()
        measured = start.copy(); measured[7] += math.radians(2.01)
        runner.start(8500., feedback(8500., measured))
        self.assertEqual(runner.status, 'FAULT')
        self.assertEqual([c[0] for c in calls], ['stop'])

    def test_rebase_and_queue_continuation_use_review_and_arrival_tolerances(self):
        runner, calls, start, _ = self.setup_runner()
        measured = start.copy(); measured[7] += .009
        self.start_running(runner, measured)
        self.assertEqual(calls[1][1], feedback_to_command(measured))
        runner, calls, start, _ = self.setup_runner()
        measured = start.copy(); measured[7] += math.radians(1.99)
        self.assertEqual(runner.start(8500., feedback(8500., measured), reviewed_deadline=8530.), 'VERIFYING')
        self.tick_until(runner,8500.31,measured)
        self.assertEqual(runner.status,'RUNNING')
        self.assertEqual(calls[1][1],feedback_to_command(measured))

    def test_workbench_repeated_waypoint_keeps_command_domain(self):
        points = [dict(q12=[0.] * 12, speeds12=[1.] * 12, jaw_gap_mm=[83., 83.]),
                  dict(q12=[0.] * 12, speeds12=[1.] * 12, jaw_gap_mm=[83., 83.])]
        plans = make_workbench_plans([0.] * 14, points, limits())
        self.assertEqual(moving_jaw_indices(plans[0]), (6, 13))
        self.assertEqual(moving_jaw_indices(plans[1]), (6, 13))
        self.assertFalse(plans[0].q_start_is_command)
        self.assertTrue(plans[1].q_start_is_command)
        self.assertEqual(plans[0].q_target, plans[1].q_start)
        self.assertEqual(plans[1].q_target[6], .0788)
        self.assertEqual(plans[1].q_target[13], .0773)

    def test_both_home_speed_bounds_and_either_side_fault(self):
        start=[.5,0.,0.,0.,0.,0.,.02,-.4,0.,0.,0.,0.,0.,.03]
        target=[.0788 if i == 6 else .0773 if i == 13 else 0. for i in range(14)]
        plan=make_plan('both',start,target,limits(),kind='both_home',created_at_s=8500.)
        self.assertEqual(plan.active_arm,'both')
        for i in range(14):
            speed=1.875*abs(target[i]-start[i])/plan.duration_s
            self.assertLessEqual(speed,10/jaw_peak_scale("left" if i == 6 else "right") if i in (6,13) else .2)
        with self.assertRaises(ValueError): make_plan('both',start,target,limits(),kind='home')
        for bad_index in (0,7,6,13):
            calls=[]
            runner=MotionRunner(plan,begin=lambda q:q,send=lambda q:calls.append(q),stop=lambda why:calls.append(why))
            self.start_running(runner,start)
            bad=start.copy();bad[bad_index]+=.09 if bad_index not in (6,13) else .005
            self.tick_until(runner,8500.6,bad)
            self.assertEqual(runner.status,'FAULT')
            self.assertIn('tracking error',runner.reason)

    def test_frozen_sdk_for_either_arm_cannot_begin(self):
        for arm in (0, 1):
            runner, calls, start, _ = self.setup_runner()
            runner.start(8500., feedback(8500., start))
            for dt in (.05, .09, .12):
                sdk = [600. + dt, 90. + dt]
                sdk[arm] = (600., 90.)[arm]
                runner.tick(8500. + dt, feedback(8500. + dt, start, sdk=sdk))
            self.assertEqual(runner.status, "FAULT")
            self.assertIn("SDK feedback stopped", runner.reason)
            self.assertEqual([x[0] for x in calls], ["stop"])

    def test_expiry_cancel_and_shared_replay_registry(self):
        consumed = set()
        runner, calls, start, _ = self.setup_runner(consumed=consumed)
        self.assertEqual(runner.cancel(), "STOPPED")
        self.assertIn(runner.plan.sha256, consumed)
        another = MotionRunner(runner.plan, begin=lambda q: None, send=lambda q: None,
                               stop=lambda r: None, consumed=consumed)
        with self.assertRaisesRegex(ValueError, "consumed"):
            another.start(8500., feedback(8500., start))
        runner, _, start, _ = self.setup_runner()
        self.assertEqual(runner.start(8530.01, feedback(8530.01, start)), "FAULT")
        self.assertIn("expired", runner.reason)
        runner, calls, start, _ = self.setup_runner()
        self.start_running(runner, start)
        runner.cancel()
        count = len(calls)
        runner.tick(8500.35, feedback(8500.35, start))
        self.assertEqual(len(calls), count)

    def test_runtime_host_sdk_limits_inactive_and_jaw_faults(self):
        for kind in ("host", "sdk", "inactive", "jaw", "limits"):
            runner, calls, start, _ = self.setup_runner()
            self.start_running(runner, start)
            state = feedback(8500.35, start)
            expected = kind
            if kind == "host":
                state["sample_time_s"] -= .101
            elif kind == "sdk":
                state["sdk_timestamps"][0] = 500.
                expected = "SDK feedback clock regressed"
            elif kind == "inactive":
                state["qpos"][0] += math.radians(2.01)
            elif kind == "jaw":
                state["qpos"][13] += .003
            else:
                state["qpos"][7] = 2.+math.radians(5.)+.00001
                expected = "position-boundary"
            self.assertEqual(runner.tick(8500.35, state), "FAULT")
            self.assertIn(expected, runner.reason)
            self.assertEqual(calls[-1][0], "stop")

    def test_sustained_tracking_fault_not_tick_gap(self):
        runner, calls, start, _ = self.setup_runner()
        self.start_running(runner, start)
        bad = start.copy(); bad[7] = -.10
        for dt in (.35, .39, .43, .47, .51):
            self.assertIn(runner.tick(8500. + dt, feedback(8500. + dt, bad)), ("RUNNING", "SETTLING"))
        self.assertEqual(runner.tick(8500.56, feedback(8500.56, bad)), "FAULT")
        self.assertIn("tracking error", runner.reason)
        self.assertEqual(calls[-1][0], "stop")

    def test_settle_reset_ready_holds_without_sends_and_monitors(self):
        runner, calls, start, target = self.setup_runner()
        self.start_running(runner, start)
        self.tick_until(runner, runner.started + runner.plan.duration_s + .2, target)
        self.assertEqual(runner.status, "SETTLING")
        v = [0.] * 14; v[7] = .07
        # Stationarity intentionally uses measured position, not noisy SDK qvel.
        moving = target.copy(); moving[7] += math.radians(2.01)
        self.tick_until(runner, runner.previous + .04, moving, v=v)
        self.assertIsNone(runner.settled_since)
        self.tick_until(runner, runner.previous + .45, target)
        self.assertEqual(runner.status, "SETTLING")
        self.tick_until(runner, runner.previous + .42, target)
        self.assertEqual(runner.status, "READY")
        sends = len(calls)
        self.tick_until(runner, runner.previous + .2, target)
        self.assertEqual(runner.status, "READY")
        self.assertEqual(len(calls), sends)
        drift = target.copy(); drift[7] += math.radians(2.01)
        self.tick_until(runner, runner.previous + .04, drift)
        self.assertEqual(runner.status, "HOLD_UNSETTLED")
        self.assertEqual(len(calls), sends)
        self.tick_until(runner, runner.previous + .9, target)
        self.assertEqual(runner.status, "READY")
        self.assertEqual(len(calls), sends)
        drift[13] += .003
        self.tick_until(runner, runner.previous + .04, drift)
        self.assertEqual(runner.status, "FAULT")
        self.assertIn("jaw drift", runner.reason)

    def test_settle_deadline_and_hold_recovery_deadline(self):
        runner, _, start, target = self.setup_runner()
        self.start_running(runner, start)
        v = [0.] * 14; v[7] = .07
        outside = target.copy(); outside[7] += .06
        self.tick_until(runner, runner.started + runner.plan.duration_s + 3.1, outside, v=v)
        self.assertEqual(runner.status, "FAULT")
        self.assertIn("settle deadline", runner.reason)
        self.assertIn(f"arrival_tolerance={math.radians(2.):.6f} rad", runner.reason)
        self.assertIn("not joint position limit", runner.reason)
        self.assertEqual(runner.target_error['arrival_deadline']['failure_kind'],'target_tolerance_exceeded')
        runner, _, start, target = self.setup_runner()
        self.start_running(runner, start)
        self.tick_until(runner, runner.started + runner.plan.duration_s + 1., target)
        self.assertEqual(runner.status, "READY")
        drift = target.copy(); drift[7] += .06
        v = [0.] * 14; v[7] = .07
        self.tick_until(runner, runner.previous + 3.2, drift, v=v)
        self.assertEqual(runner.status, "HOLD_UNSETTLED")
        self.assertEqual(runner.reason, "")
        drift[7] += .1
        self.tick_until(runner, runner.previous + .3, drift)
        self.assertEqual(runner.status, "FAULT")
        self.assertIn("tracking error", runner.reason)

    def test_deadline_reports_confirmation_failure_when_current_error_is_inside_tolerance(self):
        runner,_,start,target=self.setup_runner();self.start_running(runner,start)
        end=runner.started+runner.plan.duration_s+3.0
        outside=target.copy();outside[7]+=.0501
        self.tick_until(runner,end-.04,outside)
        # Model the observed threshold chatter while retaining the same quiet position window.
        runner.stationary.update=lambda *args:True
        self.tick_until(runner,end+.001,target)
        self.assertEqual(runner.status,'FAULT','diagnostics must not change the deadline or success policy')
        detail=runner.target_error['arrival_deadline']
        self.assertTrue(detail['instantaneous_joints_in_tolerance']);self.assertTrue(detail['stationary'])
        self.assertEqual(detail['failure_kind'],'arrival_not_confirmed_before_deadline')
        self.assertLess(detail['continuous_confirmation_s'],.5)
        self.assertIn('continuous_confirmation=',runner.reason)

    def test_noisy_velocity_requires_stationary_position_window(self):
        runner, calls, start, target = self.setup_runner()
        noisy = [0.04 if i not in (6, 13) else .001 for i in range(14)]
        self.assertEqual(runner.start(8500., feedback(8500., start, v=noisy)), "VERIFYING")
        for n in range(1, 7):
            noisy = [-value for value in noisy]
            q = start.copy()
            q[7] += .00019 if n % 2 else -.00019
            status = runner.tick(8500. + .05 * n, feedback(8500. + .05 * n, q, v=noisy))
            if n < 6:
                self.assertEqual(status, "VERIFYING")
                self.assertEqual(calls, [])
        self.assertEqual(runner.status, "RUNNING")
        self.tick_until(runner, runner.started + runner.plan.duration_s + 1., target, v=noisy)
        self.assertEqual(runner.status, "READY")
        window = StationaryWindow()
        for n in range(8):
            q = start.copy(); q[7] += math.radians(2.1) * n / 6
            self.assertFalse(window.update(8500. + .05 * n, q, noisy))

    def test_stationary_diagnostic_preserves_failure_evidence_and_units(self):
        q, v = [0.] * 14, [0.] * 14
        window = StationaryWindow()
        for n in range(7):
            self.assertEqual(window.update(n * .05, q, v), n == 6)
        d = window.last_diagnostic
        self.assertEqual(d['reasons'], ['stationary'])
        self.assertAlmostEqual(d['window_duration_s'], .3)
        self.assertEqual(d['sample_count'], 7)
        self.assertEqual(d['position_span'], [0.] * 14)
        self.assertFalse(window.update(.41, q, v))
        d = window.last_diagnostic
        self.assertEqual(d['reasons'], ['sample_gap', 'window_not_ready'])
        self.assertAlmostEqual(d['sample_gap_s'], .11)
        self.assertEqual(d['reset_window']['sample_count'], 7)
        self.assertEqual(d['sample_count'], 1)
        self.assertFalse(window.update(.41, q, v))
        self.assertEqual(window.last_diagnostic['reason'], 'nonmonotonic')
        self.assertEqual(window.last_diagnostic['sample_gap_s'], 0.)
        moved = q.copy(); moved[3] = math.radians(2.01); moved[6] = .0021
        self.assertFalse(window.update(.45, moved, v))
        d = window.last_diagnostic
        self.assertEqual(d['reason'], 'position_span')
        self.assertEqual(d['position_exceeded_indices'], [3, 6])
        self.assertEqual(d['sample_count'], 2)
        self.assertEqual(len(window.samples), 1)
        self.assertAlmostEqual(d['position_span'][3], math.radians(2.01))
        self.assertAlmostEqual(d['position_span'][6], abs(feedback_gaps(moved)[0] - feedback_gaps(q)[0]))
        self.assertEqual(d['position_units'][6], 'mm')
        self.assertEqual(d['raw_qpos_units'][6], 'SDK_units')
        v[8] = -.061; v[13] = .0051
        self.assertFalse(window.update(.49, moved, v))
        d = window.last_diagnostic
        self.assertEqual(d['reason'], 'window_not_ready')
        self.assertFalse(d['velocity_guard_enabled'])
        self.assertEqual(d['velocity_exceeded_indices'], [8, 13])
        self.assertEqual(d['velocity_units'][13], 'SDK_units/s')
        self.assertEqual(d['raw_qpos'], moved)
        self.assertEqual(d['raw_qvel'], v)
        self.assertEqual(d['sample_count'], 2)
        self.assertEqual(len(window.samples), 2)
        window.reset()
        self.assertIs(window.last_diagnostic, d)

    def test_stationary_position_span_two_degree_boundary(self):
        for axis in (*range(6), *range(7,13)):
            for span, expected in ((.0034332275390625, True), (math.radians(2.), True), (math.radians(2.0001), False)):
                window = StationaryWindow()
                for n in range(7):
                    q = [0.] * 14
                    q[axis] = span if n == 6 else 0.
                    ready = window.update(n * .05, q, [0.] * 14)
                self.assertEqual(ready, expected)
                self.assertEqual(window.last_diagnostic['position_thresholds'][axis], math.radians(2.))

    def test_stationary_window_gaps_jaws_and_velocity_is_diagnostic(self):
        q = [0.] * 14
        v = [0.] * 14
        window = StationaryWindow()
        for n in range(7):
            ready = window.update(8500. + n * .05, q, v)
        self.assertTrue(ready)
        self.assertFalse(window.update(8500.41, q, v))  # >.1 s gap resets history.
        for n in range(1, 6):
            self.assertFalse(window.update(8500.41 + n * .05, q, v))
        self.assertTrue(window.update(8500.71, q, v))
        moved_jaw = q.copy(); moved_jaw[6] = .0021
        self.assertFalse(window.update(8500.75, moved_jaw, v))
        v[0] = .061
        self.assertFalse(window.update(8500.79, moved_jaw, v))
        runner, calls, start, _ = self.setup_runner()
        self.assertEqual(runner.start(8500., feedback(8500., start, v=v)), "VERIFYING")
        self.tick_until(runner, 8503.05, start, v=v)
        self.assertEqual(runner.status, "READY")
        self.assertNotIn("stop", [x[0] for x in calls])


if __name__ == "__main__":
    unittest.main()
