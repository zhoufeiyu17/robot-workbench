"""Deterministic CPU mock tests; no SDK, robot, Isaac, wall-clock sleeps or network."""
from copy import deepcopy

from frozen_executor import FrozenExecutor
from integration import _digest
from test_app import fixture_messages
from test_harness import rejects


def fixture(delta=.01):
    source, plan, _ = fixture_messages()
    plan["ticks"] = [{"tick": i, "time_s": i/30, "target": deepcopy(plan["ticks"][0]["target"])} for i in range(3)]
    for i, tick in enumerate(plan["ticks"]):
        tick["target"]["left_arm_joints"][0] = i * delta
        tick["target"]["left_gripper"] -= i
    plan.update(timeline_digest=_digest(plan["ticks"]), control={"physics_hz": 240, "substeps": 8},
                native_substeps=[8,8,4], duration_s=20/240, simulation={"validated": True, "outcome": "success"})
    return source, plan


def make(delta=.01):
    source, plan = fixture(delta)
    targets, stops = [], []
    opened = [True]
    follower = FrozenExecutor(plan, source, sink=lambda q: targets.append(deepcopy(q)),
                              gate=lambda: opened[0], stop=stops.append)
    def feedback(now, state=None, stamp=None):
        return {"sample_monotonic_s": now if stamp is None else stamp,
                "state": deepcopy(state if state is not None else targets[-1] if targets else plan["ticks"][0]["target"])}
    return follower, plan, targets, stops, opened, feedback


def main():
    follower, plan, targets, stops, opened, feedback = make()
    before = deepcopy(plan)
    assert follower.start(10., feedback(10.))["status"] == "running"
    # A nominal 100 Hz host repeats the original 30 Hz target without interpolation or retiming.
    for step in range(1,9):
        now = 10. + step/100
        receipt = follower.tick(now, feedback(now))
        assert receipt["status"] == "running", receipt
    assert targets == [plan["ticks"][i]["target"] for i in (0,0,0,0,1,1,1,2,2)]
    assert receipt["targets_submitted"] and not receipt["timeline_elapsed"]
    assert not receipt["final_feedback_qualified"] and not stops
    done = follower.tick(10.09, feedback(10.09))
    assert done["status"] == "completed" and done["final_feedback_qualified"]
    assert done["timeline_elapsed"] and done["duration_s"] == 20/240
    assert done["validation_scope"] == "sim/mock" and done["hardware_validated"] is False
    assert plan == before and done["timeline_digest"] == _digest(plan["ticks"])
    assert follower.tick(20., feedback(20.))["status"] == "completed" and len(targets) == 9
    rejects(lambda _: follower.start(20., feedback(20.)), None)

    # Reproduce a 50 ms host pause with the physical profile's existing 100 ms
    # tolerance. All original targets remain ordered; no burst catch-up follows.
    source,long_plan=fixture(.001)
    source['calibration_profile']['divergence_thresholds']['timing_s']=.1
    long_plan['calibration_profile_digest']=_digest(source['calibration_profile'])
    long_plan['ticks']=[dict(tick=i,time_s=i/30,target=deepcopy(long_plan['ticks'][0]['target'])) for i in range(90)]
    for i,t in enumerate(long_plan['ticks']):t['target']['left_arm_joints'][0]+=.001*i
    long_plan.update(native_substeps=[8]*90,duration_s=3.,timeline_digest=_digest(long_plan['ticks']))
    sent=[];stops=[]
    def sink(target):
        if not sent or sent[-1][0]!=f.command_index:sent.append((f.command_index,now[0],target))
    f=FrozenExecutor(long_plan,source,sink=sink,gate=lambda:True,stop=stops.append)
    now=[0.]
    def fresh():return dict(sample_monotonic_s=now[0],state=sent[-1][2] if sent else long_plan['ticks'][0]['target'])
    f.start(0.,fresh());slipped_at=[]
    for step in range(1,400):
        now[0]+=.05041973799 if step in (50,150) else .01
        previous_slips=f.schedule_slips
        result=f.tick(now[0],fresh())
        if f.schedule_slips>previous_slips:slipped_at.append(f.last_tick)
        assert result['status']!='failed',result
        if result['status']=='completed':break
        assert 'tick_first_send_monotonic_s' not in result, 'steady telemetry grew with timeline'
    assert result['status']=='completed' and slipped_at and not stops
    assert [index for index,_,_ in sent]==list(range(90))
    assert [target for _,_,target in sent]==[t['target'] for t in long_plan['ticks']]
    for index in slipped_at:assert sent[index+1][1]-sent[index][1]>=1/30-1e-9
    assert now[0]+1e-9>=long_plan['duration_s']+f.scheduling_delay_s
    excessive=FrozenExecutor(long_plan,source,sink=lambda _:None,gate=lambda:True,stop=stops.append)
    first=long_plan['ticks'][0]['target'];excessive.start(0.,dict(sample_monotonic_s=0.,state=first))
    failed=excessive.tick(.15,dict(sample_monotonic_s=.15,state=first))
    assert failed['status']=='failed' and 'timing tolerance' in failed['reason']

    # End-of-timeline is distinct from a feedback sample covering that endpoint.
    f, p, sent, stopped, gate, fb = make()
    f.start(0., fb(0.))
    for now in (.01,.02,.03,.04,.05,.06,.07,.08): f.tick(now, fb(now))
    waiting = f.tick(.09, fb(.09, stamp=.08))
    assert waiting["status"] == "awaiting_final_feedback" and not waiting["final_feedback_qualified"]
    assert f.tick(.10, fb(.10))["status"] == "completed"

    f, p, sent, stopped, gate, fb = make()
    f.start(0., fb(0.))
    for now in (.01,.02,.03,.04,.05,.06,.07,.08,.09):
        f.tick(now, fb(now, stamp=min(now,.08)))
    assert f.tick(.11, fb(.11, stamp=.08))["status"] == "failed"
    assert "final feedback deadline" in stopped[0]

    # A gate change between feedback checking and the actual write blocks the sink.
    f, p, sent, stopped, gate, fb = make()
    sequence = iter((True, False))
    f.gate = lambda: next(sequence)
    assert f.start(0., fb(0.))["status"] == "failed" and not sent and len(stopped) == 1

    # Start feedback is healthy but has no submitted target to track yet.
    for offset in (.019,.5):
        f,p,sent,stopped,gate,fb=make()
        initial=deepcopy(p['ticks'][0]['target']);initial['left_arm_joints'][0]+=offset
        result=f.start(0.,fb(0.,state=initial))
        assert result['status']=='running' and result['tracking_error'] is None
        assert sent==[p['ticks'][0]['target']], 'first target changed or was not submitted'
        if offset==.019:
            assert f.tick(.01,fb(.01,state=p['ticks'][0]['target']))['status']=='running'
        else:
            assert f.tick(.01,fb(.01,state=initial))['status']=='failed'
            assert len(sent)==1 and len(stopped)==1, 'tracking guard did not stop nonfollowing start'
    for stamp in (-.2,.01):
        f,p,sent,stopped,gate,fb=make()
        assert f.start(0.,fb(0.,stamp=stamp))['status']=='failed' and not sent
    f,p,sent,stopped,gate,fb=make()
    malformed=fb(0.);malformed['state']['left_arm_joints'][0]=float('nan')
    assert f.start(0.,malformed)['status']=='failed' and not sent

    # Delayed feedback is compared with the command active at its sample timestamp.
    f, p, sent, stopped, gate, fb = make(.2)
    f.start(0., fb(0.))
    old = deepcopy(sent[-1])
    for now in (.01,.02,.03,.04): f.tick(now, fb(now, stamp=min(now,.03)))
    assert f.tick(.05, fb(.05, state=old, stamp=.03))["status"] == "running"

    # Every failure terminates once, with no further sink calls and no restart/catch-up.
    for case in ("cancel", "gate", "rollback", "skip", "late", "stale", "future", "tracking", "gripper", "sample_rollback", "sink"):
        f, p, sent, stopped, gate, fb = make()
        f.start(1., fb(1.))
        before_count = len(sent)
        now, sample = 1.01, fb(1.01)
        if case == "cancel":
            result = f.cancel()
        else:
            if case == "gate": gate[0] = False
            if case == "rollback": now, sample = .99, fb(.99)
            if case == "skip": now, sample = 1.07, fb(1.07)
            if case == "late": now, sample = 1.06, fb(1.06)
            if case == "stale": sample["sample_monotonic_s"] = .8
            if case == "future": sample["sample_monotonic_s"] = 1.02
            if case == "tracking": sample["state"]["left_arm_joints"][0] += .06
            if case == "gripper": sample["state"]["right_gripper"] += 3
            if case == "sample_rollback": sample["sample_monotonic_s"] = .999
            if case == "sink":
                def failed_sink(_): raise RuntimeError("injected sink failed")
                f.sink = failed_sink
            result = f.tick(now, sample)
        assert result["status"] in ("failed", "cancelled"), (case,result)
        assert len(stopped) == 1 and len(sent) == before_count, (case,result)
        f.tick(2., fb(2.)); f.cancel()
        assert len(stopped) == 1 and len(sent) == before_count
        rejects(lambda _: f.start(2., fb(2.)), None)

    # Stop callback failure remains visible; it is not evidence of a physical stop.
    f, p, sent, stopped, gate, fb = make()
    f.start(0., fb(0.))
    def failed_stop(_): raise RuntimeError("stop not acknowledged")
    f.stop = failed_stop
    assert f.cancel()["stop_error"] == "stop not acknowledged"

    # Commands are deep-copied; callers/sinks cannot mutate the frozen schedule.
    f, p, sent, stopped, gate, fb = make()
    p["ticks"][0]["target"]["left_gripper"] = 80
    f.start(0., {"sample_monotonic_s":0.,"state":deepcopy(f.ticks[0]["target"])})
    assert sent[0]["left_gripper"] == 38

    source, plan = fixture()
    for change in ({"native_substeps":[8,4,8]}, {"native_substeps":[8,8,True]}, {"duration_s":.1},
                   {"simulation":{"validated":False,"outcome":"success"}}, {"timeline_digest":"wrong"}):
        rejects(lambda p: FrozenExecutor(p, source, sink=lambda q:None, gate=lambda:True, stop=lambda r:None),
                {**plan, **change})
    print("Frozen executor: unchanged targets, native partial duration, final feedback and fail-closed lifecycle: PASS (CPU mock only)")


if __name__ == "__main__":
    main()
