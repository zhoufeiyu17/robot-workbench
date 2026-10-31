"""Research-only J3 accumulation schedule: real review path, continuity and Stop."""
from pathlib import Path
import hashlib,json,sys,unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
import test_precision_study as study_fixture
import test_target_interpreter as interpreter_fixture
from robot_workbench.motion_reference import ReferenceIntegral,ReferenceFeedforward,load_feedforward,FEEDFORWARD_SLEW_LIMITS
from robot_workbench.target_interpreter import TargetInterpreter


class J3IntegralPauseTest(unittest.TestCase):
    profile='x5_control50_damping_v1'
    setUp=study_fixture.PrecisionStudyTest.setUp
    cleanup=study_fixture.PrecisionStudyTest.cleanup
    create=study_fixture.PrecisionStudyTest.create
    advance=study_fixture.PrecisionStudyTest.advance
    study=study_fixture.PrecisionStudyTest.study

    def test_weighted_accumulation_preserves_bias_and_other_joints_across_jittered_windows(self):
        original=ReferenceIntegral(rapid=True,rapid_scale=8)
        paused=ReferenceIntegral(rapid=True,rapid_scale=8)
        original.bias[2]=paused.bias[2]=.2
        pause=(1043.,1044.,1051.,1052.);now=1042.;held=[];first=None
        for i in range(450):
            times=now+.03+np.arange(11)*.01
            base=np.array(original.packet(now,now,[.0006]*6,times))
            actual=np.array(paused.packet(now,now,[.0006]*6,times,j3_pause=pause))
            np.testing.assert_array_equal(actual[:,[0,1,3,4,5]],base[:,[0,1,3,4,5]])
            self.assertTrue(np.all(abs(np.diff(actual,axis=0))<=np.diff(times)[:,None]*FEEDFORWARD_SLEW_LIMITS+1e-9))
            if first is None:first=times[0]
            m=(times>=1044)&(times<=1051);held.extend(actual[m,2])
            if times[0]>=1052.15:
                # Integral of the two unit-second quintic ramps is .5s each;
                # the seven-second plateau accumulates nothing.
                expected=.2+.288*((1043-first)+.5+.5+(times[0]-1052))
                self.assertAlmostEqual(actual[0,2],expected,places=9);break
            now+=.02 if i%3 else .03
        self.assertGreater(len(held),100);self.assertLess(np.ptp(held),1e-10)
        self.assertGreater(min(held),.6,'pause must retain accumulated static bias')
        with self.assertRaisesRegex(ValueError,'cannot change'):
            paused.packet(now+.02,now+.02,[.0006]*6,times+.02)
        with self.assertRaisesRegex(ValueError,'left J3'):
            ReferenceIntegral(side='right').packet(now,now,[.001]*6,times,j3_pause=pause)

    def test_versioned_artifact_rejects_bad_scope_then_real_review_executes_and_stops(self):
        study=self.study();self.raw.native_reference_api='preserve_reference_v1';self.advance(1)
        artifact=study.config['references']['1.0']
        with np.load(artifact['path'])as f:identity=json.loads(str(f['metadata']))['reference_id']
        value=dict(schema='reference_feedforward.v2',source_sha256=identity,controller_profile=self.profile,
            phase=[0.,1.],torque_bias_sdk6=[[0.,0.,.2,0.,0.,0.]]*2,j3_integral_pause_s=[.25,.75,1.25,1.75],
            provenance=dict(study_settings=study.settings,speed_multiplier=1.,native_reference_api='preserve_reference_v1'))
        def pin(v):
            raw=json.dumps(v).encode();digest=hashlib.sha256(raw).hexdigest();path=Path(self.temp.name)/(digest+'.json');path.write_bytes(raw)
            return dict(path=str(path),sha256=digest,size_bytes=len(raw))
        for bad in ([False,.75,1.25,1.75],[.25,.5,1.25,1.75],[.25,.75,.75,1.75],[.25,.75,1.25,3.],[.25,.75,1.25,float('nan')]):
            p=pin(dict(value,j3_integral_pause_s=bad))
            with self.assertRaisesRegex(ValueError,'J3 integral pause'):
                load_feedforward(self.temp.name,p['sha256'],identity,controller_profile=self.profile,native_reference_api='preserve_reference_v1',reference_duration_s=2.)
        p=pin(value)
        with self.assertRaisesRegex(ValueError,'J3 integral pause'):
            load_feedforward(self.temp.name,p['sha256'],identity,controller_profile=self.profile,native_reference_api='preserve_reference_v1')
        legacy=pin(dict(value,schema='reference_feedforward.v1'))
        with self.assertRaisesRegex(ValueError,'source'):
            load_feedforward(self.temp.name,legacy['sha256'],identity,controller_profile=self.profile,native_reference_api='preserve_reference_v1',reference_duration_s=2.)
        command=dict(action='preview_workbench',reference_artifact=artifact,feedforward_artifact=p)
        for settings in (None,dict(study.settings,integral_enabled=False)):
            with self.assertRaisesRegex(ValueError,'explicit research'):
                self.runtime.submit(self.creds['research'],dict(command,**({}if settings is None else dict(study_settings=settings))))
        before=len(self.raw.packets);study.prepare('research','trajectory',1.,p);self.assertEqual(len(self.raw.packets),before)
        study.execute('research',study.review['id']);self.advance(120)
        interpreter=self.runtime.owner.runner.target_interpreter
        self.assertEqual(interpreter.integrals['left'].j3_pause,interpreter.feedforward.j3_pause)
        self.assertIsNone(interpreter.integrals['right'].j3_pause)
        self.runtime.request_stop();self.advance(50);study.thread.join(3)
        self.assertFalse(study.thread.is_alive());self.assertTrue(interpreter.cancelled)
        self.assertIsNone(interpreter.feedforward);self.assertFalse(interpreter.integrals);self.assertIsNone(self.raw.window)

    def test_shared_translation_unchanged_except_j3_and_new_reference_does_not_inherit_schedule(self):
        helpers=[interpreter_fixture.TargetInterpreterTests(),interpreter_fixture.TargetInterpreterTests()];executors=[]
        for i,h in enumerate(helpers):
            e=h.fixture(integral=True,profile=self.profile)
            profile=dict(controller_profile=self.profile,phase=np.array([0.,1.]),bias=np.tile([0.,0.,.2,0.,0.,0.],(2,1)))
            if i:profile['j3_integral_pause_s']=[.25,.75,1.25,1.75]
            e.feedforward=ReferenceFeedforward(profile,100.,2.,98.,initial=np.array([0.,0.,.2,0.,0.,0.]));executors.append(e)
        previous=None;differences=[]
        for i in range(110):
            for h,e in zip(helpers,executors):
                h.now=100.+i*.02;feedback=h.feedback(e);feedback['qpos'][2]-=.0006;e.tick(h.now,feedback,force=True)
            a,d=[e.last_packet for e in executors]
            np.testing.assert_array_equal(a['q_sdk14'],d['q_sdk14']);np.testing.assert_array_equal(a['dq_sdk14'],d['dq_sdk14'])
            delta=np.array(d['torque_bias_sdk14'])-a['torque_bias_sdk14'];np.testing.assert_array_equal(np.delete(delta,2,axis=1),0.)
            differences.append(float(abs(delta[:,2]).max()));times=np.array(d['host_times_s']);bias=np.array(d['torque_bias_sdk14'])[:,:6]
            if previous is not None:np.testing.assert_allclose(bias[0],[np.interp(times[0],previous[0],previous[1][:,j])for j in range(6)],atol=1e-10)
            previous=times,bias
        self.assertGreater(max(differences),.1)
        old=executors[1];h=helpers[1]
        following=TargetInterpreter(old.backend,old.reference,active_arm='both',start_s=h.now,epoch=h.epoch,current_epoch=lambda:h.epoch,gate=lambda:None,clock=lambda:h.now,profile=self.profile,integral=True)
        following.inherit_hold(old,h.now)
        self.assertTrue(old.cancelled);self.assertIsNone(following.feedforward);self.assertIsNone(following.integrals['left'].j3_pause)
        self.assertGreater(following.integrals['left'].bias[2],.2)


if __name__=='__main__':unittest.main()
