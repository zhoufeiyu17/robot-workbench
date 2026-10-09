"""Saved pre-action recovery without observing, planning, or sending commands."""
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch
import json
from playground_pipeline import restore_plan,save
from test_frozen_executor import fixture
from test_joint_space import physical_fixture
from integration import _digest
from real_recording import RealRecorder

def main():
    source,plan=fixture(.001)
    source={k:v for k,v in source.items() if k not in ('lane','adapter','instance_id','gpu')}
    source.update(schema_version='physical_collection_source.v1',evidence_source='physical_rgbd',
                  robot_id='test',camera_serials={'head':'h','left':'l','right':'r'})
    physical_fixture(plan,source)
    binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='test',
                 camera_serials=source['camera_serials'],source_digest=_digest(source),
                 calibration_profile_digest=_digest(source['calibration_profile']))
    with TemporaryDirectory() as tmp,patch('playground_pipeline.load_config',return_value=({}, {}, {})):
        root=Path(tmp);job=root/'jobs'/'planned';job.mkdir(parents=True)
        prepared=dict(validated=True,timeline_digest=plan['timeline_digest'],output=str(job))
        for key,name,value in [('source_path','source.json',source),('plan_path','plan.json',plan),
                               ('hardware_binding','binding.json',binding),('config_path','config.json',{})]:
            save(job/name,value);prepared[key]=str(job/name)
        frame=job/'first-frame.png';frame.write_bytes(b'unchanged-frame');prepared['first_frame_path']=str(frame)
        save(root/'installed.json',binding)
        bridge=SimpleNamespace(config={'trusted_binding_path':str(root/'installed.json'),'local_recording_root':str(root/'raw')})
        checkpoint=dict(schema_version='playground_prepared_plan.v1',plan=prepared,execution_pending=False,execution_output=None)
        save(root/'prepared-plan.json',checkpoint)
        assert restore_plan({},root,bridge)==prepared
        changed=dict(binding,calibration_profile_digest='changed');save(root/'installed.json',changed)
        try:restore_plan({},root,bridge)
        except ValueError:pass
        else:raise AssertionError('changed calibration resumed')
        save(root/'installed.json',binding)
        execution=root/'jobs'/'execute';execution.mkdir()
        checkpoint.update(execution_pending=True,execution_output=str(execution));save(root/'prepared-plan.json',checkpoint)
        assert restore_plan({},root,bridge) is None,'unknown execution resumed'
        phase=execution/'execution-phase.json'
        save(phase,dict(phase='pre_action',timeline_digest=plan['timeline_digest']))
        assert restore_plan({},root,bridge)==prepared,'pre-action transport failure lost checked plan'
        save(root/'prepared-plan.json',checkpoint)
        save(phase,dict(phase='confirm_requested',timeline_digest=plan['timeline_digest']))
        assert restore_plan({},root,bridge) is None,'unknown confirm outcome resumed'
        raw=root/'raw'/'run';recorder=RealRecorder(raw,source,plan,binding)
        recorder.seal('failed','startup failure with zero commands')
        save(execution/'recording-directory.json',{'directory':str(raw)})
        assert restore_plan({},root,bridge)==prepared
        assert json.loads((root/'prepared-plan.json').read_text())['execution_pending'] is False
        save(root/'prepared-plan.json',checkpoint)
        manifest=json.loads((raw/'manifest.json').read_text());manifest['command_count']=1;save(raw/'manifest.json',manifest)
        assert restore_plan({},root,bridge) is None,'partial physical execution resumed'
        save(job/'source.json',dict(source,robot_id='changed'))
        try:restore_plan({},root,bridge)
        except ValueError:pass
        else:raise AssertionError('changed scene identity resumed')
    print('PASS pre-action checkpoint, calibration/source checks, zero-command recovery, unknown/partial execution rejection; no motion')

if __name__=='__main__':main()
