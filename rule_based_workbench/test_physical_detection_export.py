"""CPU provenance checks, never physical acquisition or inference evidence."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory

from integration import _digest
from source_from_detection import build_source, convert
from export_plan import validate_export
from test_detection_bridge import fixture
from test_frozen_executor import fixture as plan_fixture
from test_harness import rejects


def main():
    summary, observations, profile = fixture()
    receipt = dict(calibration_accepted=True, evidence_source='physical_rgbd', robot_id='fixture-arx',
                   camera_serials=dict(head='h', left='l', right='r'),
                   calibration_profile_digest=_digest(profile), detection_summary_digest=_digest(summary),
                   observations_digest=_digest(observations))
    source = build_source(summary, observations, profile, 'physical-fixture', receipt)
    assert source['schema_version'] == 'physical_collection_source.v1'
    assert source['lane'] == 'physical-real' and source['adapter'] == 'foundationpose'
    assert 'gpu' not in source and 'instance_id' not in source
    for changes in [dict(calibration_accepted=False), dict(evidence_source='simulation'),
                    dict(calibration_profile_digest='wrong'), dict(observations_digest='wrong'),
                    dict(camera_serials=dict(head='same', left='same', right='same'))]:
        rejects(lambda b: build_source(summary, observations, profile, 'physical-fixture', b), {**receipt, **changes})
    with TemporaryDirectory() as directory:
        root=Path(directory)
        (root/'profile.json').write_text(json.dumps(profile))
        (root/'summary.json').write_text(json.dumps(summary))
        (root/'observations.jsonl').write_text('\n'.join(json.dumps(row) for row in observations))
        receipt['detection_files']={name+'_sha256':hashlib.sha256((root/file).read_bytes()).hexdigest()
                                    for name,file in [('summary','summary.json'),('observations','observations.jsonl')]}
        source=convert(root/'profile.json',root,root/'out','physical-fixture',receipt)
        binding_path=root/'out/hardware-binding.json'
        binding=json.loads(binding_path.read_text())
        assert binding['source_digest']==_digest(source)
        _,plan=plan_fixture(.001)
        plan.update(source_run_id=source['run_id'],source_sequence=source['sequence'],
                    scene_digest=source['scene']['scene_digest'],calibration_profile_digest=_digest(profile))
        from test_joint_space import physical_fixture
        physical_fixture(plan, source)
        assert validate_export(plan,source,binding_path)['mode']=='real'
        rejects(lambda _:validate_export(plan,source),None)
        wrong=deepcopy(source);wrong['robot_id']='other'
        rejects(lambda s:validate_export(plan,s,binding_path),wrong)
        dual=build_source(summary,observations,profile,'fixture')
        rejects(lambda s:validate_export(plan,s,binding_path),dual)
        (root/'observations.jsonl').write_text('changed')
        rejects(lambda _:convert(root/'profile.json',root,root/'bad','fixture',receipt),None)
    print('Physical detection binding and export validation: PASS (CPU fixtures only)')


if __name__=='__main__':
    main()
