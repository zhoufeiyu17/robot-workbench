"""CPU shared detector wiring; no process, remote or hardware side effects."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch
import app


def main():
    with TemporaryDirectory() as directory:
        root=Path(directory)
        service=dict(root='/tmp/rbw-detector-test/service',socket='/tmp/rbw-detector-test/service/s',
                     owner_token='template-only-identity',identity={'adapter':'sha'},allowed_read_roots=['/jobs'],
                     allowed_output_root='/jobs',max_lifetime_seconds=0)
        def template(name,native):
            contract=root/(name+'-contract.json')
            contract.write_text(json.dumps(dict(detector_service=deepcopy(service),filesystem={'writable_roots':[service['root']]},provenance={'config_sha256':{}})))
            config=root/(name+'.json');config.write_text(json.dumps(dict(native_code=native,contract_path=str(contract),detector_service=deepcopy(service))))
            return config
        dual=template('dual','/dual/native');real=template('real','/real/native')
        real_config={'preparation':{'pipeline_config':str(real),'capture_dir':'/capture','physical_binding':'/binding'}}
        frozen=app.shared_startup_config(dual,root/'state',real_config)
        real_frozen=Path(real_config['preparation']['pipeline_config'])
        d,r=[json.loads(p.read_text()) for p in (frozen,real_frozen)]
        assert d['detector_service']==r['detector_service'] and d['detector_service']['owner_token']!=service['owner_token']
        assert r['native_code']=='/real/native' and d['native_code']=='/dual/native'
        for c in (d,r):assert json.loads(Path(c['contract_path']).read_text())['detector_service']==c['detector_service']
        real_only={'preparation':{'pipeline_config':str(real)}}
        result=app.shared_startup_config(None,root/'real-only',real_only)
        assert real_only['preparation']['pipeline_config']==str(result)
        # Start gate must reject before creating jobs or starting capture/workers.
        with patch.object(app,'REAL_BRIDGE',SimpleNamespace(config=real_config)), patch.object(app,'REAL_STATE_DIR',root/'jobs'), \
             patch.object(app,'REAL_JOB',None), patch.object(app,'initialization_status',return_value={'status':'starting'}), \
             patch.object(app,'Thread') as thread:
            try:app.start_real_preparation()
            except ValueError as e:assert '尚未就绪' in str(e)
            else:raise AssertionError('unready detector accepted')
            thread.assert_not_called();assert not (root/'jobs').exists()
        changed=json.loads(real.read_text());changed['detector_service']['allowed_output_root']='/foreign'
        real.write_text(json.dumps(changed));contract=Path(changed['contract_path']);c=json.loads(contract.read_text());c['detector_service']=changed['detector_service'];contract.write_text(json.dumps(c))
        try:app.shared_startup_config(dual,root/'reject',{'preparation':{'pipeline_config':str(real)}})
        except ValueError as e:assert '不一致' in str(e)
        else:raise AssertionError('different detector scope shared silently')
    print('PASS shared startup: real-only, shared ownership, independent mode inputs, ready gate, scope mismatch rejection')

if __name__=='__main__':main()
