"""Large HTTP and artifact previews through the actual independent Owner IPC."""
from pathlib import Path
from http.client import HTTPConnection
import json,multiprocessing,os,sys,tempfile,threading,time,unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'rule_based_workbench'),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]


def owner_process(root,ready,closing,finished):
    import robot_workbench.control_owner as transport
    from robot_workbench import control_plan
    from robot_workbench.service import Device
    from test_control_plan import ControlPlanTests
    case=ControlPlanTests();case.setUp()
    runtime=case.runtime;device=Device('isolated-control','fake',None)
    device.runtime=runtime;device.credentials=case.creds
    if sys.platform!='linux':transport._identity=lambda:dict(pid=os.getpid(),start_ticks='cpu-test',boot_id='cpu-test')
    root=Path(root);raw=json.dumps(case.large()).encode();(root/'fixture.json').write_bytes(raw)
    service=transport.ControlOwnerService(device,root/'service')
    original=json.loads
    def bounded(value,*args,**kwargs):
        if isinstance(value,(bytes,str))and len(value)>control_plan.MAX_METADATA:
            raise AssertionError('independent Owner decoded full preview JSON')
        return original(value,*args,**kwargs)
    json.loads=bounded
    serving=threading.Thread(target=service.serve_forever,daemon=True);serving.start()
    done,ticker,times,errors=case.start_ticks()
    ready.put(str(service.descriptor_path))
    try:closing.wait(45)
    finally:
        done.set();ticker.join(2);json.loads=original
        finished.put(dict(max_tick_gap_s=max([b-a for a,b in zip(times,times[1:])],default=0.),errors=[str(e)for e in errors]))
        case.doCleanups();device.runtime=None;service.close();serving.join(2)


class IndependentPreviewTests(unittest.TestCase):
    def test_large_http_and_artifact_preview_keep_inputs_across_json_ipc(self):
        from robot_workbench.control_owner import ControlRuntimeClient
        from robot_workbench.control_plan import ArtifactPreview
        from robot_workbench.service import Device
        from robot_workbench.bridges import start_control_gateway
        with tempfile.TemporaryDirectory(prefix='owner-preview-',dir='/tmp')as directory:
            root=Path(directory);context=multiprocessing.get_context('spawn')
            ready,finished=context.Queue(),context.Queue();closing=context.Event()
            process=context.Process(target=owner_process,args=(directory,ready,closing,finished));process.start()
            try:
                descriptor=ready.get(timeout=12);client=ControlRuntimeClient(descriptor)
                device=Device('isolated-control','fake',None);device.runtime=client
                device.credentials={b:client.register_business(b)for b in ('calibration','collection')}
                server,thread=start_control_gateway(device,{'calibration':'c'*48,'collection':'d'*48})
                try:
                    raw=(root/'fixture.json').read_bytes();self.assertGreater(len(raw),1024**2)
                    connection=HTTPConnection('127.0.0.1',server.server_port,timeout=20)
                    connection.request('POST','/operator',raw,headers={'Authorization':'Bearer '+'d'*48,'Content-Type':'application/json'})
                    response=connection.getresponse();body=response.read();connection.close()
                    self.assertEqual(response.status,200,body.decode());review=json.loads(body)
                    self.assertEqual(review['compiled_tick_count_before_initialization'],6000)
                    command=json.loads(raw)
                    for name in ('plan','source'):(root/(name+'.json')).write_text(json.dumps(command[name]))
                    review=device.submit('collection',ArtifactPreview(plan_path=root/'plan.json',source_path=root/'source.json'))
                    self.assertEqual(review['compiled_tick_count_before_initialization'],6000)
                    state=client.refresh('collection');self.assertFalse(state['automatic_enabled']);self.assertTrue(state['feedback_healthy'])
                    self.assertEqual(list((Path(descriptor).parent/'preview-inputs').iterdir()),[])
                finally:server.shutdown();server.server_close();thread.join(2);client.detach()
            finally:
                closing.set();process.join(8)
                if process.is_alive():process.terminate();process.join(2)
            self.assertEqual(process.exitcode,0);result=finished.get(timeout=2)
            self.assertEqual(result['errors'],[]);self.assertLess(result['max_tick_gap_s'],.1)

    def test_preview_file_transport_rejects_tampering_and_path_escape(self):
        from robot_workbench.control_owner import _stage_preview,_restore_preview
        from robot_workbench.control_plan import RawPreview
        from copy import deepcopy
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);raw=b'{"action":"preview_frozen","plan":{},"source":{}}'
            wire,staged=_stage_preview(RawPreview(raw),root)
            self.assertLess(len(json.dumps(wire)),1024);self.assertEqual(_restore_preview(wire,root).raw,raw)
            for edit in ('hash','size','escape','symlink','writable'):
                bad=deepcopy(wire);ref=bad['preview_files']['files']['raw']
                if edit=='hash':ref['sha256']='0'*64
                elif edit=='size':ref['size_bytes']+=1
                elif edit=='escape':ref['path']=str(root/'outside.json')
                elif edit=='symlink':
                    link=staged/'link.json';link.symlink_to(staged/'raw.json');ref['path']=str(link)
                else:(staged/'raw.json').chmod(0o600)
                with self.subTest(edit=edit),self.assertRaises(ValueError):_restore_preview(bad,root)

    def test_unknown_receipt_retains_inputs_known_result_cleans_and_old_owner_rejects(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from robot_workbench.control_owner import ControlRuntimeClient,ControlCallUnknown,PREVIEW_FILES
        from robot_workbench.control_plan import RawPreview
        with tempfile.TemporaryDirectory()as directory:
            root=Path(directory);client=object.__new__(ControlRuntimeClient)
            client.control_recording_backend=None
            client.descriptor=root/'control-owner.json';client._business=lambda _: 'collection'
            client.channel=SimpleNamespace(descriptor={'frozen_preview_transport':PREVIEW_FILES})
            command=RawPreview(b'{"action":"preview_frozen","plan":{},"source":{}}')
            client._call=Mock(side_effect=ControlCallUnknown('unknown-cpu','lost reply'))
            with self.assertRaises(ControlCallUnknown):client.submit(None,command)
            spool=root/'preview-inputs';self.assertEqual(len(list(spool.iterdir())),1)
            client._call=Mock(side_effect=ValueError('explicit rejection'))
            with self.assertRaisesRegex(ValueError,'explicit rejection'):client.submit(None,command)
            self.assertEqual(len(list(spool.iterdir())),1)
            client._call=Mock(return_value={'token':'cpu-only'})
            self.assertEqual(client.submit(None,command),{'token':'cpu-only'})
            self.assertEqual(len(list(spool.iterdir())),1)
            client.channel.descriptor={}
            with self.assertRaisesRegex(ValueError,'lacks frozen-preview'):client.submit(None,command)


if __name__=='__main__':unittest.main()
