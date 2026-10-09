"""Run with workbench config argv; checks preview authority and recoverable refusal."""
import json,sys,tempfile,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples/scripts/debug'))
from joint_head_workbench import Workbench


def main(config_path):
    with tempfile.TemporaryDirectory() as root:
        config=json.loads(Path(config_path).read_text());config.pop('review_dataset',None);config.pop('live_backend',None)
        app=Workbench(config,root)
        start=app.q.copy()
        p=app.action('/api/preview',{'q':start.tolist(),'speeds':[.4]*12})
        assert (app.q==start).all() and p['token']
        try:app.action('/api/execute',{'token':'invalid'})
        except ValueError:pass
        else:raise AssertionError('invalid preview accepted')
        assert app.status=='READY'
        app.action('/api/execute',{'token':p['token']})
        for _ in range(100):
            if app.status!='RUNNING':break
            time.sleep(.05)
        assert app.status=='READY'
        try:app.action('/api/execute',{'token':p['token']})
        except ValueError:pass
        else:raise AssertionError('token reused')
        app.action('/api/ack',{})
        try:app.action('/api/preview',{'q':start.tolist(),'speeds':[float('nan')]*12})
        except ValueError:pass
        else:raise AssertionError('NaN speed accepted')
    print('workbench: preview-only, single-use token, recovery, invalid speed PASS')

    with tempfile.TemporaryDirectory() as root:
        root=Path(root)
        legacy={'waypoints':[{'q':start.tolist(),'capture':True,'arm':'left','kind':'grid','cell':[1,1]},
                             {'q':start.tolist(),'capture':False,'arm':'left','kind':'transition','cell':[1,1]}]}
        (root/'plan.json').write_text(json.dumps(legacy))
        dataset=root/'support-validation'/'formal-capture';dataset.mkdir(parents=True)
        (dataset/'completion.json').write_text(json.dumps({'passed':True,'captured':2,'accepted':1,'failed':1}))
        for i,valid in enumerate([True,False]):
            (dataset/f'test-{i:03}.json').write_text(json.dumps({'id':str(i),'image':f'frame-{i:03}.png','valid_for_fit':valid,'q_reported_rad':start.tolist()}))
        (dataset/'fit-truth-score.json').write_text(json.dumps({'status':'not a fit'}))
        (dataset/'fit-result.json').write_text(json.dumps({'status':'legacy','metrics_rectified_px':{}}))
        pending=root/'batch-pending';pending.mkdir()
        # History discovery must not parse large or still-being-written capture requests.
        (pending/'request.json').write_text('{in-progress-request')
        app=Workbench(config,root)
        assert len(app.queue)==2 and app.status=='READY' and not app.tokens
        assert app.state()['plan_stale']
        try:app.action('/api/run-batch',{})
        except ValueError as exc:assert '几何' in str(exc)
        else:raise AssertionError('stale geometry plan allowed to execute')
        assert {r['id'] for r in app.history()}=={'support-validation/formal-capture','batch-pending/capture'}
        review=app.action('/api/history-select',{'id':'support-validation/formal-capture'})
        assert review['fit_result']['status']=='legacy' and app.selected_batch is None
        app.action('/api/fit-source',{'id':'support-validation/formal-capture'})
        assert app.fit_result['status']=='legacy' and (app.q==start).all(), 'review must not move simulation'
        assert app.counts()=={'waypoints':2,'capture_points':1,'transition_points':1,'captured':2,'valid':1,'failed':1,'unclassified':0}
        evaluation={'status':'FROZEN_CANDIDATE_TEST','optimization_performed':False,'selected_candidate':'baseline','candidates':{'baseline':{'metrics_rectified_px':{'mean_px':.7}}},'overlays':[str(dataset/'frozen-evaluation-frame-000-overlay.png')]}
        (dataset/'frozen-evaluation.json').write_text(json.dumps(evaluation))
        (dataset/'frozen-evaluation-frame-000-overlay.png').write_bytes(b'overlay')
        review=app.action('/api/history-select',{'id':'support-validation/formal-capture'})
        assert review['evaluation']==evaluation and review['frames'][0]['overlay_url'].endswith('/frozen-evaluation-frame-000-overlay.png')
        assert app.evaluation is None
        assert app.capture_request([])['mount_chain']==app.geometry.mount_chain
        app.action('/api/queue-remove',{'index':1})
        restored=Workbench(config,root)
        assert len(restored.queue)==1 and restored.selected_batch=='support-validation/formal-capture' and restored.status=='READY'
        assert not restored.tokens and restored.process is None and len(restored.frames)==2
        review=restored.action('/api/history-select',{'id':'batch-pending/capture'})
        assert review['fit_result'] is None and review['evaluation'] is None and review['frames']==[]
        assert len(restored.frames)==2, 'history switch must preserve solver source'
        try:restored.action('/api/history-select',{'id':'../outside'})
        except ValueError:pass
        else:raise AssertionError('arbitrary history path accepted')
        restored.action('/api/queue-clear',{})
        assert Workbench(config,root).queue==[], 'empty queue must not reimport legacy plan'
        restored.action('/api/queue',{'q':start.tolist(),'speeds':[.2]*12})
        assert not Workbench(config,root).state()['plan_stale']
        changed={**config,'margin_px':99}
        assert Workbench(changed,root).state()['plan_stale'], 'changed configuration must require replanning'
        assert not (root/'workbench-state.tmp').exists()
    print('workbench: legacy discovery, atomic saved queue, safe restart, exact counts, review isolation PASS')

    with tempfile.TemporaryDirectory() as root:
        app=Workbench(config,root)
        original=app.jaw_mm.copy()
        opened=app.display(app.q,[83.,83.])
        closed=app.display(app.q,[0.,40.])
        a=next(p for p in opened['parts'] if p['side']=='left' and p['link']=='link6')
        b=next(p for p in closed['parts'] if p['side']=='left' and p['link']=='link6')
        assert a['vertices']!=b['vertices'], 'jaw slider must change mesh'
        shell=lambda p:[x for x in p['parts'] if x['link'] in ('marker','camera_shell_proxy')]
        assert shell(opened)==shell(closed), 'jaw opening must not move wrist stickers'
        app.action('/api/queue',{'q':app.q.tolist(),'jaw_gap_mm':[0.,40.]})
        assert Workbench(config,root).queue[0]['jaw_gap_mm']==[0.,40.]
        p=app.preview({'q':app.q.tolist(),'jaw_gap_mm':[81.,82.]})
        assert (app.jaw_mm==original).all() and p['duration_s']>=1.875*2/20
        app.action('/api/execute',{'token':p['token']})
        for _ in range(100):
            if app.status!='RUNNING':break
            time.sleep(.03)
        assert app.jaw_mm.tolist()==[81.,82.] and app.status=='READY'
        p=app.preview({'q':app.q.tolist(),'jaw_gap_mm':[80.,80.]})
        app.jaw_mm[0]-=1
        try:app.action('/api/execute',{'token':p['token']})
        except ValueError:pass
        else:raise AssertionError('jaw start drift accepted')
        for invalid in ([84,0],[-1,0],[float('nan'),0],[0]):
            try:app.preview({'q':app.q.tolist(),'jaw_gap_mm':invalid})
            except ValueError:pass
            else:raise AssertionError('invalid jaws accepted')
    print('workbench: jaws mesh, independent wrist markers, persistence, bounded execution and drift PASS')

    with tempfile.TemporaryDirectory() as root:
        app=Workbench(config,root)
        for jaws in ([82.,82.],[81.,83.]):app.action('/api/queue',{'q':app.q.tolist(),'jaw_gap_mm':jaws})
        app.action('/api/run-batch',{})
        for _ in range(100):
            if app.status!='RUNNING':break
            time.sleep(.03)
        assert app.status=='READY' and app.jaw_mm.tolist()==[81.,83.]
        assert not app.frames, 'manual queue must not pretend to capture'
    print('workbench: local manual queue executes joint+jaw targets without capture worker PASS')


if __name__ == '__main__':
    main(sys.argv[1])
