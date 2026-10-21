"""CPU regression: learn detection noise only after a stationary candidate window."""
from integration import ObservationWindow


def sample(i, x):
    return dict(mode='offline_replay', source_stamp_ns=(i+1)*10**9, frame_id='camera', tracks=[
        dict(object_type='coffee_machine', pose_current=True, xyz_m=[0.,0.,1.], quaternion_xyzw=[0.,0.,0.,1.]),
        dict(object_type='cup', pose_current=True, xyz_m=[x,0.,1.])])


def main():
    w=ObservationWindow({},[],mode='offline_replay')
    for i in range(5): assert not w.add(sample(i,0.))['ready']
    assert w.add(sample(5,0.))['ready']
    assert w.cup_motion_threshold_m==.001
    w=ObservationWindow({},[],mode='offline_replay')
    for i in range(6): state=w.add(sample(i,0. if i<2 else .05))
    assert state['reobserve_required'] and not state['ready']
    assert 0<w.status()['span_s']<5.
    boundary=w.frames[0]['source_stamp_ns']
    for i in range(6,12):
        state=w.add(sample(i,.05))
        assert state['ready']==(len(w.frames)>=6 and (i+1)*10**9-boundary>=3*10**9)
        if state['ready']:break
    assert state['ready']
    assert all(f['cup_position'][0]==.05 for f in w.frames)
    w=ObservationWindow({},[],mode='offline_replay')
    for i in range(18): assert not w.add(sample(i,i*.002))['ready']
    assert len(w.reobserve_events)>=3
    w=ObservationWindow({},[],mode='offline_replay',cup_motion_threshold_m=.001)
    w.add(sample(0,0.)); assert w.add(sample(1,.05))['reobserve_required']
    # Frozen measured positions from the 2026-09-23 physical capture (metres).
    xyz=[[-.265188,-.089903,.641724],[-.266785,-.091254,.641749],[-.266524,-.093170,.645369],
         [-.266704,-.091060,.643700],[-.263394,-.088366,.638450],[-.265981,-.091812,.644009],
         [-.266040,-.090064,.640083],[-.264377,-.090574,.638350]]
    w=ObservationWindow({},[],mode='offline_replay')
    for i,p in enumerate(xyz):
        row=sample(i,0.); row['tracks'][1]['xyz_m']=p; state=w.add(row)
    assert state['ready'] and not w.reobserve_events
    print('bootstrap: real jitter, early step, slow drift, static, explicit threshold PASS')


if __name__=='__main__':main()
