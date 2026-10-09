"""Check continuous solids, unchanged immutable inputs and original-height rule."""
from pathlib import Path
import importlib.util
import sys
from tempfile import TemporaryDirectory
import numpy as np

from continuous_cup import body_bands, copy_immutable
from cup_envelope import nominal_clearance
from types import ModuleType


def main():
    native=Path(__file__).resolve().parents[2]/'robot_sim-gripper-profile-20260921'
    root=native/'source/robot_sim/robot_sim'
    for suffix in ('','asset_cfg','asset_cfg.objects'):
        name='robot_sim'+('.'+suffix if suffix else '')
        package=ModuleType(name);package.__path__=[str(root.joinpath(*suffix.split('.'))) if suffix else str(root)]
        sys.modules[name]=package
    path=Path(__file__).with_name('native_overrides')/'cup.py'
    spec=importlib.util.spec_from_file_location('continuous_test_profile',path)
    module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    rows=body_bands([.03478,.03683,.03981])
    bands=[module.CupBand(**row) for row in rows]
    for a,b in zip(bands,bands[1:]):
        assert a.radius_at(a.z_max_m)==b.radius_at(b.z_min_m)
    for band in bands:
        vertices,faces=map(np.asarray,band.mesh())
        a,b,c=(vertices[faces[:,i]] for i in range(3))
        volume=np.einsum('ij,ij->i',a,np.cross(b,c)).sum()/6
        assert volume>0 and len(vertices)==64
        edges={}
        for face in faces:
            for i,j in zip(face,np.roll(face,-1)):
                key=tuple(sorted((int(i),int(j))));edges[key]=edges.get(key,0)+1
        assert set(edges.values())=={2},'collider is not watertight'
    profile=module.CupInteractionProfile('fixture','x',tuple(bands),.0347,.1,(.035,.09),())
    assert profile.required_opening_m(.04,.05)<2*bands[1].radius_m
    assert profile.sidewall_normal_limit_rad(.06)>.02
    assert np.all(nominal_clearance(np.array([[0,0,.05],[.02,0,.075]]),rows)>0)
    assert nominal_clearance(np.array([[.05,0,.06]]),rows)[0]<0
    with TemporaryDirectory() as temporary:
        root=Path(temporary);source=root/'source';source.mkdir();(source/'unchanged').write_text('source')
        copy_immutable(source,root/'derived')
        changed=root/'derived/unchanged';replacement=changed.with_suffix('.new');replacement.write_text('candidate');replacement.replace(changed)
        assert (source/'unchanged').read_text()=='source'
        from continuous_cup import prepare_source_wrapped_fingers
        relative=Path('source/robot_sim/robot_sim/asset_cfg/robots/ac_one_finger_contact.py')
        original=source/relative;original.parent.mkdir(parents=True)
        text='class Spec:\n    pads: tuple[ACOneFingerPadSpec, ...]\ndef configure():\n    for pad in []:\n        for prim in []:\n            mesh_api.GetApproximationAttr().Set(specification.body_collision_approximation)\n'
        original.write_text(text)
        wrapped=root/'wrapped';receipt=prepare_source_wrapped_fingers(source,wrapped)
        assert original.read_text()==text and receipt['body_shrink_wrap'] is True
        assert 'CreateShrinkWrapAttr(specification.body_shrink_wrap)' in (wrapped/relative).read_text()
    from motiongen_targets import GRASP_HEIGHT_RAISE_M
    assert GRASP_HEIGHT_RAISE_M==0.
    assert 'torch' not in sys.modules
    print('continuous cup: closed convex topology, 10mm seam, taper opening, immutable input and restored height PASS')


if __name__=='__main__':main()
