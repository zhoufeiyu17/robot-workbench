"""Frozen cache boundaries and coarse self checks, with no GPU or robot."""
from copy import deepcopy
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration/infra')]
from arx_env.joint_head_workbench_geometry import box_triangles
from robot_workbench.manual_self_collision import ManualSelfCollision, POLICY, components, source_fingerprint, proxy_digest


def geometry(path):
    path.write_text('<robot name="fixture"><link name="base_link"/></robot>')
    points=box_triangles([.01]*3,np.eye(4));faces=np.arange(len(points)).reshape(-1,3)
    g=SimpleNamespace(config={'model_family':'ac_one','urdf':str(path)},adjacent=set(),static_adjacent=set(),
        model=SimpleNamespace(root='base_link',joints=[],names=[f'joint{i}' for i in range(12)]),
        aliases={s:{s+'_link4':'link4'} for s in ('left','right')},
        full_side_meshes={s:[('link4',points,faces)] for s in ('left','right')},
        static_meshes=[('chassis',points+[0,0,-.5],faces)],limits=np.asarray([[-2.,2.]]*12))
    g.with_gaps=lambda a,b:g if (a,b)==(83.,83.) else None
    def q(value):
        value=np.asarray(value,dtype=float)
        if value.shape!=(12,) or not np.isfinite(value).all():raise ValueError('twelve finite joints required')
        return value
    g._q=q
    def frames(values):
        result={}
        for side,index,offset in (('left',0,-.2),('right',6,.2)):
            transform=np.eye(4);transform[0,3]=values[index]+offset
            result[side]={name:transform for name,*_ in g.full_side_meshes[side]}
        return result
    g.frames=frames
    return g


def payload(g):
    parts=[];centers=[]
    for side,rows in [*g.full_side_meshes.items(),(None,[r for r in g.static_meshes if r[0]=='chassis'])]:
        for name,points,_ in rows:
            parts.append(dict(side=side,name=name,sphere_count=1,support_member=False))
            centers.append(np.unique(points,axis=0).mean(0).tolist())
    data=dict(schema='manual_inscribed_spheres.v1',policy=deepcopy(POLICY),source_fingerprint=source_fingerprint(g),
        parts=parts,centers=centers,radii=[.003]*len(parts),original_radii=[.003]*len(parts),
        sphere_parts=list(range(len(parts))),support_local=[False]*len(parts))
    data['proxy_id']=proxy_digest(data)
    return data


class ManualSphereTests(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.g=geometry(Path(self.temp.name)/'model.urdf')

    def test_cached_both_arms_path_interior_and_limits(self):
        g=self.g;data=payload(g);checker=ManualSelfCollision(g,prebuilt=data)
        start=np.zeros(12);target=start.copy();target[0]=.4;target[6]=-.4
        self.assertTrue(checker.check_path(start,start)['ok'])
        self.assertFalse(checker.check_path(start,target)['ok'])
        target=start.copy();target[6]=-.4
        self.assertFalse(checker.check_path(start,target)['ok'])
        target[6]=2.1
        self.assertEqual(checker.check_path(target,target)['reason'],'joint limits')
        target[6]=np.nan
        with self.assertRaises(ValueError):checker.check_path(target,start)
        self.assertTrue(checker.description['cached_proxy_loaded'])
        self.assertFalse(checker.description['environment_checked'])

    def test_runtime_never_fits_and_cache_corruption_is_rejected(self):
        g=self.g;data=payload(g)
        with patch('robot_workbench.manual_self_collision.components',side_effect=AssertionError('online fit')):
            self.assertTrue(ManualSelfCollision(g,prebuilt=data).check_path(np.zeros(12),np.zeros(12))['ok'])
        for key,value in [('radii',[0.,.003,.003]),('centers',[[float('nan')]*3]*3),
                          ('sphere_parts',[0,1,9]),('support_local',[False]),('original_radii',[.001]*3)]:
            bad=deepcopy(data);bad[key]=value
            with self.assertRaises(ValueError):ManualSelfCollision(g,prebuilt=bad)
            if key!='centers':
                bad['proxy_id']=proxy_digest(bad)
                with self.assertRaises(ValueError):ManualSelfCollision(g,prebuilt=bad)
        bad=deepcopy(data);bad['source_fingerprint']['urdf_sha256']='0'*64;bad['proxy_id']=proxy_digest(bad)
        with self.assertRaises(ValueError):ManualSelfCollision(g,prebuilt=bad)
        missing=deepcopy(data);missing['parts'][1]['name']='link4_missing';missing['proxy_id']=proxy_digest(missing)
        with self.assertRaises(ValueError):ManualSelfCollision(g,prebuilt=missing)

    def test_authored_and_instantiated_sources_bound_without_camera_result(self):
        g=self.g;before=source_fingerprint(g)
        g.config['candidate']='different-camera-calibration.json'
        self.assertEqual(source_fingerprint(g),before)
        g.config['jaw_mapping_file']=str(Path(self.temp.name)/'jaw.json')
        Path(g.config['jaw_mapping_file']).write_text('{"offset":0.008}')
        self.assertNotEqual(source_fingerprint(g),before)
        data=payload(g);g.full_side_meshes['left'][0][1][0,0]+=.001
        with self.assertRaises(ValueError):ManualSelfCollision(g,prebuilt=data)

    def test_robot_static_and_front_support_remain_checked(self):
        g=self.g;name,p,f=g.static_meshes[0];g.static_meshes=[(name,p+[-.2,0,.5],f)]
        self.assertFalse(ManualSelfCollision(g,prebuilt=payload(g)).check_path(np.zeros(12),np.zeros(12))['ok'])
        g.static_adjacent={('link4','chassis')}
        self.assertTrue(ManualSelfCollision(g,prebuilt=payload(g)).check_path(np.zeros(12),np.zeros(12))['ok'])
        g.static_meshes.append(('coffee_machine',p+[-.2,0,.5],f))
        self.assertTrue(ManualSelfCollision(g,prebuilt=payload(g)).check_path(np.zeros(12),np.zeros(12))['ok'])
        points=g.full_side_meshes['left'][0][1];faces=g.full_side_meshes['left'][0][2]
        g.full_side_meshes['left']=[(n,points,faces) for n in ('link2','link3')]
        g.aliases['left']={n:n for n in ('link2','link3')};g.adjacent={frozenset(('link2','link3'))}
        self.assertFalse(ManualSelfCollision(g,prebuilt=payload(g)).check_path(np.zeros(12),np.zeros(12))['ok'])

    def test_components_do_not_merge_disconnected_material(self):
        p=self.g.full_side_meshes['left'][0][1];vertices=np.concatenate((p,p+[.2,0,0]))
        self.assertEqual(len(list(components(vertices,np.arange(len(vertices)).reshape(-1,3)))),2)


if __name__=='__main__':unittest.main()
