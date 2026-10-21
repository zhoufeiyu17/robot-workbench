"""The C10 recipe is explicit, geometry-bound, deterministic and leaves C9 intact."""
from copy import deepcopy
import json
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace as NS

from curobo_robot_model import digest,refine_pickup_recipe


def main():
    approved=json.loads(Path(__file__).with_name('a1_pickup_refinement.json').read_text())
    parts=[dict(name=name,vertices=[[0.,0.,0.]]) for name in approved['cells']]
    recipe=dict(refine_cells={name:['old-cell'] for name in approved['cells']})
    approved.update(parent_effective_recipe_sha256=digest(recipe),source_parts_sha256=digest(parts),
        part_sha256={p['name']:digest(p) for p in parts})
    original=deepcopy(recipe)
    with patch.object(Path,'read_text',return_value=json.dumps(approved)):
        result=refine_pickup_recipe(recipe,parts,'pickup-c10-v1')
        assert recipe==original
        for name,leaves in approved['cells'].items():
            assert set(result['refine_cells'][name])>=set(recipe['refine_cells'][name])
            for leaf in leaves:
                assert {leaf,leaf+'/0',leaf+'/1',*(leaf+'/'+a+'/'+b for a in '01' for b in '01')}<=set(result['refine_cells'][name])
        for changed,profile in ((parts,'unknown'),(parts[:-1],'pickup-c10-v1')):
            try:refine_pickup_recipe(recipe,changed,profile)
            except ValueError:pass
            else:raise AssertionError('unreviewed C10 profile or geometry accepted')
        try:refine_pickup_recipe(dict(recipe,body_pitch_m=.1),parts,'pickup-c10-v1')
        except ValueError:pass
        else:raise AssertionError('changed C9 parent accepted')
    descendant=json.loads(Path(__file__).with_name('a1_pickup_refinement_v2.json').read_text())
    descendant.update(parent_effective_recipe_sha256=digest(result),source_parts_sha256=digest(parts))
    with patch.object(Path,'read_text',side_effect=[json.dumps(approved),json.dumps(descendant)]):
        c11=refine_pickup_recipe(recipe,parts,'pickup-c11-v1')
    wrist=next(iter(descendant['extra_refine_cells']))
    assert set(c11['refine_cells'][wrist])-set(result['refine_cells'][wrist])==set(descendant['extra_refine_cells'][wrist])
    assert c11['pickup_refinement']['total_sphere_cap']==1819
    from physical_cup_return_geometry import verify_pickup_model_indices
    mapping={'wrist':[2,3],'left_claw':[4],'right_claw':[5],'attached_object':[8,9,10,11]}
    links=dict(zip(mapping,('left_link6','left_link7','left_link8','left_link6')))
    model=NS(total_spheres=12,get_sphere_index_from_link_name=lambda name:NS(cpu=lambda:NS(tolist=lambda:mapping[name])))
    native=NS(offset=NS(numel=lambda:12),collision_matrix=NS(shape=(12,12)),experimental_kernel=False)
    engine=NS(generator=NS(kinematics=NS(kinematics_config=model),rollout_fn=NS(robot_self_collision_constraint=NS(self_collision_kin_config=native))),
        config={'kinematics':{'collision_link_names':list(mapping),'extra_links':{k:{'parent_link_name':v}for k,v in links.items()}}},
        cup_contact=NS(wrist_indices=[2,3,8,9,10,11],claw_indices=[4,5]))
    assert verify_pickup_model_indices(engine,{'proxy':{'allocated_spheres':12}})['physical_wrist_spheres']==2
    engine.cup_contact.wrist_indices.remove(3)
    try:verify_pickup_model_indices(engine,{'proxy':{'allocated_spheres':12}})
    except ValueError:pass
    else:raise AssertionError('stale physical wrist index accepted')
    print('C10 explicit profile, fixed three binary levels, source/parent binding and unchanged input: PASS')


if __name__=='__main__':main()
