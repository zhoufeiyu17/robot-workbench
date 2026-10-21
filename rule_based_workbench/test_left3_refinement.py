"""CPU regression for geometry-bound local cover refinement."""
from copy import deepcopy
from unittest.mock import patch

import numpy as np

import curobo_robot_model as model
from conservative_spheres import cover


def main():
    vertices = [[x, y, z] for x in (-.04, .04) for y in (-.035, .035) for z in (-.14, .14)]
    part = dict(name=model.LEFT3_PART, link='left_link3', vertices=vertices)
    recipe = dict(refine_cells={'unrelated-part': ['4']}, body_pitch_m=.02)
    original = deepcopy(recipe)
    with patch.object(model, 'LEFT3_PART_SHA256', model.digest(part)):
        effective = model.refine_left3_recipe(recipe, [part], 'parent-recipe')
        assert recipe == original and effective['refine_cells']['unrelated-part'] == ['4']
        assert effective['refine_cells'][model.LEFT3_PART] == ['20', '20/0']
        for changed in (dict(part, vertices=vertices[:-1]), dict(part, name='wrong-part')):
            try:
                model.refine_left3_recipe(recipe, [changed], 'parent-recipe')
            except ValueError:
                pass
            else:
                raise AssertionError('unreviewed geometry accepted cell indices')
        try:
            model.refine_left3_recipe(effective, [part], 'changed-parent')
        except ValueError:
            pass
        else:
            raise AssertionError('changed parent refinement was silently reused')
    before_c, before_r, before = cover(vertices, pitch_m=.055)
    after_c, after_r, after = cover(vertices, pitch_m=.055, refine_cells=['20', '20/0'])
    assert before['whole_solid_covered'] and after['whole_solid_covered']
    assert len(after_c) == len(before_c) + 2
    old = {key: np.r_[c, r] for key, c, r in zip(before['cell_keys'], before_c, before_r)}
    new = {key: np.r_[c, r] for key, c, r in zip(after['cell_keys'], after_c, after_r)}
    assert all(np.array_equal(new[key], value) for key, value in old.items() if key != '20')
    assert set(new) - set(old) == {'20/0/0', '20/0/1', '20/1'}
    points = np.asarray(vertices)
    samples = np.random.default_rng(51).dirichlet(np.ones(8), 2000) @ points
    gaps = np.linalg.norm(samples[:, None] - after_c[None], axis=2) - after_r
    assert float(gaps.min(axis=1).max()) <= 0
    print('left_link3 geometry binding, unchanged cells and complete-cell cover: PASS')


if __name__ == '__main__':
    main()
