"""Regression: cap the concave extension without filling its open channel."""
from pathlib import Path
import numpy as np
from model_mast import close_extrusion_bottom
from arx_env.joint_head_workbench_geometry import stl_vertices,closed_component_representatives,point_in_mesh


def main():
    assets=Path(__file__).parent/'model_assets'
    before=stl_vertices(assets/'ac_one_baseline_v1/meshes/chassis.stl').reshape(-1,3,3)
    after=stl_vertices(assets/'ac_one_baseline_v1_r2/meshes/chassis.stl').reshape(-1,3,3)
    assert closed_component_representatives(before.reshape(-1,3)) is None
    repaired,report=close_extrusion_bottom(before)
    assert report['boundary_edges']==74 and report['cap_triangles']==72
    assert np.array_equal(before,after[:len(before)])
    assert np.array_equal(repaired.astype(np.float32),after.astype(np.float32))
    assert closed_component_representatives(after.reshape(-1,3)) is not None
    # At z=.35 only the new extrusion exists. Respect the concave U profile.
    for point,inside in [([.014,.023,.35],True),([.045,0,.35],True),([.025,0,.35],False),([.10,0,.35],False)]:
        assert point_in_mesh(np.array(point),after)==inside,(point,inside)
    try:close_extrusion_bottom(np.concatenate([before,before[:1]]))
    except ValueError:pass
    else:raise AssertionError('non-manifold duplicate face accepted')
    print('PASS: 74 boundary edges capped by 72 faces; old vertices unchanged; closed gate; concave containment; invalid input rejected')


if __name__=='__main__':main()
