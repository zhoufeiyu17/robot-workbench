from copy import deepcopy
import json
from pathlib import Path
from cup_envelope import transformed_spheres


def main():
    class Transform:
        def transform_point(self,p):return (-p[1]+1.,p[0]+2.,p[2]+3.)
    proof=json.loads(Path(__file__).with_name('a1_cup_inset.json').read_text())
    identity=proof['cup_geometry_sha256']
    actual=transformed_spheres(proof,identity,Transform(),265)
    p=proof['first_band_spheres'][0]
    assert actual[0]==[-p[1]+1.,p[0]+2.,p[2]+3.,p[3]]
    assert actual[-1]==[0.,0.,0.,-10.] and len(actual)==265
    for key,value in [('cup_geometry_sha256','changed'),('sphere_count',2),('core_coverage_proven',False),
                      ('additional_pair_allowance_m',.003),('maximum_uncovered_core_depth_m',.003),('minimum_surface_inset_m',0.),
                      ('interaction_geometry_modified',True),('external_pair_scope','machine_only'),
                      ('first_band_spheres',[[0.,0.,0.,-1.]])]:
        bad=deepcopy(proof);bad[key]=value
        try:transformed_spheres(bad,identity,Transform(),265)
        except ValueError:pass
        else:raise AssertionError(key)
    try:transformed_spheres(proof,identity,Transform(),263)
    except ValueError:pass
    else:raise AssertionError('capacity silently truncated')
    from cup_envelope import nominal_clearance,verify_offline
    import numpy as np
    from curobo_robot_model import digest
    assert verify_offline(proof)['passed']
    bad=deepcopy(proof)
    center=np.asarray([bad['first_band_spheres'][0][:3]],dtype=float)
    first=bad['source_bands'][0];center[0,2]+=(first['z_min_m']+first['z_max_m'])/2
    # Leave only 2.999 mm inside the actual source surface. A fixed 0.2 mm
    # increase previously left 3.017 mm and was therefore still legal.
    bad['first_band_spheres'][0][3]=float(nominal_clearance(center,bad['source_bands'])[0])-.003+1e-6
    bad['proof']['spheres_sha256']=digest(bad['first_band_spheres'])
    try:verify_offline(bad)
    except ValueError:pass
    else:raise AssertionError('sphere intruding into the 3mm shell accepted')
    print('264-sphere inward cup identity, transform, 3mm minimum / 6mm maximum inset and interaction separation PASS')

if __name__=='__main__':main()
