"""Geometric containment regression; no robot/physics/GPU qualification."""
import math
import unittest
import numpy as np
from scene_asset_geometry import cup_bands


class CupEnvelopeTest(unittest.TestCase):
    def test_taper_and_narrow_unsampled_rim_are_contained(self):
        vertices=[]
        rings=[(0.,.025),(.04,.03),(.08,.035),(.109999,.04),(.11,.037)]
        n=32
        for z,r in rings:
            vertices.extend((r*math.cos(i*2*math.pi/n),r*math.sin(i*2*math.pi/n),z)for i in range(n))
        faces=[]
        for k in range(len(rings)-1):
            for i in range(n):
                a=k*n+i;b=k*n+(i+1)%n;faces.extend(((a,b,b+n),(a,b+n,a+n)))
        bands,proof=cup_bands(vertices,faces)
        self.assertTrue(proof['full_triangle_surface_containment'])
        self.assertGreaterEqual(proof['minimum_vertex_containment_m'],-1e-10)
        self.assertLess(len(bands),64)
        self.assertGreaterEqual(proof['minimum_band_height_m'],.0001)
        # This protrusion falls between the uniform proposal samples. The
        # original-vertex proof must add it rather than silently losing it.
        for z,r in rings:
            b=next(b for b in bands if b['z_min_m']-1e-12<=z<=b['z_max_m']+1e-12)
            f=(z-b['z_min_m'])/(b['z_max_m']-b['z_min_m'])
            radius=b['radius_start_m']+(b['radius_end_m']-b['radius_start_m'])*f
            self.assertGreaterEqual(radius*math.cos(math.pi/32)+1e-10,r)
        bad=np.array(vertices);bad[:,2]-=.01
        with self.assertRaises(ValueError):cup_bands(bad,faces)


if __name__=='__main__':unittest.main()
