"""CPU counterexamples: delay is allowed, lost geometry/order is not hidden."""
import unittest
import numpy as np
from robot_workbench.precision_metrics import project_path,spatial_metrics,vibration_metrics,pose_residual,residual_band_metrics


class PrecisionMetricsTest(unittest.TestCase):
    def test_pose_delay_allowed_but_orientation_wobble_and_invalid_rotations_visible(self):
        from scipy.spatial.transform import Rotation
        p=np.zeros((101,2,3));p[:,:,0]=np.linspace(0.,.1,101)[:,None]
        r=np.tile(np.eye(3),(101,2,1,1))
        r[:,0]=Rotation.from_euler('z',np.linspace(0.,.3,101)).as_matrix()
        progress=np.r_[np.zeros(20),np.arange(101)]
        actual_p=p[progress.astype(int)];actual_r=r[progress.astype(int)].copy()
        pe,re=pose_residual(actual_p,actual_r,p,r,progress)
        np.testing.assert_allclose(pe,0.,atol=1e-12);np.testing.assert_allclose(re,0.,atol=1e-12)
        actual_r[:,0]=Rotation.from_euler('x',.01).as_matrix()@actual_r[:,0]
        pe,re=pose_residual(actual_p,actual_r,p,r,progress)
        np.testing.assert_allclose(np.linalg.norm(re[:,0],axis=1),.01,atol=1e-12)
        actual_r[0,0]=0.
        with self.assertRaises(ValueError):pose_residual(actual_p,actual_r,p,r,progress)

    def test_taskspace_band_amplitude_gap_and_nyquist(self):
        t=np.arange(0.,6.,.01);e=np.zeros((len(t),2,3));e[:,0,2]=.2*np.sin(2*np.pi*5*t)
        result=residual_band_metrics(t,e)
        self.assertAlmostEqual(result['vector_rms'][0],.2/np.sqrt(2),places=4)
        self.assertEqual(result['vector_rms'][1],0.);self.assertIsNone(result['passed'])
        hit=(t<1.5)|(t>4.5)
        self.assertEqual(residual_band_metrics(t[hit],e[hit])['windows'],0)
        self.assertEqual(residual_band_metrics(t,e,(1.,55.))['status'],'sampling_rate_outside_diagnostic_range')
        with self.assertRaises(ValueError):residual_band_metrics(t,e,(1.,float('nan')))

    def test_delay_and_dwell_do_not_change_path_but_offset_does(self):
        path=np.zeros((101,2,3));path[:,:,0]=np.linspace(0.,.1,101)[:,None]
        actual=np.concatenate([path[:1].repeat(30,axis=0),path,path[-1:].repeat(30,axis=0)])
        self.assertTrue(spatial_metrics(actual,path,1.)['sampled_path_checks_passed'])
        actual[:,:,1]+=.0011
        report=spatial_metrics(actual,path,1.)
        self.assertFalse(report['sampled_path_checks_passed'])
        self.assertAlmostEqual(report['path_peak_mm']['left'],1.1)

    def test_shared_progress_and_exact_minimax_not_independent_arm_matching(self):
        path=np.zeros((2,2,3));path[1,:,0]=[.1,.2]
        point=np.zeros((1,2,3));point[0,:,0]=[0.,.2]
        distances,progress=project_path(point,path)
        self.assertAlmostEqual(progress[0],2./3.)
        np.testing.assert_allclose(distances[0],[.2/3.,.2/3.],atol=1e-12)

    def test_stationary_at_closed_path_start_cannot_pass_coverage(self):
        path=np.zeros((3,2,3));path[1,:,0]=.1
        actual=np.zeros((10,2,3))
        report=spatial_metrics(actual,path,1.)
        self.assertEqual(report['endpoint_peak_mm'],0.)
        self.assertGreater(report['sampled_coverage_peak_mm'],99.)
        self.assertFalse(report['sampled_path_checks_passed'])

    def test_backtracking_not_hidden_by_zero_path_distance(self):
        path=np.zeros((101,2,3));path[:,:,0]=np.linspace(0.,.1,101)[:,None]
        actual=np.concatenate([path,path[::-1],path])
        report=spatial_metrics(actual,path,1.)
        self.assertLess(report['path_peak_mm']['left'],1e-6)
        self.assertGreater(report['maximum_progress_backtrack_mm'],99.)
        self.assertTrue(report['sampled_path_checks_passed'],'geometry alone does not establish smoothness or ordered traversal')
        self.assertTrue(report['projected_backtrack_requires_review'])

    def test_repeated_segments_and_bad_data(self):
        path=np.zeros((3,2,3))
        np.testing.assert_allclose(project_path(path,path)[0],0.)
        for value in (np.full((3,2,3),np.nan),np.zeros((3,3))):
            with self.assertRaises(ValueError):project_path(value,path)

    def test_high_frequency_normalization_and_gap_not_filled(self):
        times=np.arange(0.,4.,.01)
        error=np.tile((.1*np.sin(2*np.pi*38*times))[:,None],(1,12))
        result=vibration_metrics(times,error)
        np.testing.assert_allclose(result['mean_band_rms_deg'],np.degrees(.1/np.sqrt(2)),rtol=.02)
        self.assertIsNone(result['passed'],'spectral RMS is not a universal stability qualification')
        times=np.r_[np.arange(0.,.6,.01),np.arange(2.,2.6,.01)]
        self.assertEqual(vibration_metrics(times,np.zeros((len(times),12)))['windows'],0)
        with self.assertRaises(ValueError):vibration_metrics([0.,0.],np.zeros((2,12)))


if __name__=='__main__':unittest.main()
