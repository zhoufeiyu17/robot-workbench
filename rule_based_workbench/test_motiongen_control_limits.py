"""Native success must not hide a short endpoint step exceeding 30Hz limits."""
from motiongen_motion import discrete_motion

def main():
    # Actual p2 approach tail: 6.564975rad/s², although native success=True.
    values=[1.7253470582962036,1.7273470582962036,1.7333470582962036,1.7414090633392334,1.749471187591553,1.7502388954162595,1.7502388954162595]
    path=[[v,0.,0.,0.,0.,0.] for v in values]
    result=discrete_motion(path,2.,5.)
    assert not result['passed'] and abs(result['max_acceleration_rad_s2']-6.564974784851674)<1e-9
    assert not discrete_motion([[0.]*6,[.006]*6,[.012]*6],2.,5.)['passed']  # Join to/from rest matters.
    assert discrete_motion([[0.]*6,[.001]*6,[.002]*6,[.002]*6],2.,5.)['passed']
    for bad in ([[0.]*6,[float('nan')]*6],[[0.]*5,[0.]*5]):
        try:discrete_motion(bad,2.,5.)
        except ValueError:pass
        else:raise AssertionError('invalid control path accepted')
    print('30Hz finite differences, zero-speed joins and invalid paths PASS')

if __name__=='__main__':main()
