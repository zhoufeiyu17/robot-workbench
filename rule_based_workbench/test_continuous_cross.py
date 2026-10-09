"""Joined geometry must keep moving through former segment boundaries."""
import math
from motiongen_motion import continuous_path, discrete_motion


def main():
    path=[]
    for segment in range(3):
        for i in range(61):
            t=i/60; eased=10*t**3-15*t**4+6*t**5
            path.append([.1*(segment+eased),.2*(segment+eased),0.,0.,0.,0.])
    joined,receipt=continuous_path(path,2.,5.)
    assert joined[0]==path[0] and joined[-1]==path[-1]
    assert discrete_motion(joined,2.,5.)['passed']
    assert all(abs(q[1]-2*q[0])<1e-12 for q in joined)
    for boundary in (.1,.2):
        index=min(range(len(joined)),key=lambda i:abs(joined[i][0]-boundary))
        assert (joined[index+1][0]-joined[index][0])*30>.03
    assert receipt['final_gpu_path_query_required'] and not receipt['internal_stop_commands']
    approach=[[.2*(i/200)**3,0.,0.,0.,0.,0.] for i in range(201)]
    quick,proof=continuous_path(approach,2.,5.,positions=[q[:3] for q in approach],tcp_speed=.08,frontloaded=True)
    speed=[(b[0]-a[0])*30 for a,b in zip(quick,quick[1:])]
    n=len(speed)//4
    assert sum(speed[n:2*n])/n > 2*sum(speed[-n:])/n
    assert len(quick)<len(approach) and discrete_motion(quick,2.,5.)['passed']
    slower,_=continuous_path(path,.02,.1)
    assert len(slower)>len(joined) and discrete_motion(slower,.02,.1)['passed']
    for invalid in ([[0.]*6]*3,[[0.]*6,[math.nan]*6]):
        try:continuous_path(invalid,2.,5.)
        except ValueError:pass
        else:raise AssertionError('invalid continuous path accepted')
    print('cross retains geometry/endpoints, nonzero internal speed and original limits PASS')


if __name__=='__main__':main()
