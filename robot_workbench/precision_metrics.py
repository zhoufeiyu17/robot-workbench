"""CPU-only spatial and vibration diagnostics; never imported by the Owner tick."""
import numpy as np

SPATIAL_CONTRACT = 'spatial_path.v2'


def pose_residual(actual_p, actual_r, reference_p, reference_r, progress):
    """Position and world-frame rotation error at ONE explicitly selected progress.

    progress is a reference sample index plus segment fraction, shared by both
    arms. The caller must resolve phase/dwell/self-intersection ambiguity; this
    function never independently realigns orientation to hide a rotation error.
    """
    from scipy.spatial.transform import Rotation, Slerp
    p, r, target_p, target_r = map(lambda x: np.asarray(x, float),
                                  (actual_p, actual_r, reference_p, reference_r))
    progress = np.asarray(progress, float)
    if (p.ndim != 3 or p.shape[1:] != (2, 3) or len(p) == 0
            or target_p.ndim != 3 or target_p.shape[1:] != (2, 3) or len(target_p) < 2
            or r.shape != (len(p), 2, 3, 3) or target_r.shape != (len(target_p), 2, 3, 3)
            or progress.shape != (len(p),) or np.any(progress < 0)
            or np.any(progress > len(target_p)-1)
            or not all(np.isfinite(x).all() for x in (p, r, target_p, target_r, progress))):
        raise ValueError('finite paired poses and shared in-range progress required')
    for matrices in (r, target_r):
        if (not np.allclose(matrices @ matrices.swapaxes(-1, -2), np.eye(3), atol=1e-6)
                or not np.allclose(np.linalg.det(matrices), 1., atol=1e-6)):
            raise ValueError('proper rotation matrices required')
    index = np.minimum(progress.astype(int), len(target_p)-2)
    alpha = progress-index
    expected_p = target_p[index] + alpha[:, None, None]*(target_p[index+1]-target_p[index])
    rotation_error = []
    for side in (0, 1):
        expected_r = Slerp(np.arange(len(target_p)), Rotation.from_matrix(target_r[:, side]))(progress)
        rotation_error.append((Rotation.from_matrix(r[:, side])*expected_r.inv()).as_rotvec())
    return p-expected_p, np.stack(rotation_error, axis=1)


def residual_band_metrics(times, residual, band_hz=(1., 12.), window_s=2., max_gap_s=.03):
    """Vector RMS of paired TCP residuals, in the input unit; no noisy derivatives.

    Linear trend removal and Hann PSD normalization; never interpolate across
    missing feedback. A spectral observation is not a stability pass/fail gate.
    """
    t, e = np.asarray(times, float), np.asarray(residual, float)
    band = np.asarray(band_hz, float)
    if (t.ndim != 1 or len(t) < 2 or e.shape != (len(t), 2, 3)
            or not np.isfinite(t).all() or not np.isfinite(e).all() or np.any(np.diff(t) <= 0)
            or band.shape != (2,) or not np.isfinite(band).all() or not 0 < band[0] < band[1]
            or not np.isfinite(window_s) or window_s <= 0
            or not np.isfinite(max_gap_s) or max_gap_s <= 0):
        raise ValueError('ordered finite paired residuals and positive spectral bounds required')
    dt = float(np.median(np.diff(t)))
    n = round(window_s/dt)
    result = dict(status='insufficient_contiguous_feedback', passed=None, windows=0,
                  band_hz=band.tolist(), window_s=n*dt, max_gap_s=max_gap_s,
                  scope='encoder-FK residual spectrum; not independent physical-motion metrology')
    if not 16 <= n <= 8192 or band[1] > .45/dt:
        result['status'] = 'sampling_rate_outside_diagnostic_range'
        return result
    window = np.hanning(n)
    frequency = np.fft.rfftfreq(n, dt)
    selected = (frequency >= band[0]) & (frequency <= band[1])
    design = np.column_stack([np.ones(n), np.linspace(-1., 1., n)])
    powers = []
    for indices in np.split(np.arange(len(t)), np.flatnonzero(np.diff(t) > max_gap_s)+1):
        if len(indices) < 2:
            continue
        grid = np.arange(t[indices[0]], t[indices[-1]], dt)
        flat = e[indices].reshape(-1, 6)
        values = np.column_stack([np.interp(grid, t[indices], flat[:, j]) for j in range(6)])
        for start in range(0, len(grid)-n+1, max(1, n//2)):
            segment = values[start:start+n]
            detrended = segment-design @ np.linalg.lstsq(design, segment, rcond=None)[0]
            transform = np.fft.rfft(detrended*window[:, None], axis=0)
            psd = abs(transform)**2*dt/np.sum(window**2)
            psd[1:-1 if n % 2 == 0 else None] *= 2.
            powers.append(np.sum(psd[selected], axis=0).reshape(2, 3).sum(axis=1)/(n*dt))
    if powers:
        result.update(status='observed', windows=len(powers),
                      vector_rms=np.sqrt(np.mean(powers, axis=0)).tolist(),
                      worst_window_vector_rms=np.sqrt(np.max(powers, axis=0)).tolist())
    return result


def project_path(points, path):
    """Exact minimax distance to a paired-TCP polyline, one shared arm progress.

    Minimize max(left distance, right distance) on each segment. A minimum of
    two convex quadratics occurs at a stationary point, crossing, or endpoint.
    No timing alignment, IK, or control calls take place here.
    """
    points, path = np.asarray(points, float), np.asarray(path, float)
    if (points.ndim != 3 or path.ndim != 3 or points.shape[1:] != (2, 3)
            or path.shape[1:] != (2, 3) or not 0 < len(points) <= 200000
            or not 2 <= len(path) <= 20000
            or not np.isfinite(points).all() or not np.isfinite(path).all()):
        raise ValueError('bounded finite paired TCP paths required')
    delta = np.diff(path, axis=0)
    a = np.sum(delta**2, axis=2)
    distances, progress = [], []
    # ponytail: exact O(samples*segments) CPU scan; spatial indexing only if profiling warrants it.
    for start in range(0, len(points), 16):
        offset = points[start:start+16, None] - path[None, :-1]
        b = np.sum(offset * delta, axis=3)
        c = np.sum(offset**2, axis=3)
        qa = a[:, 0] - a[:, 1]
        qb = -2. * (b[:, :, 0]-b[:, :, 1])
        qc = c[:, :, 0]-c[:, :, 1]
        discriminant = np.maximum(0., qb**2-4.*qa*qc)
        linear = np.divide(-qc, qb, out=np.zeros_like(qb), where=abs(qb)>1e-18)
        root1 = np.divide(-qb+np.sqrt(discriminant), 2.*qa,
                          out=linear.copy(), where=abs(qa)>1e-18)
        root2 = np.divide(-qb-np.sqrt(discriminant), 2.*qa,
                          out=linear.copy(), where=abs(qa)>1e-18)
        shape = b.shape[:2]
        candidates = [np.zeros(shape), np.ones(shape), root1, root2]
        candidates += [np.divide(b[:, :, side], a[:, side],
                                 out=np.zeros(shape), where=a[:, side]>1e-18) for side in (0, 1)]
        best, alpha = np.full(shape, np.inf), np.zeros(shape)
        for candidate in candidates:
            t = np.clip(candidate, 0., 1.)
            cost = np.maximum(0., np.max(a*t[:, :, None]**2-2.*b*t[:, :, None]+c, axis=2))
            improved = cost < best
            best[improved], alpha[improved] = cost[improved], t[improved]
        segment = np.argmin(best, axis=1)
        row = np.arange(len(segment))
        fraction = alpha[row, segment]
        projected = path[segment] + fraction[:, None, None]*delta[segment]
        distances.extend(np.linalg.norm(points[start:start+16]-projected, axis=2))
        progress.extend(segment+fraction)
    return np.asarray(distances), np.asarray(progress)


def spatial_metrics(actual, reference, tolerance_mm):
    """Separate path adherence, sampled coverage, endpoints and path reversal."""
    if type(tolerance_mm) not in (int, float) or not np.isfinite(tolerance_mm) or tolerance_mm <= 0:
        raise ValueError('positive finite path tolerance required')
    actual, reference = np.asarray(actual, float), np.asarray(reference, float)
    forward, progress = project_path(actual, reference)
    backward, _ = project_path(reference, actual)
    endpoints = np.linalg.norm(actual[[0, -1]]-reference[[0, -1]], axis=2)
    increments = np.max(np.linalg.norm(np.diff(reference, axis=0), axis=2), axis=1)
    arc = np.r_[0., np.cumsum(increments)]
    projected_arc = np.interp(progress, np.arange(len(arc)), arc)
    backtrack_mm = float(np.max(np.maximum.accumulate(projected_arc)-projected_arc)*1000.)
    peak, coverage, endpoint = [float(np.max(x)*1000.) for x in (forward, backward, endpoints)]
    return dict(contract=SPATIAL_CONTRACT,metric='双臂同一空间进度的TCP折线距离，允许延迟；非外部空间测量',
        path_peak_mm=dict(left=float(np.max(forward[:, 0])*1000.),right=float(np.max(forward[:, 1])*1000.)),
        sampled_coverage_peak_mm=coverage,endpoint_peak_mm=endpoint,
        maximum_progress_backtrack_mm=backtrack_mm,
        projected_backtrack_requires_review=backtrack_mm>tolerance_mm,
        reference_max_segment_mm=float(np.max(increments)*1000.),
        actual_max_segment_mm=float(np.max(np.linalg.norm(np.diff(actual, axis=0), axis=2))*1000.),
        sampled_path_checks_passed=bool(max(peak, coverage, endpoint)<=tolerance_mm),
        scope='sampled paired-TCP polylines only; does not certify continuous motion, orientation or smoothness')


def vibration_metrics(times, error_rad):
    """Observe 3–45Hz error-band RMS without differentiating noisy feedback."""
    times, error = np.asarray(times, float), np.asarray(error_rad, float)
    if (times.ndim != 1 or error.shape != (len(times), 12) or len(times)<2
            or not np.isfinite(times).all() or not np.isfinite(error).all()
            or np.any(np.diff(times)<=0)):
        raise ValueError('ordered finite 12-joint feedback diagnostics required')
    dt = float(np.median(np.diff(times))); n = round(1./dt)
    if not 32 <= n <= 4096:
        return dict(status='sampling_rate_outside_diagnostic_range',passed=None)
    window = np.hanning(n); frequency = np.fft.rfftfreq(n, dt)
    upper = min(45., .45/dt); band = (frequency>=3.) & (frequency<=upper)
    powers = []
    for indices in np.split(np.arange(len(times)),np.flatnonzero(np.diff(times)>.03)+1):
        if len(indices)<2:continue
        t=times[indices];grid=np.arange(t[0],t[-1],dt)
        e=np.column_stack([np.interp(grid,t,error[indices,j])for j in range(12)])
        for start in range(0,len(grid)-n+1,n//2):
            segment=e[start:start+n]
            transform=np.fft.rfft((segment-segment.mean(axis=0))*window[:,None],axis=0)
            psd=abs(transform)**2*dt/np.sum(window**2)
            psd[1:-1 if n%2==0 else None]*=2.
            powers.append(np.sum(psd[band],axis=0)/(n*dt))
    result=dict(status='observed'if powers else'insufficient_contiguous_feedback',passed=None,
        windows=len(powers),window_s=n*dt,band_hz=[3.,upper],
        scope='host-sampled diagnostic; aliasing/noise possible; low-frequency vibration requires separate inspection')
    if powers:
        result.update(mean_band_rms_deg=np.degrees(np.sqrt(np.mean(powers,axis=0))).tolist(),
            worst_window_band_rms_deg=np.degrees(np.sqrt(np.max(powers,axis=0))).tolist())
    return result
