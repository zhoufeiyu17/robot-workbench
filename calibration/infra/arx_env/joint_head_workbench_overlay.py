"""Full-mesh image overlays; camera and reported joints remain unchanged."""
import numpy as np
from .joint_head_jaw_mapping import feedback_gaps, measured_jaw_gaps


def mesh_overlay(geometry, head, q, candidate):
    """DSW full-mesh silhouette over its paired RTX image, no nominal mutation."""
    import cv2
    output = np.asarray(head).copy()
    gaps = measured_jaw_gaps(candidate['jaw_gap_mm']) if 'jaw_gap_mm' in candidate else None
    articulated = getattr(geometry, 'hand', None) is None and hasattr(geometry, 'with_feedback_gaps')
    view = geometry.with_feedback_gaps(*gaps) if articulated and gaps is not None else geometry
    frames = view.frames(np.asarray(q)+candidate['offsets'])
    camera = candidate['camera_T_world_optical']
    k = geometry.k
    for side, color in [('left', (30, 230, 255)), ('right', (255, 100, 220))]:
        mask = np.zeros(output.shape[:2], np.uint8)
        polygons = []
        for name, vertices, _ in geometry.side_meshes[side]:
            if name == 'link6' and gaps is not None and not articulated:
                vertices = vertices.copy()
                vertices[:,1] += geometry.hand['signs']*(gaps[0 if side=='left' else 1]/1000-geometry.gaps[side])/2
            T = frames[side][name]
            world = vertices@T[:3,:3].T+T[:3,3]
            optical = (world-camera[:3,3])@camera[:3,:3]
            triangles = optical.reshape(-1, 3, 3)
            triangles = triangles[np.all(triangles[:,:,2] > .005, axis=1)]
            uv = triangles[:,:,:2]/triangles[:,:,2,None]*[k['fx'],k['fy']]+[k['cx'],k['cy']]
            # Bound offscreen projections before int conversion/OpenCV clipping.
            uv = np.clip(uv, -32768, 32768).astype(np.int32)
            polygons.extend(uv)
        if polygons:
            # Each triangle is convex; avoid even/odd holes from a single fillPoly call.
            for polygon in polygons:
                cv2.fillConvexPoly(mask, polygon, 255)
            pixels = mask != 0
            output[pixels] = (output[pixels]*.78+np.asarray(color)*.22).astype(np.uint8)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(output, contours, -1, color, 2, cv2.LINE_AA)
    return output


def render_mesh_overlay(geometry, row, candidate, image_path, output_path):
    """Historical overlay uses only the image's recorded joints and intrinsics."""
    import copy
    import json
    from pathlib import Path
    import cv2
    image_path, output_path = Path(image_path), Path(output_path)
    k = json.loads((image_path.parent/'camera.json').read_text())
    if not k.get('rectified') and any(k.get('coefficients', [])):
        raise ValueError('历史图像含畸变；请先生成绑定原帧的 rectified 图像再叠加 mesh')
    head = cv2.imread(str(image_path))
    if head is None or head.shape[:2] != (k['height'], k['width']):
        raise ValueError('historical image/intrinsics dimensions disagree')
    q = np.asarray(row.get('q_reported_rad'), float)
    camera = np.asarray(candidate['camera_T_world_optical'], float)
    offsets = np.asarray([candidate['joint_offsets_rad'][s] for s in ('left','right')], float).reshape(-1)
    if q.shape != (12,) or offsets.shape != (12,) or camera.shape != (4,4) or not all(np.isfinite(x).all() for x in (q,offsets,camera)):
        raise ValueError('invalid historical feedback or calibration')
    if not np.allclose(camera[3], [0,0,0,1]) or not np.allclose(camera[:3,:3].T@camera[:3,:3], np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(camera[:3,:3]),1,atol=1e-5):
        raise ValueError('calibration camera must be rigid')
    view = copy.copy(geometry)
    view.k = k
    result = mesh_overlay(view, cv2.cvtColor(head,cv2.COLOR_BGR2RGB), q,
                          {'camera_T_world_optical':camera,'offsets':offsets,**({'jaw_gap_mm':row['jaw_gap_mm']} if 'jaw_gap_mm' in row else {})})
    if not cv2.imwrite(str(output_path),cv2.cvtColor(result,cv2.COLOR_RGB2BGR)):
        raise ValueError('failed to save historical mesh overlay')
    return output_path


def pair_real_feedback(metadata, history, owner_pid, max_delta_ms=200.):
    """Same robot-host receive/sample clock, not exposure synchronization."""
    stamp = metadata.get('frame_timestamp_ns')
    if type(stamp) is not int or stamp<=0:raise ValueError('图像缺少主机时间戳')
    samples=[v['owner']['calibration_feedback'] for v in history
             if v.get('owner',{}).get('pid')==owner_pid
             and type(v.get('owner',{}).get('calibration_feedback',{}).get('sample_wall_ns')) is int]
    if not samples:raise ValueError('等待图像时刻的真机关节反馈')
    sample=min(samples,key=lambda f:abs(f['sample_wall_ns']-stamp))
    delta=(sample['sample_wall_ns']-stamp)/1e6
    if abs(delta)>max_delta_ms:raise ValueError(f'图像/关节时间差 {delta:+.1f} ms 超过 {max_delta_ms:g} ms')
    return sample,dict(image_joint_delta_ms=delta,image_timestamp_ns=str(stamp),joint_timestamp_ns=str(sample['sample_wall_ns']),
                       timestamp_basis='同一真机主机：图像接收时间与关节采样时间；非曝光硬同步',max_delta_ms=max_delta_ms)


def real_mesh_overlay(geometry, bgr, raw_q, candidate, camera):
    """Project actual raw joints onto real distorted pixels, with selected offsets."""
    import cv2
    from .joint_head_isaac_fit import rigid
    q=np.asarray(raw_q,float)
    if q.shape!=(14,) or not np.isfinite(q).all():raise ValueError('真机关节无效')
    if bgr.shape[:2]!=(camera['height'],camera['width']):raise ValueError('真机图像与内参尺寸不匹配')
    T=rigid(candidate['camera_T_world_optical'])
    offsets=np.asarray([candidate['joint_offsets_rad'][s] for s in ('left','right')],float).reshape(12)
    gaps_mm = measured_jaw_gaps(feedback_gaps(q))
    view = geometry.with_feedback_gaps(*gaps_mm) if getattr(geometry, 'hand', None) is None and hasattr(geometry, 'with_feedback_gaps') else geometry
    frames=view.frames(np.r_[q[:6],q[7:13]]+offsets)
    output=bgr.copy();coeff=np.asarray(camera.get('coefficients',[0.]*5),float)
    model=camera.get('distortion_model','')
    if coeff.shape!=(5,) or not np.isfinite(coeff).all():raise ValueError('相机畸变系数无效')
    if np.any(coeff) and not camera.get('rectified') and model not in ('distortion.inverse_brown_conrady','distortion.brown_conrady','distortion.modified_brown_conrady'):raise ValueError('不支持当前真机畸变模型')
    def distort(x):
        r=(x*x).sum(axis=-1);k1,k2,p1,p2,k3=coeff;rad=1+k1*r+k2*r*r+k3*r*r*r
        return x*rad[...,None]+np.stack([2*p1*x[...,0]*x[...,1]+p2*(r+2*x[...,0]**2),p1*(r+2*x[...,1]**2)+2*p2*x[...,0]*x[...,1]],axis=-1)
    for side,color in [('left',(255,230,30)),('right',(220,100,255))]:
        mask=np.zeros(bgr.shape[:2],np.uint8)
        for name,vertices,_ in geometry.side_meshes[side]:
            # Use the same measured mapping as feedback display and DSW mirror.
            if name=='link6' and getattr(geometry,'hand',None) is not None:
                gap=float(gaps_mm[0 if side=='left' else 1])/1000.
                vertices=vertices.copy();vertices[:,1]+=geometry.hand['signs']*(gap-geometry.gaps[side])/2
            F=frames[side][name];world=vertices@F[:3,:3].T+F[:3,3];opt=(world-T[:3,3])@T[:3,:3]
            tri=opt.reshape(-1,3,3);tri=tri[np.all(tri[:,:,2]>.005,axis=1)]
            xy=tri[:,:,:2]/tri[:,:,2,None]
            keep=np.all(np.abs(xy)<3,axis=(1,2));xy=xy[keep]
            if np.any(coeff) and not camera.get('rectified'):
                if model=='distortion.inverse_brown_conrady':
                    target=xy.copy()
                    for _ in range(20):
                        residual=target-distort(xy)
                        if np.all(np.abs(residual)<1e-5):break
                        xy+=np.clip(residual,-.1,.1)
                    valid=np.all(np.abs(distort(xy)-target)<1e-5,axis=(1,2));xy=xy[valid]
                else:xy=distort(xy)
            uv=xy*[camera['fx'],camera['fy']]+[camera['cx'],camera['cy']]
            for polygon in np.clip(uv,-32768,32768).astype(np.int32):cv2.fillConvexPoly(mask,polygon,255)
        pixels=mask!=0;output[pixels]=(output[pixels]*.78+np.asarray(color)*.22).astype(np.uint8)
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(output,contours,-1,color,2,cv2.LINE_AA)
    return output
