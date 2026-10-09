"""CUDA real-camera silhouettes. No simulation scene or pixels are consumed."""
import threading
import numpy as np
import torch
import warp as wp
from .joint_head_jaw_mapping import feedback_gaps, measured_jaw_gaps
from .joint_head_isaac_fit import rigid


@wp.kernel
def rasterize(tri: wp.array(dtype=wp.vec3), mask: wp.array2d(dtype=wp.int32), width: int, height: int):
    i = wp.tid()
    a, b, c = tri[3*i], tri[3*i+1], tri[3*i+2]
    if a[2] == 0.0 or b[2] == 0.0 or c[2] == 0.0:
        return
    lo_x = wp.max(0, int(wp.floor(wp.min(a[0], wp.min(b[0], c[0])))))
    hi_x = wp.min(width-1, int(wp.ceil(wp.max(a[0], wp.max(b[0], c[0])))))
    lo_y = wp.max(0, int(wp.floor(wp.min(a[1], wp.min(b[1], c[1])))))
    hi_y = wp.min(height-1, int(wp.ceil(wp.max(a[1], wp.max(b[1], c[1])))))
    area = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    if wp.abs(area) < 0.000001:
        return
    for y in range(lo_y, hi_y+1):
        for x in range(lo_x, hi_x+1):
            px, py = float(x)+0.5, float(y)+0.5
            e0 = (b[0]-a[0])*(py-a[1])-(b[1]-a[1])*(px-a[0])
            e1 = (c[0]-b[0])*(py-b[1])-(c[1]-b[1])*(px-b[0])
            e2 = (a[0]-c[0])*(py-c[1])-(a[1]-c[1])*(px-c[0])
            if (e0 >= 0.0 and e1 >= 0.0 and e2 >= 0.0) or (e0 <= 0.0 and e1 <= 0.0 and e2 <= 0.0):
                wp.atomic_max(mask, y, x, 1)


class GPUOverlay:
    """One CUDA launch per arm per frame, not one per mesh part.

    Every part is a rigid body of the same kinematic chain and the rasterizer is
    per-triangle, so the parts can be concatenated into a single vertex block per
    side and transformed by per-vertex part matrices.  Doing that removes, per
    overlay: 18 of the 20 launches, 36 of the 40 small host transfers and 18 of the
    20 distortion inversions.  This render runs inside the simulation renderer's own
    process, where every host round trip waits for the render loop's queued GPU work
    (measured 2026-09-22: 1.3 s per overlay in that process, 0.15 s for the same
    render off it).  Projection, distortion inversion and rasterization are unchanged
    per vertex, so the pixels are the same as the per-part version.
    """
    def __init__(self, geometry):
        wp.init()
        self.geometry = geometry
        self.lock = threading.Lock()
        self.device = 'cuda:0'
        self.sides = {}
        for side, meshes in geometry.side_meshes.items():
            names = [name for name, _, _ in meshes]
            bounds, owner = [], []
            for index, (name, vertices, _) in enumerate(meshes):
                bounds.append((name, len(owner), len(owner)+len(vertices)))
                owner.extend([index]*len(vertices))
            base = np.concatenate([np.asarray(vertices, float) for _, vertices, _ in meshes])
            self.sides[side] = dict(names=names, bounds=bounds,
                                    base=torch.as_tensor(base.astype(np.float32), device=self.device),
                                    owner=torch.as_tensor(np.asarray(owner), dtype=torch.int64, device=self.device),
                                    link6=next(((lo, hi) for name, lo, hi in bounds if name == 'link6'), None))
        self.signs = torch.as_tensor(geometry.hand['signs'], device=self.device) if geometry.hand is not None else None

    @torch.inference_mode()
    def render(self, bgr, raw_q, candidate, camera):
        import cv2
        q = np.asarray(raw_q, float)
        if q.shape != (14,) or not np.isfinite(q).all():raise ValueError('invalid real joint feedback')
        h, w = bgr.shape[:2]
        if (h,w) != (camera['height'],camera['width']):raise ValueError('camera dimensions disagree')
        T = rigid(candidate['camera_T_world_optical'])
        offsets = np.asarray([candidate['joint_offsets_rad'][s] for s in ('left','right')]).reshape(12)
        gaps = measured_jaw_gaps(feedback_gaps(q))
        view = self.geometry.with_feedback_gaps(*gaps) if self.geometry.hand is None else self.geometry
        frames = view.frames(np.r_[q[:6],q[7:13]]+offsets)
        coeff = np.asarray(camera.get('coefficients',[0.]*5),float)
        if coeff.shape != (5,) or not np.isfinite(coeff).all():raise ValueError('invalid distortion coefficients')
        model = camera.get('distortion_model','')
        distorted = np.any(coeff) and not camera.get('rectified')
        if distorted and model not in ('distortion.inverse_brown_conrady','distortion.brown_conrady','distortion.modified_brown_conrady'):raise ValueError('unsupported distortion')
        # Focal/principal as one transfer, not one per part.
        focal = torch.tensor([camera['fx'],camera['fy']],device=self.device)
        centre = torch.tensor([camera['cx'],camera['cy']],device=self.device)
        with self.lock:
            masks = {s: torch.zeros((h,w),dtype=torch.int32,device=self.device) for s in ('left','right')}
            for side, part in self.sides.items():
                bounds = part['bounds']
                # Both matrices for every part go up in two transfers.
                rotation = torch.as_tensor(np.stack([frames[side][name][:3,:3].T@T[:3,:3] for name in part['names']]).astype(np.float32),device=self.device)
                translation = torch.as_tensor(np.stack([(frames[side][name][:3,3]-T[:3,3])@T[:3,:3] for name in part['names']]).astype(np.float32),device=self.device)
                vertices = part['base']
                if part['link6'] is not None and self.signs is not None:
                    lo, hi = part['link6']
                    gap = gaps[0 if side=='left' else 1]/1000
                    vertices = part['base'].clone()
                    vertices[lo:hi,1] += self.signs*(gap-self.geometry.gaps[side])/2
                owner = part['owner']
                optical = torch.einsum('ni,nij->nj',vertices,rotation[owner])+translation[owner]
                valid = (optical[:,2]>.005).reshape(-1,3).all(1)
                xy = optical[:,:2]/optical[:,2,None]
                valid &= (xy.abs()<3).reshape(-1,6).all(1)
                def distort(x):
                    k1,k2,p1,p2,k3 = coeff
                    r = (x*x).sum(1);rad = 1+k1*r+k2*r*r+k3*r*r*r
                    return x*rad[:,None]+torch.stack((2*p1*x[:,0]*x[:,1]+p2*(r+2*x[:,0]**2),p1*(r+2*x[:,1]**2)+2*p2*x[:,0]*x[:,1]),1)
                if distorted:
                    if model == 'distortion.inverse_brown_conrady':
                        # Fixed iteration budget, and never a host read inside the loop:
                        # `if torch.all(...)` forces a device-to-host synchronization, and
                        # this render shares its stream with the simulation loop, so every
                        # check waited for the render loop's queued work. 138 syncs per
                        # overlay used to cost 1.4 s (measured 2026-09-22). The clamped
                        # fixed point is reached either way, and acceptance is still
                        # decided by the convergence test below.
                        target = xy.clone()
                        for _ in range(20):
                            xy += (target-distort(xy)).clamp(-.1,.1)
                        valid &= ((distort(xy)-target).abs()<1e-5).reshape(-1,6).all(1)
                    else:xy = distort(xy)
                uv = xy*focal+centre
                projected = torch.cat((uv.clamp(-32768,32768),valid.repeat_interleave(3)[:,None]),1).contiguous()
                stream = wp.stream_from_torch(torch.cuda.current_stream())
                wp.launch(rasterize,dim=len(part['base'])//3,inputs=[wp.from_torch(projected,dtype=wp.vec3),wp.from_torch(masks[side]),w,h],stream=stream,device=self.device)
            image = torch.as_tensor(bgr,device=self.device).float()
            for side,color in [('left',(255,230,30)),('right',(220,100,255))]:
                mask = masks[side].bool()
                image = torch.where(mask[:,:,None],image*.78+torch.tensor(color,device=self.device)*.22,image)
            output = image.byte().cpu().numpy()
            # Contour decoration is small; projection, rasterization and blending are CUDA.
            for side,color in [('left',(255,230,30)),('right',(220,100,255))]:
                mask = masks[side].byte().cpu().numpy()
                contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(output,contours,-1,color,2,cv2.LINE_AA)
            return output
