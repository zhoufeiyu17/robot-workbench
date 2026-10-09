#!/usr/bin/env python3
"""Render true X5 exported geometry in Isaac RTX; detect pixels, never ideal corners.

This is a kinematic camera experiment, not a PhysX joint-control qualification.
Run with Isaac python, --request JSON --output NEW_DIRECTORY. A request contains
camera T_world_optical, fx/fy/cx/cy,width,height and stations with id,q,split,
parts (vertices+faces), markers (id,corners_world ordered TL,TR,BR,BL).
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import time


def validate(request):
    import numpy as np
    camera = request['camera']
    for key in ('fx', 'fy', 'cx', 'cy', 'width', 'height'):
        if not np.isfinite(camera[key]):
            raise ValueError('nonfinite camera')
    if not 16 <= camera['width'] <= 4096 or not 16 <= camera['height'] <= 4096:
        raise ValueError('invalid resolution')
    if min(camera['fx'], camera['fy']) <= 0:
        raise ValueError('invalid focal length')
    T = np.asarray(camera['T_world_optical'], float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError('invalid optical transform')
    if not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-5):
        raise ValueError('camera is not a rigid transform')
    if not request['stations'] or len(request['stations']) > 200:
        raise ValueError('expect 1..200 stations')
    ids = set()
    for s in request['stations']:
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', s['id']) or s['id'] in ids:
            raise ValueError('invalid or duplicate station id')
        ids.add(s['id'])
        if len(s.get('recovery',[])) > 2:
            raise ValueError('at most two recovery attempts')
        q = np.asarray(s['q'], float)
        if q.shape != (12,) or not np.isfinite(q).all() or s['split'] not in ('fit', 'holdout', 'test'):
            raise ValueError('invalid station')
        for p in s.get('parts', request.get('shared_parts', [])):
            v, f = np.asarray(p['vertices'], float), np.asarray(p['faces'])
            if v.ndim != 2 or v.shape[1] != 3 or not np.isfinite(v).all():
                raise ValueError('invalid mesh vertices')
            if f.ndim != 2 or f.shape[1] != 3 or f.dtype.kind not in 'iu' or f.min() < 0 or f.max() >= len(v):
                raise ValueError('invalid triangle indices')
        for transforms in s.get('transforms', {}).values():
            for matrix in transforms.values():
                matrix = np.asarray(matrix, float)
                if matrix.shape != (4,4) or not np.isfinite(matrix).all():
                    raise ValueError('invalid link transform')
        for retry in s.get('recovery', [])[:2]:
            validate({**request, 'stations':[{**s, **retry, 'recovery':[]}]})
        for m in s['markers']:
            corners = np.asarray(m['corners_world'], float)
            if m['id'] not in (24, 30) or corners.shape != (4, 3) or not np.isfinite(corners).all():
                raise ValueError('invalid marker')
    return request


def render(request, output):
    import numpy as np
    import cv2
    from PIL import Image
    from pxr import Gf, UsdGeom, UsdShade, UsdLux, Sdf
    import omni.usd
    import omni.replicator.core as rep

    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    mats = []
    for i, color in enumerate(((.008,)*3, (.95,)*3, (.26,.29,.34))):
        mat = UsdShade.Material.Define(stage, f'/World/Materials/m{i}')
        shader = UsdShade.Shader.Define(stage, f'/World/Materials/m{i}/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.85)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        mats.append(mat)
    light = UsdLux.DomeLight.Define(stage, '/World/Light')
    light.CreateIntensityAttr(1200.)
    c = request['camera']
    (output/'camera.json').write_text(json.dumps({**{k:v for k,v in c.items() if k!='T_world_optical'}, 'distortion_model':'distortion.inverse_brown_conrady', 'coefficients':[0.,0.,0.,0.,0.], 'rectified':True}, indent=2))
    cam = UsdGeom.Camera.Define(stage, '/World/Camera')
    # USD camera looks along -Z with +Y up; input optical is +Z forward,+Y down.
    pose = np.asarray(c['T_world_optical']) @ np.diag([1,-1,-1,1])
    cam.AddTransformOp().Set(Gf.Matrix4d(*pose.T.ravel().tolist()))
    unit = .01
    cam.CreateFocalLengthAttr(c['fx'] * unit)
    cam.CreateHorizontalApertureAttr(c['width'] * unit)
    cam.CreateVerticalApertureAttr(c['height'] * unit * c['fx'] / c['fy'])
    cam.CreateHorizontalApertureOffsetAttr((c['width']/2-c['cx']) * unit)
    cam.CreateVerticalApertureOffsetAttr((c['cy']-c['height']/2) * unit * c['fx']/c['fy'])
    cam.CreateClippingRangeAttr(Gf.Vec2f(.005, 20.))
    rp = rep.create.render_product(str(cam.GetPath()), (int(c['width']), int(c['height'])))
    rgb = rep.AnnotatorRegistry.get_annotator('rgb')
    rgb.attach([rp])
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    def mesh(path, vertices, faces, material):
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr(np.asarray(vertices).tolist())
        m.CreateFaceVertexCountsAttr([len(f) for f in faces])
        m.CreateFaceVertexIndicesAttr(np.asarray(faces).ravel().tolist())
        m.CreateDoubleSidedAttr(True)
        m.CreateSubdivisionSchemeAttr('none')
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material)

    frames = []
    pending = [(s, s, 0) for s in request['stations']]
    while pending:
        original, station, attempt = pending.pop(0)
        if stage.GetPrimAtPath('/World/Robot'):
            stage.RemovePrim('/World/Robot')
        for i, p in enumerate(station.get('parts', request.get('shared_parts', []))):
            vertices = np.asarray(p['vertices'])
            if 'side' in p:
                T = np.asarray(station['transforms'][p['side']][p['link']])
                vertices = vertices @ T[:3,:3].T + T[:3,3]
            mesh(f'/World/Robot/Part{i}', vertices, p['faces'], mats[2])
        for m in station['markers']:
            corners = np.asarray(m['corners_world'])
            origin, dx, dy = corners[0], corners[1]-corners[0], corners[3]-corners[0]
            pattern = np.pad(cv2.aruco.generateImageMarker(dictionary, m['id'], 6), 1, constant_values=255)
            for y in range(8):
                for x in range(8):
                    a, b = (x-1)/6, (y-1)/6
                    v = [origin+u*dx+w*dy for u,w in ((a,b),(a+1/6,b),(a+1/6,b+1/6),(a,b+1/6))]
                    mesh(f"/World/Robot/Tag{m['id']}/Cell{x}_{y}", v, [[0,1,2,3]], mats[int(pattern[y,x]>0)])
        for _ in range(8):
            rep.orchestrator.step(rt_subframes=8)
        image = np.asarray(rgb.get_data())[...,:3].copy()
        Image.fromarray(image).save(output / (station['id']+'.png'))
        corners, ids, _ = detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY))
        detections = {} if ids is None else {str(int(i)):p.reshape(4,2).tolist() for i,p in zip(ids.ravel(),corners)}
        expected = original.get('expected_marker_id')
        accepted = str(expected) in detections if expected is not None else bool(detections)
        alternatives = original.get('recovery', [])[:2]
        if not accepted and attempt < len(alternatives):
            retry = {**original, **alternatives[attempt], 'id':original['id'], 'split':original['split']}
            pending.insert(0, (original, retry, attempt+1))
            continue
        q = station['q']
        row = {'id':station['id'], 'split':station['split'], 'image':station['id']+'.png',
               'qpos':q[:6]+[.083]+q[6:]+[.083], 'q_reported_rad':q,
               'detections':detections, 'valid_for_fit':True, 'marker_detected':accepted, 'transition':bool(original.get('transition',False)), 'recovered':attempt>0 and accepted, 'recovery_attempts':attempt, 'expected_marker_id':expected, 'bracket_gap_s':0., 'pose_span_rad':[0.]*12,
               'source':'Isaac RTX X5 kinematic rendered pixels', 'feedback_kind':'commanded kinematic state; not motor feedback'}
        (output/(station['id']+'.json')).write_text(json.dumps(row,indent=2))
        frames.append(row)
        print(json.dumps({'captured':station['id'],'detected_ids':list(detections)}),flush=True)
    (output/'dataset.json').write_text(json.dumps({'schema':'x5-rtx-kinematic-v1','camera':{k:v for k,v in c.items() if k!='T_world_optical'},'frames':frames,
        'physics_tracking_validated':False,'ideal_corners_used_as_measurements':False},indent=2))
    return frames


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--request',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--dry-run',action='store_true')
    args=p.parse_args()
    request=validate(json.loads(args.request.read_text()))
    if args.output.exists():
        raise FileExistsError(args.output)
    if args.dry_run:
        print(json.dumps({'valid':True,'stations':len(request['stations']),'request_sha256':hashlib.sha256(args.request.read_bytes()).hexdigest()}))
        return
    args.output.mkdir(parents=True)
    (args.output/'run.json').write_text(json.dumps({'pid':os.getpid(),'started':time.time(),'runner_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'request_sha256':hashlib.sha256(args.request.read_bytes()).hexdigest()}))
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'active_gpu':0,'multi_gpu':False,'renderer':'RayTracedLighting'})
    try:
        frames=render(request,args.output)
        (args.output/'completion.json').write_text(json.dumps({'passed':True,'captured':len(frames),'accepted':len(frames),'failed':0,'marker_detected':sum(r.get('marker_detected',False) for r in frames),'all_requested_markers_detected':all(all(str(m['id']) in r['detections'] for m in s['markers']) for s,r in zip(request['stations'],frames))}))
    finally:
        app.close()

if __name__=='__main__':
    main()
