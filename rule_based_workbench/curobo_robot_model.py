"""Part-labelled A1-cal MotionGen mesh/sphere model, never a geometry certificate.

The default proxy uses deterministic interior-distance balls and bounded surface
refinement for the moving left arm. Fixed right-arm/chassis components use their
original world meshes; the two left mounting components retain separate sphere
links so mounting exemptions never exempt an entire chassis. Every generated
route still requires native collision/contact audit.
"""
import hashlib
import json
import math
from pathlib import Path

from curobo_planning import robot_config
from a1_asset_derivation import certificate_path

REQUIRED_LINKS = {'base_link', *(f'left_link{i}' for i in range(1, 9)), *(f'right_link1{i}' for i in range(1, 9))}
SUPPORT_PAIRS = tuple((f'{side}_link{offset+2}', f'{side}_link{offset+i}')
                      for side, offset in (('left', 0), ('right', 10)) for i in (3, 5))
PROXY_NUMERIC_STANDARDS = {
    'arm': dict(max_gap_m=.002, max_outward_m=.004),
    'chassis': dict(max_gap_m=.002, max_outward_m=.004),
    'finger': dict(max_gap_m=.0015, max_outward_m=.0015),
    'pad': dict(max_gap_m=.008, max_outward_m=.0005),
}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def official_fit(vertices, faces, count, surface_radius):
    """Historical attached-object fitter; not the default robot proxy fitter."""
    import numpy as np
    import trimesh
    import curobo
    from curobo.geom.sphere_fit import fit_spheres_to_mesh, SphereFitType
    if curobo.__version__ != '0.7.8':
        raise ValueError('sphere fitting requires cuRobo 0.7.8')
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    if not mesh.is_watertight:
        raise ValueError('sphere proxy fitting requires closed source parts')
    mesh.fix_normals()
    if not mesh.is_winding_consistent or mesh.volume <= 0:
        raise ValueError('source part must have consistent outward normals and positive volume')
    centers, radii = fit_spheres_to_mesh(mesh, count, surface_sphere_radius=surface_radius,
        fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE, voxelize_method='subdivide')
    if centers is None or len(centers) == 0:
        raise ValueError('official sphere fitting returned no spheres')
    centers, radii = np.asarray(centers, dtype=float), np.asarray(radii, dtype=float).reshape(-1)
    signed = trimesh.proximity.ProximityQuery(mesh).signed_distance(centers)
    if not np.isfinite(signed).all():
        raise ValueError('nonfinite proxy center signed distance')
    return centers, radii, dict(
        method='cuRobo0.7.8.VOXEL_VOLUME_SAMPLE_SURFACE/subdivide',
        official_surface_fallback_possible=True, stochastic_surface_sampling=True,
        # An interior center contains a ball of radius signed_distance; outside
        # centers use distance+r as an upper bound. This does not prove coverage.
        numeric_inflation_upper_bound_m=float(np.maximum(radii-signed, 0).max()),
        closed_mesh=True, trimesh_version=trimesh.__version__)


def surface_net(np, triangles, max_edge, maximum_cells=1_000_000):
    """Deterministic longest-edge bisection; each centroid has a surface-cover radius."""
    pending = np.asarray(triangles, dtype=float)
    centers, radii, normals = [], [], []
    count = 0
    while len(pending):
        edges = np.roll(pending, -1, axis=1)-pending
        lengths = np.einsum('nij,nij->ni', edges, edges)
        longest = lengths.argmax(axis=1)
        ready = lengths.max(axis=1) <= max_edge*max_edge*(1+1e-12)
        finished = pending[ready]
        if len(finished):
            mid = finished.mean(axis=1)
            normal = np.cross(finished[:, 1]-finished[:, 0], finished[:, 2]-finished[:, 0])
            normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-30)
            centers.append(mid)
            radii.append(np.linalg.norm(finished-mid[:, None], axis=2).max(axis=1))
            normals.append(normal)
            count += len(finished)
        if count+2*int((~ready).sum()) > maximum_cells:
            raise ValueError(f'surface coverage refinement exceeds {maximum_cells} cells; do not silently subsample')
        split = []
        for edge in range(3):
            rows = pending[(~ready) & (longest == edge)]
            if not len(rows):
                continue
            a, b, c = rows[:, edge], rows[:, (edge+1)%3], rows[:, (edge+2)%3]
            mid = (a+b)/2
            split.extend((np.stack((a, mid, c), axis=1), np.stack((mid, b, c), axis=1)))
        pending = np.concatenate(split) if split else np.empty((0, 3, 3))
    return np.concatenate(centers), np.concatenate(radii), np.concatenate(normals)


def refine_balls(np, samples, tolerances, initial_centers, initial_radii, candidate, maximum_spheres):
    """Greedy farthest residual with deterministic ties; never truncate an unfinished fit."""
    centers, radii = [], []
    gaps = np.full(len(samples), np.inf)

    def append(center, radius):
        if len(centers) >= maximum_spheres:
            raise ValueError(f'proxy sphere budget {maximum_spheres} exhausted; residual gap excess={float((gaps-tolerances).max()):.6g} m')
        centers.append(center)
        radii.append(radius)
        np.minimum(gaps, np.linalg.norm(samples-center, axis=1)-radius, out=gaps)

    for center, radius in zip(initial_centers, initial_radii):
        # A fully contained ball adds no volume or surface coverage.
        if centers and any(np.linalg.norm(center-old)+radius <= previous+1e-10 for old, previous in zip(centers, radii)):
            continue
        append(center, float(radius))
    while float((gaps-tolerances).max()) > 1e-8:
        index = int((gaps-tolerances).argmax())
        center, radius = candidate(index)
        if np.linalg.norm(samples[index]-center)-radius > tolerances[index]+1e-8:
            raise ValueError('candidate sphere cannot cover its selected residual probe')
        append(center, float(radius))
    return np.asarray(centers), np.asarray(radii), gaps


def adaptive_fit(vertices, faces, initial_count, outward, *, max_gap, max_spheres):
    """Fit material volume, filtering voxel fill through the original closed mesh.

    r <= signed_inside_distance(center)+outward bounds the ball's exterior reach;
    it cannot fill a real cavity beyond outward. Surface centroid gap plus the
    complete triangle radius bounds every surface point, not just old vertices.
    """
    import numpy as np
    import trimesh
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    if not mesh.is_watertight:
        raise ValueError('adaptive proxy requires a closed source part')
    mesh.fix_normals()
    if not mesh.is_winding_consistent or mesh.volume <= 0:
        raise ValueError('adaptive proxy requires outward normals and positive material volume')
    proximity = trimesh.proximity.ProximityQuery(mesh)
    pitch = max((float(mesh.volume)/initial_count)**(1/3), 2*outward)
    voxels = mesh.voxelized(pitch, method='subdivide').fill('base').points
    voxels = np.unique(voxels, axis=0)
    signed = proximity.signed_distance(voxels)
    if not np.isfinite(signed).all():
        raise ValueError('nonfinite closed-mesh voxel distance')
    discarded = int((signed <= 1e-9).sum())
    seeds, distances = voxels[signed > 1e-9], signed[signed > 1e-9]
    # independent off-lattice material probes; holes introduced by fill remain rejected
    shifts = np.asarray([[0, 0, 0], [.25, 0, 0], [-.25, 0, 0], [0, .25, 0], [0, -.25, 0], [0, 0, .25], [0, 0, -.25]])*pitch
    probes = (voxels[:, None]+shifts).reshape(-1, 3)
    probe_signed = proximity.signed_distance(probes)
    if not np.isfinite(probe_signed).all():
        raise ValueError('nonfinite material probe distance')
    volume = np.unique(probes[probe_signed > 1e-9], axis=0)
    surface, cover_radius, normals = surface_net(np, mesh.triangles, max_gap)
    samples = np.concatenate((surface, volume))
    tolerances = np.concatenate((max_gap-cover_radius, np.full(len(volume), max_gap)))
    limit = float(mesh.extents.max())
    depths = np.unique(np.r_[0., outward*np.asarray([.25, .5, 1, 2, 4, 8, 16, 32, 64]), limit*np.asarray([.125, .25, .5])])
    depths = depths[depths <= limit]

    def candidate(index):
        point = samples[index]
        candidates = (point[None]-normals[index][None]*depths[:, None]
                      if index < len(surface) else point[None])
        inside = proximity.signed_distance(candidates)
        # A surface centroid can measure slightly outside a thin cooked mesh.
        # Subtract that distance from its radius, preserving the outward bound
        # instead of relaxing containment or enlarging the physical mesh.
        radii = inside+outward
        valid = np.isfinite(inside) & (radii > 0.) & (np.linalg.norm(candidates-point, axis=1)-radii <= tolerances[index]+1e-8)
        if not valid.any():
            raise ValueError(f'no finite interior/boundary ball for residual surface probe: '
                             f'point={point.tolist()}, tolerance={tolerances[index]}, '
                             f'signed={inside.tolist()}, normal={normals[index].tolist() if index < len(surface) else None}')
        options = np.flatnonzero(valid)
        chosen = options[int(radii[options].argmax())]
        return candidates[chosen], radii[chosen]

    # Largest interior balls first; stable lexicographic voxel order breaks ties.
    order = np.argsort(-distances, kind='stable')
    centers, radii, gaps = refine_balls(np, samples, tolerances, seeds[order], distances[order]+outward,
                                      candidate, max_spheres)
    actual_signed = proximity.signed_distance(centers)
    if not np.isfinite(actual_signed).all():
        raise ValueError('nonfinite selected proxy center distance')
    inflation = float(np.maximum(radii-actual_signed, 0).max())
    surface_gap = float(np.maximum(gaps[:len(surface)]+cover_radius, 0).max())
    volume_gap = float(np.maximum(gaps[len(surface):], 0).max()) if len(volume) else None
    if inflation > outward+1e-7 or surface_gap > max_gap+1e-7 or (volume_gap is not None and volume_gap > max_gap+1e-7):
        raise ValueError('adaptive proxy missed its explicit numerical quality gate')
    return centers, radii, dict(method='deterministic_interior_distance_and_surface_refinement.v1',
        deterministic=True, random_surface_sampling=False, whole_aabb_fallback=False,
        max_gap_target_m=max_gap, max_outward_target_m=outward, quality_gate_passed=True,
        initial_seed_hint=initial_count, initial_voxel_pitch_m=pitch, retained_interior_seeds=len(seeds),
        discarded_void_or_outside_voxels=discarded, surface_cells=len(surface), volume_probes=len(volume),
        surface_gap_upper_bound_m=surface_gap, max_uncovered_volume_probe_gap_m=volume_gap,
        numeric_inflation_upper_bound_m=inflation, closed_mesh=True, trimesh_version=trimesh.__version__,
        radius_range_m=[float(radii.min()), float(radii.max())], radius_median_m=float(np.median(radii)),
        positive_material_center_count=int((actual_signed > 1e-9).sum()),
        bound_scope='triangle surface Lipschitz gap bound and interior probes; full solid-volume coverage is not certified')


def sample_coverage(np, vertices, faces, centers, radii):
    """Vertices and all face centroids only; explicitly not a solid-volume proof."""
    samples = np.concatenate((vertices, vertices[faces].mean(1)))
    gaps = []
    for begin in range(0, len(samples), 512):
        distance = np.linalg.norm(samples[begin:begin+512, None]-centers[None], axis=2)-radii
        gaps.extend(np.maximum(distance.min(1), 0).tolist())
    return dict(sample_count=len(samples), covered_fraction=sum(value <= 1e-6 for value in gaps)/len(gaps),
                max_uncovered_sample_gap_m=max(gaps), coverage_certificate=False,
                scope='all source vertices and face centroids; surface interiors and volume unproven')


def outward_fixed_mesh(vertices, faces):
    """Repair winding only; preserve every original triangle and every cavity."""
    import numpy as np
    import trimesh
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    if not mesh.is_watertight:
        raise ValueError('fixed robot world mesh must be closed')
    mesh.fix_normals()
    if not mesh.is_winding_consistent or mesh.volume <= 0:
        raise ValueError('fixed robot world mesh needs consistent outward normals')
    original = np.sort(np.asarray(faces), axis=1)
    repaired = np.sort(mesh.faces, axis=1)
    if not np.array_equal(np.asarray(vertices), mesh.vertices) or not np.array_equal(original, repaired):
        raise ValueError('fixed robot mesh repair changed source triangle geometry')
    return mesh.faces.tolist()


def fixed_world_mesh_descriptors(exported, report, fk, positions):
    """Return original fixed components with base-frame poses for world_config.

    The caller composes its world/base transform as appropriate. These shapes
    must accompany every MotionGen world update; omitting them loses the mast
    and fixed right-arm collision coverage.
    """
    from dataclasses import asdict
    if digest(exported['parts']) != report['source_parts_sha256']:
        raise ValueError('fixed robot mesh source differs from the model')
    if any(positions.get(name) != value for name, value in report['fixed_world_geometry']['locked_positions'].items()):
        raise ValueError('fixed right-arm/gripper command changed; rebuild world meshes')
    frames = fk.forward(positions)
    descriptors = []
    for part in exported['parts']:
        item = report['part_map'][part['name']]
        if item['representation'] != 'fixed_world_mesh':
            continue
        descriptors.append(dict(name=item['alias'], vertices=part['vertices'],
            faces=outward_fixed_mesh(part['vertices'], part['faces']), base_from_shape=asdict(fk.pose(frames[part['link']])),
            source_part_name=part['name'], source_part_sha256=item['sha256'], link=part['link']))
    if len(descriptors) != report['fixed_world_geometry']['part_count']:
        raise ValueError('fixed robot mesh descriptor coverage differs')
    return descriptors


def build_conservative_robot_model(exported, urdf_path, movable, positions, *, ignored_link_pairs, domain_certificate=None, attached_spheres=264):
    """Task candidate: spend the approximately 1000 left spheres on known tight regions."""
    import numpy as np
    from conservative_spheres import cover
    recipe=json.loads(certificate_path('a1_conservative_refinement.json',urdf_path).read_text())
    if (recipe['urdf_sha256']!=exported['urdf_sha256'] or recipe['source_parts_sha256']!=digest(exported['parts'])):
        raise ValueError('local conservative refinement recipe requires its frozen model geometry')
    if domain_certificate is None:
        domain_certificate=json.loads(certificate_path('a1_self_domain_certificate.json',urdf_path).read_text())
    mounts={name for name,link in exported['planner_identity']['chassis_collision']['base_arm_exemptions'] if link=='left_link1'}
    if domain_certificate:
        links={p['name']:p['link'] for p in exported['parts']}
        for row in domain_certificate.get('checks',[]):
            if row.get('certified'):
                mounts.update(n for n in row['parts'] if links.get(n)=='base_link')
    parts=iter(p for p in exported['parts'] if p['link'].startswith('left_') or p['name'] in mounts)
    coverage=[]
    def fitter(vertices,faces,count,radius):
        part=next(parts)
        if not np.array_equal(vertices,np.asarray(part['vertices'])):
            raise ValueError('conservative part order differs from model builder')
        link=part['link']
        if link in ('left_link7','left_link8'):
            pad='/RealFingerRubberPad/' in part['name'] or part['name'].endswith('/rubber_pad')
            options=dict(face_overhang_m=recipe['pad_face_overhang_m'] if pad else recipe['finger_overhang_m'],
                protected_normal=[0,-1 if link=='left_link7' else 1,0],
                backing_m=recipe['pad_backing_m'] if pad else 0.,clip_boundary=not pad)
        else:
            options=dict(pitch_m=recipe['base_pitch_m'] if link=='base_link' else recipe['body_pitch_m'] if link in ('left_link1','left_link2','left_link4','left_link5') else .035 if link=='left_link6' else .055,clip_boundary=True)
        centers,radii,proof=cover(vertices,faces=faces,refine_cells=recipe['refine_cells'].get(part['name'],()),**options)
        coverage.append(dict(name=part['name'],link=link,count=len(centers),**proof))
        return centers,radii,proof
    config,report=build_robot_model(exported,urdf_path,movable,positions,
        ignored_link_pairs=ignored_link_pairs,sphere_budget=2048,fit=fitter,domain_certificate=domain_certificate,claw_cup_allowance_m=.003,attached_spheres=attached_spheres)
    # One physical 264-ball cup, in either a fixed world slot or the held slot.
    # The inactive slot has negative radii; parent changes never invalidate graphs.
    if attached_spheres!=264:raise ValueError('inward cup asset requires its fixed 264 slots')
    kin=config['kinematics'];alias='world_cup'
    kin['extra_links'][alias]=dict(parent_link_name='base_link',link_name=alias,
        fixed_transform=[0.,0.,0.,1.,0.,0.,0.],joint_type='FIXED',joint_name='world_cup_joint')
    kin['collision_link_names'].append(alias)
    kin['collision_spheres'][alias]=[dict(center=[0.,0.,0.],radius=-10.) for _ in range(264)]
    kin['extra_collision_spheres'][alias]=264
    kin['collision_sphere_buffer'][alias]=0.;kin['self_collision_buffer'][alias]=0.
    kin['self_collision_ignore'].setdefault(alias,[]).append('attached_object')
    interaction_aliases=[name for name,link in kin['extra_links'].items()
        if link['parent_link_name'] in ('left_link6','left_link7','left_link8') and name!='attached_object']
    kin['self_collision_ignore'][alias].extend(interaction_aliases)
    report['cup_hand_interaction']=dict(links=['left_link6','left_link7','left_link8'],
        backend='original cup geometry GPU query before grasp/release; fixed relative certificate while held',
        geometry_inset_applied=False)
    ignored_world=set(kin['self_collision_ignore'][alias])|{alias}
    report['enabled_sphere_pairs']+=264*sum(len(kin['collision_spheres'][name]) for name in kin['collision_link_names'] if name not in ignored_world)
    report['self_collision_dense_initialization_bytes']=4*(report['allocated_spheres']+264)**2
    report.update(cup_proxy_mode='inward_surface_3mm',cup_active_spheres=264,
        inactive_cup_buffer_slots=264,allocated_spheres=report['allocated_spheres']+264,
        robot_config_sha256=digest(config))
    left_count=sum(p['count'] for p in coverage if p['link'].startswith('left_'))
    if left_count>1088 or not all(p['whole_solid_covered'] for p in coverage):
        raise ValueError(f'left arm conservative coverage/budget not met: {left_count} spheres; counts='+str([(p['link'],p['count']) for p in coverage]))
    report.update(refinement_recipe_sha256=digest(recipe),fit_policy='conservative_box_cells_local_density_v2',fixture_fitter_used=False,
        conservative_coverage_proven=True,coverage=coverage,left_sphere_count=left_count,left_sphere_budget=1088,
        numeric_quality_gate_passed=False,numeric_standards_scope='old adaptive quality gate does not apply',
        requires_complete_native_final_audit=False,online_validation_backend='curobo_gpu',
        production_ready=False,readiness_note='contact policy and complete MotionGen path/replay still require validation')
    return config,report


def build_robot_model(exported, urdf_path, movable, positions, *, ignored_link_pairs,
                      sphere_budget=4096, counts=None, attached_spheres=64, fit=None, domain_certificate=None, claw_cup_allowance_m=0.):
    """Return (RobotConfig dictionary, diagnostics) using only frozen model-space data.

    ignored_link_pairs must be the existing native whole.ignores, not new contact
    policy. The four native support pairs are explicitly delegated to final
    region/outside_support acceptance. No plan from this proxy is execution-ready.
    ``counts`` are initial seed hints, not hard per-part caps. 4096 is a quality
    candidate ceiling, not a target allocation. ``fit`` is only a fixture hook.
    """
    import numpy as np
    if claw_cup_allowance_m not in (0.,.003):raise ValueError('only the reviewed 3mm claw/cup policy is supported')
    path = Path(urdf_path).resolve()
    if exported['urdf_sha256'] != hashlib.sha256(path.read_bytes()).hexdigest():
        raise ValueError('frozen export and actual A1-cal URDF differ')
    config, kinematic_identity = robot_config(path, movable, positions)
    if any(abs(positions[a]-positions[b]) > 1e-9 for a, b in
           (('left_joint7', 'left_joint8'), ('right_joint17', 'right_joint18'))):
        raise ValueError('A1 mechanically linked jaws require symmetric model positions')
    if type(sphere_budget) is not int or sphere_budget < 1:
        raise ValueError('positive integer sphere budget required')
    if type(attached_spheres) is not int or not 0 <= attached_spheres <= 512:
        raise ValueError('attached sphere reservation must be an integer from 0 to 512')
    limits = dict(arm=24, chassis=6, finger=2, pad=8)
    if counts is not None:
        if set(counts)-set(limits):
            raise ValueError('unknown sphere-count category')
        limits.update(counts)
    if any(type(value) is not int or value < 1 for value in limits.values()):
        raise ValueError('each category requires a positive sphere count')
    parts = exported['parts']
    if not parts or len({part['name'] for part in parts}) != len(parts):
        raise ValueError('unique nonempty native parts required')
    if {part['link'] for part in parts} != REQUIRED_LINKS:
        raise ValueError('native export must cover chassis, both arms and all four fingers')
    if not REQUIRED_LINKS <= set(config['kinematics']['link_names']):
        raise ValueError('exported collision links absent from actual URDF')
    ignored = {frozenset(pair) for pair in ignored_link_pairs}
    if any(len(pair) != 2 or not pair <= REQUIRED_LINKS or 'base_link' in pair for pair in ignored):
        raise ValueError('invalid native arm-link exclusions; chassis requires part-specific exemptions')
    part_map, prepared = {}, []
    for index, part in enumerate(parts):
        vertices, faces = np.asarray(part['vertices'], dtype=float), np.asarray(part['faces'])
        if (vertices.ndim != 2 or vertices.shape[1:] != (3,) or len(vertices) < 4 or not np.isfinite(vertices).all()
                or faces.ndim != 2 or faces.shape[1:] != (3,) or faces.dtype.kind not in 'iu'
                or len(faces) < 4 or faces.min() < 0 or faces.max() >= len(vertices)):
            raise ValueError('invalid native collision vertices/faces: '+part['name'])
        pad = '/RealFingerRubberPad/' in part['name'] or part['name'].endswith('/rubber_pad')
        finger = part['link'] in ('left_link7', 'left_link8', 'right_link17', 'right_link18')
        category = 'pad' if pad else 'chassis' if part['link'] == 'base_link' else 'finger' if finger else 'arm'
        alias = f'collision_part_{index:03d}_{digest(part)[:10]}'
        part_map[part['name']] = dict(alias=alias, link=part['link'], category=category, sha256=digest(part))
        prepared.append((part, vertices, faces, category, alias))
    pads = {part['link'] for part, _, _, category, _ in prepared if category == 'pad'}
    if pads != {'left_link7', 'left_link8', 'right_link17', 'right_link18'}:
        raise ValueError('each finger requires its separate native rubber pad')
    chassis = exported['planner_identity']['chassis_collision']
    if chassis.get('component_count') != sum(part['link'] == 'base_link' for part in parts):
        raise ValueError('exported chassis component count differs from its frozen identity')
    mount_pairs = set()
    for name, link in chassis.get('base_arm_exemptions', []):
        if name not in part_map or part_map[name]['link'] != 'base_link' or link not in ('left_link1', 'right_link11'):
            raise ValueError('mount exemption is not an exported chassis component/base arm pair')
        number = name.rsplit('_', 1)[-1]
        expected = chassis.get('mount_component_sha256', {}).get(number)
        part = next(row for row in prepared if row[0]['name'] == name)
        triangles = part[1][part[2]]
        canonical = sorted(tuple(sorted(tuple(float(v)+0. for v in point) for point in triangle)) for triangle in triangles)
        actual = hashlib.sha256(np.asarray(canonical, dtype='<f8').tobytes()).hexdigest()
        if expected is None or actual != expected:
            raise ValueError('mount exemption component geometry hash mismatch: '+name)
        mount_pairs.add((name, link))
    support = exported['planner_identity'].get('support_contact')
    if not support or support['urdf_sha256'] != kinematic_identity['urdf_sha256']:
        raise ValueError('explicit native support identity for this URDF required')

    # The right arm is frozen by this planner. Exact world meshes avoid trying
    # to approximate large thin-walled fixed parts with thousands of tiny balls.
    # World kernels have no link/obstacle pair mask: retain only left mounting
    # parts as self-only spheres to keep their existing local link1 exemption.
    retained_mounts = {name for name, link in mount_pairs if link == 'left_link1'}
    certified_pairs=set()
    if domain_certificate:
        if (domain_certificate.get('urdf_sha256')!=kinematic_identity['urdf_sha256']
                or domain_certificate.get('source_parts_sha256')!=digest(parts)):
            raise ValueError('offline domain certificate model identity differs')
        bounds={n:[v['lower'],v['upper']] for n,v in kinematic_identity['joints'].items()}
        for row in domain_certificate.get('checks',[]):
            if row.get('certified') is not True:continue
            names=row['parts']
            if (len(names)!=2 or any(n not in part_map for n in names)
                    or row.get('reason')!='continuous_interval_separation' or row.get('joint_domain')!=bounds
                    or not math.isfinite(row.get('minimum_separation_lower_bound_m',float('nan')))
                    or row['minimum_separation_lower_bound_m']<=1e-7):
                raise ValueError('unbound or incomplete offline joint domain proof')
            pair_links={part_map[n]['link'] for n in names}
            if pair_links not in ({'base_link','left_link1'},{'base_link','left_link2'}):
                raise ValueError('fixed-world remap requires certified root/upper-arm mount pairs')
            reserve=row.get('numerical_distance_reserve_m',0.)
            if 'left_link2' in pair_links and (type(reserve) not in (float,int) or not math.isfinite(reserve) or reserve<1e-5):
                raise ValueError('upper-arm domain proof requires its numerical distance reserve')
            certified_pairs.add(frozenset(names))
            retained_mounts.update(n for n in names if part_map[n]['link']=='base_link')
    all_prepared = prepared
    prepared = [row for row in all_prepared if row[0]['link'].startswith('left_') or row[0]['name'] in retained_mounts]
    fixed = [row for row in all_prepared if not row[0]['link'].startswith('left_') and row[0]['name'] not in retained_mounts]
    if any(any(link.startswith('left_') for link in pair) and any(link.startswith('right_') for link in pair) for pair in ignored):
        raise ValueError('fixed world meshes cannot represent cross-arm pair exemptions')
    for row in all_prepared:
        part_map[row[0]['name']]['representation'] = ('robot_spheres'
            if row[0]['link'].startswith('left_') or row[0]['name'] in retained_mounts else 'fixed_world_mesh')
    requested = sum(limits[category] for _, _, _, category, _ in prepared)
    if len(prepared)+attached_spheres > sphere_budget or (fit is not None and requested+attached_spheres > sphere_budget):
        raise ValueError(f'requested {requested}+{attached_spheres} reserved spheres exceed budget {sphere_budget}; adjust explicit counts/budget')

    kin = config['kinematics']
    kin.update(collision_link_names=[], collision_spheres={}, extra_links={},
               collision_sphere_buffer={}, self_collision_buffer={}, self_collision_ignore={})
    reports, allocated = [], 0
    for part, vertices, faces, category, alias in prepared:
        quality = PROXY_NUMERIC_STANDARDS[category]
        surface_radius = quality['max_outward_m']
        remaining = sphere_budget-attached_spheres-allocated
        if remaining <= 0:
            raise ValueError('proxy sphere budget exhausted before part '+part['name'])
        try:
            centers, radii, fit_report = (fit(vertices, faces, limits[category], surface_radius) if fit is not None else
                adaptive_fit(vertices, faces, limits[category], surface_radius,
                             max_gap=quality['max_gap_m'], max_spheres=remaining))
        except Exception as error:
            raise ValueError(f'sphere fitting failed for {part["name"]}: {error}') from error
        centers, radii = np.asarray(centers, dtype=float), np.asarray(radii, dtype=float).reshape(-1)
        if (centers.shape != (len(radii), 3) or not 0 < len(radii) <= remaining
                or not np.isfinite(centers).all() or not np.isfinite(radii).all() or np.any(radii <= 0)):
            raise ValueError('fitter returned invalid or excess spheres: '+part['name'])
        allocated += len(radii)
        spheres = [dict(center=center.tolist(), radius=float(radius)) for center, radius in zip(centers, radii)]
        kin['collision_link_names'].append(alias)
        kin['collision_spheres'][alias] = spheres
        kin['collision_sphere_buffer'][alias] = 0.
        kin['self_collision_buffer'][alias] = 0.
        if part['link'] == 'base_link':
            # cuRobo 0.7.8 world kernels disable negative radii. Its generator
            # clamps (-1, 0] to 1mm, so stay below -2. Self-collision adds back
            # self_collision_buffer - collision_sphere_buffer before comparing.
            # The 1um self padding covers float32 cancellation; base/self stays.
            buffer = -(float(radii.max())+2.)
            kin['collision_sphere_buffer'][alias] = buffer
            kin['self_collision_buffer'][alias] = 1e-6
            world_radii = (radii+buffer).astype(np.float32)
            restored = world_radii+np.float32(1e-6-buffer)
            if np.any(world_radii >= -1.) or np.any(restored.astype(float) < radii):
                raise ValueError('base world/self radius separation failed numeric configuration check')
        kin['extra_links'][alias] = dict(parent_link_name=part['link'], link_name=alias,
            fixed_transform=[0., 0., 0., 1., 0., 0., 0.], joint_type='FIXED', joint_name=alias+'_joint')
        reports.append(dict(name=part['name'], link=part['link'], alias=alias, category=category,
            requested_spheres=limits[category], actual_spheres=len(spheres), spheres_sha256=digest(spheres),
            surface_radius_m=surface_radius, numerical_standard=quality, fit=fit_report,
            sampled_coverage=sample_coverage(np, vertices, faces, centers, radii)))

    exemptions, enabled_sphere_pairs = [], 0
    delegated = {frozenset(pair) for pair in SUPPORT_PAIRS}
    for i, (part, _, _, _, alias) in enumerate(prepared):
        for other, _, _, _, other_alias in prepared[:i]:
            links = frozenset((part['link'], other['link']))
            reason = ('same_rigid_link' if part['link'] == other['link'] else
                      'offline_joint_domain_proof' if frozenset((part['name'],other['name'])) in certified_pairs else
                      'native_local_support_delegated_to_exact_final_audit' if links in delegated else
                      'existing_native_link_exclusion' if links in ignored else
                      'pinned_mount_component' if (part['name'], other['link']) in mount_pairs
                          or (other['name'], part['link']) in mount_pairs else None)
            if reason:
                kin['self_collision_ignore'].setdefault(alias, []).append(other_alias)
                exemptions.append(dict(parts=[part['name'], other['name']], reason=reason))
            else:
                enabled_sphere_pairs += len(kin['collision_spheres'][alias])*len(kin['collision_spheres'][other_alias])
    total = sum(len(values) for values in kin['collision_spheres'].values())
    attachment = None
    if attached_spheres:
        alias = 'attached_object'
        if alias in kin['link_names'] or alias in kin['extra_links']:
            raise ValueError('attached_object already exists in robot model')
        kin['extra_links'][alias] = dict(parent_link_name='left_link6', link_name=alias,
            fixed_transform=[0., 0., 0., 1., 0., 0., 0.], joint_type='FIXED', joint_name='cup_attach_joint')
        kin['collision_link_names'].append(alias)
        kin['extra_collision_spheres'] = {alias: attached_spheres}
        kin['collision_spheres'][alias] = [dict(center=[0., 0., 0.], radius=-10.) for _ in range(attached_spheres)]
        kin['collision_sphere_buffer'][alias] = 0.
        kin['self_collision_buffer'][alias] = 0.
        ignored_pads,ignored_claws = [],[]
        for part, _, _, category, part_alias in prepared:
            if part['link']=='left_link6' and claw_cup_allowance_m:
                kin['self_collision_ignore'].setdefault(part_alias, []).append(alias)
                exemptions.append(dict(parts=[part['name'],alias],reason='fixed_relative_wrist_cup_strict_gpu_checked_at_attachment'))
            elif part['link'] in ('left_link7', 'left_link8') and (category=='pad' or claw_cup_allowance_m):
                kin['self_collision_ignore'].setdefault(part_alias, []).append(alias)
                ignored_claws.append(part_alias)
                if category=='pad':ignored_pads.append(part_alias)
                exemptions.append(dict(parts=[part['name'], alias], reason=('fixed_relative_claw_cup_3mm_checked_at_attachment' if claw_cup_allowance_m else 'attached_cup_grasp_pad_delegated_to_exact_final_audit')))
            else:
                enabled_sphere_pairs += len(kin['collision_spheres'][part_alias])*attached_spheres
        attachment = dict(link_name=alias, parent_link='left_link6', reserved_spheres=attached_spheres,
                          initial_state='disabled negative-radius placeholders; attach measured cup before held motion',
                          proxy_ignored_pad_aliases=ignored_pads, proxy_ignored_claw_aliases=ignored_claws,
                          claw_cup_allowance_m=claw_cup_allowance_m,
                          fixed_relative_contact_certificate_required=bool(claw_cup_allowance_m),
                          metal_and_wrist_collision_enabled=not bool(claw_cup_allowance_m), wrist_collision_enabled=True,
                          requires_native_pad_band_and_penetration_validation=not bool(claw_cup_allowance_m),
                          physical_grasp_requires_replay=True)
    allocated = total+attached_spheres
    combined_fingers = {}
    for link in ('left_link7', 'left_link8'):
        selected = [row for row in prepared if row[0]['link'] == link]
        vertices, faces, centers, radii, offset = [], [], [], [], 0
        for _, v, f, _, alias in selected:
            vertices.append(v); faces.append(f+offset); offset += len(v)
            centers.extend(s['center'] for s in kin['collision_spheres'][alias])
            radii.extend(s['radius'] for s in kin['collision_spheres'][alias])
        combined_fingers[link] = sample_coverage(np, np.concatenate(vertices), np.concatenate(faces), np.asarray(centers), np.asarray(radii))
        report = dict(schema_version='a1_curobo_optimization_proxy.v1', curobo_version='0.7.8',
        optimization_proxy_only=True, conservative_coverage_proven=False, requires_complete_native_final_audit=True,
        robot_config_sha256=digest(config), source_parts_sha256=digest(parts),
        domain_certificate_sha256=digest(domain_certificate) if domain_certificate else None,
        certified_domain_pair_count=len(certified_pairs),
        kinematics=kinematic_identity, support_identity=support,
        world_sha256=digest([*exported['observation']['scene']['shapes'], exported['table'], *exported['obstacles']]),
        represented_links=sorted(REQUIRED_LINKS), part_map=part_map, part_reports=reports,
        fixed_world_geometry=dict(part_count=len(fixed),
            parts=[dict(name=row[0]['name'], link=row[0]['link'], alias=row[4], sha256=digest(row[0]),
                        vertices=len(row[1]), triangles=len(row[2])) for row in fixed],
            locked_positions={name: value for name, value in kin['lock_joints'].items() if name.startswith('right_')},
            cross_arm_exemptions=[], retained_left_mount_parts=sorted(retained_mounts),
            representation='original closed component triangle meshes; outward winding repair only',
            sphere_approximation=False, cavities_filled=False, convex_hull=False,
            requires_every_world_update=True, native_fixed_initial_world_and_support_validation_required=True),
        part_coverage=dict(source_parts=len(parts), sphere_parts=len(prepared), fixed_mesh_parts=len(fixed),
                           complete=len(prepared)+len(fixed)==len(parts), omitted_parts=[]),
        fit_policy='moving_left_adaptive_spheres_fixed_robot_meshes.v3', numeric_standards=PROXY_NUMERIC_STANDARDS,
        numeric_standards_scope='proxy quality criteria for this candidate only; not user collision/contact allowances',
        numeric_quality_gate_passed=fit is None and all(row['fit'].get('quality_gate_passed') for row in reports),
        numeric_quality_gate_scope='sphere parts only; fixed original meshes are checked by fixed_world_mesh_descriptors',
        fixture_fitter_used=fit is not None, combined_metal_pad_sampled_coverage=combined_fingers,
        promotion_ready=False, historical_mast_gpu_collision_verified=False, nearby_safe_gpu_cases_verified=False,
        base_world_exclusion_required=True,
        base_world_exclusion_configured=True, base_world_exclusion_gpu_verified=False,
        base_self_numeric_guard_m=1e-6,
        base_collision_aliases=[alias for part, _, _, _, alias in prepared if part['link'] == 'base_link'],
        base_world_policy='nonmount chassis/right arm are original world meshes; left mount spheres skip static world and retain exact native component/link1 self exemption',
        required_gpu_fixtures=['retained left mount spheres overlap static table: their world cost remains zero',
                               'recorded historical mast collision pose: implicated original mesh/moving-link GPU collision is positive',
                               'native-clear nearby mast poses: report GPU false positives before model promotion',
                               'moving link3 proxy overlaps nonexempt fixed base mesh: world collision remains positive',
                               'pinned mounting component/link1 exemption remains local to that component',
                               'attached cup versus fixed base/right mesh checked in world; versus retained left mounts checked in self',
                               'unattached cup/base and fixed right world/support checked by native final audit'],
        sphere_budget=sphere_budget, requested_spheres=requested, actual_spheres=total,
        requested_spheres_scope='initial seed hints; adaptive fitting is allowed to exceed each hint, never the global budget',
        attached_object=attachment, allocated_spheres=allocated,
        self_collision_dense_initialization_bytes=4*allocated*allocated, enabled_sphere_pairs=enabled_sphere_pairs,
        slow_kernel_expected=enabled_sphere_pairs > 32*512,
        slow_kernel_requirement='apply configure_zero_home_collision_cache before warmup if experimental_kernel is false',
        exemptions=exemptions, native_support_pairs=[list(pair) for pair in SUPPORT_PAIRS],
        native_final_acceptance=['full robot/chassis/world geometry', 'local support ROI and outside_support',
                                 'cup/table/tray and 3mm machine contact rules', 'finger metal and rubber-pad contact bands'],
        missing_coverage_is_reported_not_waived=True,
        max_sampled_gap_m=max(row['sampled_coverage']['max_uncovered_sample_gap_m'] for row in reports))
    return config, report
