"""Continuous convex pad/cup contact regions; no IK, force or collision certificate."""
from dataclasses import asdict, is_dataclass
import hashlib
import json
import math

import numpy as np
from scipy.optimize import linprog
from scipy.spatial import ConvexHull, QhullError

LINKS = ('left_link7', 'left_link8')
JOINTS = ('left_joint7', 'left_joint8')
HEIGHT_BAND_M = (.035, .100)
HEIGHT_RESERVE_M = .002
NUMERICAL_M = 1e-9


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _pose(value):
    value = asdict(value) if is_dataclass(value) else value
    p = np.asarray(value['position'], dtype=float)
    q = np.asarray(value['orientation_wxyz'], dtype=float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[p, q]).all() or abs(np.linalg.norm(q)-1) > 1e-7:
        raise ValueError('finite position and unit quaternion required')
    w, x, y, z = q
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return p, rotation


def _hull(vertices, faces):
    """Reject incomplete/nonconvex source meshes before using their halfspaces."""
    points = np.asarray(vertices, dtype=float)
    triangles = np.asarray(faces)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 4 or not np.isfinite(points).all()
            or triangles.ndim != 2 or triangles.shape[1] != 3 or triangles.dtype.kind not in 'iu'
            or triangles.min() < 0 or triangles.max() >= len(points)):
        raise ValueError('finite convex vertices and triangular source faces required')
    edges = {}
    for face in triangles:
        a, b, c = points[face]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length == 0:
            raise ValueError('degenerate convex source face')
        # Cooked hulls include almost collinear corner triangles. Keep every
        # vertex in the hull; do not infer a plane from their unstable normal.
        if length > 1e-12:
            distance = (points-a) @ (normal/length)
            if distance.min() < -NUMERICAL_M and distance.max() > NUMERICAL_M:
                raise ValueError('source mesh is not convex')
        for i, j in zip(face, np.roll(face, -1)):
            key = tuple(sorted((int(i), int(j))))
            edges[key] = edges.get(key, 0)+1
    if any(count != 2 for count in edges.values()) or set(triangles.ravel()) != set(range(len(points))):
        raise ValueError('closed complete convex source mesh required')
    equations = ConvexHull(points).equations
    return equations[:, :3], -equations[:, 3]


def _solve(objective, constraints, bounds):
    a, b = constraints
    fit = linprog(objective, A_ub=a, b_ub=b, bounds=bounds, method='highs',
                  options={'primal_feasibility_tolerance': 1e-9, 'dual_feasibility_tolerance': 1e-9})
    if fit.status == 2:
        return None
    if not fit.success or not np.isfinite(fit.x).all() or np.max(a @ fit.x-b) > NUMERICAL_M:
        raise ValueError('convex contact LP did not produce a bounded feasible solution')
    return fit.x


def _constraints(pad, cup, axis, offset):
    # x is in cup coordinates; pad(q) = pad(0) + axis*q + offset.
    a, b = pad
    c, d = cup
    return np.vstack((np.column_stack((a, -a @ axis)), np.column_stack((c, np.zeros(len(c)))))), np.r_[b+a @ offset, d]


def analyze_contact_pose(planner, data, cup_from_tool, *, closed_jaw_m,
                         table_normal_cup=(0., 0., 1.), closing_geometry=None, saved=None):
    """Bound all pad/cup solid intersections over linked, piecewise affine closing.

    Intersections include the whole original convex solids, conservatively
    covering their contact surfaces. Empty regions never count as a grasp.
    Cup centering follows the native projection onto the observed table plane.
    An optional native closing result must match the LP first-contact stops
    within the original 50 micrometre sample interval, in the closing direction.
    """
    try:
        opening = float(planner.open_position_m)
        if (isinstance(closed_jaw_m, bool) or not math.isfinite(closed_jaw_m)
                or not 0 <= closed_jaw_m < opening <= .044):
            raise ValueError('finite commanded gap within the native jaw range required')
        position, rotation = _pose(cup_from_tool)
        normal = np.asarray(table_normal_cup, dtype=float)
        if normal.shape != (3,) or not np.isfinite(normal).all() or abs(np.linalg.norm(normal)-1) > 1e-8:
            raise ValueError('finite unit observed table normal in cup coordinates required')
        original_pose = dict(position=position.tolist(), orientation_wxyz=list(
            (asdict(cup_from_tool) if is_dataclass(cup_from_tool) else cup_from_tool)['orientation_wxyz']))
        state = dict(planner.home_state, **dict.fromkeys(JOINTS, 0.))
        frames = planner.fk.forward(state)
        tool_inverse = np.linalg.inv(frames['left_link6'])
        relative = {link: tool_inverse @ frames[link] for link in LINKS}
        moved = planner.fk.forward(dict(state, **dict.fromkeys(JOINTS, opening)))
        halfway = planner.fk.forward(dict(state, **dict.fromkeys(JOINTS, opening/2)))
        axes = {}
        for link in LINKS:
            end = tool_inverse @ moved[link]
            mid = tool_inverse @ halfway[link]
            axis = (end[:3, 3]-relative[link][:3, 3])/opening
            if (abs(np.linalg.norm(axis)-1) > 1e-8
                    or not np.allclose(end[:3, :3], relative[link][:3, :3], rtol=0, atol=1e-10)
                    or not np.allclose(mid, (end+relative[link])/2, rtol=0, atol=1e-10)):
                raise ValueError('fixed orientation and unit affine prismatic jaw axes required')
            axes[link] = rotation @ axis
        if np.linalg.norm(axes[LINKS[0]]+axes[LINKS[1]]) > 1e-8:
            raise ValueError('opposed native jaw axes required')
        axis = axes[LINKS[0]]
        if abs(float(axis @ normal)) > .01:
            raise ValueError('centering requires horizontal jaw axes within the native table-plane gate')
        axis = axis-(axis @ normal)*normal
        axis /= np.linalg.norm(axis)
        pads = [p for p in planner.parts if p.link in LINKS and
                ('/RealFingerRubberPad/' in p.name or p.name.endswith('/rubber_pad'))]
        if len(pads) != 2 or {p.link for p in pads} != set(LINKS):
            raise ValueError('exactly one original convex hull per active pad required')
        pad_shapes, pad_inputs = {}, []
        for part in pads:
            sources = [row for row in data['parts'] if row['name'] == part.name and row['link'] == part.link]
            if (len(sources) != 1 or not np.array_equal(sources[0]['vertices'], part.vertices)
                    or not np.array_equal(sources[0]['faces'], part.faces)):
                raise ValueError('pad geometry differs from the frozen source parts')
            frame = relative[part.link]
            vertices = (np.asarray(part.vertices) @ frame[:3, :3].T+frame[:3, 3]) @ rotation.T+position
            pad_shapes[part.link] = _hull(vertices, part.faces)
            pad_inputs.append(dict(link=part.link, name=part.name, vertices=vertices.tolist(), faces=part.faces))
        cup_shapes, cup_inputs = {}, []
        for band in planner.cup.bands:
            if not callable(getattr(band, 'mesh', None)):
                raise ValueError('explicit native cup-band mesh required; analytic cylinder approximation is unqualified')
            vertices, faces = band.mesh()
            vertices = np.asarray(vertices, dtype=float)+[0., 0., (band.z_min_m+band.z_max_m)/2]
            if band.shape_path in cup_shapes:
                raise ValueError('unique cup-band geometry required')
            cup_shapes[band.shape_path] = _hull(vertices, faces)
            cup_inputs.append(dict(name=band.shape_path, vertices=vertices.tolist(), faces=faces))
        if not cup_shapes:
            raise ValueError('complete original cup-band geometry required')
        inputs = dict(urdf_sha256=data['urdf_sha256'], source_parts_sha256=_digest(data['parts']),
                      cup_profile_sha256=planner.cup.profile_sha256, cup_from_tool=original_pose,
                      pad_geometry=pad_inputs, cup_geometry=cup_inputs,
                      jaw_axes_cup={link: axis.tolist() for link, axis in axes.items()},
                      open_jaw_m=opening, commanded_closed_jaw_m=closed_jaw_m,
                      table_normal_cup=normal.tolist(), native_closing_geometry=closing_geometry)
        stops = {}
        for link in LINKS:
            hits = [_solve([0., 0., 0., -1.], _constraints(pad_shapes[link], cup, axes[link], np.zeros(3)),
                           [(None, None)]*3+[(0., opening)]) for cup in cup_shapes.values()]
            hits = [row for row in hits if row is not None]
            if not hits:
                raise ValueError('no pad contact in the native jaw range: '+link)
            stops[link] = max(float(row[3]) for row in hits)
            if opening-stops[link] <= NUMERICAL_M:
                raise ValueError('open pad already contacts cup: '+link)
        lp_stops = dict(zip(JOINTS, (stops[link] for link in LINKS)))
        if closing_geometry is not None:
            supplied = closing_geometry['contact_geometry_stops_m']
            if (set(supplied) != set(JOINTS) or any(isinstance(value, bool) or not math.isfinite(value)
                    or not -NUMERICAL_M <= lp_stops[joint]-value <= .00005+NUMERICAL_M
                    for joint, value in supplied.items())):
                raise ValueError('native stops differ from LP contact by more than the original closing sample interval')
            stops = {link: float(supplied[joint]) for link, joint in zip(LINKS, JOINTS)}
        terminal = sum(stops.values())/2
        if closed_jaw_m > terminal+NUMERICAL_M:
            raise ValueError('commanded closure does not reach bilateral rigid contact')
        shift = (stops[LINKS[1]]-stops[LINKS[0]])/2
        displacement = shift*axis
        if abs(shift) > .01:
            raise ValueError('cup centering exceeds the existing 10 mm geometry bound')
        if closing_geometry is not None:
            given = np.asarray(closing_geometry['centering_displacement_cup_m'], dtype=float)
            gap = closing_geometry['terminal_jaw_position_m']
            if (given.shape != (3,) or not np.isfinite(given).all() or np.linalg.norm(given-displacement) > NUMERICAL_M
                    or isinstance(gap, bool) or not math.isfinite(gap) or abs(gap-terminal) > NUMERICAL_M):
                raise ValueError('native terminal gap or centering differs from linked-jaw table-plane geometry')
        def center(gap):
            return axis*math.copysign(min(abs(shift), max(0., max(stops.values())-gap)), shift)
        breaks = sorted(set((opening, max(stops.values()), terminal, float(closed_jaw_m))), reverse=True)
        regions = []
        for start, end in zip(breaks, breaks[1:]):
            if start-end <= NUMERICAL_M:
                continue
            slope = (center(start)-center(end))/(start-end)
            intercept = center(start)-slope*start
            for link in LINKS:
                for name, cup in cup_shapes.items():
                    constraints = _constraints(pad_shapes[link], cup, axes[link]-slope, -intercept)
                    bounds = [(None, None)]*3+[(end, start)]
                    low = _solve([0., 0., 1., 0.], constraints, bounds)
                    if low is None:
                        continue
                    high = _solve([0., 0., -1., 0.], constraints, bounds)
                    if high is None:
                        raise ValueError('inconsistent convex contact LP feasibility')
                    regions.append(dict(link=link, band=name, jaw_interval_m=[end, start],
                        height_interval_m=[float(low[2])-NUMERICAL_M, float(high[2])+NUMERICAL_M],
                        lower_witness_cup_m=low[:3].tolist(), upper_witness_cup_m=high[:3].tolist(),
                        extremum_jaws_m=[float(low[3]), float(high[3])]))
        if {row['link'] for row in regions} != set(LINKS):
            raise ValueError('both pads require a nonempty continuous potential contact region')
        low, high = min(r['height_interval_m'][0] for r in regions), max(r['height_interval_m'][1] for r in regions)
        permitted = [HEIGHT_BAND_M[0]+HEIGHT_RESERVE_M, HEIGHT_BAND_M[1]-HEIGHT_RESERVE_M]
        passed = low >= permitted[0] and high <= permitted[1]
        result = dict(schema_version='grasp_contact_geometry.v1', passed=passed,
            reason='passed' if passed else 'potential_contact_height_outside_reserved_band',
            input_digest=_digest(inputs), urdf_sha256=inputs['urdf_sha256'],
            source_parts_sha256=inputs['source_parts_sha256'], cup_profile_sha256=inputs['cup_profile_sha256'],
            original_cup_from_tool=original_pose, commanded_closed_jaw_m=closed_jaw_m,
            contact_geometry_stops_m=dict(zip(JOINTS, (stops[link] for link in LINKS))),
            independent_contact_stops_lp_m=lp_stops, native_closing_checked=closing_geometry is not None,
            table_normal_cup=normal.tolist(), centering_axis_cup=axis.tolist(),
            terminal_jaw_position_m=terminal, centering_displacement_cup_m=displacement.tolist(),
            original_cup_from_held_cup=dict(position=displacement.tolist(), orientation_wxyz=[1., 0., 0., 0.]),
            jaw_breakpoints_m=breaks, nominal_height_band_m=list(HEIGHT_BAND_M), reserve_m=HEIGHT_RESERVE_M,
            permitted_height_band_m=permitted, contact_height_interval_m=[low, high], regions=regions,
            numerical_bound_padding_m=NUMERICAL_M, native_acceptance_tolerance_consumed_m=0.,
            method='4D linear programming over original convex pad/cup solid intersections',
            scope='continuous linked-jaw geometric hypothesis through commanded closure; no force, IK, collision or held-contact qualification')
        result['geometry_digest'] = _digest(result)
        if saved is not None and saved != result:
            raise ValueError('saved contact geometry differs from recomputed input-bound result')
        return result
    except (ValueError, KeyError, TypeError, QhullError, np.linalg.LinAlgError) as error:
        return dict(schema_version='grasp_contact_geometry.v1', passed=False,
                    reason='contact_geometry_unproven', error=str(error))
