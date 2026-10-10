"""CPU checks for continuous potential contact, independent of FCL/Isaac/GPU."""
from copy import deepcopy
from dataclasses import dataclass
from itertools import product
from types import SimpleNamespace

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from grasp_contact_geometry import analyze_contact_pose


@dataclass
class Part:
    link: str
    name: str
    vertices: list
    faces: list


def mesh(points):
    return points, ConvexHull(points).simplices.tolist()


class Band:
    shape_path = '/Cup/Collisions/MiddleBody'
    z_min_m, z_max_m = 0., .13

    def mesh(self):
        return mesh([[x, y, z] for x, y, z in product((-.02, .02), (-.02, .02), (-.065, .065))])


class FK:
    axes = (np.array([0., 1., 0.]), np.array([0., -1., 0.]))

    def forward(self, state):
        result = {'left_link6': np.eye(4)}
        for number, axis in zip((7, 8), self.axes):
            frame = np.eye(4)
            frame[:3, 3] = axis*state[f'left_joint{number}']
            result[f'left_link{number}'] = frame
        return result


def fixture(low=.05, high=.075):
    vertices, faces = mesh([[x, y, z] for x, y, z in product((-.04, .04), (-.001, .001), (low, high))])
    parts = [Part(f'left_link{i}', f'/left_link{i}/RealFingerRubberPad/cooked_0', vertices, faces) for i in (7, 8)]
    planner = SimpleNamespace(open_position_m=.04, home_state={}, fk=FK(), parts=parts,
                              cup=SimpleNamespace(profile_sha256='c'*64, bands=[Band()]))
    data = dict(urdf_sha256='a'*64, parts=[vars(p) for p in parts])
    return planner, data, dict(position=[0., 0., 0.], orientation_wxyz=[1., 0., 0., 0.])


def main():
    planner, data, pose = fixture()
    result = analyze_contact_pose(planner, data, pose, closed_jaw_m=.015)
    assert result['passed'], result
    assert abs(result['terminal_jaw_position_m']-.021) < 1e-9
    assert result['permitted_height_band_m'] == [.037000000000000005, .098]
    assert result['native_acceptance_tolerance_consumed_m'] == 0.
    assert analyze_contact_pose(planner, data, pose, closed_jaw_m=.015, saved=result) == result
    bad = deepcopy(result)
    bad['contact_height_interval_m'][0] += .001
    assert not analyze_contact_pose(planner, data, pose, closed_jaw_m=.015, saved=bad)['passed']
    altered = dict(data, urdf_sha256='b'*64)
    assert not analyze_contact_pose(planner, altered, pose, closed_jaw_m=.015, saved=result)['passed']
    native = dict(contact_geometry_stops_m={'left_joint7': .021-.00003, 'left_joint8': .021-.00001},
                  terminal_jaw_position_m=.021-.00002, centering_displacement_cup_m=[0., .00001, 0.])
    checked = analyze_contact_pose(planner, data, pose, closed_jaw_m=.015, closing_geometry=native)
    assert checked['passed'] and checked['native_closing_checked'], checked
    for key, value in [('terminal_jaw_position_m', .02), ('centering_displacement_cup_m', [0., .001, 0.]),
                       ('contact_geometry_stops_m', {'left_joint7': .021+.00001, 'left_joint8': .021}),
                       ('contact_geometry_stops_m', {'left_joint7': .021-.000051, 'left_joint8': .021})]:
        assert not analyze_contact_pose(planner, data, pose, closed_jaw_m=.015,
                                        closing_geometry={**native, key: value})['passed']
    assert not analyze_contact_pose(planner, data, pose, closed_jaw_m=.015,
                                    table_normal_cup=[0., 1., 0.])['passed']
    for invalid in (True, float('nan'), -.001, .041):
        assert not analyze_contact_pose(planner, data, pose, closed_jaw_m=invalid)['passed']
    missing_mesh = deepcopy(planner)
    missing_mesh.cup.bands = [SimpleNamespace(shape_path='/cup', z_min_m=0., z_max_m=.13, radius_m=.04)]
    assert not analyze_contact_pose(missing_mesh, data, pose, closed_jaw_m=.015)['passed']

    # Outer pad vertices exceed 100 mm, while only its cup-overlapping portion
    # can make contact. The solid-intersection bound must accept this geometry.
    points = [[x, y, z] for x, y in product((-.08, .04), (-.001, .001)) for z in (.05, .085-.3*x)]
    vertices, faces = mesh(points)
    for part in planner.parts:
        part.vertices, part.faces = vertices, faces
    result = analyze_contact_pose(planner, data, pose, closed_jaw_m=.015)
    assert max(v[2] for v in vertices) > .1
    assert result['passed'] and result['contact_height_interval_m'][1] < .092, result

    for low, high in ((.0369, .07), (.05, .0981), (.035, .1), (.037, .098)):
        planner, data, pose = fixture(low, high)
        result = analyze_contact_pose(planner, data, pose, closed_jaw_m=.015)
        assert not result['passed'], (low, high, result)
    planner, data, pose = fixture(.03701, .09799)
    assert analyze_contact_pose(planner, data, pose, closed_jaw_m=.015)['passed']

    # Oblique jaws pass a narrow cup inside a segment. The minimum is reached
    # at an interior gap when the pad leaves the cup, not at either endpoint.
    planner, data, pose = fixture(.049, .051)
    s = 2**-.5
    planner.fk.axes = (np.array([s, 0., s]), np.array([-s, 0., -s]))
    for index, part in enumerate(planner.parts):
        offset = -.02 if index == 0 else .02
        part.vertices, part.faces = mesh([[x+offset, y, z] for x, y, z in
                                         product((-.001, .001), (-.01, .01), (.049, .051))])
    class NarrowBand(Band):
        def mesh(self):
            return mesh([[x, y, z] for x, y, z in product((-.005, .005), (-.02, .02), (-.065, .065))])
    planner.cup.bands = [NarrowBand()]
    result = analyze_contact_pose(planner, data, pose, closed_jaw_m=0., table_normal_cup=[s, 0., -s])
    assert not result['passed'] and result['reason'] == 'potential_contact_height_outside_reserved_band', result
    terminal = result['terminal_jaw_position_m']
    assert any(1e-8 < q < terminal-1e-8 for row in result['regions'] for q in row['extremum_jaws_m']), result

    planner, data, pose = fixture()
    pose['position'][0] = 1.
    assert not analyze_contact_pose(planner, data, pose, closed_jaw_m=.015)['passed']
    pose['position'][0] = 0.
    assert not analyze_contact_pose(planner, data, pose, closed_jaw_m=.03)['passed']
    planner.fk.axes = (np.array([0., 1., 0.]), np.array([0., 1., 0.]))
    assert not analyze_contact_pose(planner, data, pose, closed_jaw_m=.015)['passed']
    planner.fk = FK()
    planner.parts[0].faces = planner.parts[0].faces[:-1]
    assert not analyze_contact_pose(planner, data, pose, closed_jaw_m=.015)['passed']

    # Seven source-pad vertices minimized from the 040630 failure. The old
    # transformed-face check misclassifies a ~1.6e-12 m² triangle as nonconvex.
    planner, data, pose = fixture()
    vertices = np.array([
        [.07100038230746372, .02692000587099421, .011270925531377788],
        [.07100100232458115, .026920087644716536, .011270699667960477],
        [.0009990001197512088, .017689823021660372, .03677142959892582],
        [.0009990001197416054, .017690025111115962, .0367714292569962],
        [.05599990940599344, .026140995972027004, .016733355094208535],
        [.07100100232458115, .025419767515893484, .011270700541896862],
        [.0009990214687390017, .016189082369588926, .036771097908232825]])
    faces = [[4,6,3], [5,4,1], [5,4,6], [0,5,1], [2,6,3],
             [2,5,6], [2,0,5], [2,4,1], [2,4,3], [2,0,1]]
    for part in planner.parts:
        points = vertices.copy()
        if part.link == 'left_link7':
            points[:, 1] *= -1
        part.vertices, part.faces = points.tolist(), faces
    data['parts'] = [vars(p) for p in planner.parts]
    pose = dict(position=[-.121462024215198, 0., .14], orientation_wxyz=[
        .9497051983623903, .00029650889100494085, .3131452358370428, -9.77675459307028e-5])
    class WorldFK:
        world = np.eye(4)
        def forward(self, state):
            result = {'left_link6': self.world}
            for number, sign in ((7, 1), (8, -1)):
                frame = np.array([
                    [.9999999999999999, -1.4482008771693869e-16, 1.786519601833262e-17, .08397000000000002],
                    [-8.928561046024588e-17, .9999999999999998, 2.7216121192794418e-17, 0.],
                    [1.9737889168124518e-17, 2.727832392728769e-17, 1., -.006270300000000034],
                    [0., 0., 0., 1.]])
                frame[1, 3] = sign*(.023637999999999965+state[f'left_joint{number}'])
                result[f'left_link{number}'] = self.world @ frame
            return result
    planner.fk = WorldFK()
    planner.open_position_m = .041496
    baseline = analyze_contact_pose(planner, data, pose, closed_jaw_m=.015)
    assert baseline['passed'], baseline
    # A global arm rigid transform must preserve the contact bounds.
    for angles in ((.3, -.5, .8), (-1.3, .8, 2.1)):
        planner.fk.world = np.eye(4)
        planner.fk.world[:3, :3] = Rotation.from_euler('xyz', angles).as_matrix()
        planner.fk.world[:3, 3] = [.255113, .234719, .882812]
        transformed = analyze_contact_pose(planner, data, pose, closed_jaw_m=.015)
        assert transformed['passed'], transformed
        assert np.allclose(transformed['contact_height_interval_m'],
                           baseline['contact_height_interval_m'], atol=1e-9, rtol=0)

    # A closed mesh with a dent remains invalid in the source frame: the
    # transform fix must not replace its geometry with its convex hull.
    planner, data, pose = fixture()
    planner.parts[0].vertices = deepcopy(planner.parts[0].vertices)
    planner.parts[0].vertices[-1] = [0., 0., .0625]
    data['parts'] = [vars(p) for p in planner.parts]
    dented = analyze_contact_pose(planner, data, pose, closed_jaw_m=.015)
    assert not dented['passed'] and dented['error'] == 'source mesh is not convex', dented
    print('continuous pad/cup LP: stops, interior extrema, 37–98 mm bounds, noncontact pad vertices and stale evidence PASS')


if __name__ == '__main__':
    main()
