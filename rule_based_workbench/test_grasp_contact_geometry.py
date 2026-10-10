"""CPU checks for continuous potential contact, independent of FCL/Isaac/GPU."""
from copy import deepcopy
from dataclasses import dataclass
from itertools import product
from types import SimpleNamespace

import numpy as np
from scipy.spatial import ConvexHull

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
    print('continuous pad/cup LP: stops, interior extrema, 37–98 mm bounds, noncontact pad vertices and stale evidence PASS')


if __name__ == '__main__':
    main()
