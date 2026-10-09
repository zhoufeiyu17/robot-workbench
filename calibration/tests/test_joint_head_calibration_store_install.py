"""The installed calibration is one group at one path, replaced atomically.

The versioned files in the library are history; `active/` is the one calibration in
force. What matters is that it holds exactly one head plus the wrist solved against it,
that its bytes are the published ones (so a stale copy cannot pass as installed), that a
second install leaves no second group behind, and that an install interrupted between its
two renames comes back rather than leaving the path empty.
"""
import json
import tempfile
import unittest
from pathlib import Path

from arx_env.joint_head_calibration_store import CalibrationStore


def payload(kind, marker):
    return {'row': {'kind': kind, 'marker': marker}}


class ActiveInstall(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        # macOS hands out /var/... paths, and /var is a symlink; the store refuses those.
        self.root = Path(self.temporary.name).resolve() / 'hardware'
        self.store = CalibrationStore(self.root, 'test-robot', 'hardware')

    def tearDown(self):
        self.temporary.cleanup()

    def publish(self, kind, identifier, *, dependency=None, marker=1):
        return self.store.register(kind, identifier, payload(kind, marker),
                                   'batch-'+identifier, 'hardware', dependency)

    def pair(self, head_identifier='head-a', wrist_identifier='wrist-a', *, marker=1):
        head = self.publish('head', head_identifier)
        wrist = self.publish('wrist', wrist_identifier, dependency=head['id'], marker=marker)
        return head, wrist

    def test_nothing_is_installed_until_it_is_installed(self):
        self.assertIsNone(self.store.active())

    def test_install_publishes_one_group_with_the_published_bytes(self):
        head, wrist = self.pair()
        manifest = self.store.install(head, wrist, geometry_id='abc123', acceptance={'max_holdout_rms_px': 2.0})
        active = self.root / 'active'
        self.assertEqual(manifest['path'], str(active))
        self.assertEqual(sorted(path.name for path in active.iterdir()),
                         ['head.json', 'manifest.json', 'wrist.json'])
        for kind, record in (('head', head), ('wrist', wrist)):
            self.assertEqual((active / (kind + '.json')).read_bytes(),
                             (self.root / (record['version'] + '.json')).read_bytes())
        read = self.store.active()
        self.assertTrue(read['verified'])
        self.assertEqual(read['problems'], [])
        self.assertEqual(read['geometry_id'], 'abc123')
        self.assertEqual(read['head']['version'], head['version'])
        self.assertEqual(read['wrist']['version'], wrist['version'])
        self.assertEqual(read['wrist']['dependency_id'], head['id'])
        self.assertEqual(read['acceptance'], {'max_holdout_rms_px': 2.0})
        self.assertIsNone(read['replaces'])

    def test_reinstalling_the_active_group_is_a_refresh_not_a_replacement(self):
        head, wrist = self.pair()
        self.store.install(head, wrist, geometry_id='one')
        manifest = self.store.install(head, wrist, geometry_id='one')
        self.assertIsNone(manifest['replaces'], 'the same group twice is a refresh, not a change')

    def test_a_second_install_replaces_instead_of_accumulating(self):
        first_head, first_wrist = self.pair()
        self.store.install(first_head, first_wrist, geometry_id='one')
        second_head, second_wrist = self.pair('head-b', 'wrist-b')
        manifest = self.store.install(second_head, second_wrist, geometry_id='two')
        self.assertEqual(manifest['replaces'],
                         {'head_version': first_head['version'], 'wrist_version': first_wrist['version']})
        active = self.root / 'active'
        self.assertEqual(sorted(path.name for path in active.iterdir()),
                         ['head.json', 'manifest.json', 'wrist.json'])
        self.assertEqual(json.loads((active / 'manifest.json').read_text())['geometry_id'], 'two')
        self.assertEqual([path.name for path in self.root.glob('.active.*')], [])

    def test_a_wrist_solved_against_another_head_is_refused(self):
        head, _ = self.pair()
        other_head = self.publish('head', 'head-b')
        other_wrist = self.publish('wrist', 'wrist-b', dependency=other_head['id'])
        with self.assertRaisesRegex(ValueError, 'solved against this head'):
            self.store.install(head, other_wrist, geometry_id='x')
        self.assertIsNone(self.store.active())

    def test_a_single_sided_record_is_refused(self):
        head, _ = self.pair()
        single = self.publish('left', 'left-a', dependency=head['id'])
        with self.assertRaisesRegex(ValueError, 'one head plus one paired wrist'):
            self.store.install(head, single, geometry_id='x')

    def test_another_environments_record_is_refused(self):
        other = CalibrationStore(self.root.parent / 'simulation', 'test-robot', 'simulation')
        head = other.register('head', 'head-sim', payload('head', 1), 'batch', 'simulation')
        wrist = other.register('wrist', 'wrist-sim', payload('wrist', 1), 'batch', 'simulation',
                               head['id'])
        with self.assertRaisesRegex(ValueError, 'from this library'):
            self.store.install(head, wrist, geometry_id='x')

    def test_a_tampered_copy_is_reported_instead_of_used(self):
        head, wrist = self.pair()
        self.store.install(head, wrist, geometry_id='abc123')
        (self.root / 'active' / 'head.json').write_text('{"tampered": true}\n')
        read = self.store.active()
        self.assertFalse(read['verified'])
        self.assertTrue(any('differs from library' in problem for problem in read['problems']), read['problems'])

    def test_an_install_interrupted_between_renames_is_recovered(self):
        head, wrist = self.pair()
        self.store.install(head, wrist, geometry_id='one')
        self.store.install(*self.pair('head-b', 'wrist-b'), geometry_id='two')
        (self.root / 'active').rename(self.root / '.active.superseded-crash')
        self.assertTrue(self.store.active()['verified'], 'the interrupted install must come back')
        self.store.install(*self.pair('head-c', 'wrist-c'), geometry_id='three')
        self.assertEqual(self.store.active()['geometry_id'], 'three')
        self.assertEqual([path.name for path in self.root.glob('.active.*')], [])

    def test_the_library_still_lists_only_versions(self):
        head, wrist = self.pair()
        self.store.install(head, wrist, geometry_id='abc123')
        versions = [row['version'] for row in self.store.list()]
        self.assertEqual(versions, sorted(versions))
        self.assertNotIn('active', versions)

    def test_physical_install_requires_both_published_sources(self):
        head, wrist = self.pair()
        with self.assertRaisesRegex(ValueError, 'explicit physical'):
            self.store.install(head, wrist, geometry_id='x', robot_source='physical')
        for source, simulated in (('physical', False), ('simulated', True)):
            row = {'robot_source': source, 'simulation_only': simulated}
            h = self.store.register('head', source+'-head', {'row': row}, 'batch', 'hardware')
            w = self.store.register('wrist', source+'-wrist', {'row': row}, 'batch', 'hardware', h['id'])
            if source == 'simulated':
                with self.assertRaisesRegex(ValueError, 'explicit physical'):
                    self.store.install(h, w, geometry_id='x', robot_source='physical')
            else:
                manifest = self.store.install(h, w, geometry_id='x', robot_source='physical')
                self.assertEqual(manifest['robot_source'], 'physical')
                self.assertIs(manifest['simulation_only'], False)
                with self.assertRaisesRegex(ValueError, 'explicit physical'):
                    self.store.install(h, w, geometry_id='x', robot_source='simulated')
                for index, incomplete in enumerate(({}, {'robot_source': 'simulated', 'simulation_only': True}, {'robot_source': 'physical'})):
                    mixed = self.store.register('wrist', 'mixed-'+str(index), {'row': incomplete}, 'batch', 'hardware', h['id'])
                    with self.assertRaisesRegex(ValueError, 'explicit physical'):
                        self.store.install(h, mixed, geometry_id='x')
        head['payload']['row'].update(robot_source='physical', simulation_only=False)
        wrist['payload']['row'].update(robot_source='physical', simulation_only=False)
        with self.assertRaisesRegex(ValueError, 'unchanged published'):
            self.store.install(head, wrist, geometry_id='x', robot_source='physical')
        self.assertEqual(self.store.active()['robot_source'], 'physical')


if __name__ == '__main__':
    unittest.main()
