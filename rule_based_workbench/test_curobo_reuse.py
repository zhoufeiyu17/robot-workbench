"""Check per-search native context wiring and cleanup without GPU or robot services."""
from types import SimpleNamespace

from curobo_relative_geometry import initial


def main():
    pose = SimpleNamespace(position=(0., 0., 0.), orientation_wxyz=(1., 0., 0., 0.))
    contexts = []

    class Context:
        def __init__(self, geo, fk, options):
            self.geo, self.fk, self.options = geo, fk, options
            self.stats = dict(visits=0, hits=0, misses=0)
            self.closed = False

        def __enter__(self):
            contexts.append(self)
            return self

        def __exit__(self, *args):
            self.closed = True

        def check(self, a, b, **kw):
            self.stats['visits'] += 1
            return self.geo.corridor(self.fk, a, b, _state_reuse=self, **self.options, **kw)

    class Geometry:
        scene = object()

        def __init__(self):
            self._candidate_audit_stats = {}

        def corridor(self, fk, a, b, **options):
            return options

        def trajectory_state_reuse(self, fk, **options):
            return Context(self, fk, options)

    geo, fk = Geometry(), object()

    def solve(obs, margin):
        def call(**kw):
            return geo.corridor(fk, {}, {}, world_from_cup=pose, **kw)
        a = call()
        assert a['_state_reuse'] is call(maximum_joint_step=.00005)['_state_reuse']
        assert a['_state_reuse'] is not call(grasp_contact=True)['_state_reuse']
        pose.position = (-0., 0., 0.)
        assert a['_state_reuse'] is not call()['_state_reuse'], 'do not round/sign-normalize pose keys'
        for i in range(40):
            pose.position = (float(i), 0., 0.)
            call()
        assert len(contexts) == 32
        geo.scene = object()
        assert call()['_state_reuse'] is None, 'changed scene must use the native path'
        return dict(passed=True)

    planner = SimpleNamespace(geo=geo, initial=solve)
    assert initial(planner, None, .004)['passed']
    assert 'corridor' not in geo.__dict__ and all(c.closed for c in contexts)
    assert geo._candidate_audit_stats['initial_state_reuse']['contexts'] == 32
    original = geo.corridor
    geo.corridor = original
    planner.initial = lambda *args: (_ for _ in ()).throw(ValueError('native failure'))
    try:
        initial(planner, None, .004)
    except ValueError as error:
        assert str(error) == 'native failure'
    else:
        raise AssertionError('failure swallowed')
    assert geo.corridor is original, 'instance override must also restore on failure'
    print('exact per-search context selection, bound, scene fallback and cleanup passed')


if __name__ == '__main__':
    main()
