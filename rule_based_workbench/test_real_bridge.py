"""Local HTTP mode isolation checks; no hardware connection."""
from unittest.mock import Mock
import app
from test_app import serving


def main():
    original = app.WORKBENCH_MODE, app.REAL_BRIDGE
    try:
        app.WORKBENCH_MODE = 'dual_dsw'
        app.REAL_BRIDGE = Mock()
        app.REAL_BRIDGE.config = {}
        app.REAL_BRIDGE.owner.return_value = {'automatic_enabled': False}
        with serving(app.WorkbenchState()) as request:
            mode = request('GET', '/api/mode')
            headers = {'X-Workbench-CSRF': mode['csrf_token']}
            request('POST', '/api/mode', {'mode': 'real'}, expected=403)
            request('POST', '/api/mode', {'mode': 'real'}, headers={**headers, 'Origin': 'http://evil.example'}, expected=403)
            request('POST', '/api/mode', {'mode': 'real'}, headers=headers)
            assert not app.REAL_BRIDGE.control.called
            request('POST', '/api/pipeline/run', {}, expected=403)
            app.REAL_BRIDGE.workflow_status.return_value = {
                'pipeline': {'schema_version': 'physical_collection_pipeline.v1'},
                'telemetry': {'status': 'unavailable'}, 'calibration_profile': None,
                'scene': None, 'trajectory': None, 'pipeline_configured': False, 'workflow': {}}
            real = request('GET', '/api/state?mode=real')
            assert real['mode'] == 'real' and real['scene'] is None
            dual = request('GET', '/api/state?mode=dual_dsw')
            assert dual['pipeline'] != real['pipeline']
            request('GET', '/api/state?mode=unknown', expected=400)
            request('GET', '/api/state?mode=real&mode=dual_dsw', expected=400)
            request('GET', '/api/live-preview/source.jpg?mode=dual_dsw&camera=unknown', expected=400)
            app.REAL_BRIDGE.prepare_collection.return_value = {'status': 'awaiting-physical-confirmation', 'preview': {'token': 'test'}}
            result = request('POST', '/api/pipeline/run', {'mode':'real'}, headers=headers, expected=202)
            assert result['status'] == 'awaiting-physical-confirmation'
            assert app.REAL_BRIDGE.prepare_collection.call_count == 1
            request('POST', '/api/pipeline/run', {'mode':'dual_dsw'}, headers=headers, expected=400)
            request('POST', '/api/pipeline/source', {}, expected=400)
            assert not app.REAL_BRIDGE.control.called
            app.REAL_BRIDGE.owner.return_value = {'automatic_enabled': True}
            request('POST', '/api/mode', {'mode': 'dual_dsw'}, headers=headers, expected=400)
            app.REAL_BRIDGE.owner.return_value = {'automatic_enabled': False}
            request('POST', '/api/mode', {'mode': 'dual_dsw'}, headers=headers)
            request('POST', '/api/real/control', {'action': 'acquire', 'confirmation': 'ARMS_SUPPORTED_CONNECT'}, headers=headers, expected=400)
            assert not app.REAL_BRIDGE.control.called
        print('Mode isolation, CSRF, no implicit acquisition and busy guard: PASS')
    finally:
        app.WORKBENCH_MODE, app.REAL_BRIDGE = original


if __name__ == '__main__':
    main()
