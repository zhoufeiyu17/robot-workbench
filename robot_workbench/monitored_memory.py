"""Composition adapter for observing the existing memory session lifecycle."""
import threading

from .runtime_monitor import probes
from .wam_mem import MemorySession


class MonitoredMemorySession(MemorySession):
    def __init__(self, *args, record=None, **kwargs):
        def observed_record(request, prepared, call):
            node = request['memory_sample']['node']
            fields = dict(request_id=request['request_id'], kind=node['kind'], step=node['step'],
                          model_time_s=node['model_time_s'], epoch=request['epoch'],
                          run_id=request['run_id'], generation=self.generation,
                          stride=self.sampling_stride if node['kind']=='MODEL' else self.manual_stride)
            probes.mark('memory/write', force=True, active=True, stage='write', **fields)
            try:
                result = record(request, prepared, call) if record is not None else call()
            except BaseException:
                probes.mark('memory/write', force=True, active=False, stage='failed', **fields)
                raise
            probes.mark('memory/write', force=True, active=False, stage='committed',
                        context_revision=result['audit']['memory'].get('context_revision'), **fields)
            return result
        super().__init__(*args, record=observed_record, **kwargs)

    def _enqueue(self, metadata, observation, epoch, run_id=None):
        super()._enqueue(metadata, observation, epoch, run_id)
        node = metadata['node']
        probes.mark('memory/queue', force=True, stage='queued', kind=node['kind'], step=node['step'],
                    model_time_s=node['model_time_s'], target_source_s=node['target_source_time'],
                    pending=len(self.pending), epoch=epoch)

    def _resolve_pending(self, until):
        if not self.pending:
            return super()._resolve_pending(until)
        with probes.span('memory/prepare', 'resolve_input'):
            return super()._resolve_pending(until)

    def _drain(self, until=None):
        if not self.pending:
            return super()._drain(until)
        background = threading.current_thread() is self.thread
        channel = 'memory/background' if background else 'memory/infer'
        probes.mark(channel, force=True, active=True, stage='drain')
        try:
            result = super()._drain(until)
        except BaseException:
            probes.mark(channel, force=True, active=False, stage='failed')
            raise
        probes.mark(channel, force=True, active=not background, stage='idle' if background else 'predict')
        return result

    def infer(self, request, *, prepared_input=None):
        with probes.span('memory/infer', 'wait_io', request_id=request['request_id']):
            return super().infer(request, prepared_input=prepared_input)
