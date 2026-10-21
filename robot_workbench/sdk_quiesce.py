"""Pinned, opt-in SDK candidate contract. Never accepts browser configuration."""
import hashlib
from pathlib import Path
import re


REPORT_FIELDS=('library_build_id','controller_instance_id','caller_generation','send_thread_joined',
               'last_send_attempt_monotonic_ns','quiesce_returned_monotonic_ns','receiver_state','transport_basis','verification_scope','idempotent_hit')


def validate_contract(value):
    if (not isinstance(value,dict) or set(value)!={'library_build_id','module','vendor_libraries','vendor_trust'}
            or not isinstance(value['library_build_id'],str)
            or not re.fullmatch('[0-9a-f]{64}',value['library_build_id'])
            or value['vendor_trust']!='human-approved-unmodified-binary'
            or not isinstance(value['vendor_libraries'],list) or not value['vendor_libraries']):
        raise ValueError('explicit pinned SDK/vendor trust contract required')
    for row in [value['module'],*value['vendor_libraries']]:
        if (not isinstance(row,dict) or set(row)!={'path','sha256'}
                or not isinstance(row['path'],str) or not Path(row['path']).is_absolute()
                or '..' in Path(row['path']).parts or not isinstance(row['sha256'],str)
                or not re.fullmatch('[0-9a-f]{64}',row['sha256'])):
            raise ValueError('invalid SDK artifact identity')
        if hashlib.sha256(Path(row['path']).read_bytes()).hexdigest()!=row['sha256']:
            raise ValueError('SDK artifact hash differs: '+row['path'])
    return value


def verify_module(sdk, contract):
    if (Path(sdk.__file__).resolve()!=Path(contract['module']['path']).resolve()
            or getattr(sdk,'workbench_build_id',None)!=contract['library_build_id']):
        raise ValueError('loaded SDK differs from pinned candidate')


def quiesce_report(controller, generation):
    report=controller.quiesce(generation)
    return {key:getattr(report,key) for key in REPORT_FIELDS}
