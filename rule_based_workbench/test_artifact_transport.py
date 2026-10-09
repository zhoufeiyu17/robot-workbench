"""CPU-only transport checks; never SSH, camera, or GPU."""
import io
import json
from pathlib import Path
import shlex
import subprocess
import sys
import tarfile
import tempfile

from artifact_transport import (PARTIAL, manifest, pack, relative, ssh_argv,
                                transfer, unpack, verify)


def rejects(call):
    try:
        call()
    except (ValueError, OSError, tarfile.TarError):
        return
    raise AssertionError('unsafe transfer accepted')


def run():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory).resolve()
        robot, dsw = root/'robot', root/'dsw'
        robot.mkdir(); dsw.mkdir()
        capture = robot/'sealed-capture'; capture.mkdir()
        for name, value in {'rgbd-manifest.json': b'{"physical":true}', 'profile.json': b'{}',
                            'frames/000.npz': b'fake paired RGB-D', 'acquisition-binding.json': b'{}'}.items():
            path = capture/name; path.parent.mkdir(exist_ok=True); path.write_bytes(value)
        expected = manifest(capture, ['rgbd-manifest.json','profile.json','frames/000.npz','acquisition-binding.json'])
        args = dict(direction='upload', local_root=robot, local_directory='sealed-capture',
                    remote_root=dsw, remote_directory='new-capture', expected=expected)
        target = transfer(**args)
        verify(target, expected)
        assert transfer(**args) == target  # exact retry is idempotent
        (target/'frames/000.npz').write_bytes(b'corrupt')
        rejects(lambda: transfer(**args))  # never silently repairs an existing artifact
        assert (target/'frames/000.npz').read_bytes() == b'corrupt'

        playback = dsw/'target-playback'; playback.mkdir()
        for name in ('manifest.json','target-base_0.mp4','target-left_wrist_0.mp4','target-right_wrist_0.mp4'):
            (playback/name).write_bytes(name.encode())
        videos = manifest(playback, [p.name for p in playback.iterdir()])
        pulled = transfer(direction='download',local_root=robot,local_directory='target-results',
                          remote_root=dsw,remote_directory='target-playback',expected=videos)
        verify(pulled, videos)
        (playback/'target-base_0.mp4').write_bytes(b'bad')
        rejects(lambda: transfer(direction='download',local_root=robot,local_directory='bad-results',
                                 remote_root=dsw,remote_directory='target-playback',expected=videos))
        assert not (robot/'bad-results').exists()

        bad = io.BytesIO()
        with tarfile.open(fileobj=bad,mode='w') as archive:
            row = tarfile.TarInfo('profile.json'); row.size=2
            archive.addfile(row,io.BytesIO(b'xx'))
        bad.seek(0)
        rejects(lambda: unpack(dsw,'broken',expected,bad))
        assert (dsw/'broken'/PARTIAL).is_file()
        rejects(lambda: unpack(dsw,'broken',expected,io.BytesIO()))
        for name in ('../escape','/absolute','a/../../b','a//b','a/./b','.transport.json'):
            rejects(lambda: relative(name))
        (capture/'link').symlink_to(capture/'profile.json')
        rejects(lambda: manifest(capture,['link']))
        (dsw/'link-root').symlink_to(robot,target_is_directory=True)
        rejects(lambda: transfer(**{**args,'remote_directory':'link-root/new'}))
        for kind in (tarfile.SYMTYPE,tarfile.LNKTYPE,tarfile.DIRTYPE):
            buffer=io.BytesIO()
            with tarfile.open(fileobj=buffer,mode='w') as archive:
                row=tarfile.TarInfo('profile.json'); row.type=kind; row.linkname='/tmp/escape'
                archive.addfile(row)
            buffer.seek(0)
            rejects(lambda: unpack(dsw,'type-'+kind.decode(),expected,buffer))
        # Execute precisely the encoded remote worker through local Python only.
        request=dict(operation='receive',root=str(dsw),directory='worker-capture',manifest=expected)
        argv=ssh_argv(dict(host='user@example.com',port=739,key='/tmp/key'),request)
        payload=io.BytesIO(); pack(capture,expected,payload)
        subprocess.run([sys.executable,*shlex.split(argv[-1])[1:]],input=payload.getvalue(),check=True)
        verify(dsw/'worker-capture',expected)
        request.update(operation='send')
        argv=ssh_argv(dict(host='user@example.com',port=739,key='/tmp/key'),request)
        result=subprocess.run([sys.executable,*shlex.split(argv[-1])[1:]],stdout=subprocess.PIPE,check=True)
        unpack(robot,'worker-return',expected,io.BytesIO(result.stdout))
        verify(robot/'worker-return',expected)
        request=dict(operation='receive',root='/tmp/root;$(echo bad)',directory='capture',manifest=expected)
        argv=ssh_argv(dict(host='user@example.com',port=739,key='/tmp/key'),request)
        command=shlex.split(argv[-1])
        assert command[:3] == ['python3','-B','-c'] and len(command)==4
        assert '/tmp/root;$(echo bad)' not in argv[-1]  # encoded data, never shell syntax
        rejects(lambda: ssh_argv(dict(host='host;echo bad',port=739,key='/tmp/key'),request))
    print('artifact transport: capture upload, target download, idempotence and trust-boundary checks passed')


if __name__ == '__main__':
    run()
