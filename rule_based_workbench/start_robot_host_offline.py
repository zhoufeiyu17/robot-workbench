"""Start the robot-host UI without cameras, controller, detector or GPU workers."""
import argparse
import json
import os
from pathlib import Path
import secrets
import sys


def prepare(directory, host, port):
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    token = directory / 'offline-control.token'
    # Offline-only token cannot grant access to any existing owner.
    if not token.exists():
        with open(token, 'x', opener=lambda p, flags: os.open(p, flags, 0o600)) as out:
            out.write(secrets.token_urlsafe(32))
    config = directory / 'offline-real-config.json'
    config.write_text(json.dumps({
        'hardware_access_enabled': False,
        'robot_id': 'offline-unbound',
        'control_url': 'http://127.0.0.1:18402',
        'token_file': str(token),
        'cameras': {name: {'url': f'http://127.0.0.1:{18403 + i}'}
                    for i, name in enumerate(('base_0', 'left_wrist_0', 'right_wrist_0'))},
    }, indent=2))
    return [sys.executable, '-u', str(Path(__file__).with_name('app.py')),
            '--host', host, '--allow-host', host, '--port', str(port),
            '--state-dir', str(directory / 'ui-state'), '--real-config', str(config)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state-dir', type=Path, required=True)
    parser.add_argument('--host', default='10.17.32.5')
    parser.add_argument('--port', type=int, default=8119)
    parser.add_argument('--print-command', action='store_true', help='prepare offline config but do not start')
    args = parser.parse_args()
    command = prepare(args.state_dir, args.host, args.port)
    if args.print_command:
        import shlex
        print(shlex.join(command))
    else:
        os.execv(sys.executable, command)


if __name__ == '__main__':
    main()
