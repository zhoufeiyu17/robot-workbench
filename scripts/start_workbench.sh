#!/bin/sh
# Install at the isolated task root. This starts services, never robot control.
set -eu
WORKBENCH_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ACTION=start
if [ "${1:-}" = "--check" ]; then
    ACTION=check
    shift
fi
exec "$WORKBENCH_ROOT/env/bin/python" -B \
    "$WORKBENCH_ROOT/deployment-tools-v1/workbench_deploy.py" "$ACTION" \
    --manifest "$WORKBENCH_ROOT/startup/current-release.json" "$@"
