#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$(pwd):$(pwd)/infra:$(pwd)/examples/scripts/debug${PYTHONPATH:+:$PYTHONPATH}"
python -m unittest discover -s tests -p 'test_joint_head_*.py' -v
python tests/test_joint_head_isaac_harness.py
python examples/scripts/debug/joint_head_isaac_review.py --self-test
