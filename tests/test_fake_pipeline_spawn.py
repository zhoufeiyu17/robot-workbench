"""Native fake reuses the physical orchestration process and cancellation protocol."""
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench'),str(ROOT/'tests')]
from test_playground_execution_process import check


if __name__=='__main__':print(json.dumps(check(environment='fake'),indent=2))
