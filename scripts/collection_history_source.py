"""Print a pinned completed fake history specification; read-only, no GPU."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_workbench.collection_history import source_spec

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run',type=Path,required=True)
    parser.add_argument('--job',type=Path,action='append',required=True)
    args=parser.parse_args()
    print(json.dumps(source_spec(args.source_run,args.job),ensure_ascii=False,indent=2))
