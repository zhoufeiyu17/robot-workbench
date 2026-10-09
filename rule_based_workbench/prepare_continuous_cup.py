"""Prepare a new, pinned geometry candidate without starting Isaac or a GPU."""
import argparse
import json
from pathlib import Path
import time

from continuous_cup import prepare_assets, prepare_native
from playground_geometry import load


def main(args):
    args.output.mkdir(parents=True,exist_ok=False)
    started=time.perf_counter()
    native=args.output/'native'
    native_files=prepare_native(args.native_code,native)
    # Reuse the existing numerical-package loader; no renderer/GPU is loaded.
    load(args.input,json.loads(args.source.read_text()),native,args.assets)
    model=prepare_assets(args.assets,args.output/'assets')
    model.update(native_overrides_sha256=native_files,hardware_access=False,
                 elapsed_seconds=time.perf_counter()-started)
    (args.output/'cup-model.json').write_text(json.dumps(model,indent=2))
    print(json.dumps(model,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('native-code','assets','input','source','output'):
        p.add_argument('--'+name,type=Path,required=True)
    main(p.parse_args())
