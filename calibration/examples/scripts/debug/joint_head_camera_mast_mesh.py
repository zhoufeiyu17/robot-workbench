"""Restore the original camera mount seat mid-rod in the extended mast mesh.

Why this exists
---------------
The physical setup: an extension rod is bolted on top of the original camera
mounting rod, so the original mount seat (plate + side bracket) is still there
half way up the mast. The mesh shipped earlier was built by *moving* the whole
top assembly up by 0.24 m and stretching the rod behind it, which deleted the
seat from the middle and left it only at the top.

This derives the corrected chassis mesh without touching anything else:

    corrected = current_extension + upper_block_of_original

i.e. the extension rod and its top assembly stay exactly as modelled, and the
original seat is put back where it physically is. The block is cut out of the
original mesh and re-closed with a centroid cap so the chassis keeps satisfying
the loader's closed two-manifold requirement.

    python3 joint_head_camera_mast_mesh.py --help
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

# The original plate starts at z=0.24 and the rod's top ring sits at 0.236, so
# anything at or above this plane is the seat (plate + the rod's last ring),
# nothing below it is part of the seat.
SEAT_CUT_Z_M = 0.2198


def read_stl(path):
    raw = Path(path).read_bytes()
    n = int.from_bytes(raw[80:84], 'little')
    if len(raw) != 84 + 50 * n:
        raise ValueError('%s is not a binary STL' % path)
    dt = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    return np.frombuffer(raw, dtype=dt, offset=84)['v'].astype(float).reshape(-1, 3, 3)


def write_stl(path, triangles):
    rec = np.zeros((len(triangles), 50), dtype=np.uint8)
    rec[:, 12:48] = triangles.astype('<f4').reshape(len(triangles), 9).view(np.uint8).reshape(len(triangles), 36)
    Path(path).write_bytes(bytes(80) + np.array([len(triangles)], '<u4').tobytes() + rec.tobytes())


def edge_incidence(triangles):
    unique, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    edges = np.sort(np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]])), axis=1)
    keys, counts = np.unique(edges[:, 0] * len(unique) + edges[:, 1], return_counts=True)
    return unique, counts, keys


def open_loops(triangles):
    """Boundary cycles of a triangle soup, as vertex index rings."""
    unique, counts, keys = edge_incidence(triangles)
    free = keys[counts == 1]
    adjacency = {}
    for key in free:
        a, b = divmod(int(key), len(unique))
        adjacency.setdefault(a, []).append(b)
        adjacency.setdefault(b, []).append(a)
    odd = {v: n for v, n in adjacency.items() if len(n) != 2}
    if odd:
        raise ValueError('open boundary is not a set of simple rings (vertices with degree != 2: %d)' % len(odd))
    loops, seen = [], set()
    for start in adjacency:
        if start in seen:
            continue
        ring, previous, current = [start], None, start
        while True:
            seen.add(current)
            nxt = [v for v in adjacency[current] if v != previous]
            if not nxt:
                break
            previous, current = current, nxt[0]
            if current == start:
                break
            ring.append(current)
        loops.append(ring)
    return unique, loops


def cap_boundary(triangles):
    unique, loops = open_loops(triangles)
    extra = []
    for ring in loops:
        points = unique[ring]
        centre = points.mean(0)
        for i in range(len(ring)):
            extra.append([centre, points[i], points[(i + 1) % len(ring)]])
    if not extra:
        return triangles, 0
    return np.concatenate([triangles, np.array(extra)]), len(extra)


def closed_or_raise(triangles, label):
    _, counts, _ = edge_incidence(triangles)
    bad = counts != 2
    if bad.any():
        raise ValueError('%s: %d edges are not shared by exactly two triangles' % (label, int(bad.sum())))


def triangle_keys(triangles):
    """Order-independent identity of a triangle set, so derivation stays idempotent."""
    vertices = np.round(triangles.reshape(-1, 3, 3), 6)
    vertices = np.sort(vertices, axis=1)
    return np.unique(vertices.reshape(len(triangles), 9), axis=0)


def contains_all(haystack, needles):
    keys = triangle_keys(haystack)
    return len(np.setdiff1d(triangle_keys(needles), keys, assume_unique=False)) == 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--original', required=True, help='chassis mesh without the extension rod')
    parser.add_argument('--extended', required=True, help='chassis mesh currently in use (extension modelled)')
    parser.add_argument('--output', required=True, help='corrected mesh to write')
    parser.add_argument('--report', help='optional JSON report path')
    parser.add_argument('--cut-z', type=float, default=SEAT_CUT_Z_M)
    args = parser.parse_args()

    original = read_stl(args.original)
    extended = read_stl(args.extended)
    if len(original) != len(extended):
        pass  # only informational; the derivation does not require equal counts

    block = original[original[:, :, 2].min(1) >= args.cut_z]
    if not len(block):
        raise ValueError('no original geometry above z=%.4f; wrong mesh?' % args.cut_z)
    _, counts, _ = edge_incidence(extended)
    if counts.min() == 1:
        raise ValueError('the extended mesh is not closed; refusing to modify it')
    if contains_all(extended, block):
        print('the extended mesh already carries the original seat at z>=%.4f; nothing to restore' % args.cut_z)
        write_stl(args.output, extended)
        return 0

    capped, cap_triangles = cap_boundary(block)
    closed_or_raise(capped, 'restored seat')
    corrected = np.concatenate([extended, capped])
    closed_or_raise(corrected, 'corrected chassis')

    seat = capped[:, :, 2]
    write_stl(args.output, corrected)
    report = {
        'original_triangles': int(len(original)),
        'extension_triangles': int(len(extended)),
        'restored_seat_triangles': int(len(block)),
        'cap_triangles': int(cap_triangles),
        'corrected_triangles': int(len(corrected)),
        'cut_z_m': args.cut_z,
        'restored_seat_z_m': [float(seat.min()), float(seat.max())],
        'top_block_z_m': [float(extended[:, :, 2].max() - 0.0587), float(extended[:, :, 2].max())],
        'output': str(args.output),
        'closed_manifold': True,
    }
    print(json.dumps(report, indent=2, ensure_ascii=False))
    if args.report:
        Path(args.report).write_text(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    sys.exit(main())
