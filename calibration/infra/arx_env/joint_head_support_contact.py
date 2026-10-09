"""Closed-solid contact checks, optionally restricted to a front support region."""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

from .joint_head_workbench_geometry import point_in_mesh, surface_intersection_pairs


class PreparedMesh:
    """Keep triangle ordering when supplying rigidly transformed coordinates."""
    def __init__(self, triangles, *, require_closed=True):
        self.triangles = _triangles(triangles)
        vertices, inverse = np.unique(self.triangles.reshape(-1, 3), axis=0, return_inverse=True)
        self.faces = inverse.reshape(-1, 3)
        self.vertex_count = len(vertices)
        edges = self._edges(self.faces)
        _, counts = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
        area = np.linalg.norm(np.cross(self.triangles[:, 1]-self.triangles[:, 0],
                                       self.triangles[:, 2]-self.triangles[:, 0]), axis=1)
        self.closed = bool(np.all(counts == 2) and np.all(area > 0))
        if require_closed and not self.closed:
            raise ValueError('support contact requires closed, nondegenerate mesh components')
        labels = self._labels(edges)
        self.components = [np.flatnonzero(labels[self.faces[:, 0]] == label)
                           for label in np.unique(labels)]
        self.nodes = []
        centers = self.triangles.mean(axis=1)
        def build(indices):
            if len(indices) <= 16:
                node = (None, None, indices)
            else:
                axis = np.ptp(centers[indices], axis=0).argmax()
                ordered = indices[np.argsort(centers[indices, axis])]
                middle = len(ordered)//2
                node = (build(ordered[:middle]), build(ordered[middle:]), None)
            self.nodes.append(node)
            return len(self.nodes)-1
        self.root = build(np.arange(len(self.triangles)))


    def bounds(self, triangles):
        low, high = np.empty((len(self.nodes), 3)), np.empty((len(self.nodes), 3))
        for i, (left, right, indices) in enumerate(self.nodes):
            if indices is not None:
                low[i], high[i] = triangles[indices].min((0, 1)), triangles[indices].max((0, 1))
            else:
                low[i], high[i] = np.minimum(low[left], low[right]), np.maximum(high[left], high[right])
        return low, high

    @staticmethod
    def _edges(faces):
        return np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]))

    def _labels(self, edges):
        graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                           shape=(self.vertex_count, self.vertex_count))
        return connected_components(graph, directed=False)[1]

    def region_points(self, triangles, selected):
        """One actual region vertex for each connected selected surface patch."""
        if not len(selected):
            return np.empty((0, 3))
        faces = self.faces[selected]
        labels = self._labels(self._edges(faces))
        _, first = np.unique(labels[faces[:, 0]], return_index=True)
        patch = triangles[selected[first]]
        # Across the ROI boundary retain whole triangles conservatively, but
        # choose a point inside the ROI rather than on the discarded side.
        return patch[np.arange(len(patch)), patch[:, :, 0].argmax(axis=1)]


def _triangles(value):
    result = np.asarray(value, dtype=float)
    if result.ndim != 3 or result.shape[1:] != (3, 3) or not len(result) or not np.isfinite(result).all():
        raise ValueError('expected nonempty finite (N,3,3) triangles')
    return result


def region_overlap(a, b, *, a_triangles=None, b_triangles=None, roi_x_min=None):
    """Intersect closed component unions in a common frame.

    ROI only selects surface patches; all inside tests use original closed
    components. Crossing triangles are retained whole, conservatively counting
    contacts slightly behind the ROI boundary. No open cropped mesh is filled.
    """
    if not a.closed or not b.closed:
        raise ValueError('overlap requires closed nondegenerate components')
    at = a.triangles if a_triangles is None else _triangles(a_triangles)
    bt = b.triangles if b_triangles is None else _triangles(b_triangles)
    if at.shape != a.triangles.shape or bt.shape != b.triangles.shape:
        raise ValueError('transformed mesh must preserve triangle ordering and shape')
    if roi_x_min is not None and not np.isfinite(roi_x_min):
        raise ValueError('ROI boundary must be finite')
    ai = np.arange(len(at)) if roi_x_min is None else np.flatnonzero(at[:, :, 0].max(1) >= roi_x_min)
    bi = np.arange(len(bt)) if roi_x_min is None else np.flatnonzero(bt[:, :, 0].max(1) >= roi_x_min)
    if not len(ai) or not len(bi):
        return False
    # These bounds enclose the clipped solids too: triangles crossing the
    # region boundary are retained whole, including every cap-edge vertex.
    ar, br = at[ai], bt[bi]
    if np.any(ar.max((0, 1)) < br.min((0, 1))-1e-9) or np.any(br.max((0, 1)) < ar.min((0, 1))-1e-9):
        return False
    al, ah = a.bounds(at)
    bl, bh = b.bounds(bt)
    stack = [(a.root, b.root)]
    while stack:
        i, j = stack.pop()
        if np.any(ah[i] < bl[j]-1e-9) or np.any(bh[j] < al[i]-1e-9):
            continue
        if roi_x_min is not None and max(ah[i, 0], bh[j, 0]) < roi_x_min:
            continue
        la, ra, ia = a.nodes[i]
        lb, rb, ib = b.nodes[j]
        if ia is not None and ib is not None:
            for fa, fb in surface_intersection_pairs(at[ia], bt[ib]):
                if roi_x_min is None or max(at[ia[fa], :, 0].max(), bt[ib[fb], :, 0].max()) >= roi_x_min:
                    return True
        elif ia is None and (ib is not None or np.prod(ah[i]-al[i]) >= np.prod(bh[j]-bl[j])):
            stack.extend(((la, j), (ra, j)))
        else:
            stack.extend(((i, lb), (i, rb)))
    for mesh, triangles, selected, other, other_triangles in ((a, at, ai, b, bt), (b, bt, bi, a, at)):
        points = mesh.region_points(triangles, selected)
        for component in other.components:
            solid = other_triangles[component]
            lo, hi = solid.min((0, 1)), solid.max((0, 1))
            candidates = points[np.all(points >= lo, axis=1) & np.all(points <= hi, axis=1)]
            if any(point_in_mesh(point, solid) for point in candidates):
                return True
    return False


def region_separated_with_margin(a, b, *, a_triangles=None, b_triangles=None,
                                 roi_x_min=None, margin_m):
    """Prove separation for every B-point displacement of at most margin_m.

    A is fixed. A separating unit axis with projection gap greater than the
    displacement bound remains separating after any permitted displacement.
    Missing such a certificate is uncertainty, never permission to move.
    """
    if not np.isfinite(margin_m) or margin_m < 0:
        raise ValueError('nonnegative finite displacement margin required')
    at = a.triangles if a_triangles is None else _triangles(a_triangles)
    bt = b.triangles if b_triangles is None else _triangles(b_triangles)
    region = None if roi_x_min is None else roi_x_min-margin_m
    # No boundary crossing under the bound can create solid overlap unless
    # the initial solids already overlap; test containment before certificates.
    if region_overlap(a, b, a_triangles=at, b_triangles=bt, roi_x_min=region):
        return False
    al, ah = a.bounds(at)
    bl, bh = b.bounds(bt)
    stack = [(a.root, b.root)]
    while stack:
        i, j = stack.pop()
        if np.any(ah[i]+margin_m < bl[j]-1e-9) or np.any(bh[j]+margin_m < al[i]-1e-9):
            continue
        if region is not None and max(ah[i, 0], bh[j, 0]) < region:
            continue
        la, ra, ia = a.nodes[i]
        lb, rb, ib = b.nodes[j]
        if ia is not None and ib is not None:
            for triangle in at[ia]:
                candidates = bt[ib]
                if region is not None:
                    candidates = candidates[np.maximum(candidates[:, :, 0].max(1), triangle[:, 0].max()) >= region]
                if len(candidates) and not _separated_triangles(triangle, candidates, margin_m).all():
                    return False
        elif ia is None and (ib is not None or np.prod(ah[i]-al[i]) >= np.prod(bh[j]-bl[j])):
            stack.extend(((la, j), (ra, j)))
        else:
            stack.extend(((i, lb), (i, rb)))
    return True


def _separated_triangles(a, b, margin):
    """The shared triangle SAT axes, retaining the metric projection gap."""
    ea = np.roll(a, -1, axis=0)-a
    eb = np.roll(b, -1, axis=1)-b
    na = np.cross(ea[0], ea[1])
    nb = np.cross(eb[:, 0], eb[:, 1])
    axes = np.concatenate((np.broadcast_to(na, (len(b), 1, 3)), nb[:, None, :],
        np.cross(ea[None, :, None, :], eb[:, None, :, :]).reshape(-1, 9, 3),
        np.broadcast_to(np.cross(na, ea), (len(b), 3, 3)), np.cross(nb[:, None, :], eb)), axis=1)
    length = np.linalg.norm(axes, axis=2)
    axes = axes/np.maximum(length[:, :, None], 1e-30)
    pa = np.einsum('vj,naj->nav', a, axes)
    pb = np.einsum('nvj,naj->nav', b, axes)
    gap = np.maximum(pa.min(2)-pb.max(2), pb.min(2)-pa.max(2))
    return ((gap > margin+1e-9) & (length > 1e-14)).any(1)
