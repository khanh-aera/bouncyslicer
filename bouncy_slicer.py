#!/usr/bin/env python3
"""
Bouncy Slicer - Arch vase mode G-code generator.

Supports: cylinder, box (rectangular prism), pentagon, hexagon, arbitrary n-gon, STL files.
Toolpath traces bouncy arch patterns instead of flat layers.
Open output in PrusaSlicer (File > Open G-code) to visualize.
"""

import math
import argparse
import struct
from dataclasses import dataclass


@dataclass
class Pt:
    x: float
    y: float
    z: float


# ── Shape classes ─────────────────────────────────────────────────────


class Shape:
    """Base class. Shapes provide surface_point(t, z) -> (x, y)."""

    def surface_point(self, t, z):
        """(x, y) at parametric position t in [0,1) around perimeter, height z."""
        raise NotImplementedError

    def max_height(self):
        raise NotImplementedError

    def max_inset(self):
        """Max inward offset before shape degenerates (for bottom fill)."""
        raise NotImplementedError

    def inset_point(self, t, z, d):
        """(x, y) offset d mm inward from surface."""
        raise NotImplementedError

    def describe(self):
        """Short description for G-code header."""
        return "shape"


class Cylinder(Shape):
    def __init__(self, radius, height):
        self.radius = radius
        self._height = height

    def max_height(self):
        return self._height

    def max_inset(self):
        return self.radius

    def surface_point(self, t, z):
        a = 2 * math.pi * t
        return (self.radius * math.cos(a), self.radius * math.sin(a))

    def inset_point(self, t, z, d):
        r = max(0, self.radius - d)
        a = 2 * math.pi * t
        return (r * math.cos(a), r * math.sin(a))

    def describe(self):
        return f"Cylinder: {self.radius*2:.1f}mm dia x {self._height:.1f}mm"


class ExtrudedPolygon(Shape):
    """Regular n-gon extruded along Z."""

    def __init__(self, n, circumradius, height, rotation=0):
        self.n = n
        self.cr = circumradius
        self._height = height
        self.rot = rotation
        self.verts = []
        for i in range(n):
            a = 2 * math.pi * i / n + rotation
            self.verts.append(
                (circumradius * math.cos(a), circumradius * math.sin(a))
            )
        self.edge_lens = []
        self.cum_lens = [0.0]
        for i in range(n):
            v1 = self.verts[i]
            v2 = self.verts[(i + 1) % n]
            el = math.hypot(v2[0] - v1[0], v2[1] - v1[1])
            self.edge_lens.append(el)
            self.cum_lens.append(self.cum_lens[-1] + el)
        self.perim = self.cum_lens[-1]

    def max_height(self):
        return self._height

    def max_inset(self):
        return self.cr * math.cos(math.pi / self.n)

    def _sample(self, t, ratio=1.0):
        total = self.perim * ratio
        if total < 1e-10:
            return (0.0, 0.0)
        dist = (t % 1.0) * total
        cum = 0.0
        for i in range(self.n):
            el = self.edge_lens[i] * ratio
            if cum + el >= dist - 1e-10:
                frac = (dist - cum) / el if el > 1e-10 else 0.0
                frac = max(0.0, min(1.0, frac))
                v1 = self.verts[i]
                v2 = self.verts[(i + 1) % self.n]
                return (
                    (v1[0] + frac * (v2[0] - v1[0])) * ratio,
                    (v1[1] + frac * (v2[1] - v1[1])) * ratio,
                )
            cum += el
        return (self.verts[0][0] * ratio, self.verts[0][1] * ratio)

    def surface_point(self, t, z):
        return self._sample(t)

    def inset_point(self, t, z, d):
        cos_half = math.cos(math.pi / self.n)
        new_cr = max(0, self.cr - d / cos_half)
        return self._sample(t, ratio=new_cr / self.cr if self.cr > 0 else 0)

    def describe(self):
        names = {3: "Triangle", 4: "Rectangular Prism", 5: "Pentagon", 6: "Hexagon"}
        name = names.get(self.n, f"{self.n}-gon")
        side = 2 * self.cr * math.sin(math.pi / self.n)
        return f"{name}: {side:.1f}mm side x {self._height:.1f}mm"


class STLShape(Shape):
    """Shape loaded from an STL file."""

    def __init__(self, filepath, scale=1.0):
        self.tris = _subdivide_mesh(parse_stl(filepath))
        if scale != 1.0:
            self.tris = [
                tuple(tuple(c * scale for c in v) for v in tri)
                for tri in self.tris
            ]
        xs, ys, zs = [], [], []
        for v1, v2, v3 in self.tris:
            for v in (v1, v2, v3):
                xs.append(v[0])
                ys.append(v[1])
                zs.append(v[2])
        self.min_z = min(zs)
        self._max_z = max(zs)
        self._cx = (min(xs) + max(xs)) / 2
        self._cy = (min(ys) + max(ys)) / 2
        self._contour_cache = {}
        self.filepath = filepath
        # Z-sorted index: triangles sorted by z_min, with z_max for early exit
        self._z_index = sorted(
            ((min(v1[2], v2[2], v3[2]), max(v1[2], v2[2], v3[2]), v1, v2, v3)
             for v1, v2, v3 in self.tris),
            key=lambda t: t[0]
        )
        # Pre-compute reference alignment point, contour length, and area
        self._ref_align_point = None  # will be set by _compute_ref_contour_length
        self._ref_contour_area = 0.0
        self._ref_contour_length = self._compute_ref_contour_length()

    def max_height(self):
        return self._max_z - self.min_z

    def max_inset(self):
        c = self._contour_at(self.max_height() * 0.01)
        if not c:
            return 10.0
        return _contour_length(c) / (2 * math.pi)

    def _compute_ref_contour_length(self):
        """Sample contours at several Z heights to establish reference length, area, and alignment point."""
        h = self._max_z - self.min_z
        samples = []
        for frac in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9):
            c = self._extract_contour(frac * h)
            if c:
                samples.append((_contour_length(c), _contour_area(c), c))
        if not samples:
            return 0.0
        # Use the median-length contour as reference
        samples.sort(key=lambda x: x[0])
        median_len, median_area, median_contour = samples[len(samples) // 2]
        # Set reference alignment point = t=0 of the median contour (aligned by angle=0)
        self._ref_align_point = median_contour[0]
        # Reference area = median of all sampled areas (robust to outliers)
        areas = sorted(s[1] for s in samples)
        self._ref_contour_area = areas[len(areas) // 2]
        return median_len

    def _extract_contour(self, z):
        """Extract and center contour at height z (no caching, no quality check)."""
        model_z = z + self.min_z
        segs = _slice_indexed(self._z_index, model_z)
        contours = chain_segments(segs) if segs else []
        if not contours:
            return []
        best = _pick_best_contour(contours)
        if not best:
            return []
        centered = [(x - self._cx, y - self._cy) for x, y in best]
        return _align_contour(centered, ref_point=self._ref_align_point)

    def _is_good_contour(self, contour):
        """Check if a contour meets quality criteria: length, area, and alignment distance."""
        if not contour:
            return False
        if self._ref_contour_length <= 0:
            return True
        # Length check: contour must be at least 70% of reference
        if _contour_length(contour) < 0.7 * self._ref_contour_length:
            return False
        # Area check: contour must enclose at least 70% of reference area
        # (catches fragmented contours that have decent perimeter but small area)
        if self._ref_contour_area > 0:
            if _contour_area(contour) < 0.7 * self._ref_contour_area:
                return False
        # Alignment check: t=0 vertex must be close to the reference point
        if self._ref_align_point is not None:
            approx_r = self._ref_contour_length / (2 * math.pi)
            dx = contour[0][0] - self._ref_align_point[0]
            dy = contour[0][1] - self._ref_align_point[1]
            if math.hypot(dx, dy) > approx_r * 0.5:
                return False
        return True

    def _contour_at(self, z):
        zk = round(z, 2)
        if zk in self._contour_cache:
            return self._contour_cache[zk]
        contour = self._extract_contour(z)
        # If contour fails quality checks, try nearby Z offsets
        if not self._is_good_contour(contour):
            for dz in (0.05, -0.05, 0.1, -0.1, 0.2, -0.2, 0.4, -0.4):
                alt = self._extract_contour(z + dz)
                if self._is_good_contour(alt):
                    contour = alt
                    break
        self._contour_cache[zk] = contour
        return contour

    def surface_point(self, t, z):
        c = self._contour_at(z)
        return _point_on_contour(c, t) if c else (0.0, 0.0)

    def inset_point(self, t, z, d):
        c = self._contour_at(z)
        if not c:
            return (0.0, 0.0)
        cl = _contour_length(c, closed=_is_closed(c))
        if cl < 1e-6:
            return (0.0, 0.0)
        approx_r = cl / (2 * math.pi)
        scale = max(0, (approx_r - d) / approx_r)
        x, y = _point_on_contour(c, t)
        return (x * scale, y * scale)

    def describe(self):
        import os

        name = os.path.basename(self.filepath)
        h = self.max_height()
        return f"STL: {name}, {h:.1f}mm tall, {len(self.tris)} triangles"


# ── STL utilities ─────────────────────────────────────────────────────


def parse_stl(path):
    """Parse binary or ASCII STL. Returns list of ((x,y,z), (x,y,z), (x,y,z))."""
    with open(path, "rb") as f:
        header = f.read(80)
        data = f.read(4)
        if len(data) == 4:
            n_tris = struct.unpack("<I", data)[0]
            tris = []
            for _ in range(n_tris):
                chunk = f.read(50)
                if len(chunk) < 50:
                    break
                vals = struct.unpack("<12fH", chunk)
                tris.append(
                    (
                        (vals[3], vals[4], vals[5]),
                        (vals[6], vals[7], vals[8]),
                        (vals[9], vals[10], vals[11]),
                    )
                )
            if tris:
                return tris
    # Fallback: ASCII STL
    tris = []
    verts = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip().lower()
            if line.startswith("vertex"):
                parts = line.split()
                verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
                if len(verts) == 3:
                    tris.append(tuple(verts))
                    verts = []
    return tris


def _subdivide_mesh(tris, max_edge=5.0, max_tris=800000):
    """Subdivide triangles until no edge exceeds max_edge length.

    Iterative work-stack version with a hard triangle budget so dense
    source meshes cannot exhaust Pyodide memory (MemoryError guard)."""
    work = list(tris)
    out = []
    while work:
        if len(out) + len(work) >= max_tris:
            # Budget exhausted: accept remaining triangles unsplit.
            out.extend(work)
            break
        v1, v2, v3 = work.pop()
        e0 = math.sqrt((v2[0]-v1[0])**2 + (v2[1]-v1[1])**2 + (v2[2]-v1[2])**2)
        e1 = math.sqrt((v3[0]-v2[0])**2 + (v3[1]-v2[1])**2 + (v3[2]-v2[2])**2)
        e2 = math.sqrt((v1[0]-v3[0])**2 + (v1[1]-v3[1])**2 + (v1[2]-v3[2])**2)
        longest = max(e0, e1, e2)
        if longest <= max_edge:
            out.append((v1, v2, v3))
            continue
        if e0 >= e1 and e0 >= e2:  # split v1-v2
            mid = ((v1[0]+v2[0])/2, (v1[1]+v2[1])/2, (v1[2]+v2[2])/2)
            work.append((v1, mid, v3))
            work.append((mid, v2, v3))
        elif e1 >= e2:  # split v2-v3
            mid = ((v2[0]+v3[0])/2, (v2[1]+v3[1])/2, (v2[2]+v3[2])/2)
            work.append((v1, v2, mid))
            work.append((v1, mid, v3))
        else:  # split v3-v1
            mid = ((v3[0]+v1[0])/2, (v3[1]+v1[1])/2, (v1[2]+v3[2])/2)
            work.append((v1, v2, mid))
            work.append((mid, v2, v3))
    return out


def slice_mesh(tris, z):
    """Intersect triangles with Z plane. Returns list of ((x1,y1),(x2,y2)) segments."""
    segs = []
    eps = 1e-6
    for v1, v2, v3 in tris:
        verts = [v1, v2, v3]
        above = [v for v in verts if v[2] > z + eps]
        below = [v for v in verts if v[2] < z - eps]
        on = [v for v in verts if abs(v[2] - z) <= eps]

        if len(on) == 2:
            p1, p2 = (on[0][0], on[0][1]), (on[1][0], on[1][1])
            if math.hypot(p1[0] - p2[0], p1[1] - p2[1]) > eps:
                segs.append((p1, p2))
        elif len(on) == 1 and len(above) == 1 and len(below) == 1:
            p1 = (on[0][0], on[0][1])
            p2 = _edge_intersect(above[0], below[0], z)
            if p2 and math.hypot(p1[0] - p2[0], p1[1] - p2[1]) > eps:
                segs.append((p1, p2))
        elif above and below and not on:
            single = above[0] if len(above) == 1 else below[0]
            pair = below if len(above) == 1 else above
            p1 = _edge_intersect(single, pair[0], z)
            p2 = _edge_intersect(single, pair[1], z)
            if p1 and p2:
                segs.append((p1, p2))
    return segs


def _slice_indexed(z_index, z):
    """Fast Z-plane slice using sorted index. Skips triangles entirely above/below."""
    import bisect
    # Find first triangle whose z_min could reach z (z_min <= z)
    # z_index is sorted by z_min; skip all with z_min > z
    segs = []
    eps = 1e-6
    # Binary search: find rightmost entry with z_min <= z + eps
    hi = bisect.bisect_right(z_index, z + eps, key=lambda t: t[0])
    for idx in range(hi):
        z_min, z_max, v1, v2, v3 = z_index[idx]
        if z_max < z - eps:
            continue  # entirely below
        # Triangle spans z — intersect it
        verts = [v1, v2, v3]
        above = [v for v in verts if v[2] > z + eps]
        below = [v for v in verts if v[2] < z - eps]
        on = [v for v in verts if abs(v[2] - z) <= eps]
        if len(on) == 2:
            p1, p2 = (on[0][0], on[0][1]), (on[1][0], on[1][1])
            if math.hypot(p1[0] - p2[0], p1[1] - p2[1]) > eps:
                segs.append((p1, p2))
        elif len(on) == 1 and len(above) == 1 and len(below) == 1:
            p1 = (on[0][0], on[0][1])
            p2 = _edge_intersect(above[0], below[0], z)
            if p2 and math.hypot(p1[0] - p2[0], p1[1] - p2[1]) > eps:
                segs.append((p1, p2))
        elif above and below and not on:
            single = above[0] if len(above) == 1 else below[0]
            pair = below if len(above) == 1 else above
            p1 = _edge_intersect(single, pair[0], z)
            p2 = _edge_intersect(single, pair[1], z)
            if p1 and p2:
                segs.append((p1, p2))
    return segs


def _edge_intersect(v1, v2, z):
    dz = v2[2] - v1[2]
    if abs(dz) < 1e-10:
        return None
    t = (z - v1[2]) / dz
    if t < -1e-6 or t > 1 + 1e-6:
        return None
    t = max(0.0, min(1.0, t))
    return (v1[0] + t * (v2[0] - v1[0]), v1[1] + t * (v2[1] - v1[1]))


def chain_segments(segments):
    """Chain line segments into contours, merging fragments with loose tolerance."""
    if not segments:
        return []
    eps = 0.01
    remaining = list(segments)
    chains = []
    while remaining:
        chain = [remaining.pop(0)]
        changed = True
        while changed:
            changed = False
            for i, seg in enumerate(remaining):
                p1, p2 = seg
                head = chain[0][0]
                tail = chain[-1][1]
                if _close(tail, p1, eps):
                    chain.append(seg)
                    remaining.pop(i)
                    changed = True
                    break
                if _close(tail, p2, eps):
                    chain.append((p2, p1))
                    remaining.pop(i)
                    changed = True
                    break
                if _close(head, p2, eps):
                    chain.insert(0, seg)
                    remaining.pop(i)
                    changed = True
                    break
                if _close(head, p1, eps):
                    chain.insert(0, (p2, p1))
                    remaining.pop(i)
                    changed = True
                    break
        chains.append(chain)
    # Merge pass: join open (non-closed) chains whose endpoints are nearby
    merge_eps = 1.0
    merged = True
    while merged:
        merged = False
        for i in range(len(chains)):
            if chains[i] is None:
                continue
            if _close(chains[i][0][0], chains[i][-1][1], eps):
                continue  # already closed, skip
            for j in range(i + 1, len(chains)):
                if chains[j] is None:
                    continue
                a_head, a_tail = chains[i][0][0], chains[i][-1][1]
                b_head, b_tail = chains[j][0][0], chains[j][-1][1]
                if _close(a_tail, b_head, merge_eps):
                    chains[i] = chains[i] + chains[j]
                    chains[j] = None
                    merged = True
                    break
                if _close(a_tail, b_tail, merge_eps):
                    chains[i] = chains[i] + [(s[1], s[0]) for s in reversed(chains[j])]
                    chains[j] = None
                    merged = True
                    break
                if _close(a_head, b_tail, merge_eps):
                    chains[i] = chains[j] + chains[i]
                    chains[j] = None
                    merged = True
                    break
                if _close(a_head, b_head, merge_eps):
                    chains[i] = [(s[1], s[0]) for s in reversed(chains[j])] + chains[i]
                    chains[j] = None
                    merged = True
                    break
            if merged:
                break
    contours = []
    for chain in chains:
        if chain is None:
            continue
        pts = [s[0] for s in chain]
        if len(pts) >= 3:
            contours.append(pts)
    return contours


def _close(a, b, eps):
    return abs(a[0] - b[0]) < eps and abs(a[1] - b[1]) < eps


def _ensure_ccw(pts):
    """Ensure contour winds counter-clockwise (positive signed area)."""
    if len(pts) < 3:
        return pts
    # Shoelace signed area
    area = 0.0
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        area += pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1]
    if area < 0:
        return list(reversed(pts))
    return pts


def _contour_area(pts):
    """Absolute enclosed area via shoelace formula."""
    if len(pts) < 3:
        return 0.0
    area = 0.0
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        area += pts[i][0] * pts[j][1] - pts[j][0] * pts[i][1]
    return abs(area) / 2.0


def _align_contour(pts, ref_point=None):
    """Normalize winding to CCW, then rotate so t=0 starts near a reference.

    If ref_point is given, aligns t=0 to the vertex closest to that point
    (Euclidean distance). Otherwise falls back to the vertex nearest angle=0
    from origin. Using a fixed ref_point ensures alignment continuity across
    Z heights even when contour shape changes (e.g. skull jaw area).
    """
    if len(pts) < 3:
        return pts
    pts = _ensure_ccw(pts)
    best_i = 0
    best_dist = float("inf")
    if ref_point is not None:
        rx, ry = ref_point
        for i, p in enumerate(pts):
            d = (p[0] - rx) ** 2 + (p[1] - ry) ** 2
            if d < best_dist:
                best_dist = d
                best_i = i
    else:
        for i, p in enumerate(pts):
            a = math.atan2(p[1], p[0]) % (2 * math.pi)
            d = min(a, 2 * math.pi - a)
            if d < best_dist:
                best_dist = d
                best_i = i
    return pts[best_i:] + pts[:best_i]


def _is_closed(pts, eps=1.0):
    """Check if last point is near first point."""
    if len(pts) < 3:
        return False
    return math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]) < eps


def _clean_contour(pts, jump_factor=10.0, min_abs_jump=5.0):
    """Split contour at abnormally large edges, return longest piece."""
    if len(pts) < 4:
        return pts
    n = len(pts)
    # Compute all edge lengths including closing edge
    edges = []
    for i in range(n):
        j = (i + 1) % n
        edges.append(math.hypot(pts[j][0] - pts[i][0], pts[j][1] - pts[i][1]))
    sorted_e = sorted(edges)
    median = sorted_e[len(sorted_e) // 2]
    if median < 1e-6:
        return pts
    # Edge must be both > jump_factor * median AND > min_abs_jump to be "bad"
    cutoff = max(median * jump_factor, min_abs_jump)
    bad = [i for i, e in enumerate(edges) if e > cutoff]
    if not bad:
        return pts
    # Split at bad edges, keep longest sub-contour
    subs = []
    for k in range(len(bad)):
        start = (bad[k] + 1) % n
        end = bad[(k + 1) % len(bad)]
        sub = []
        i = start
        while True:
            sub.append(pts[i])
            if i == end:
                break
            i = (i + 1) % n
            if len(sub) > n:
                break
        if len(sub) >= 3:
            subs.append(sub)
    return max(subs, key=len) if subs else pts


def _pick_best_contour(contours):
    """Pick the best contour: prefer properly closed ones, then longest."""
    if not contours:
        return []
    # Clean all contours
    cleaned = []
    for c in contours:
        cc = _clean_contour(c)
        if len(cc) >= 3:
            cleaned.append(cc)
    if not cleaned:
        return []
    # Prefer properly closed contours, but only if they have real perimeter
    def score(c):
        area = _contour_area(c)
        cl = _contour_length(c, closed=_is_closed(c))
        # Only prefer closed contours if they have meaningful size
        closed_bonus = 1 if (_is_closed(c) and cl > 10.0) else 0
        return (closed_bonus, area)
    return max(cleaned, key=score)


def _contour_length(pts, closed=True):
    if len(pts) < 2:
        return 0.0
    total = 0.0
    end = len(pts) if closed else len(pts) - 1
    for i in range(end):
        j = (i + 1) % len(pts)
        total += math.hypot(pts[j][0] - pts[i][0], pts[j][1] - pts[i][1])
    return total


def _sample_by_angle(angle_map, t):
    """Sample contour at angle 2*pi*t using precomputed angle-sorted points."""
    ccx, ccy, apts = angle_map
    n = len(apts)
    target = (2 * math.pi * (t % 1.0)) % (2 * math.pi)
    # Binary search for the first angle >= target
    lo, hi = 0, n
    while lo < hi:
        mid = (lo + hi) // 2
        if apts[mid][0] < target:
            lo = mid + 1
        else:
            hi = mid
    i2 = lo % n
    i1 = (lo - 1) % n
    a1 = apts[i1][0]
    a2 = apts[i2][0]
    # Angular gap between the two bracketing points
    da = a2 - a1
    if da < 0:
        da += 2 * math.pi
    dt = target - a1
    if dt < 0:
        dt += 2 * math.pi
    frac = dt / da if da > 1e-10 else 0.0
    frac = max(0.0, min(1.0, frac))
    return (
        apts[i1][1] + frac * (apts[i2][1] - apts[i1][1]),
        apts[i1][2] + frac * (apts[i2][2] - apts[i1][2]),
    )


def _build_cumlen(pts, closed):
    """Precompute cumulative arc-length array for a contour."""
    n = len(pts)
    end = n if closed else n - 1
    cum = [0.0] * (end + 1)
    for i in range(end):
        j = (i + 1) % n
        cum[i + 1] = cum[i] + math.hypot(pts[j][0] - pts[i][0], pts[j][1] - pts[i][1])
    return cum


def _point_on_contour(pts, t, _cache={}):
    if not pts:
        return (0.0, 0.0)
    if len(pts) == 1:
        return pts[0]
    # Cache cumulative arc-length by contour identity
    key = id(pts)
    if key not in _cache:
        closed = _is_closed(pts)
        _cache[key] = (closed, _build_cumlen(pts, closed))
    closed, cum = _cache[key]
    total = cum[-1]
    if total < 1e-10:
        return pts[0]
    target = (t % 1.0) * total
    n = len(pts)
    # Binary search for the segment containing target
    lo, hi = 0, len(cum) - 2
    while lo < hi:
        mid = (lo + hi) // 2
        if cum[mid + 1] < target - 1e-10:
            lo = mid + 1
        else:
            hi = mid
    i = lo
    j = (i + 1) % n
    seg_len = cum[i + 1] - cum[i]
    frac = (target - cum[i]) / seg_len if seg_len > 1e-10 else 0.0
    frac = max(0.0, min(1.0, frac))
    return (pts[i][0] + frac * (pts[j][0] - pts[i][0]),
            pts[i][1] + frac * (pts[j][1] - pts[i][1]))


def clear_caches():
    """Clear mutable caches for clean re-runs (e.g., in Pyodide)."""
    _point_on_contour.__defaults__[0].clear()


# ── G-code templates ──────────────────────────────────────────────────

DEFAULT_START_GCODE = """\
G90 ; absolute positioning
M82 ; absolute extrusion
M107 ; fan off during warmup
M104 S{hotend_temp} ; set hotend temp
M140 S{bed_temp} ; set bed temp
M190 S{bed_temp} ; wait for bed
M109 S{hotend_temp} ; wait for hotend
G28 ; home all axes
G21 ; mm units
G92 E0 ; reset extruder"""

DEFAULT_END_GCODE = """\
M104 S0 ; hotend off
M140 S0 ; bed off
M107 ; fan off
G91 ; relative positioning
G0 Z10 F3000 ; lift nozzle
G90 ; absolute positioning
G0 X0 Y{bed_y} F{travel_speed} ; present part
M84 ; disable steppers"""


# ── G-code generator ─────────────────────────────────────────────────


class BouncySlicer:
    def __init__(self, shape, args):
        self.shape = shape
        self.args = args
        self.lines = []
        self.e_total = 0.0
        self.prev = None
        self.cx = args.bed_x / 2
        self.cy = args.bed_y / 2
        self.max_descent_deg = 0.0

    def emit(self, line):
        self.lines.append(line)

    def _e_per_mm(self, bead_h):
        a = self.args
        return (a.nozzle_diameter * bead_h) / (
            math.pi * (a.filament_diameter / 2) ** 2
        )

    def move(self, x, y, z, speed=None):
        if speed is None:
            speed = self.args.travel_speed
        self.emit(f"G0 F{speed:.0f} X{x:.3f} Y{y:.3f} Z{z:.3f}")
        self.prev = Pt(x, y, z)

    def extrude_to(self, x, y, z, speed=None, bead_h=None):
        if speed is None:
            speed = self.args.print_speed
        if bead_h is None:
            bead_h = self.args.bead_height
        if self.prev is None:
            self.prev = Pt(x, y, z)
            return
        dx = x - self.prev.x
        dy = y - self.prev.y
        dz = z - self.prev.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dz < 0:
            horiz = math.sqrt(dx * dx + dy * dy)
            if horiz > 1e-10:
                descent = math.degrees(math.atan2(-dz, horiz))
                if descent > self.max_descent_deg:
                    self.max_descent_deg = descent
        self.e_total += dist * self._e_per_mm(bead_h)
        self.emit(
            f"G1 F{speed:.0f} X{x:.3f} Y{y:.3f} Z{z:.3f} E{self.e_total:.5f}"
        )
        self.prev = Pt(x, y, z)

    def retract(self):
        self.e_total -= self.args.retract_distance
        self.emit(f"G1 F{self.args.retract_speed:.0f} E{self.e_total:.5f}")

    def unretract(self):
        self.e_total += self.args.retract_distance
        self.emit(f"G1 F{self.args.retract_speed:.0f} E{self.e_total:.5f}")

    def travel_to(self, x, y, z):
        """Retract, Z-hop, travel, lower, unretract."""
        a = self.args
        self.retract()
        if self.prev and a.z_hop > 0:
            hop_z = self.prev.z + a.z_hop
            self.emit(f"G0 F{a.travel_speed:.0f} Z{hop_z:.3f}")
        self.move(x, y, z + (a.z_hop if a.z_hop > 0 else 0))
        if a.z_hop > 0:
            self.emit(f"G0 F{a.travel_speed:.0f} Z{z:.3f}")
            self.prev = Pt(x, y, z)
        self.unretract()

    # ── G-code sections ──────────────────────────────────────────

    def _emit_template(self, template):
        a = self.args
        replacements = {
            'bed_temp': str(a.bed_temp),
            'hotend_temp': str(a.hotend_temp),
            'bed_x': f'{a.bed_x:.0f}',
            'bed_y': f'{a.bed_y:.0f}',
            'travel_speed': f'{a.travel_speed:.0f}',
            'nozzle_diameter': f'{a.nozzle_diameter}',
            'filament_diameter': f'{a.filament_diameter}',
        }
        for line in template.splitlines():
            for key, val in replacements.items():
                line = line.replace('{' + key + '}', val)
            self.emit(line)

    def header(self):
        a = self.args
        s = self.shape
        start_z = a.bottom_layers * a.bottom_layer_height
        n_sl = max(1, int((s.max_height() - start_z) / a.arch_layer_height))

        self.emit("; Generated by Bouncy Slicer - Arch Vase Mode")
        self.emit(f"; {s.describe()}")
        self.emit(
            f"; Arches: {a.num_arches}/ring, "
            f"{a.arch_layer_height:.1f}mm tall, {n_sl} layers"
        )
        self.emit(f"; Segments/arc: {a.segments_per_arc*2}")
        self.emit("")
        template = getattr(a, 'start_gcode', None)
        if template is None:
            template = DEFAULT_START_GCODE
        self._emit_template(template)
        self.emit("")

    def footer(self):
        self.emit("")
        self.emit("; === End ===")
        template = getattr(self.args, 'end_gcode', None)
        if template is None:
            template = DEFAULT_END_GCODE
        self._emit_template(template)

    def bottom_layers(self):
        """Concentric bottom fill using shape's inset_point."""
        a = self.args
        if a.bottom_layers <= 0:
            return

        self.emit("; === Bottom solid layers ===")
        max_d = self.shape.max_inset()
        num_rings = max(1, int(max_d / a.nozzle_diameter))

        for layer_i in range(a.bottom_layers):
            z = (layer_i + 1) * a.bottom_layer_height
            speed = a.first_layer_speed if layer_i == 0 else a.print_speed

            self.emit(";LAYER_CHANGE")
            self.emit(f";Z:{z:.3f}")
            self.emit(f";HEIGHT:{a.bottom_layer_height:.3f}")
            self.emit(";TYPE:Solid infill")

            for ring_i in range(num_rings):
                d = ring_i * a.nozzle_diameter
                if d >= max_d:
                    break

                # Estimate perimeter for segment count
                p0 = self.shape.inset_point(0, z, d)
                p25 = self.shape.inset_point(0.25, z, d)
                p50 = self.shape.inset_point(0.5, z, d)
                p75 = self.shape.inset_point(0.75, z, d)
                est_perim = (
                    math.hypot(p25[0] - p0[0], p25[1] - p0[1])
                    + math.hypot(p50[0] - p25[0], p50[1] - p25[1])
                    + math.hypot(p75[0] - p50[0], p75[1] - p50[1])
                    + math.hypot(p0[0] - p75[0], p0[1] - p75[1])
                )
                if est_perim < a.nozzle_diameter:
                    break
                num_segs = max(32, int(est_perim / 0.5))

                sx, sy = self.shape.inset_point(0, z, d)
                sx += self.cx
                sy += self.cy

                if self.prev is not None:
                    self.travel_to(sx, sy, z)
                else:
                    self.move(sx, sy, z)

                for seg in range(1, num_segs + 1):
                    t = seg / num_segs
                    px, py = self.shape.inset_point(t, z, d)
                    self.extrude_to(
                        px + self.cx,
                        py + self.cy,
                        z,
                        speed,
                        bead_h=a.bottom_layer_height,
                    )

        self.emit("M106 S255 ; part cooling fan on")
        self.emit("")

    def arch_layers(self):
        """Generate bouncy arcs using shape.surface_point."""
        a = self.args
        s = self.shape

        start_z = a.bottom_layers * a.bottom_layer_height
        available = s.max_height() - start_z
        num_layers = max(1, int(available / a.arch_layer_height))
        segs = 2 * a.segments_per_arc

        self.emit("; === Arch layers ===")
        self.emit(f"; {num_layers} layers, {a.num_arches} arcs each")
        self.emit("")

        for li in range(num_layers):
            z_bot = start_z + li * a.arch_layer_height
            z_top = z_bot + a.arch_layer_height
            # Phase offset alternates each layer for brick-laying pattern
            phase = (li % 2) * (0.5 / a.num_arches)

            self.emit(";LAYER_CHANGE")
            self.emit(f";Z:{z_top:.3f}")
            self.emit(f";HEIGHT:{a.arch_layer_height:.3f}")
            self.emit(";TYPE:External perimeter")
            self.emit(f"; Arch layer {li+1}/{num_layers}: z={z_bot:.2f}-{z_top:.2f}")

            # Travel to start
            sx, sy = s.surface_point(phase, z_bot)
            self.travel_to(sx + self.cx, sy + self.cy, z_bot)

            # Trace arcs — blend XY between bottom and top contours
            # so arcs are smooth curves instead of following mesh jagginess
            for si in range(a.num_arches):
                for seg in range(1, segs + 1):
                    t_arc = seg / segs
                    t_perim = (si + t_arc) / a.num_arches + phase
                    blend = math.sin(math.pi * t_arc)  # 0 at bottom, 1 at peak
                    z = z_bot + a.arch_layer_height * blend

                    bx, by = s.surface_point(t_perim, z_bot)
                    tx, ty = s.surface_point(t_perim, z_top)
                    px = bx + blend * (tx - bx)
                    py = by + blend * (ty - by)
                    self.extrude_to(px + self.cx, py + self.cy, z)

            self.emit("")

    def top_layers(self):
        """Flat perimeter rings at the top of the shape, like bottom_layers."""
        a = self.args
        n_top = getattr(a, "top_layers", getattr(a, "top_layers_count", 0))
        if n_top <= 0:
            return
        top_lh = getattr(a, "top_layer_height", a.bead_height)

        start_z = a.bottom_layers * a.bottom_layer_height
        available = self.shape.max_height() - start_z
        num_arch = max(1, int(available / a.arch_layer_height))
        arch_top_z = start_z + num_arch * a.arch_layer_height

        self.emit("; === Top perimeter layers ===")

        for layer_i in range(n_top):
            z = arch_top_z + (layer_i + 1) * top_lh
            if z > self.shape.max_height() + top_lh:
                break

            self.emit(";LAYER_CHANGE")
            self.emit(f";Z:{z:.3f}")
            self.emit(f";HEIGHT:{top_lh:.3f}")
            self.emit(";TYPE:External perimeter")

            # Estimate segment count from perimeter at this height
            p0 = self.shape.surface_point(0, z)
            p25 = self.shape.surface_point(0.25, z)
            p50 = self.shape.surface_point(0.5, z)
            p75 = self.shape.surface_point(0.75, z)
            est_perim = (
                math.hypot(p25[0] - p0[0], p25[1] - p0[1])
                + math.hypot(p50[0] - p25[0], p50[1] - p25[1])
                + math.hypot(p75[0] - p50[0], p75[1] - p50[1])
                + math.hypot(p0[0] - p75[0], p0[1] - p75[1])
            )
            num_segs = max(32, int(est_perim / 0.5))

            sx, sy = self.shape.surface_point(0, z)
            self.travel_to(sx + self.cx, sy + self.cy, z)

            for seg in range(1, num_segs + 1):
                t = seg / num_segs
                px, py = self.shape.surface_point(t, z)
                self.extrude_to(px + self.cx, py + self.cy, z,
                                bead_h=top_lh)

        self.emit("")

    def generate(self):
        self.header()
        self.bottom_layers()
        self.arch_layers()
        self.top_layers()
        self.footer()
        return "\n".join(self.lines)


# ── CLI ───────────────────────────────────────────────────────────────


def build_shape(args):
    """Create Shape from CLI args."""
    if args.stl:
        stl_scale = getattr(args, 'stl_scale', 1.0)
        shape = STLShape(args.stl, scale=stl_scale)
        if args.height is None:
            args.height = shape.max_height()
        return shape

    h = args.height or 100.0

    if args.shape == "cylinder":
        return Cylinder(args.radius, h)
    elif args.shape == "box":
        side = args.side
        cr = side * math.sqrt(2) / 2
        return ExtrudedPolygon(4, cr, h, rotation=math.pi / 4)
    elif args.shape == "pentagon":
        return ExtrudedPolygon(5, args.circumradius, h)
    elif args.shape == "hexagon":
        return ExtrudedPolygon(6, args.circumradius, h)
    elif args.shape == "polygon":
        return ExtrudedPolygon(args.polygon_sides, args.circumradius, h)
    else:
        return Cylinder(args.radius, h)


def main():
    p = argparse.ArgumentParser(
        description="Bouncy Slicer - Arch Vase Mode",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    g = p.add_argument_group("Shape selection")
    g.add_argument(
        "--shape",
        choices=["cylinder", "box", "pentagon", "hexagon", "polygon"],
        default="cylinder",
    )
    g.add_argument("--stl", type=str, help="STL file (overrides --shape)")
    g.add_argument("--stl-scale", type=float, default=1.0, help="Scale factor for STL mesh")

    g = p.add_argument_group("Shape geometry")
    g.add_argument("--radius", type=float, default=25.0, help="Cylinder radius (mm)")
    g.add_argument("--side", type=float, default=50.0, help="Box side length (mm)")
    g.add_argument(
        "--circumradius",
        type=float,
        default=25.0,
        help="Polygon circumradius (mm)",
    )
    g.add_argument("--polygon-sides", type=int, default=6, help="Sides for --shape polygon")
    g.add_argument("--height", type=float, default=None, help="Height (mm, auto for STL)")

    g = p.add_argument_group("Arch parameters")
    g.add_argument("--num-arches", type=int, default=12, help="Arcs per ring")
    g.add_argument(
        "--arch-layer-height", type=float, default=8.0, help="Arch layer height (mm)"
    )
    g.add_argument(
        "--segments-per-arc",
        type=int,
        default=20,
        help="Segments per half-arc (total = 2x)",
    )

    g = p.add_argument_group("Print settings")
    g.add_argument("--nozzle-diameter", type=float, default=0.4)
    g.add_argument("--filament-diameter", type=float, default=1.75)
    g.add_argument("--bead-height", type=float, default=0.3, help="Bead height (mm)")
    g.add_argument("--print-speed", type=float, default=1800, help="mm/min (30mm/s)")
    g.add_argument("--first-layer-speed", type=float, default=900, help="mm/min (15mm/s)")
    g.add_argument("--travel-speed", type=float, default=7200, help="mm/min (120mm/s)")
    g.add_argument("--retract-distance", type=float, default=0.8)
    g.add_argument("--retract-speed", type=float, default=2400)
    g.add_argument("--z-hop", type=float, default=2.0, help="Z-hop during travels (mm)")

    g = p.add_argument_group("Temperature")
    g.add_argument("--bed-temp", type=int, default=60)
    g.add_argument("--hotend-temp", type=int, default=210)

    g = p.add_argument_group("Bed size")
    g.add_argument("--bed-x", type=float, default=250)
    g.add_argument("--bed-y", type=float, default=210)

    g = p.add_argument_group("Foundation")
    g.add_argument("--bottom-layers", type=int, default=3)
    g.add_argument("--bottom-layer-height", type=float, default=0.2)

    g = p.add_argument_group("Top finish")
    g.add_argument("--top-layers", type=int, default=0, help="Flat perimeter layers on top (0=none)")
    g.add_argument("--top-layer-height", type=float, default=0.3, help="Layer height for top perimeters (mm)")

    g = p.add_argument_group("Custom G-code")
    g.add_argument("--start-gcode-file", type=str, default=None,
                    help="File with custom start G-code (variables: {bed_temp} etc.)")
    g.add_argument("--end-gcode-file", type=str, default=None,
                    help="File with custom end G-code (variables: {bed_temp} etc.)")

    g = p.add_argument_group("Output")
    g.add_argument("-o", "--output", type=str, default="bouncy_vase.gcode")

    args = p.parse_args()
    if args.height is None:
        args.height = 100.0

    # Load custom G-code from files if specified
    if args.start_gcode_file:
        with open(args.start_gcode_file, "r") as f:
            args.start_gcode = f.read()
    else:
        args.start_gcode = None
    if args.end_gcode_file:
        with open(args.end_gcode_file, "r") as f:
            args.end_gcode = f.read()
    else:
        args.end_gcode = None

    shape = build_shape(args)
    slicer = BouncySlicer(shape, args)
    gcode = slicer.generate()

    with open(args.output, "w") as f:
        f.write(gcode)

    start_z = args.bottom_layers * args.bottom_layer_height
    n_layers = max(1, int((shape.max_height() - start_z) / args.arch_layer_height))

    print(f"Generated: {args.output}")
    print(f"  {shape.describe()}")
    print(
        f"  Arches: {args.num_arches}/ring, "
        f"{args.arch_layer_height:.1f}mm tall, {n_layers} layers"
    )
    print(f"  Segments/arc: {args.segments_per_arc*2}")
    print(f"  Filament: {slicer.e_total/1000:.2f}m")
    print(f"  G-code lines: {len(slicer.lines)}")
    print(f"  Max descent angle: {slicer.max_descent_deg:.1f}° (must be < nozzle cone half-angle)")
    print(f"\n  View: PrusaSlicer > File > Open G-code > {args.output}")


if __name__ == "__main__":
    main()
