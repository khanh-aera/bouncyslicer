#!/usr/bin/env python3
"""Tests for bouncy_slicer.py — stdlib unittest only."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))

import bouncy_slicer as bs


# ── Helpers ───────────────────────────────────────────────────────────

def _make_circle(cx, cy, r, n=64):
    """CCW circle of n points."""
    return [
        (cx + r * math.cos(2 * math.pi * i / n),
         cy + r * math.sin(2 * math.pi * i / n))
        for i in range(n)
    ]


def _make_square(cx, cy, half):
    """CCW unit-ish square."""
    return [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
    ]


def _make_serpentine(cx, cy, r, zigzags=20, n_per_zig=8):
    """Zigzag contour that has long perimeter but small enclosed area.

    Creates a roughly circular path that serpentines in/out, producing a
    perimeter much longer than a simple circle of the same radius while
    enclosing less area.
    """
    pts = []
    total = zigzags * n_per_zig
    for i in range(total):
        angle = 2 * math.pi * i / total
        # Alternate between 0.5r and r
        phase = (i % n_per_zig) / n_per_zig
        wave = 0.5 + 0.5 * abs(math.sin(math.pi * phase * 2))
        rad = r * wave
        pts.append((cx + rad * math.cos(angle), cy + rad * math.sin(angle)))
    return pts


STL_PATH = os.path.join(os.path.dirname(__file__), "test_cube.stl")
SLOW = os.environ.get("BOUNCY_SLOW_TESTS", "") == "1"


# ── Edge intersect ────────────────────────────────────────────────────

class TestEdgeIntersect(unittest.TestCase):
    def test_midpoint(self):
        p = bs._edge_intersect((0, 0, 0), (0, 0, 10), 5)
        self.assertIsNotNone(p)
        self.assertAlmostEqual(p[0], 0.0)
        self.assertAlmostEqual(p[1], 0.0)

    def test_at_vertex(self):
        p = bs._edge_intersect((1, 2, 0), (3, 4, 10), 0)
        self.assertIsNotNone(p)
        self.assertAlmostEqual(p[0], 1.0)
        self.assertAlmostEqual(p[1], 2.0)

    def test_horizontal_edge(self):
        p = bs._edge_intersect((0, 0, 5), (10, 0, 5), 5)
        self.assertIsNone(p)

    def test_out_of_range(self):
        p = bs._edge_intersect((0, 0, 0), (0, 0, 10), 20)
        self.assertIsNone(p)


# ── Slice mesh ────────────────────────────────────────────────────────

class TestSliceMesh(unittest.TestCase):
    def test_single_tri_crossing(self):
        tri = ((0, 0, 0), (10, 0, 10), (10, 10, 0))
        segs = bs.slice_mesh([tri], 5)
        self.assertGreater(len(segs), 0)

    def test_above(self):
        tri = ((0, 0, 0), (10, 0, 0), (5, 5, 0))
        segs = bs.slice_mesh([tri], 10)
        self.assertEqual(len(segs), 0)

    def test_below(self):
        tri = ((0, 0, 10), (10, 0, 10), (5, 5, 10))
        segs = bs.slice_mesh([tri], 0)
        self.assertEqual(len(segs), 0)

    def test_two_on_plane(self):
        tri = ((0, 0, 5), (10, 0, 5), (5, 5, 10))
        segs = bs.slice_mesh([tri], 5)
        self.assertGreater(len(segs), 0)

    @unittest.skipUnless(os.path.exists(STL_PATH), "test_cube.stl missing")
    def test_cube_midplane(self):
        tris = bs.parse_stl(STL_PATH)
        zs = [v[2] for tri in tris for v in tri]
        mid_z = (min(zs) + max(zs)) / 2
        segs = bs.slice_mesh(tris, mid_z)
        self.assertGreater(len(segs), 0)


# ── Slice indexed ─────────────────────────────────────────────────────

class TestSliceIndexed(unittest.TestCase):
    @unittest.skipUnless(os.path.exists(STL_PATH), "test_cube.stl missing")
    def test_matches_slice_mesh(self):
        tris = bs.parse_stl(STL_PATH)
        zs = [v[2] for tri in tris for v in tri]
        mid_z = (min(zs) + max(zs)) / 2
        z_index = sorted(
            ((min(v1[2], v2[2], v3[2]), max(v1[2], v2[2], v3[2]), v1, v2, v3)
             for v1, v2, v3 in tris),
            key=lambda t: t[0]
        )
        segs_old = bs.slice_mesh(tris, mid_z)
        segs_new = bs._slice_indexed(z_index, mid_z)
        self.assertEqual(len(segs_old), len(segs_new))


# ── Chain segments ────────────────────────────────────────────────────

class TestChainSegments(unittest.TestCase):
    def test_square(self):
        segs = [
            ((0, 0), (1, 0)),
            ((1, 0), (1, 1)),
            ((1, 1), (0, 1)),
            ((0, 1), (0, 0)),
        ]
        contours = bs.chain_segments(segs)
        self.assertEqual(len(contours), 1)
        self.assertGreaterEqual(len(contours[0]), 4)

    def test_reversed_segs(self):
        segs = [
            ((0, 0), (1, 0)),
            ((1, 1), (1, 0)),  # reversed
            ((1, 1), (0, 1)),
            ((0, 0), (0, 1)),  # reversed
        ]
        contours = bs.chain_segments(segs)
        self.assertEqual(len(contours), 1)

    def test_two_separate(self):
        segs = [
            ((0, 0), (1, 0)),
            ((1, 0), (1, 1)),
            ((1, 1), (0, 0)),
            # second contour far away
            ((100, 100), (101, 100)),
            ((101, 100), (101, 101)),
            ((101, 101), (100, 100)),
        ]
        contours = bs.chain_segments(segs)
        self.assertEqual(len(contours), 2)

    def test_tolerance(self):
        segs = [
            ((0, 0), (1, 0)),
            ((1.005, 0.005), (1, 1)),  # within 0.01 tolerance
            ((1, 1), (0, 0)),
        ]
        contours = bs.chain_segments(segs)
        self.assertEqual(len(contours), 1)

    def test_empty(self):
        self.assertEqual(bs.chain_segments([]), [])


# ── Clean contour ─────────────────────────────────────────────────────

class TestCleanContour(unittest.TestCase):
    def test_no_jumps(self):
        pts = _make_circle(0, 0, 10, 32)
        cleaned = bs._clean_contour(pts)
        self.assertEqual(len(cleaned), len(pts))

    def test_single_jump_splits(self):
        pts = _make_circle(0, 0, 10, 32)
        # Insert a huge jump
        pts[16] = (100, 100)
        cleaned = bs._clean_contour(pts)
        self.assertLess(len(cleaned), len(pts))

    def test_short_passthrough(self):
        pts = [(0, 0), (1, 0), (2, 0)]
        result = bs._clean_contour(pts)
        self.assertEqual(len(result), 3)


# ── Contour area (new) ───────────────────────────────────────────────

class TestContourArea(unittest.TestCase):
    def test_unit_square(self):
        pts = [(0, 0), (1, 0), (1, 1), (0, 1)]
        self.assertAlmostEqual(bs._contour_area(pts), 1.0)

    def test_cw_winding(self):
        pts = [(0, 0), (0, 1), (1, 1), (1, 0)]  # CW
        self.assertAlmostEqual(bs._contour_area(pts), 1.0)

    def test_triangle(self):
        pts = [(0, 0), (4, 0), (0, 3)]
        self.assertAlmostEqual(bs._contour_area(pts), 6.0)

    def test_degenerate_line(self):
        pts = [(0, 0), (1, 0)]
        self.assertAlmostEqual(bs._contour_area(pts), 0.0)

    def test_circle_approx(self):
        r = 10
        pts = _make_circle(0, 0, r, 1000)
        expected = math.pi * r * r
        self.assertAlmostEqual(bs._contour_area(pts), expected, delta=0.5)

    def test_empty(self):
        self.assertAlmostEqual(bs._contour_area([]), 0.0)


# ── Pick best contour ────────────────────────────────────────────────

class TestPickBestContour(unittest.TestCase):
    def test_single(self):
        c = _make_circle(0, 0, 10, 32)
        best = bs._pick_best_contour([c])
        self.assertGreater(len(best), 0)

    def test_empty(self):
        self.assertEqual(bs._pick_best_contour([]), [])

    def test_prefer_closed(self):
        closed = _make_circle(0, 0, 10, 32)
        closed.append(closed[0])  # close it
        open_c = [(i, 0) for i in range(100)]  # long open line
        best = bs._pick_best_contour([open_c, closed])
        # Should pick the closed one
        self.assertTrue(bs._is_closed(best))

    def test_largest_area_wins(self):
        small = _make_circle(0, 0, 5, 32)
        small.append(small[0])
        big = _make_circle(0, 0, 20, 32)
        big.append(big[0])
        best = bs._pick_best_contour([small, big])
        area = bs._contour_area(best)
        self.assertGreater(area, bs._contour_area(small))


# ── Pick best contour — regression / bug fix ─────────────────────────

class TestPickBestContourBugFix(unittest.TestCase):
    """Regression tests: area-based selection beats perimeter-based."""

    def test_serpentine_inner_vs_simple_outer(self):
        """Zigzag contour has longer perimeter but smaller area than simple circle."""
        outer = _make_circle(0, 0, 20, 64)
        outer.append(outer[0])
        inner = _make_serpentine(0, 0, 15)
        inner.append(inner[0])

        # Verify precondition: serpentine has longer perimeter
        outer_len = bs._contour_length(outer, closed=True)
        inner_len = bs._contour_length(inner, closed=True)
        self.assertGreater(inner_len, outer_len,
                           "Test setup: serpentine should have longer perimeter")

        # Verify precondition: outer has larger area
        outer_area = bs._contour_area(outer)
        inner_area = bs._contour_area(inner)
        self.assertGreater(outer_area, inner_area,
                           "Test setup: outer should have larger area")

        best = bs._pick_best_contour([inner, outer])
        best_area = bs._contour_area(best)
        self.assertAlmostEqual(best_area, outer_area, delta=1.0)

    def test_concentric_circles(self):
        """Must pick outermost (largest area) among concentric circles."""
        circles = []
        for r in (5, 10, 15, 20, 25):
            c = _make_circle(0, 0, r, 64)
            c.append(c[0])
            circles.append(c)

        best = bs._pick_best_contour(circles)
        best_area = bs._contour_area(best)
        max_area = max(bs._contour_area(c) for c in circles)
        self.assertAlmostEqual(best_area, max_area, delta=1.0)

    def test_eye_socket_scenario(self):
        """Outer oval + two small inner loops (skull cross-section)."""
        outer = _make_circle(0, 0, 30, 80)
        outer.append(outer[0])
        eye_l = _make_circle(-10, 5, 4, 20)
        eye_l.append(eye_l[0])
        eye_r = _make_circle(10, 5, 4, 20)
        eye_r.append(eye_r[0])

        best = bs._pick_best_contour([eye_l, outer, eye_r])
        best_area = bs._contour_area(best)
        outer_area = bs._contour_area(outer)
        self.assertAlmostEqual(best_area, outer_area, delta=1.0)


# ── Align contour ─────────────────────────────────────────────────────

class TestAlignContour(unittest.TestCase):
    def test_ccw_maintained(self):
        pts = _make_circle(0, 0, 10, 16)
        aligned = bs._align_contour(pts)
        # Signed area should be positive (CCW)
        area = 0.0
        n = len(aligned)
        for i in range(n):
            j = (i + 1) % n
            area += aligned[i][0] * aligned[j][1] - aligned[j][0] * aligned[i][1]
        self.assertGreater(area, 0)

    def test_cw_reversed(self):
        pts = list(reversed(_make_circle(0, 0, 10, 16)))
        aligned = bs._align_contour(pts)
        area = 0.0
        n = len(aligned)
        for i in range(n):
            j = (i + 1) % n
            area += aligned[i][0] * aligned[j][1] - aligned[j][0] * aligned[i][1]
        self.assertGreater(area, 0)

    def test_ref_point_alignment(self):
        pts = _make_circle(0, 0, 10, 32)
        ref = (10, 0)  # rightmost point
        aligned = bs._align_contour(pts, ref_point=ref)
        # First point should be nearest to ref
        d_first = math.hypot(aligned[0][0] - ref[0], aligned[0][1] - ref[1])
        for p in aligned[1:]:
            d = math.hypot(p[0] - ref[0], p[1] - ref[1])
            self.assertGreaterEqual(d, d_first - 0.01)


# ── Contour closure ──────────────────────────────────────────────────

class TestContourClosure(unittest.TestCase):
    def test_closed(self):
        pts = [(0, 0), (1, 0), (1, 1), (0, 0.1)]  # last near first
        self.assertTrue(bs._is_closed(pts, eps=1.0))

    def test_open(self):
        pts = [(0, 0), (1, 0), (1, 1), (0, 5)]
        self.assertFalse(bs._is_closed(pts, eps=1.0))

    def test_exact_closure(self):
        pts = [(0, 0), (1, 0), (1, 1), (0, 0)]
        self.assertTrue(bs._is_closed(pts))


# ── Contour length ───────────────────────────────────────────────────

class TestContourLength(unittest.TestCase):
    def test_unit_square_closed(self):
        pts = [(0, 0), (1, 0), (1, 1), (0, 1)]
        self.assertAlmostEqual(bs._contour_length(pts, closed=True), 4.0)

    def test_unit_square_open(self):
        pts = [(0, 0), (1, 0), (1, 1), (0, 1)]
        self.assertAlmostEqual(bs._contour_length(pts, closed=False), 3.0)

    def test_single_point(self):
        self.assertAlmostEqual(bs._contour_length([(0, 0)]), 0.0)

    def test_empty(self):
        self.assertAlmostEqual(bs._contour_length([]), 0.0)


# ── Point on contour ─────────────────────────────────────────────────

class TestPointOnContour(unittest.TestCase):
    def setUp(self):
        bs.clear_caches()

    def test_t_zero(self):
        pts = [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)]
        p = bs._point_on_contour(pts, 0.0)
        self.assertAlmostEqual(p[0], 0.0, places=1)
        self.assertAlmostEqual(p[1], 0.0, places=1)

    def test_t_quarter(self):
        # Closed square: t=0.25 should be ~1/4 around perimeter
        pts = [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)]
        p = bs._point_on_contour(pts, 0.25)
        self.assertIsInstance(p, tuple)
        self.assertEqual(len(p), 2)

    def test_empty(self):
        p = bs._point_on_contour([], 0.5)
        self.assertEqual(p, (0.0, 0.0))

    def test_single_point(self):
        p = bs._point_on_contour([(5, 5)], 0.5)
        self.assertEqual(p, (5, 5))

    def test_wrap_around(self):
        pts = [(0, 0), (10, 0), (10, 10), (0, 10), (0, 0)]
        p1 = bs._point_on_contour(pts, 0.0)
        p2 = bs._point_on_contour(pts, 1.0)
        self.assertAlmostEqual(p1[0], p2[0], places=1)
        self.assertAlmostEqual(p1[1], p2[1], places=1)


# ── Cylinder ──────────────────────────────────────────────────────────

class TestCylinder(unittest.TestCase):
    def test_surface_point(self):
        c = bs.Cylinder(10, 50)
        x, y = c.surface_point(0, 0)
        self.assertAlmostEqual(math.hypot(x, y), 10.0)

    def test_max_height(self):
        c = bs.Cylinder(10, 50)
        self.assertEqual(c.max_height(), 50)

    def test_max_inset(self):
        c = bs.Cylinder(10, 50)
        self.assertEqual(c.max_inset(), 10)

    def test_inset_reduces_radius(self):
        c = bs.Cylinder(10, 50)
        x, y = c.inset_point(0, 0, 3)
        self.assertAlmostEqual(math.hypot(x, y), 7.0)


# ── Extruded polygon ─────────────────────────────────────────────────

class TestExtrudedPolygon(unittest.TestCase):
    def test_corners(self):
        sq = bs.ExtrudedPolygon(4, 10, 50)
        self.assertEqual(len(sq.verts), 4)

    def test_max_height(self):
        sq = bs.ExtrudedPolygon(4, 10, 50)
        self.assertEqual(sq.max_height(), 50)

    def test_perimeter(self):
        sq = bs.ExtrudedPolygon(4, 10, 50)
        expected = 4 * 2 * 10 * math.sin(math.pi / 4)
        self.assertAlmostEqual(sq.perim, expected, places=3)

    def test_z_independence(self):
        sq = bs.ExtrudedPolygon(4, 10, 50)
        p1 = sq.surface_point(0.25, 0)
        p2 = sq.surface_point(0.25, 25)
        self.assertAlmostEqual(p1[0], p2[0])
        self.assertAlmostEqual(p1[1], p2[1])


# ── STL shape ─────────────────────────────────────────────────────────

@unittest.skipUnless(os.path.exists(STL_PATH), "test_cube.stl missing")
class TestSTLShape(unittest.TestCase):
    def setUp(self):
        bs.clear_caches()
        self.shape = bs.STLShape(STL_PATH)

    def test_dimensions(self):
        self.assertGreater(self.shape.max_height(), 0)

    def test_center(self):
        self.assertIsNotNone(self.shape._cx)
        self.assertIsNotNone(self.shape._cy)

    def test_contour_at_midheight(self):
        c = self.shape._contour_at(self.shape.max_height() / 2)
        self.assertGreater(len(c), 0)

    def test_caching(self):
        z = self.shape.max_height() / 2
        c1 = self.shape._contour_at(z)
        c2 = self.shape._contour_at(z)
        self.assertIs(c1, c2)


# ── Parse STL ─────────────────────────────────────────────────────────

@unittest.skipUnless(os.path.exists(STL_PATH), "test_cube.stl missing")
class TestParseSTL(unittest.TestCase):
    def test_binary_cube(self):
        tris = bs.parse_stl(STL_PATH)
        self.assertEqual(len(tris), 12)

    def test_vertex_ranges(self):
        tris = bs.parse_stl(STL_PATH)
        xs = [v[0] for tri in tris for v in tri]
        ys = [v[1] for tri in tris for v in tri]
        zs = [v[2] for tri in tris for v in tri]
        self.assertGreaterEqual(min(xs), -0.1)
        self.assertLessEqual(max(xs), 50.1)
        self.assertGreaterEqual(min(ys), -0.1)
        self.assertLessEqual(max(ys), 50.1)
        self.assertGreaterEqual(min(zs), -0.1)
        self.assertLessEqual(max(zs), 50.1)


# ── Subdivide mesh ───────────────────────────────────────────────────

class TestSubdivideMesh(unittest.TestCase):
    def test_small_tri_unchanged(self):
        tri = ((0, 0, 0), (1, 0, 0), (0, 1, 0))
        result = bs._subdivide_mesh([tri], max_edge=5.0)
        self.assertEqual(len(result), 1)

    def test_large_tri_split(self):
        tri = ((0, 0, 0), (100, 0, 0), (50, 100, 0))
        result = bs._subdivide_mesh([tri], max_edge=5.0)
        self.assertGreater(len(result), 1)

    def test_all_edges_within_limit(self):
        tri = ((0, 0, 0), (100, 0, 0), (50, 100, 0))
        result = bs._subdivide_mesh([tri], max_edge=5.0)
        for v1, v2, v3 in result:
            e0 = math.sqrt(sum((a - b) ** 2 for a, b in zip(v1, v2)))
            e1 = math.sqrt(sum((a - b) ** 2 for a, b in zip(v2, v3)))
            e2 = math.sqrt(sum((a - b) ** 2 for a, b in zip(v3, v1)))
            self.assertLessEqual(max(e0, e1, e2), 5.0 + 0.01)


# ── Build shape ──────────────────────────────────────────────────────

class TestBuildShape(unittest.TestCase):
    def _args(self, **kw):
        defaults = dict(
            shape="cylinder", stl=None, stl_scale=1.0, radius=25.0,
            side=50.0, circumradius=25.0, polygon_sides=6, height=100.0,
        )
        defaults.update(kw)

        class A:
            pass

        a = A()
        for k, v in defaults.items():
            setattr(a, k, v)
        return a

    def test_cylinder(self):
        s = bs.build_shape(self._args(shape="cylinder"))
        self.assertIsInstance(s, bs.Cylinder)

    def test_box(self):
        s = bs.build_shape(self._args(shape="box"))
        self.assertIsInstance(s, bs.ExtrudedPolygon)
        self.assertEqual(s.n, 4)

    @unittest.skipUnless(os.path.exists(STL_PATH), "test_cube.stl missing")
    def test_stl_factory(self):
        s = bs.build_shape(self._args(stl=STL_PATH, height=None))
        self.assertIsInstance(s, bs.STLShape)


# ── End-to-end generate ──────────────────────────────────────────────

class TestBouncySlicerGenerate(unittest.TestCase):
    def _args(self, **kw):
        defaults = dict(
            shape="cylinder", stl=None, stl_scale=1.0, radius=25.0,
            side=50.0, circumradius=25.0, polygon_sides=6, height=50.0,
            num_arches=6, arch_layer_height=8.0, segments_per_arc=5,
            nozzle_diameter=0.4, filament_diameter=1.75, bead_height=0.3,
            print_speed=1800, first_layer_speed=900, travel_speed=7200,
            retract_distance=0.8, retract_speed=2400, z_hop=2.0,
            bed_temp=60, hotend_temp=210, bed_x=250, bed_y=210,
            bottom_layers=1, bottom_layer_height=0.2,
            top_layers=0, top_layer_height=0.3,
            output="test_output.gcode",
        )
        defaults.update(kw)

        class A:
            pass

        a = A()
        for k, v in defaults.items():
            setattr(a, k, v)
        return a

    def test_cylinder_gcode(self):
        args = self._args()
        shape = bs.build_shape(args)
        slicer = bs.BouncySlicer(shape, args)
        gcode = slicer.generate()
        self.assertIn("G90", gcode)
        self.assertIn("E", gcode)
        self.assertGreater(slicer.e_total, 0)

    def test_box_gcode(self):
        args = self._args(shape="box")
        shape = bs.build_shape(args)
        slicer = bs.BouncySlicer(shape, args)
        gcode = slicer.generate()
        self.assertIn("G90", gcode)
        self.assertGreater(slicer.e_total, 0)

    @unittest.skipUnless(os.path.exists(STL_PATH), "test_cube.stl missing")
    def test_stl_gcode(self):
        args = self._args(stl=STL_PATH, height=None)
        shape = bs.build_shape(args)
        slicer = bs.BouncySlicer(shape, args)
        gcode = slicer.generate()
        self.assertIn("G90", gcode)
        self.assertGreater(slicer.e_total, 0)

    def test_top_layers_zero(self):
        """No top layers by default — should not emit top section."""
        args = self._args(top_layers=0)
        shape = bs.build_shape(args)
        slicer = bs.BouncySlicer(shape, args)
        gcode = slicer.generate()
        self.assertNotIn("Top perimeter layers", gcode)

    def test_top_layers_present(self):
        """With top_layers=2, gcode should contain the top perimeter section."""
        args = self._args(top_layers=2, top_layer_height=0.3)
        shape = bs.build_shape(args)
        slicer = bs.BouncySlicer(shape, args)
        gcode = slicer.generate()
        self.assertIn("Top perimeter layers", gcode)
        self.assertGreater(slicer.e_total, 0)

    def test_top_layers_more_extrusion(self):
        """Top layers should add extrusion vs no top layers."""
        args_no = self._args(top_layers=0)
        shape_no = bs.build_shape(args_no)
        slicer_no = bs.BouncySlicer(shape_no, args_no)
        slicer_no.generate()

        args_yes = self._args(top_layers=3, top_layer_height=0.3)
        shape_yes = bs.build_shape(args_yes)
        slicer_yes = bs.BouncySlicer(shape_yes, args_yes)
        slicer_yes.generate()

        self.assertGreater(slicer_yes.e_total, slicer_no.e_total)


# ── Slow tests (gated) ──────────────────────────────────────────────

@unittest.skipUnless(SLOW, "Set BOUNCY_SLOW_TESTS=1 to run")
class TestSkullVaseRegression(unittest.TestCase):
    SKULL_PATH = os.path.join(os.path.dirname(__file__), "Skull%20Vase.stl")

    @unittest.skipUnless(
        os.path.exists(os.path.join(os.path.dirname(__file__), "Skull%20Vase.stl")),
        "Skull Vase STL missing"
    )
    def test_skull_vase_generates(self):
        """Skull vase should generate without error using problem params."""

        class A:
            pass

        args = A()
        for k, v in dict(
            shape="cylinder", stl=self.SKULL_PATH, stl_scale=0.25,
            radius=25.0, side=50.0, circumradius=25.0, polygon_sides=6,
            height=None, num_arches=60, arch_layer_height=1.5,
            segments_per_arc=7, nozzle_diameter=0.4, filament_diameter=1.75,
            bead_height=0.3, print_speed=1800, first_layer_speed=900,
            travel_speed=7200, retract_distance=0.8, retract_speed=2400,
            z_hop=2.0, bed_temp=60, hotend_temp=210, bed_x=250, bed_y=210,
            bottom_layers=3, bottom_layer_height=0.2,
            output="skull_test.gcode",
        ).items():
            setattr(args, k, v)

        shape = bs.build_shape(args)
        slicer = bs.BouncySlicer(shape, args)
        gcode = slicer.generate()
        self.assertIn("G90", gcode)
        self.assertGreater(slicer.e_total, 0)


if __name__ == "__main__":
    unittest.main()
