#!/usr/bin/env python3
"""
Coverage path planning for OmniMow (no ROS dependencies, so it can be tested on a PC).

All coordinates are metres in the local odom frame (x = east, y = north).

A plan for one zone is:
  1. Perimeter passes along the edge of the lawn and around every exclusion zone (flower beds)
  2. Parallel stripes (boustrophedon) over the rest, in the direction of the zone's longest side
Passes are ordered greedily so the mower always drives to the nearest unmowed pass next.
"""
import math

from shapely import affinity
from shapely.geometry import LineString, MultiLineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union


def mowable_area(boundary, exclusions, edge_margin):
    """Area the mower's centre may cover: lawn shrunk by edge_margin, exclusions grown by it."""
    area = Polygon(boundary).buffer(0)  # buffer(0) repairs self-intersecting drawings
    if exclusions:
        holes = unary_union([Polygon(e).buffer(0) for e in exclusions if len(e) >= 3])
        area = area.difference(holes)
    return area.buffer(-edge_margin, join_style=2)


def keepout_area(boundary, exclusions, clearance):
    """
    Region Nav2 must not plan through: everything outside the lawn plus the inner part of each
    exclusion. `clearance` (robot radius + a little) keeps the perimeter passes themselves legal.
    Returns (allowed_polygon, list_of_forbidden_polygons_inside).
    """
    allowed = Polygon(boundary).buffer(0).buffer(clearance, join_style=2)
    forbidden = []
    for e in exclusions or []:
        inner = Polygon(e).buffer(0).buffer(-clearance, join_style=2)
        if not inner.is_empty:
            forbidden.extend(_polygons(inner))
    return allowed, forbidden


def _polygons(geom):
    if geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        return [geom]
    if isinstance(geom, MultiPolygon):
        return list(geom.geoms)
    return [g for g in getattr(geom, 'geoms', []) if isinstance(g, Polygon)]


def _lines(geom):
    if geom.is_empty:
        return []
    if isinstance(geom, LineString):
        return [geom]
    if isinstance(geom, MultiLineString):
        return list(geom.geoms)
    return [g for g in getattr(geom, 'geoms', []) if isinstance(g, LineString)]


def stripe_angle(area):
    """Direction of the longest side of the minimum rotated rectangle (fewest turns)."""
    rect = area.minimum_rotated_rectangle
    coords = list(rect.exterior.coords)
    best, angle = 0.0, 0.0
    for (x1, y1), (x2, y2) in zip(coords, coords[1:]):
        length = math.hypot(x2 - x1, y2 - y1)
        if length > best:
            best, angle = length, math.atan2(y2 - y1, x2 - x1)
    return angle


def densify(points, step=0.10):
    """Insert points so consecutive points are at most `step` apart (smooth path for the controller)."""
    out = [points[0]]
    for (x1, y1), (x2, y2) in zip(points, points[1:]):
        n = max(1, int(math.ceil(math.hypot(x2 - x1, y2 - y1) / step)))
        out.extend((x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n) for i in range(1, n + 1))
    return out


def plan_zone(boundary, exclusions=(), cut_width=0.18, overlap=0.03, edge_margin=0.20,
              perimeter_passes=1, angle=None, min_pass_length=0.3, start=(0.0, 0.0)):
    """
    Returns a list of passes: {"type": "perimeter"|"stripe", "points": [(x, y), ...]}.
    """
    spacing = cut_width - overlap
    if spacing <= 0:
        raise ValueError("overlap must be smaller than cut_width")
    area = mowable_area(boundary, exclusions, edge_margin)
    if area.is_empty:
        return []

    passes = []
    # 1. Perimeter loops: outer edge and around every exclusion, moving inward each loop
    for i in range(perimeter_passes):
        ring_area = area.buffer(-i * spacing, join_style=2)
        for poly in _polygons(ring_area):
            for ring in [poly.exterior, *poly.interiors]:
                pts = list(ring.coords)
                if LineString(pts).length >= min_pass_length:
                    passes.append({"type": "perimeter", "points": pts})

    # 2. Stripes over what is left inside the perimeter loops
    inner = area.buffer(-perimeter_passes * spacing + spacing / 2, join_style=2) if perimeter_passes else area
    stripes = []
    if not inner.is_empty:
        theta = stripe_angle(area) if angle is None else angle
        origin = area.centroid
        rotated = affinity.rotate(inner, -theta, origin=origin, use_radians=True)
        minx, miny, maxx, maxy = rotated.bounds
        y = miny + spacing / 2
        while y < maxy:
            scan = LineString([(minx - 1, y), (maxx + 1, y)])
            for seg in _lines(rotated.intersection(scan)):
                if seg.length >= min_pass_length:
                    back = affinity.rotate(seg, theta, origin=origin, use_radians=True)
                    stripes.append(list(back.coords))
            y += spacing

    return _order_passes(passes, start) + _order_passes(
        [{"type": "stripe", "points": s} for s in stripes],
        passes[-1]["points"][-1] if passes else start)


def _order_passes(passes, start):
    """Greedy nearest-neighbour ordering; stripes may be driven in either direction."""
    remaining = list(passes)
    ordered = []
    pos = start
    while remaining:
        best_i, best_d, best_rev = 0, float('inf'), False
        for i, p in enumerate(remaining):
            pts = p["points"]
            d_start = math.dist(pos, pts[0])
            d_end = math.dist(pos, pts[-1]) if p["type"] == "stripe" else float('inf')
            if d_start < best_d:
                best_i, best_d, best_rev = i, d_start, False
            if d_end < best_d:
                best_i, best_d, best_rev = i, d_end, True
        p = remaining.pop(best_i)
        if best_rev:
            p = {"type": p["type"], "points": list(reversed(p["points"]))}
        ordered.append(p)
        pos = p["points"][-1]
    return ordered


def point_inside(polygon_points, x, y):
    return Polygon(polygon_points).buffer(0).contains(Point(x, y))
