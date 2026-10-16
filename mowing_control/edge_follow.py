#!/usr/bin/env python3
"""
Edge following for automatic lawn mapping (no ROS dependencies, so it can be simulated on a PC).

Input: points in base_link (x forward, y left, metres) of everything that is not mowable lawn:
the AI lawn edge (/vision/boundary_points), AI obstacles and the stereo scan.

Behaviour (lawn edge kept on the RIGHT, where the Worx cutter sits):
  SEEK         drive straight until the edge or an obstacle is right in front
  TURN_LEFT    something in front: turn on the spot until the way ahead is clear
  FOLLOW       keep the edge at `target` metres to the right of the mower's centre
  SEARCH_RIGHT the edge disappeared on the right (outer corner). The camera looks ahead, so the
               corner is still in front (where the edge was last seen): drive past it first,
               then arc around it with radius = target so the mower's body stays clear of the edge

The loop is closed when the mower returns to where it started following the edge.
"""
import math

import numpy as np


class EdgeParams:
    def __init__(self, **kw):
        self.target = 0.28        # metres from the mower's centre to the lawn edge (right side)
        self.half_width = 0.20    # half the mower's width
        self.speed = 0.15         # m/s while following
        self.search_speed = 0.10  # m/s while driving around an outer corner
        self.turn_rate = 0.40     # rad/s turning on the spot at corners/obstacles
        self.k = 2.0              # steering gain on the distance error (rad/s per metre)
        self.k_angle = 1.5        # steering gain on the edge's angle relative to the mower (rad/s per rad)
        self.bin = 0.10           # Edge profile: closest edge point per 10 cm ahead
        self.max_w = 0.6
        self.front_min = 0.30     # Strip in front checked for "blocked" (inner corner, obstacle)
        self.front_max = 0.65
        self.front_half = 0.12    # Narrower than target: an edge met at a small angle is steered away from
        self.look_min = 0.30      # Region on the right used to measure the edge distance
        self.look_max = 1.20
        self.search_width = 1.20
        self.min_points = 5
        self.cell = 0.10          # Noise filter: points need several neighbours within ~10 cm
        self.min_cell_points = 4
        self.__dict__.update(kw)


def denoise(pts, cell, min_count):
    """Drop isolated points (single misclassified pixels); a real edge gives dense clusters.
    Counts points in each cell plus its 8 neighbours, so points on the border of the
    non-grass area are kept (counting the cell alone would shift the edge by a cell)."""
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / cell).astype(np.int64)
    uniq, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    lookup = {(int(a), int(b)): int(c) for (a, b), c in zip(uniq, counts)}
    neighbourhood = np.array([
        sum(lookup.get((a + da, b + db), 0) for da in (-1, 0, 1) for db in (-1, 0, 1))
        for a, b in uniq])
    return pts[neighbourhood[inverse.ravel()] >= min_count]


def edge_command(points, p, edge_found):
    """
    One control step. points: (N, 2) array in base_link.
    Returns (linear m/s, angular rad/s, state, distance to the edge or None, furthest x where
    the edge is visible or None).
    """
    pts = denoise(np.asarray(points, dtype=float).reshape(-1, 2), p.cell, p.min_cell_points)
    x, y = pts[:, 0], pts[:, 1]

    front = (x >= p.front_min) & (x <= p.front_max) & (np.abs(y) <= p.front_half)
    if np.count_nonzero(front) >= p.min_points:
        return 0.0, p.turn_rate, 'TURN_LEFT', None, None

    if not edge_found:
        # Only start following an edge the mower has actually driven up to
        return p.speed, 0.0, 'SEEK', None, None

    right = (x >= p.look_min) & (x <= p.look_max) & (y <= 0.0) & (y >= -p.search_width)
    if np.count_nonzero(right) < p.min_points:
        return p.search_speed, -p.search_speed / p.target, 'SEARCH_RIGHT', None, None

    # Edge profile: for every 10 cm ahead, the closest non-grass point on the right.
    # A line through it gives the perpendicular distance to the edge and the edge's angle,
    # so the mower also straightens up when it meets the edge at an angle (after corners).
    xr, yr = x[right], y[right]
    bins = np.floor((xr - p.look_min) / p.bin).astype(int)
    profile = [(xr[bins == b].mean(), yr[bins == b].max()) for b in np.unique(bins)]
    if len(profile) >= 3:
        px, py = np.array(profile).T
        slope, offset = np.polyfit(px, py, 1)
        distance = -offset / math.hypot(1.0, slope)
        angle = math.atan(slope)   # > 0: the edge comes closer ahead
    else:
        distance = -np.percentile(yr, 90)
        angle = 0.0
    error = distance - p.target
    w = float(np.clip(-p.k * error + p.k_angle * angle, -p.max_w, p.max_w))
    v = p.speed * (1.0 - 0.5 * min(abs(error) / 0.3, 1.0))
    return v, w, 'FOLLOW', distance, float(xr.max())


class EdgeMapper:
    """Edge following + recording of the driven path + loop closure detection."""

    def __init__(self, params=None, record_step=0.25, min_loop_length=8.0, close_radius=0.6,
                 min_excursion=2.0, settle_steps=20):
        self.p = params or EdgeParams()
        self.record_step = record_step
        self.min_loop_length = min_loop_length
        self.close_radius = close_radius
        self.min_excursion = min_excursion  # Must have been this far from the start before closing
        self.max_excursion = 0.0
        self.search_turned = 0.0   # Rotation while searching for a lost edge
        self.last_yaw = None
        self.settle_steps = settle_steps  # Steady following needed before the loop start is set
        self.stable = 0
        self.lost_distance = 0.0   # Distance driven since the edge disappeared
        self.edge_ahead = 0.0      # How far ahead the edge was last seen (where the corner is)
        self.last_pose = None
        self.edge_found = False
        self.start = None          # (x, y, yaw) where edge following started
        self.path = []             # recorded [x, y] in the map frame
        self.length = 0.0
        self.state = 'SEEK'

    def step(self, points, pose):
        """points in base_link, pose (x, y, yaw) in the map frame. Returns (v, w, closed)."""
        v, w, self.state, distance, edge_x = edge_command(points, self.p, self.edge_found)
        if edge_x is not None:
            self.edge_ahead = edge_x
        if self.state != 'SEEK' and not self.edge_found:
            self.edge_found = True

        # Outer corner: first drive past the corner the camera saw disappear, then arc around it
        moved = math.dist(self.last_pose[:2], pose[:2]) if self.last_pose else 0.0
        self.last_pose = pose
        if self.state == 'SEARCH_RIGHT':
            self.lost_distance += moved
            if self.lost_distance < self.edge_ahead + 0.05:
                w = 0.0
        else:
            self.lost_distance = 0.0

        # Searching for a lost edge for more than a full turn: it was not an outer corner but a
        # false edge (e.g. a misclassified patch) - drive straight again until a real edge is in front
        if self.state == 'SEARCH_RIGHT' and self.last_yaw is not None:
            self.search_turned += abs(math.atan2(math.sin(pose[2] - self.last_yaw), math.cos(pose[2] - self.last_yaw)))
            if self.search_turned > math.radians(330):
                self.edge_found = False
                self.search_turned = 0.0
        elif self.state != 'SEARCH_RIGHT':
            self.search_turned = 0.0
        self.last_yaw = pose[2]

        # The loop starts once the mower follows the edge steadily, so it passes the start point
        # again with the same heading
        steady = self.state == 'FOLLOW' and abs(distance - self.p.target) < 0.10
        self.stable = self.stable + 1 if steady else 0
        if self.start is None and self.stable >= self.settle_steps:
            self.start = pose
            self.path = [[pose[0], pose[1]]]
        if self.start is not None:
            last = self.path[-1]
            d = math.dist(last, pose[:2])
            if d >= self.record_step:
                self.path.append([pose[0], pose[1]])
                self.length += d
            self.max_excursion = max(self.max_excursion, math.dist(pose[:2], self.start[:2]))
            if self.closed(pose):
                return 0.0, 0.0, True
        return v, w, False

    def closed(self, pose):
        if self.length < self.min_loop_length or self.max_excursion < self.min_excursion:
            return False
        near = math.dist(pose[:2], self.start[:2]) < self.close_radius
        heading = abs(math.atan2(math.sin(pose[2] - self.start[2]), math.cos(pose[2] - self.start[2])))
        return near and heading < math.radians(60)

    def boundary(self):
        """Recorded loop as a zone boundary (the mower's centre line along the edge)."""
        return [[round(x, 3), round(y, 3)] for x, y in self.path]
