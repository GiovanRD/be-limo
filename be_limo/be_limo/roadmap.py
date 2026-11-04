"""Road map of the QCar SDCS large track, in the Gazebo world frame.

The track is the Quanser "Self-Driving Car Studio" mat used in room 207.
Its road network is described as a directed graph:

* 24 nodes, each with a pose (x, y, heading), placed on the lane centre
  line at the entry/exit of the intersections (right-hand traffic);
* edges between nodes, each drawn as Straight-Curve-Straight (SCS): the
  lane leaves the first node in a straight line, turns on a circular arc of
  a given radius, then reaches the second node in a straight line.

The node positions were measured on the Quanser road-map figure
(``SDCS_RoadMap_RightHandTraffic.png`` in the ``qcar_large_track`` package)
and converted into the Gazebo world frame, where the 4.88 m x 6.20 m mat is
centred on the origin. On the physical robot, the same frame is used once
the robot's odometry is initialised at a known node (see README).

This module has no ROS dependency, so it can be tested on any computer.
"""

import heapq
import math

# ---------------------------------------------------------------------------
# Frame conversion: Quanser map frame (figure) -> Gazebo world frame
# ---------------------------------------------------------------------------

# Centre of the mat in the Quanser frame, and scale between both frames
# (measured on the figure axes: 4.833 m x 6.159 m drawn for a 4.88 x 6.20 mat).
_QUANSER_CENTRE = (0.1345, 1.693)
_QUANSER_SCALE = (4.88 / 4.833, 6.20 / 6.159)


def quanser_to_world(x, y):
    """Convert a point from the Quanser map frame to the Gazebo world frame."""
    return ((x - _QUANSER_CENTRE[0]) * _QUANSER_SCALE[0],
            (y - _QUANSER_CENTRE[1]) * _QUANSER_SCALE[1])


# ---------------------------------------------------------------------------
# Nodes: (x, y) in the Quanser frame [m], heading [deg]
# ---------------------------------------------------------------------------

_E, _N, _W, _S = 0.0, 90.0, 180.0, -90.0

_QUANSER_NODES = [
    (-0.002, 0.130, _S),    # 0
    (0.266, 0.084, _N),     # 1
    (1.122, -1.076, _E),    # 2
    (1.119, -0.807, _W),    # 3
    (2.235, 0.084, _N),     # 4
    (1.967, 0.081, _S),     # 5
    (1.009, 1.097, _W),     # 6
    (1.224, 0.828, _E),     # 7
    (-0.740, 1.097, _W),    # 8
    (-0.738, 0.828, _E),    # 9
    (-1.269, -0.455, -45.0),  # 10
    (-0.002, 2.144, _S),    # 11
    (-0.002, 1.834, _S),    # 12
    (0.266, 1.836, _N),     # 13
    (2.235, 2.946, _N),     # 14
    (1.967, 1.833, _S),     # 15
    (0.900, 3.681, _S),     # 16
    (1.459, 3.131, _E),     # 17
    (0.618, 3.048, -135.0),  # 18
    (0.783, 2.841, 45.0),   # 19
    (-0.003, 4.469, _W),    # 20
    (-0.001, 4.200, _E),    # 21
    (-1.964, 2.943, _S),    # 22
    (-1.703, 2.946, _N),    # 23
]

# Lane radii of the Quanser road map [m]
INNER = 0.622        # inner lane of a corner
OUTER = 0.891        # outer lane of a corner
CIRCLE = 0.678       # traffic circle (roundabout)
ONE_WAY = 0.712      # one-way street
KINK = 0.763         # kinked street
STRAIGHT = 0.0

# Directed edges: (from, to, radius[, via poses in the Quanser frame]).
# A via pose splits an edge that turns by more than 180 degrees, such as the
# full loop of the roundabout from node 17 back to node 16.
EDGES = [
    (0, 2, OUTER), (1, 7, INNER), (1, 8, OUTER), (2, 4, OUTER),
    (3, 1, INNER), (4, 6, OUTER), (5, 3, INNER), (6, 0, OUTER),
    (6, 8, STRAIGHT), (7, 5, INNER), (8, 10, ONE_WAY), (9, 0, INNER),
    (9, 7, STRAIGHT), (10, 1, INNER), (10, 2, INNER), (1, 13, STRAIGHT),
    (4, 14, STRAIGHT), (6, 13, INNER), (7, 14, OUTER), (8, 23, INNER),
    (9, 13, OUTER), (11, 12, STRAIGHT), (12, 0, STRAIGHT), (12, 7, OUTER),
    (12, 8, INNER), (13, 19, INNER), (14, 16, CIRCLE), (14, 20, CIRCLE),
    (15, 5, OUTER), (15, 6, INNER), (16, 17, CIRCLE), (16, 18, INNER),
    (17, 15, INNER), (17, 16, CIRCLE, [(2.235, 3.681, _N)]),
    (18, 11, KINK), (19, 17, INNER),
    (20, 22, OUTER), (21, 16, INNER), (22, 9, OUTER), (22, 10, OUTER),
    (23, 21, INNER),
]

# Spacing between two consecutive points of a generated path [m]
STEP = 0.02


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def node_poses():
    """Return the list of node poses (x, y, yaw) in the world frame."""
    poses = []
    for x, y, heading in _QUANSER_NODES:
        wx, wy = quanser_to_world(x, y)
        poses.append((wx, wy, math.radians(heading)))
    return poses


# ---------------------------------------------------------------------------
# Edge geometry
# ---------------------------------------------------------------------------

def _line(p, q, step=STEP):
    """Points from p to q (p included, q excluded)."""
    length = math.hypot(q[0] - p[0], q[1] - p[1])
    n = max(1, int(length / step))
    return [(p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n)
            for i in range(n)]


def _arc(centre, radius, a0, sweep, step=STEP):
    """Points on a circle from angle a0, over a signed sweep (end excluded)."""
    n = max(1, int(abs(sweep) * radius / step))
    return [(centre[0] + radius * math.cos(a0 + sweep * i / n),
             centre[1] + radius * math.sin(a0 + sweep * i / n))
            for i in range(n)]


def scs_path(start, end, radius, step=STEP):
    """Straight-Curve-Straight path between two poses.

    start, end: (x, y, yaw). Returns a list of (x, y) points, end included.
    """
    x0, y0, h0 = start
    x1, y1, h1 = end
    d0 = (math.cos(h0), math.sin(h0))
    d1 = (math.cos(h1), math.sin(h1))
    turn = _wrap(h1 - h0)

    if radius <= 0.0 or abs(turn) < 1e-3:
        return _line((x0, y0), (x1, y1), step) + [(x1, y1)]

    u_turn = abs(abs(turn) - math.pi) < 1e-3
    if u_turn:
        # Turn towards the side where the end pose lies.
        turn = math.pi if d0[0] * (y1 - y0) - d0[1] * (x1 - x0) > 0 else -math.pi
    side = 1.0 if turn > 0 else -1.0          # +1: left turn, -1: right turn
    n0 = (-d0[1] * side, d0[0] * side)          # from the lane towards the centre

    if u_turn:
        # U-turn (around the roundabout): straight, half circle, straight.
        lateral = (x1 - x0) * n0[0] + (y1 - y0) * n0[1]
        r = abs(lateral) / 2.0
        along = (x1 - x0) * d0[0] + (y1 - y0) * d0[1]
        s0 = max(0.0, along)
        p = (x0 + s0 * d0[0], y0 + s0 * d0[1])
        centre = (p[0] + r * n0[0], p[1] + r * n0[1])
        a0 = math.atan2(p[1] - centre[1], p[0] - centre[0])
        pts = _line((x0, y0), p, step) if s0 > 0 else []
        pts += _arc(centre, r, a0, side * math.pi, step)
        q = (centre[0] - (p[0] - centre[0]), centre[1] - (p[1] - centre[1]))
        pts += _line(q, (x1, y1), step) + [(x1, y1)]
        return pts

    # Intersection of the two lane lines.
    den = d0[0] * d1[1] - d0[1] * d1[0]
    t = ((x1 - x0) * d1[1] - (y1 - y0) * d1[0]) / den
    ix, iy = x0 + t * d0[0], y0 + t * d0[1]

    # Tangent points of the arc, then shrink the radius if they do not fit.
    dist0 = (ix - x0) * d0[0] + (iy - y0) * d0[1]
    dist1 = (x1 - ix) * d1[0] + (y1 - iy) * d1[1]
    tan_half = math.tan(abs(turn) / 2.0)
    r = min(radius, max(dist0, 0.0) / tan_half, max(dist1, 0.0) / tan_half)
    tangent = r * tan_half
    t0 = (ix - tangent * d0[0], iy - tangent * d0[1])
    t1 = (ix + tangent * d1[0], iy + tangent * d1[1])
    centre = (t0[0] + r * n0[0], t0[1] + r * n0[1])
    a0 = math.atan2(t0[1] - centre[1], t0[0] - centre[0])

    pts = _line((x0, y0), t0, step)
    pts += _arc(centre, r, a0, turn, step)
    pts += _line(t1, (x1, y1), step) + [(x1, y1)]
    return pts


def _length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:]))


# ---------------------------------------------------------------------------
# Road map
# ---------------------------------------------------------------------------

class RoadMap:
    """Directed road graph with A* shortest paths."""

    def __init__(self, nodes=None, edges=None, step=STEP):
        self.nodes = nodes if nodes is not None else node_poses()
        self.edges = {}
        self.successors = {i: [] for i in range(len(self.nodes))}
        for edge in (edges if edges is not None else EDGES):
            a, b, radius = edge[:3]
            via = [(*quanser_to_world(x, y), math.radians(h))
                   for x, y, h in (edge[3] if len(edge) > 3 else [])]
            poses = [self.nodes[a]] + via + [self.nodes[b]]
            pts = []
            for p, q in zip(poses, poses[1:]):
                part = scs_path(p, q, radius, step)
                pts += part if not pts else part[1:]
            self.edges[(a, b)] = (pts, _length(pts))
            self.successors[a].append(b)

    def nearest_node(self, x, y, yaw=None, max_heading_error=math.pi / 2):
        """Closest node, optionally among those heading like the robot."""
        best, best_d = None, float('inf')
        for i, (nx, ny, nyaw) in enumerate(self.nodes):
            if yaw is not None and abs(_wrap(nyaw - yaw)) > max_heading_error:
                continue
            d = math.hypot(nx - x, ny - y)
            if d < best_d:
                best, best_d = i, d
        return best

    def shortest_path(self, start, goal):
        """Node sequence of the shortest route (A*), or None."""
        if start == goal:
            return [start]
        gx, gy, _ = self.nodes[goal]
        h = lambda n: math.hypot(self.nodes[n][0] - gx, self.nodes[n][1] - gy)
        open_set = [(h(start), 0.0, start)]
        came_from = {start: None}
        cost = {start: 0.0}
        while open_set:
            _, g, node = heapq.heappop(open_set)
            if node == goal:
                route = [node]
                while came_from[route[-1]] is not None:
                    route.append(came_from[route[-1]])
                return route[::-1]
            if g > cost[node]:
                continue
            for nxt in self.successors[node]:
                g2 = g + self.edges[(node, nxt)][1]
                if g2 < cost.get(nxt, float('inf')):
                    cost[nxt] = g2
                    came_from[nxt] = node
                    heapq.heappush(open_set, (g2 + h(nxt), g2, nxt))
        return None

    def route_points(self, route):
        """Concatenate the geometry of a node sequence into one polyline."""
        pts = []
        for a, b in zip(route, route[1:]):
            edge = self.edges[(a, b)][0]
            pts += edge if not pts else edge[1:]
        return pts

    def plan(self, waypoints):
        """Plan through a list of nodes (start, via..., goal).

        Returns (node_route, points) or (None, []) if a leg is impossible.
        """
        route = [waypoints[0]]
        for a, b in zip(waypoints, waypoints[1:]):
            leg = self.shortest_path(a, b)
            if leg is None:
                return None, []
            route += leg[1:]
        return route, self.route_points(route)
