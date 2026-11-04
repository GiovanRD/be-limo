"""Global planner: shortest route on the road map of the track.

Inputs
  /robot_pose     geometry_msgs/PoseStamped  current pose (map frame)
  /goal_pose      geometry_msgs/PoseStamped  goal clicked in RViz ("2D Goal Pose")
  /planner/goal   std_msgs/String            "14" (one node), "14,20,9" (mission)
                                             or "demo" (demonstration loop)
  /controller/state std_msgs/String (JSON)   to restart the loop at the goal

Outputs
  /plan             nav_msgs/Path            path to follow (latched)
  /plan/route       std_msgs/String (JSON)   node sequence and length (latched)
  /roadmap/markers  visualization_msgs/MarkerArray   road map for RViz (latched)
"""

import json
import math

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from be_limo.roadmap import RoadMap
from be_limo.ros_utils import LATCHED, quaternion_from_yaw, run, yaw_from_quaternion

DEMO_MISSION = [14, 20, 22, 9, 0, 2]


class GlobalPlanner(Node):

    def __init__(self):
        super().__init__('global_planner')
        self.declare_parameter('mission', '')          # e.g. "14,20,22,9,0,2" or "demo"
        self.declare_parameter('loop', False)
        self.declare_parameter('heading_tolerance_deg', 60.0)

        self.roadmap = RoadMap()
        self.pose = None
        self.mission = []
        self.loop = self.get_parameter('loop').value
        self.last_goal_reached = None

        self.path_pub = self.create_publisher(Path, '/plan', LATCHED)
        self.route_pub = self.create_publisher(String, '/plan/route', LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, '/roadmap/markers', LATCHED)

        self.create_subscription(PoseStamped, '/robot_pose', self.on_pose, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, 10)
        self.create_subscription(String, '/planner/goal', self.on_goal_text, 10)
        self.create_subscription(String, '/controller/state', self.on_controller, 10)

        self.publish_roadmap()
        text = self.get_parameter('mission').value
        if text:
            self.mission = self.parse(text)
            self.pending = bool(self.mission)
        else:
            self.pending = False
        self.create_timer(0.5, self.try_pending)
        self.get_logger().info('Planificateur prêt (%d noeuds, %d arcs)' % (
            len(self.roadmap.nodes), len(self.roadmap.edges)))

    # ------------------------------------------------------------------ input
    def parse(self, text):
        text = text.strip().lower()
        if text in ('demo', 'boucle', 'loop'):
            self.loop = True
            return list(DEMO_MISSION)
        try:
            nodes = [int(t) for t in text.replace(';', ',').split(',') if t.strip()]
        except ValueError:
            self.get_logger().error('Objectif invalide : "%s"' % text)
            return []
        bad = [n for n in nodes if not 0 <= n < len(self.roadmap.nodes)]
        if bad:
            self.get_logger().error('Noeuds inconnus : %s' % bad)
            return []
        return nodes

    def on_pose(self, msg):
        p = msg.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_goal_pose(self, msg):
        p = msg.pose
        goal = self.roadmap.nearest_node(p.position.x, p.position.y,
                                         yaw_from_quaternion(p.orientation))
        self.get_logger().info('Objectif RViz -> noeud %s' % goal)
        if goal is not None:
            self.loop = False
            self.mission = [goal]
            self.pending = True
            self.try_pending()

    def on_goal_text(self, msg):
        mission = self.parse(msg.data)
        if mission:
            if msg.data.strip().lower() not in ('demo', 'boucle', 'loop'):
                self.loop = False
            self.mission = mission
            self.pending = True
            self.try_pending()

    def on_controller(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        if state.get('state') == 'GOAL_REACHED' and self.loop and self.mission:
            stamp = state.get('plan_id')
            if stamp != self.last_goal_reached:
                self.last_goal_reached = stamp
                self.pending = True

    # --------------------------------------------------------------- planning
    def start_node(self):
        """Closest node ahead of the robot, heading the same way."""
        x, y, yaw = self.pose
        tol = math.radians(self.get_parameter('heading_tolerance_deg').value)
        best, best_d = None, float('inf')
        for i, (nx, ny, nyaw) in enumerate(self.roadmap.nodes):
            if abs(math.atan2(math.sin(nyaw - yaw), math.cos(nyaw - yaw))) > tol:
                continue
            ahead = (nx - x) * math.cos(yaw) + (ny - y) * math.sin(yaw)
            d = math.hypot(nx - x, ny - y)
            if ahead > -0.05 and d < best_d:
                best, best_d = i, d
        return best

    def try_pending(self):
        if not self.pending or self.pose is None:
            return
        self.pending = False
        start = self.start_node()
        if start is None:
            self.get_logger().error('Aucun noeud de la carte devant le robot')
            return
        mission = list(self.mission)
        if mission and mission[0] == start:
            mission = mission[1:]
        waypoints = [start] + mission
        if len(waypoints) == 1:
            self.get_logger().info('Le robot est déjà au noeud %d' % start)
            return
        route, points = self.roadmap.plan(waypoints)
        if route is None:
            self.get_logger().error('Pas de chemin pour %s' % waypoints)
            return
        # Join the robot to the first node of the route.
        x, y, _ = self.pose
        if math.hypot(points[0][0] - x, points[0][1] - y) > 0.05:
            points = [(x, y)] + points
        self.publish_plan(route, points)

    # ------------------------------------------------------------------ output
    def publish_plan(self, route, points):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for i, (x, y) in enumerate(points):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            j = min(i + 1, len(points) - 1)
            k = max(j - 1, 0) if j == i else i
            yaw = math.atan2(points[j][1] - points[k][1], points[j][0] - points[k][0])
            q = quaternion_from_yaw(yaw)
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = q
            path.poses.append(ps)
        self.path_pub.publish(path)
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))
        info = {'route': route, 'length': round(length, 2), 'loop': self.loop,
                'mission': self.mission}
        self.route_pub.publish(String(data=json.dumps(info)))
        self.get_logger().info('Itinéraire %s (%.1f m)' % (' -> '.join(map(str, route)), length))

    def publish_roadmap(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        lines = Marker()
        lines.header.frame_id, lines.header.stamp = 'map', now
        lines.ns, lines.id, lines.type = 'edges', 0, Marker.LINE_LIST
        lines.scale.x = 0.01
        lines.color.r, lines.color.g, lines.color.b, lines.color.a = 0.2, 0.8, 0.2, 0.6
        lines.pose.orientation.w = 1.0
        for pts, _ in self.roadmap.edges.values():
            for a, b in zip(pts[::3], pts[3::3] + [pts[-1]]):
                lines.points.append(Point(x=a[0], y=a[1], z=0.005))
                lines.points.append(Point(x=b[0], y=b[1], z=0.005))
        markers.markers.append(lines)
        for i, (x, y, yaw) in enumerate(self.roadmap.nodes):
            arrow = Marker()
            arrow.header.frame_id, arrow.header.stamp = 'map', now
            arrow.ns, arrow.id, arrow.type = 'nodes', i, Marker.ARROW
            arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = x, y, 0.01
            q = quaternion_from_yaw(yaw)
            (arrow.pose.orientation.x, arrow.pose.orientation.y,
             arrow.pose.orientation.z, arrow.pose.orientation.w) = q
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.12, 0.03, 0.03
            arrow.color.r, arrow.color.a = 1.0, 0.9
            markers.markers.append(arrow)
            label = Marker()
            label.header = arrow.header
            label.ns, label.id, label.type = 'labels', i, Marker.TEXT_VIEW_FACING
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 0.08
            label.pose.orientation.w = 1.0
            label.scale.z = 0.09
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = str(i)
            markers.markers.append(label)
        self.marker_pub.publish(markers)


def main(args=None):
    run(GlobalPlanner, args)
