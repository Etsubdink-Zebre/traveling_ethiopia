import math
import os
import sys
import time
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

current_file = os.path.abspath(__file__)
path_parts = os.path.normpath(current_file).split(os.sep)
try:
    ros_nav_idx = path_parts.index('ros_navigation')
    project_root = os.sep.join(path_parts[:ros_nav_idx])
except ValueError:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))

if os.path.exists(os.path.join(project_root, 'uninformed_search')):
    sys.path.insert(0, project_root)

from uninformed_search.bfs import bfs
from data.graph_relaxed import ethiopia_relaxed_graph

CITY_COORDINATES = {
    'Addis Ababa': (0.0, 0.0),
    'Debre Birhan': (1.5, 3.0),
    'Ambo': (-3.0, 0.5),
    'Adama': (1.5, -2.0),
    'Wolkite': (-2.0, -1.5),
    'Buta Jirra': (-0.5, -2.5),
    'Worabe': (-1.0, -3.0),
    'Batu': (0.5, -3.0),
    'Nekemte': (-5.0, 1.0),
    'Gimbi': (-6.5, 2.0),
    'Dembi Dollo': (-7.5, 1.5),
    'Bedelle': (-5.0, -1.0),
    'Gore': (-6.5, -2.5),
    'Gambella': (-8.5, -2.0),
    'Tepi': (-6.5, -4.0),
    'Bonga': (-5.5, -3.0),
    'Mezan Tefari': (-6.0, -4.5),
    'Jimma': (-4.0, -2.5),
    'Hossana': (-1.0, -4.5),
    'Shashemene': (0.5, -5.0),
    'Hawassa': (1.0, -5.5),
    'Dilla': (1.0, -7.0),
    'Wolaita Sodo': (-2.0, -5.5),
    'Dawro': (-3.0, -5.0),
    'Arba Minch': (-2.5, -8.0),
    'Matahara': (3.0, -1.5),
    'Awash': (4.5, -0.5),
    'Chiro': (5.5, 1.0),
    'Dire Dawa': (6.5, 2.5),
    'Harar': (7.5, 1.5),
    'Babile': (7.5, 0.5),
    'Jijiga': (8.5, -0.5),
    'Dega Habur': (7.0, -2.5),
    'Kebri Dehar': (8.5, -4.0),
    'Gode': (8.5, -7.0),
    'Assella': (2.5, -2.5),
    'Assasa': (3.5, -3.5),
    'Dodola': (3.5, -5.5),
    'Bale': (4.5, -5.5),
    'Goba': (5.5, -3.5),
    'Sof Oumer': (6.5, -4.5),
}


class EthiopiaRosController(Node):
    """ROS 2 navigation controller using BFS on the Ethiopia relaxed graph."""

    GOAL_THRESHOLD = 0.3
    LINEAR_SPEED = 0.5
    ANGULAR_KP = 0.5
    LINEAR_KP = 0.3
    HEADING_KP = 0.2
    HEADING_TOLERANCE = 0.1

    def __init__(self):
        super().__init__('ethiopia_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._odom_callback, 10)

        self.pose = [0.0, 0.0, 0.0]
        self.path = []
        self.path_index = 0
        self.navigating = False

    def _odom_callback(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose[2] = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def plan_and_execute(self, start, goal):
        self.get_logger().info(f'Planning path from {start} to {goal}...')
        path = bfs(ethiopia_relaxed_graph, start, goal)

        if not path:
            self.get_logger().error(f'No path found from {start} to {goal}.')
            return

        self.get_logger().info(f'BFS Path Found: {" -> ".join(path)}')
        self.get_logger().info(f'Path length: {len(path)} cities')
        self.path = path
        self.path_index = 0
        self.navigating = True
        self._advance()

    def _advance(self):
        if not self.navigating or self.path_index >= len(self.path):
            self.navigating = False
            self.get_logger().info('Navigation complete!')
            return

        city = self.path[self.path_index]
        coords = CITY_COORDINATES.get(city)

        if coords:
            self.get_logger().info(f'Navigating to {city} at {coords}')
            self.nav_timer = self.create_timer(0.1, self._control_loop)
        else:
            self.get_logger().warn(f'No coordinates for {city}, skipping...')
            self.path_index += 1
            self._advance()

    def _control_loop(self):
        if not self.navigating or self.path_index >= len(self.path):
            if hasattr(self, 'nav_timer'):
                self.nav_timer.cancel()
            return

        city = self.path[self.path_index]
        coords = CITY_COORDINATES.get(city)

        if not coords:
            self.path_index += 1
            if self.path_index < len(self.path):
                self._advance()
            else:
                self.navigating = False
            return

        dx = coords[0] - self.pose[0]
        dy = coords[1] - self.pose[1]
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = math.atan2(
            math.sin(angle_to_goal - self.pose[2]),
            math.cos(angle_to_goal - self.pose[2]),
        )

        msg = Twist()

        if distance < self.GOAL_THRESHOLD:
            self.get_logger().info(f'Reached {city}!')
            self.publisher_.publish(msg)
            self.path_index += 1
            if self.path_index < len(self.path):
                self._advance()
            else:
                self.navigating = False
                self.get_logger().info('Navigation complete! All cities reached.')
                if hasattr(self, 'nav_timer'):
                    self.nav_timer.cancel()
            return

        if abs(angle_error) > self.HEADING_TOLERANCE:
            msg.angular.z = self.ANGULAR_KP * angle_error
        else:
            msg.linear.x = min(self.LINEAR_SPEED, self.LINEAR_KP * distance)
            msg.angular.z = self.HEADING_KP * angle_error

        self.publisher_.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='Ethiopia Navigation Controller')
    parser.add_argument('--start', type=str, default='Addis Ababa')
    parser.add_argument('--goal', type=str, default='Harar')

    try:
        import rclpy.utilities
        ros_args = rclpy.utilities.remove_ros_args(args or sys.argv)
        parsed = parser.parse_args(ros_args[1:] if len(ros_args) > 1 else [])
    except Exception:
        parsed = parser.parse_args(args[1:] if args and len(args) > 1 else [])

    rclpy.init(args=args)
    controller = EthiopiaRosController()

    controller.get_logger().info(
        f'Starting navigation from {parsed.start} to {parsed.goal}'
    )
    time.sleep(1.0)
    controller.plan_and_execute(parsed.start, parsed.goal)

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
