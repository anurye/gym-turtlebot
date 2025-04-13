"""
RvizPublisher: Publishes rviz visualization msgs.

Author: Ahmed Yesuf Nurye
Date: 2025-04-08
"""

import random

from geometry_msgs.msg import (
    Pose, Twist
)
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class RvizPublisher(Node):
    def __init__(self, node_name: str = 'rviz_publisher') -> None:
        super().__init__(node_name)

        self.goal_marker_pub_ = self.create_publisher(MarkerArray, 'goal_point', 10)
        self.linear_vel_marker_pub_ = self.create_publisher(MarkerArray, 'linear_velocity', 10)
        self.angular_vel_marker_pub_ = self.create_publisher(MarkerArray, 'angular_velocity', 10)

    def pub_goal_marker(self, pose: Pose) -> None:
        self.get_logger().debug(f'pose X: {pose.position.x}, pose Y: {pose.position.y}')
        pose_marker_spec = {
            'frame_id': 'odom',
            'marker_type': Marker.CYLINDER,
            'scale': (0.1, 0.1, 0.01),
            'color': (1.0, 0.0, 1.0, 0.0),
            'position': (pose.position.x, pose.position.y, 0.0),
            'orientation': (0.0, 0.0, 0.0, 1.0),
            'action': Marker.ADD,
            'ns': '',
            'marker_id': 0,
        }

        pose_marker = self.create_marker(**pose_marker_spec)
        marker_array = MarkerArray()
        marker_array.markers.append(pose_marker)

        self.goal_marker_pub_.publish(marker_array)

    def pub_vel_marker(self, twist: Twist) -> None:
        linear, angular = twist.linear, twist.angular
        self.get_logger().debug(
            f'Linear X: {linear.x}, Angular Z: {angular.z}'
        )
        marker_specs = [
            {
                'frame_id': 'odom',
                'marker_type': Marker.CUBE,
                'scale': (abs(linear.x), 0.1, 0.01),
                'color': (1.0, 1.0, 0.0, 0.0),
                'position': (5.0, 0.0, 0.0),
                'orientation': (0.0, 0.0, 0.0, 1.0),
                'action': Marker.ADD,
                'ns': '',
                'marker_id': 1,
                'publisher': self.linear_vel_marker_pub_,
            },
            {
                'frame_id': 'odom',
                'marker_type': Marker.CUBE,
                'scale': (abs(angular.z), 0.1, 0.01),
                'color': (1.0, 1.0, 0.0, 0.0),
                'position': (5.0, 0.2, 0.0),
                'orientation': (0.0, 0.0, 0.0, 1.0),
                'action': Marker.ADD,
                'ns': '',
                'marker_id': 2,
                'publisher': self.angular_vel_marker_pub_,
            }
        ]

        for marker_spec in marker_specs:
            marker = self.create_marker(**marker_spec)
            marker_array = MarkerArray()
            marker_array.markers.append(marker)
            marker_spec['publisher'].publish(marker_array)

    def create_marker(self, **marker_spec) -> Marker:
        marker = Marker()
        marker.header.frame_id = marker_spec.get('frame_id', 'odom')
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = marker_spec.get('ns', '')
        marker.id = marker_spec.get('marker_id', 0)
        marker.type = marker_spec.get('marker_type', Marker.CYLINDER)
        marker.action = marker_spec.get('action', Marker.ADD)

        (marker.scale.x,
         marker.scale.y,
         marker.scale.z) = marker_spec.get('scale', (1.0, 1.0, 1.0))

        (marker.color.a,
         marker.color.r,
         marker.color.g,
         marker.color.b) = marker_spec.get('color', (1.0, 0.0, 1.0, 0.0))

        (marker.pose.position.x,
         marker.pose.position.y,
         marker.pose.position.z) = marker_spec.get('position', (0.0, 0.0, 0.0))

        (marker.pose.orientation.x,
         marker.pose.orientation.y,
         marker.pose.orientation.z,
         marker.pose.orientation.w) = marker_spec.get('orientation', (0.0, 0.0, 0.0, 1.0))

        return marker


def main(args=None) -> None:
    rclpy.init(args=args)

    rviz_pub = RvizPublisher()

    while rclpy.ok():
        try:
            pose = Pose()
            pose.position.x = float(random.randint(2, 8))
            pose.position.y = float(random.randint(2, 8))
            pose.position.z = 0.0

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            rviz_pub.pub_goal_marker(pose)

        except KeyboardInterrupt:
            pass

    rviz_pub.destroy_node()


if __name__ == '__main__':
    main()
