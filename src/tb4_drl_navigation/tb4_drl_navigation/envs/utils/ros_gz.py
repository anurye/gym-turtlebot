import subprocess
from typing import (
    Dict,
    List,
    Optional,
    Tuple,
)

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
    TwistStamped,
)
from nav_msgs.msg import Odometry, Path
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from robot_localization.srv import (
    SetPose,
)
from ros_gz_interfaces.srv import (
    ControlWorld,
    DeleteEntity,
    SetEntityPose,
    SpawnEntity
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from transforms3d.euler import quat2euler
from visualization_msgs.msg import Marker, MarkerArray


class Sensors(Node):
    """ROS2 node that subscribes to LaserScan and Odometry topics."""

    def __init__(self, node_name: str = 'sensors_node') -> None:
        super().__init__(node_name=node_name)

        self.qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            self.qos_profile
        )
        self.scan_subscription  # prevent unused variable warning

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            self.qos_profile
        )
        self.odom_subscription

        self._latest_scan: Optional[list] = None
        self._range_min: Optional[float] = None
        self._range_max: Optional[float] = None
        self._angle_min: Optional[float] = None
        self._angle_max: Optional[float] = None

        self._latest_pose_stamped: Optional[PoseStamped] = None

    def get_data(self) -> Dict:
        return {
            'latest_scan': self._latest_scan,
            'latest_odom': self._latest_pose_stamped,
        }

    def scan_callback(self, msg: LaserScan) -> None:
        self._latest_scan = msg.ranges[:]
        if None in [self._angle_min, self._angle_max, self._range_min, self._range_max]:
            self._angle_min = msg.angle_min
            self._angle_max = msg.angle_max
            self._range_min = msg.range_min
            self._range_max = msg.range_max

    def get_latest_scan(self) -> Optional[List[float]]:
        return self._latest_scan

    def get_range_min_max(self) -> Optional[Tuple[float, float]]:
        return self._range_min, self._range_max

    def get_angle_min_max(self) -> Optional[Tuple[float, float]]:
        return self._angle_min, self._angle_max

    def odom_callback(self, msg: Odometry) -> None:
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self._latest_pose_stamped = pose_stamped

    def get_latest_pose_stamped(self) -> Optional[PoseStamped]:
        return self._latest_pose_stamped


class Publisher(Node):
    """ROS2 node to publish velocity commands and visualization markers."""

    def __init__(
            self,
            base_frame_id: str = 'base_link',
            odom_frame_id: str = 'odom',
            node_name: str = 'ros_gz_publisher'
    ) -> None:
        super().__init__(
            node_name=node_name,
            parameter_overrides=[
                Parameter(
                    name='use_sim_time', type_=Parameter.Type.BOOL, value=True
                ),
            ],
        )

        self.base_frame_id = base_frame_id
        self.odom_frame_id = odom_frame_id

        self.cmd_vel_publisher_ = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.marker_publisher = self.create_publisher(
            Marker,
            '/visualization_marker',
            10
        )
        self.debug_markers_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10
        )
        self.path_publisher = self.create_publisher(
            Path,
            '/path',
            10
        )

        self.path = Path()
        self.path.header.frame_id = self.odom_frame_id

    def pub_cmd_vel(self, action: Twist) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame_id
        msg.twist = action
        self.cmd_vel_publisher_.publish(msg)

    def pub_goal_marker(self, goal_pose: Pose, marker_id: int = 0) -> None:
        marker = Marker()
        marker.header.frame_id = self.odom_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = goal_pose
        marker.scale.x = 0.5
        marker.scale.y = 0.05
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        self.marker_publisher.publish(marker)

    def pub_robot_path(self, pose_stamped: PoseStamped) -> None:
        self.path.poses.append(pose_stamped)
        self.path.header.stamp = pose_stamped.header.stamp
        self.path_publisher.publish(self.path)

    def clear_path(self) -> None:
        self.path.poses = []
        self.path_publisher.publish(self.path)

    def publish_observation(
            self,
            observation: Dict,
            robot_pose: PoseStamped,
            goal_pose: Pose,
            robot_radius: float = 1.0,
    ) -> None:
        odom_header = Header()
        odom_header.frame_id = self.odom_frame_id
        odom_header.stamp = self.get_clock().now().to_msg()

        base_link_header = Header()
        base_link_header.frame_id = self.base_frame_id
        base_link_header.stamp = robot_pose.header.stamp

        marker_array = MarkerArray()

        # Get the robots pose and orientation
        robot_x, robot_y, robot_z = (
            robot_pose.pose.position.x,
            robot_pose.pose.position.y,
            robot_pose.pose.position.z
        )
        q_robot = [
            robot_pose.pose.orientation.w,
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z
        ]
        _, _, robot_yaw = quat2euler(q_robot, 'sxyz')

        # Homogeneous transformation matrix: base_link -> odom
        T = np.array([
            [np.cos(robot_yaw), -np.sin(robot_yaw), robot_x],
            [np.sin(robot_yaw),  np.cos(robot_yaw), robot_y],
            [0,             0,              1]
        ])

        # Min ranges markers
        min_ranges = observation['min_ranges']
        min_angles = observation['min_ranges_angle']
        for i, (min_range, min_angle) in enumerate(zip(min_ranges, min_angles)):
            # Convert polar to Cartesian in base_link frame
            x_local = min_range * np.cos(min_angle)
            y_local = min_range * np.sin(min_angle)

            p_odom = T @ np.array([x_local, y_local, 1], dtype=np.float32)

            marker = Marker()
            marker.header = odom_header
            marker.ns = 'min_ranges'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.025
            marker.color.a = 1.0
            marker.color.r = 1.0
            start = Point(x=robot_x, y=robot_y, z=robot_z)
            end = Point(x=float(p_odom[0]), y=float(p_odom[1]), z=robot_z)
            marker.points = [start, end]
            marker_array.markers.append(marker)

        # Visualize relative distance to goal and orientation
        marker = self._create_arrow(
            start_point=Point(x=robot_x, y=robot_y, z=0.0),
            end_point=Point(x=goal_pose.position.x, y=goal_pose.position.y, z=0.0),
            header=odom_header,
            ns='orient_arrow',
            rgb=(0.0, 1.0, 0.0),
            marker_id=0
        )
        marker_array.markers.append(marker)
        # Horizontal line in front of the robot wrt baselink
        ref_line_odom = T @ np.array([2.0, 0.0, 1.0], dtype=np.float32)
        marker = self._create_arrow(
            start_point=Point(x=robot_x, y=robot_y, z=robot_z),
            end_point=Point(x=float(ref_line_odom[0]), y=float(ref_line_odom[1]), z=0.0),
            header=odom_header,
            ns='orient_arrow',
            rgb=(0.0, 1.0, 0.0),
            marker_id=1
        )
        marker_array.markers.append(marker)

        orient_to_goal = observation['orient_to_goal'][0]
        marker = self._create_arc(
            center=(0.0, 0.0),
            radius=robot_radius,
            theta_start=0.0,
            theta_end=orient_to_goal,
            header=base_link_header,
            ns='arc',
            rgb=(0.0, 1.0, 0.0),
        )
        marker_array.markers.append(marker)

        marker = self._create_text(
            text=f'{orient_to_goal:.4}rad',
            midpoint_theta=orient_to_goal / 2,
            radius=robot_radius + 0.5,
            header=base_link_header,
        )
        marker_array.markers.append(marker)

        self.debug_markers_pub.publish(marker_array)

    def _create_arrow(
            self,
            start_point: Point,
            end_point: Point,
            header: Header,
            ns: str = 'arrow',
            marker_id: int = 0,
            rgb: Tuple[float, float, float] = (1.0, 0.0, 0.0),
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.id = marker_id
        marker.ns = ns
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = 1.0
        marker.points = [start_point, end_point]
        return marker

    def _create_arc(
            self,
            center: Tuple[float, float],
            radius: float,
            theta_start: float,
            theta_end: float,
            header: Header,
            ns: str = 'arc',
            marker_id: int = 0,
            rgb: Tuple[float, float, float] = (0.0, 1.0, 0.0),
            num_points: int = 50,
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.ns = ns
        marker.id = marker_id
        marker.scale.x = 0.02
        marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = 1.0

        delta = (theta_end - theta_start) / num_points
        for i in range(num_points + 1):
            theta = theta_start + i * delta
            x = center[0] + radius * np.cos(theta)
            y = center[1] + radius * np.sin(theta)
            marker.points.append(Point(x=float(x), y=float(y), z=0.0))
        return marker

    def _create_text(
            self,
            text: str,
            midpoint_theta: float,
            radius: float,
            header: Header,
            marker_id: int = 0,
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = text
        marker.id = marker_id
        marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        # Position at arc midpoint
        marker.pose.position.x = float(radius * np.cos(midpoint_theta))
        marker.pose.position.y = float(radius * np.sin(midpoint_theta))
        marker.pose.position.z = 0.1

        return marker


class SimulationControl(Node):
    """ROS2 node for Gazebo world control and entity management."""

    def __init__(
            self,
            world_name: str,
            node_name: str = 'simulation_controller'
    ):
        super().__init__(
            node_name=node_name,
            parameter_overrides=[
                Parameter(
                    name='use_sim_time', type_=Parameter.Type.BOOL, value=True
                ),
            ],
        )

        self.world_name = world_name

        self.control_world_client = self.create_client(
            ControlWorld,
            f'/world/{self.world_name}/control'
        )
        self.set_entity_pose_client = self.create_client(
            SetEntityPose,
            f'world/{self.world_name}/set_pose'
        )
        self.set_pose_client = self.create_client(
            SetPose,
            'set_pose'
        )
        self.spawn_entity_client = self.create_client(
            SpawnEntity,
            f'world/{self.world_name}/create'
        )
        self.delete_entity_client = self.create_client(
            DeleteEntity,
            f'world/{self.world_name}/remove'
        )

        self._cached_gz_models: Optional[List[str]] = None

    def reset_world(self) -> None:
        request = ControlWorld.Request()
        request.world_control.reset.model_only = True

        while not self.control_world_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Service /world/{self.world_name}/control not available, waiting again...'
            )
        try:
            future = self.control_world_client.call_async(request=request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except Exception as e:
            self.get_logger().error(
                f'Service call /world/{self.world_name}/control failed: {e}'
            )

    def pause_unpause(self, pause: bool) -> None:
        request = ControlWorld.Request()
        request.world_control.pause = pause

        while not self.control_world_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Service /world/{self.world_name}/control not available, waiting again...'
            )
        try:
            future = self.control_world_client.call_async(request=request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except Exception as e:
            self.get_logger().error(
                f'Service call /world/{self.world_name}/control failed: {e}'
            )

    def spawn_entity(
            self,
            name: str,
            pose: Pose,
            sdf_filename: str,
    ):
        request = SpawnEntity.Request()
        request.entity_factory.name = name
        request.entity_factory.pose = pose
        request.entity_factory.sdf_filename = sdf_filename

        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Service /world/{self.world_name}/create not available, waiting again...'
            )
        try:
            future = self.spawn_entity_client.call_async(request=request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except Exception as e:
            self.get_logger().error(
                f'Service call /world/{self.world_name}/create failed: {e}'
            )

    def delete_entity(self, entity_name: str) -> None:
        request = DeleteEntity.Request()
        request.entity.name = entity_name

        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Service /world/{self.world_name}/remove not available, waiting again...'
            )
        try:
            future = self.delete_entity_client.call_async(request=request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except Exception as e:
            self.get_logger().error(
                f'Service call /world/{self.world_name}/remove failed: {e}'
            )

    def set_pose(self, pose: Pose, frame_id: str = 'odom') -> None:
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.pose = pose

        covariance = 1e-9*np.eye(6)
        # covariance[0][0] = 0.01  # x variance
        # covariance[1][1] = 0.01  # y variance
        # covariance[5][5] = 0.01  # yaw variance
        pose_msg.pose.covariance = covariance.flatten().tolist()

        request = SetPose.Request()
        request.pose = pose_msg
        try:
            future = self.set_pose_client.call_async(request=request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except Exception as e:
            self.get_logger().error(
                f'Service call /set_pose failed: {e}'
            )

    def set_entity_pose(self, entity_name: str, pose: Pose) -> None:
        request = SetEntityPose.Request()
        request.entity.name = entity_name
        request.pose = pose

        while not self.set_entity_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Service /world/{self.world_name}/set_pose not available, waiting again...'
            )

        try:
            future = self.set_entity_pose_client.call_async(request=request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except Exception as e:
            self.get_logger().error(
                f'Service call /world/{self.world_name}/set_pose failed: {e}'
            )

    def get_gz_models(self, force_refresh: bool = False) -> List[str]:
        """Get list of Gazebo model names."""
        if force_refresh:
            self._cached_gz_models = None
        if self._cached_gz_models:
            return self._cached_gz_models

        try:
            result = subprocess.run(
                ['gz', 'model', '--list'],
                capture_output=True,
                text=True,
                check=True,
                timeout=10,
            )

            full_output = result.stdout + '\n' + result.stderr
            models = []
            in_selection = False

            for line in full_output.splitlines():
                if 'Available models:' in line:
                    in_selection = True
                    continue

                if in_selection:
                    if not line.strip():
                        break

                    if line.strip().startswith(('- ', '* ')):
                        models.append(line.split(maxsplit=1)[1].strip())

            self._cached_gz_models = models

            return models

        except subprocess.CalledProcessError as e:
            print(f'Command failed (code {e.returncode})')
            print('Output:', e.output)
            return []
        except Exception as e:
            print(f'Error: {str(e)}')
            return []

    def get_obstacles(self, starts_with: str = 'obstacle') -> List[str]:
        """Get list of Gazebo model names that starts with 'starts_with'."""
        return [obs for obs in self.get_gz_models() if obs.startswith(starts_with)]


def test_world() -> None:
    world_ctrl = SimulationControl(world_name='static_world')
    entity_name = 'turtlebot4'

    # Options
    delete_entity = False
    set_pose = True
    pub = False

    # Example pose
    pose = Pose()
    pose.position.x = 5.0
    pose.position.y = 3.0
    pose.position.z = 0.1
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    if delete_entity:
        world_ctrl.delete_entity(entity_name=entity_name)

    if set_pose:
        world_ctrl.set_pose(pose=pose)
        world_ctrl.set_entity_pose(entity_name=entity_name, pose=pose)

    if pub:
        publisher = Publisher()
        publisher.pub_goal_marker(goal_pose=pose)

    world_ctrl.destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    # test_laser()
    # test_odom()
    test_world()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
