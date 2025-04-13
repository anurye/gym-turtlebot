"""
ROS-Gazebo Interface: Handles communication between ROS and Gazebo.

Author: Ahmed Yesuf Nurye
Date: 2025-04-07
"""

import math  # noqa: F401
import subprocess
from typing import (
    Dict,
    List,
    Optional,
    Tuple,
)

from geometry_msgs.msg import (
    Pose,
    Twist,
    TwistStamped,
)
from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from ros_gz_interfaces.srv import (
    ControlWorld,
    SetEntityPose,
    SpawnEntity
)
from sensor_msgs.msg import LaserScan
from tb4_drl_navigation.utils.launch import Launcher
from transforms3d.euler import euler2quat, quat2euler  # noqa: F401


class Sensors(Node):
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
            '/odom',
            self.odom_callback,
            self.qos_profile
        )
        self.odom_subscription

        self._latest_scan: Optional[list] = None
        self._range_min: Optional[float] = None
        self._range_max: Optional[float] = None
        self._angle_min: Optional[float] = None
        self._angle_max: Optional[float] = None

        self._latest_odom: Optional[Pose] = None

    def get_data(self) -> Dict:
        return {
            'latest_scan': self._latest_scan,
            'latest_odom': self._latest_odom,
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
        self._latest_odom = msg.pose.pose

    def get_latest_odom(self) -> Optional[Pose]:
        return self._latest_odom

    def get_world_pose_from_odom(self, start_pose: Pose):
        """Convert odometry pose to world coordinates using initial spawn position."""
        pass

    def get_absolute_pose(self, model_name: str = 'turtlebot4') -> Pose:
        # Use full to get the exact location but, takes a lot of time.
        try:
            result = subprocess.run(
                ['gz', 'model', '-m', f'{model_name}', '-p'],
                capture_output=True,
                text=True,
                check=True
            )
            # Not sure if appending the error stream is necessary
            full_output = result.stdout + '\n' + result.stderr
            output_lines = full_output.split('\n')

            for i, line in enumerate(output_lines):
                if 'Pose [ XYZ (m) ] [ RPY (rad) ]:' in line:
                    # Extract XYZ and RPY values from subsequent lines
                    xyz_line = output_lines[i+1].strip()
                    rpy_line = output_lines[i+2].strip()

                    xyz = list(map(float, xyz_line.strip('[]').split()))
                    rpy = list(map(float, rpy_line.strip('[]').split()))

                    return {
                        'position': {'x': xyz[0], 'y': xyz[1], 'z': xyz[2]},
                        'orientation': {'roll': rpy[0], 'pitch': rpy[1], 'yaw': rpy[2]}
                    }
            print('Pose information not found in the output.')
            return None

        except subprocess.CalledProcessError as e:
            print(f'Command failed with error: {e}')
            return None
        except Exception as e:
            print(f'Unexpected error occurred: {e}')
            return None


class LidarSensor(Node):
    def __init__(self, node_name: str = 'scan_subscription') -> None:
        super().__init__(node_name=node_name)

        self.qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            self.qos_profile
        )
        self.subscription  # prevent unused variable warning

        self.ranges: Optional[List[float]] = None
        self.range_min: Optional[float] = None
        self.range_max: Optional[float] = None
        self.angle_min: Optional[float] = None
        self.angle_max: Optional[float] = None

    def scan_callback(self, msg: LaserScan) -> None:
        self.ranges = msg.ranges[:]
        print(f'Scan call back: min = {min(self.ranges)}, max = {max(self.ranges)}')
        if None in [self.angle_min, self.angle_max, self.range_min, self.range_max]:
            self.angle_min = msg.angle_min
            self.angle_max = msg.angle_max
            self.range_min = msg.range_min
            self.range_max = msg.range_max

    def get_ranges(self) -> Optional[List[float]]:
        return self.ranges

    def get_range_min_max(self) -> Optional[Tuple[float, float]]:
        return self.range_min, self.range_max

    def get_angle_min_max(self) -> Optional[Tuple[float, float]]:
        return self.angle_min, self.angle_max


class OdometrySensor(Node):
    def __init__(self, node_name: str = 'odom_subscription') -> None:
        super().__init__(node_name=node_name)

        self.qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            self.qos_profile
        )
        self.subscription  # prevent unused variable warning

        self.pose: Optional[Pose] = None

    def odom_callback(self, msg: Odometry) -> None:
        self.pose = msg.pose.pose

    def get_pose(self) -> Optional[Pose]:
        return self.pose

    def get_absolute_pose(self, model_name: str = 'turtlebot4') -> Pose:
        # TODO: this is temporary fix
        # NOTE: It used to be that odometry source can be set to world in clasic.
        try:
            result = subprocess.run(
                ['gz', 'model', '-m', f'{model_name}', '-p'],
                capture_output=True,
                text=True,
                check=True
            )
            # Not sure if appending the error stream is necessary
            full_output = result.stdout + '\n' + result.stderr
            output_lines = full_output.split('\n')

            for i, line in enumerate(output_lines):
                if 'Pose [ XYZ (m) ] [ RPY (rad) ]:' in line:
                    # Extract XYZ and RPY values from subsequent lines
                    xyz_line = output_lines[i+1].strip()
                    rpy_line = output_lines[i+2].strip()

                    xyz = list(map(float, xyz_line.strip('[]').split()))
                    rpy = list(map(float, rpy_line.strip('[]').split()))

                    return {
                        'position': {'x': xyz[0], 'y': xyz[1], 'z': xyz[2]},
                        'orientation': {'roll': rpy[0], 'pitch': rpy[1], 'yaw': rpy[2]}
                    }
            print('Pose information not found in the output.')
            return None

        except subprocess.CalledProcessError as e:
            print(f'Command failed with error: {e}')
            return None
        except Exception as e:
            print(f'Unexpected error occurred: {e}')
            return None


class Publisher(Node):
    def __init__(
            self,
            base_frame_id: str = 'base_link',
            node_name: str = 'ros_gz_publisher'
    ) -> None:
        super().__init__(node_name=node_name)

        self._base_frame_id = base_frame_id

        self.cmd_vel_publisher_ = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

    def pub_cmd_vel(self, action: Twist) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame_id
        msg.twist = action
        self.cmd_vel_publisher_.publish(msg)


class WorldControl(Node):
    def __init__(
            self,
            world_name: str,
            node_name: str = 'simulation_controller'
    ):
        super().__init__(node_name=node_name)
        self.world_name = world_name

        self.control_world_client = self.create_client(
            ControlWorld,
            f'/world/{self.world_name}/control'
        )
        self.set_pose_client = self.create_client(
            SetEntityPose,
            f'world/{self.world_name}/set_pose'
        )
        self.spawn_entity_client = self.create_client(
            SpawnEntity,
            f'world/{self.world_name}/create'
        )

        self._cached_gz_models: Optional[List[str]] = None

    def reset_world(self) -> None:
        request = ControlWorld.Request()
        request.world_control.reset.all = True

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

    ):
        request = SpawnEntity.Request()
        request.entity_factory.name

    def spawn_robot(
            self,
            package_name: str = 'tb4_gz_sim',
            launch_file: str = 'spawn_tb4.launch.py',
            *launch_args,
            build_first: bool = False
    ) -> None:
        workspace = Launcher.find_workspace()
        launcher = Launcher(workspace_dir=workspace)
        launcher.launch(
            package_name,
            launch_file,
            *launch_args,
            build_first=build_first
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

    def set_entity_pose(self, entity_name: str, pose: Pose) -> None:
        request = SetEntityPose.Request()
        request.entity.name = entity_name
        request.pose = pose

        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'Service /world/{self.world_name}/set_pose not available, waiting again...'
            )

        try:
            future = self.set_pose_client.call_async(request=request)
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


def main(args=None) -> None:
    rclpy.init(args=args)

    laser = LidarSensor()
    odom = OdometrySensor()

    while rclpy.ok():
        try:
            rclpy.spin_once(laser)
            rclpy.spin_once(odom)
            scan = laser.get_ranges()
            if scan is not None:
                laser.get_logger().info(f'Min: {min(scan)}')
                laser.get_logger().info(f'Max: {max(scan)}')
            pose = odom.get_pose()
            position, orientation = pose.position, pose.orientation
            if pose is not None:
                odom.get_logger().info(
                    f'position: [{position.x}, {position.y}, {position.z}], '
                    f'orientation: '
                    f'[{orientation.x}, {orientation.y}, {orientation.z}, {orientation.w}]'
                )
        except KeyboardInterrupt:
            pass

    laser.destroy_node()
    odom.destroy_node()

    # ctrl = WorldControl('static_world')
    # goal_pose = Pose()
    # goal_pose.position.x = 3.0
    # goal_pose.position.y = 0.0
    # goal_pose.position.z = 0.1
    # goal_pose.orientation.x = 0.0
    # goal_pose.orientation.y = 0.0
    # goal_pose.orientation.z = 0.0
    # goal_pose.orientation.w = 1.0
    # entity_name = "turtlebot4"

    # ctrl.pause_unpause(pause=True)

    # ctrl.set_entity_pose(entity_name=entity_name, pose=goal_pose)
    # print(ctrl.get_gz_models())
    # print('\n\n')
    # print(ctrl.get_obstacles())


if __name__ == '__main__':
    main()
