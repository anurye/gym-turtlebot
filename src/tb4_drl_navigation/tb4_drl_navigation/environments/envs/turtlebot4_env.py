"""
RL environment node for Gazebo-ROS2.

Author: Ahmed Yesuf Nurye
Date: 2025-04-08
"""

import math
import os
from pathlib import Path
import time
from typing import (
    Dict,
    List,
    Optional,
    Tuple,
)

from geometry_msgs.msg import Pose

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from tb4_drl_navigation.environments.utils.env_helper import EnvHelper
from tb4_drl_navigation.environments.utils.ros_gz import (
    Publisher,
    Sensors,
    WorldControl,
)
from tb4_drl_navigation.utils.dtype_convertor import (  # noqa: F401
    PoseConverter,
    TwistConverter,
)
from tb4_drl_navigation.utils.rviz import RvizPublisher
from transforms3d.euler import (  # noqa: F401
    euler2quat,
    quat2euler,
)


class Turtlebot4Env(gym.Env):

    def __init__(self,
                 world_name: str = 'static_world',
                 agent_name: str = 'turtlebot4',
                 map_path: Optional[str] = None,
                 yaml_path: Optional[str] = None,
                 spawn_launch_path: Optional[str] = None,
                 robot_radius: float = 0.3,
                 min_separation: float = 1.5,
                 obstacle_prefix: str = 'obstacle',
                 obstacle_clearance: float = 2.0,
                 num_bins: int = 20,
                 goal_threshold: float = 0.35,
                 collision_threshold: float = 0.4,
                 time_delta: float = 0.4,
                 shuffle_on_reset: bool = True):
        super(Turtlebot4Env, self).__init__()

        current_dir = Path(__file__).resolve().parent
        if map_path is None:
            map_path = os.path.join(f'{current_dir.parent}', 'maps', f'{world_name}.pgm')
        if yaml_path is None:
            yaml_path = os.path.join(f'{current_dir.parent}', 'maps', f'{world_name}.yaml')

        if not Path(map_path).resolve().exists():
            raise FileNotFoundError(f'Map file missing at: {map_path}')
        if not Path(yaml_path).resolve().exists():
            raise FileNotFoundError(f'Map metadata missing at: {yaml_path}')

        self.world_name = world_name
        self.agent_name = agent_name
        self.map_path = map_path
        self.yaml_path = yaml_path
        self.spawn_launch_path = spawn_launch_path
        self.robot_radius = robot_radius
        self.min_separation = min_separation
        self.obstacle_clearance = obstacle_clearance
        self.obstacle_prefix = obstacle_prefix

        self.num_bins = num_bins

        self.time_delta = time_delta
        self.shuffle_on_reset = shuffle_on_reset
        self.goal_threshold = goal_threshold
        self.collision_threshold = collision_threshold

        # rclpy.init(args=None)

        self.sensors = Sensors(node_name=f'{self.agent_name}_sensors')
        self.ros_gz_pub = Publisher(node_name=f'{self.agent_name}_gz_pub')
        self.rviz_pub = RvizPublisher(node_name=f'{self.agent_name}_rviz_pub')
        self.world_control = WorldControl(
            world_name=self.world_name, node_name=f'{self.agent_name}_world_control'
        )

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.sensors)
        self.executor.add_node(self.ros_gz_pub)
        self.executor.add_node(self.rviz_pub)
        self.executor.add_node(self.world_control)

        self.executor.spin_once(timeout_sec=1.0)

        self.pose_converter = PoseConverter()
        self.twist_converter = TwistConverter()

        self.env_helper = EnvHelper(
            map_path=self.map_path,
            yaml_path=self.yaml_path,
            robot_radius=self.robot_radius,
            min_separation=self.min_separation,
            obstacle_clearance=self.obstacle_clearance
        )

        self.action_space = spaces.Box(
            low=np.array([0.0, 0.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )
        self.observation_space = self._get_observation_space()

        self._last_action = np.zeros(self.action_space.shape, dtype=np.float32)

        self._target_pose: Optional[Pose] = None
        self._start_pose: Optional[Pose] = None

    def _get_observation_space(self) -> spaces.Dict:
        self.world_control.pause_unpause(pause=False)
        # Wait for scan and odometry to initialize
        while True:
            self.executor.spin_once(timeout_sec=0.1)
            range_min, range_max = self.sensors.get_range_min_max()
            angle_min, angle_max = self.sensors.get_angle_min_max()
            if None not in [range_min, range_max, angle_min, angle_max]:
                break

        return spaces.Dict({
            'min_ranges': spaces.Box(
                low=range_min, high=range_max, shape=(self.num_bins,), dtype=np.float32
            ),
            'min_ranges_angle': spaces.Box(
                low=angle_min, high=angle_max, shape=(self.num_bins,), dtype=np.float32
            ),
            'dist_to_goal': spaces.Box(low=0.0, high=100.0, shape=(1,), dtype=np.float32),
            'orient_to_goal': spaces.Box(low=-math.pi, high=math.pi, shape=(1,), dtype=np.float32),
            'action': spaces.Box(
                low=self.action_space.low,
                high=self.action_space.high,
                shape=self.action_space.shape,
                dtype=np.float32
            ),
        })

    def _get_info(self) -> Optional[Dict[str, float]]:
        pass

    def _get_obs(self) -> Dict:
        # TODO: may be do spin_once here in addition to the one done during step & reset
        min_ranges, min_ranges_angle = self._process_lidar()
        dist_to_goal, orient_to_goal = self._process_odom()

        return {
            'min_ranges': np.array(min_ranges, dtype=np.float32),
            'min_ranges_angle': np.array(min_ranges_angle, dtype=np.float32),
            'dist_to_goal': np.array([dist_to_goal], dtype=np.float32),
            'orient_to_goal': np.array([orient_to_goal], dtype=np.float32),
            'action': self._last_action.astype(np.float32),
        }

    def _process_lidar(self) -> Tuple[List[float], List[float]]:
        # Get laser scan data
        ranges = self.sensors.get_latest_scan()
        range_min, range_max = self.sensors.get_range_min_max()
        angle_min, angle_max = self.sensors.get_angle_min_max()
        num_ranges = len(ranges)

        # Calculate bin width and mid
        self.num_bins = min(max(1, self.num_bins), num_ranges)
        bin_width = (angle_max - angle_min) / self.num_bins

        # Initialize bins with default values centred at bin centre
        min_ranges = [range_max] * self.num_bins
        min_ranges_angle = [
            angle_min + (i * bin_width) + bin_width/2 for i in range(self.num_bins)
        ]

        # Process ranges
        for i in range(num_ranges):
            current_range = ranges[i]
            current_angle = angle_min + i * (angle_max - angle_min) / (num_ranges - 1)
            # Clip current_angle to handle floating point precision
            current_angle = max(angle_min, min(current_angle, angle_max))

            # Take the default for invalid range
            if not (range_min <= current_range <= range_max) or not math.isfinite(current_range):
                continue

            # Calculate bin index
            bin_idx = (current_angle - angle_min) // bin_width
            bin_idx = int(max(0, min(bin_idx, self.num_bins - 1)))

            # Update min range and angle
            if current_range < min_ranges[bin_idx]:
                min_ranges[bin_idx] = current_range
                min_ranges_angle[bin_idx] = current_angle

        return min_ranges, min_ranges_angle

    def _process_odom(self) -> Tuple[float, float]:
        # Get current pose
        # agent_pose = self.sensors.get_world_pose_from_odom(
        #     start_pose=self._start_pose
        # )
        agent_pose = self.sensors.get_latest_odom()

        # Extract positions
        agent_x = agent_pose.position.x
        agent_y = agent_pose.position.y
        goal_x = self._target_pose.position.x
        goal_y = self._target_pose.position.y

        # Calculate relative distance
        dx = goal_x - agent_x
        dy = goal_y - agent_y
        distance = math.hypot(dx, dy)

        # Handle zero-distance edge case
        if math.isclose(distance, 0.0, abs_tol=1e-3):
            return (0.0, 0.0)

        # Calculate bearing to goal (global frame)
        bearing = math.atan2(dy, dx)

        # Extract current orientation
        q = [
            agent_pose.orientation.w,
            agent_pose.orientation.x,
            agent_pose.orientation.y,
            agent_pose.orientation.z
        ]
        _, _, yaw = quat2euler(q, 'sxyz')

        # Calculate relative angle (robot's frame)
        relative_angle = bearing - yaw

        # Normalize angle to [-pi, pi]
        relative_angle = math.atan2(math.sin(relative_angle), math.cos(relative_angle))

        return distance, relative_angle

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool]:
        # Store action for inclusion in next observation
        self._last_action = action.copy()

        # Execute the action, propaget for time_delta
        twist_msg = self.twist_converter.from_dict({
            'linear': (float(action[0]), 0.0, 0.0),
            'angular': (0.0, 0.0, float(action[1]))
        })
        self.rviz_pub.pub_vel_marker(twist=twist_msg)
        self.ros_gz_pub.pub_cmd_vel(twist_msg)

        observation, info = self._propagate_state(time_delta=self.time_delta)

        # MDP
        truncated = False
        terminated = (
            self._goal_reached(
                dist_to_goal=observation['dist_to_goal'].item()
            ) or self._collision(min_ranges=observation['min_ranges'])
        )
        reward = self._get_reward(
            action=action,
            min_ranges=observation['min_ranges'],
            dist_to_goal=observation['dist_to_goal']
        )

        return observation, reward, terminated, truncated, info

    def reset(self,
              seed: Optional[int] = None,
              options: Optional[dict] = None
              ) -> Tuple[Dict]:
        super().reset(seed=seed)

        # Initialize last action to zeros
        self._last_action = np.zeros(self.action_space.shape, dtype=np.float32)

        twist_msg = self.twist_converter.from_dict({
            'linear': (float(self._last_action[0]), 0.0, 0.0),
            'angular': (0.0, 0.0, float(self._last_action[1]))
        })
        self.rviz_pub.pub_vel_marker(twist=twist_msg)
        self.ros_gz_pub.pub_cmd_vel(twist_msg)

        options = options or {}
        start_pos = options.get('start_pos')  # (x, y)
        goal_pos = options.get('goal_pos')    # (x, y)
        if start_pos is None or goal_pos is None:
            start_pos, goal_pos = self.env_helper.generate_start_goal()
            # print(f'Start: {start_pos}, Goal: {goal_pos}')
            # TODO: Odometry doesn't start relative to the world.
            start_pos = (0.0, 0.0)

        # Convert pos to pose
        self._start_pose = self.pose_converter.from_dict({
            'position': (start_pos[0], start_pos[1], 0.01),
            'orientation': (0.0, 0.0, 0.0, 1.0)
        })
        self._target_pose = self.pose_converter.from_dict({
            'position': (goal_pos[0], goal_pos[1], 0.01),
            'orientation': (0.0, 0.0, 0.0, 1.0)
        })
        self.rviz_pub.pub_goal_marker(pose=self._target_pose)

        self.world_control.reset_world()

        # Unfortunately gazebo reset unspawns the robot
        self.world_control.spawn_robot(
            'tb4_gz_sim',
            'spawn_tb4.launch.py',
            f'x_pose:={start_pos[0]}',
            f'y_pose:={start_pos[1]}',
            'z_pose:=0.01',
            'roll:=0.00',
            'pitch:=0.00',
            'yaw:=0.00',
            build_first=False,
        )
        # TODO: will be used after the reset problem is figured out
        # self.world_control.set_entity_pose(self.agent_name, self._start_pose)

        # Shuffle obstacles
        if self.shuffle_on_reset:
            self._shuffle_obstacles(
                start_pos=start_pos, goal_pos=goal_pos
            )

        observation, info = self._propagate_state(time_delta=self.time_delta)

        return observation, info

    def _propagate_state(self, time_delta: float = 0.2) -> Tuple[Dict]:
        self.world_control.pause_unpause(pause=False)

        end_time = time.time() + time_delta
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=max(0, end_time - time.time()))

        self.world_control.pause_unpause(pause=True)

        observation = self._get_obs()
        info = {'distance_to_goal': observation['dist_to_goal'].item()}

        return observation, info

    def _shuffle_obstacles(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> None:
        # Get random obstacle locations
        obstacles = self.world_control.get_obstacles(starts_with=self.obstacle_prefix)
        obstacles_pos = self.env_helper.generate_obstacles(
            num_obstacles=len(obstacles), start_pos=start_pos, goal_pos=goal_pos
        )
        for obs_pos, obs_name in zip(obstacles_pos, obstacles[:len(obstacles_pos)]):
            # Convert to Pose
            obs_pose = self.pose_converter.from_dict({
                'position': (obs_pos[0], obs_pos[1], 0.01),
                'orientation': (0.0, 0.0, 0.0, 1.0)
            })
            self.world_control.set_entity_pose(
                entity_name=obs_name, pose=obs_pose
            )

    def _get_reward(self,
                    action: np.ndarray,
                    min_ranges: np.ndarray,
                    dist_to_goal: float) -> float:
        if self._goal_reached(dist_to_goal=dist_to_goal):
            return 100.0
        if self._collision(min_ranges=min_ranges):
            return -100.0

        # Obstacle reward
        obstacle_reward = (min(min_ranges) - 1)/2 if min(min_ranges) < 1.0 else 0.0

        # Action reward
        action_reward = action[0]/2 - abs(action[1])/2 - 0.001

        return obstacle_reward + action_reward

    def _goal_reached(self, dist_to_goal: float) -> bool:
        if dist_to_goal < self.goal_threshold:
            self.sensors.get_logger().info('Goal reached!')
            return True
        return False

    def _collision(self, min_ranges) -> bool:
        if min(min_ranges) < self.collision_threshold:
            self.sensors.get_logger().info('Colission detected!')
            return True
        return False

    def close(self) -> None:
        self.world_control.destroy_node()
        self.ros_gz_pub.destroy_node()
        self.sensors.destroy_node()
        self.rviz_pub.destroy_node()
        rclpy.try_shutdown()


def main():
    import tb4_drl_navigation.environments  # noqa: F401
    env = gym.make('Turtlebot4Env-v0', world_name='static_world')

    try:
        i = 0
        while i < 3:
            obs, info = env.reset()
            print('Obs:', obs, '\n\n', 'Info: ', info, '\n\n')
            i += 1
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
