import math
from pathlib import Path
import time
from typing import (
    Any,
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

from tb4_drl_navigation.envs.diffdrive.scenario_generator import ScenarioGenerator
from tb4_drl_navigation.envs.utils.ros_gz import (
    Publisher,
    Sensors,
    SimulationControl,
)
from tb4_drl_navigation.utils.dtype_convertor import (
    PoseConverter,
    TwistConverter,
)
from transforms3d.euler import (
    euler2quat,
    quat2euler,
)


class Turtlebot4Env(gym.Env):
    """
    Gymnasium environment for a ROS2/Gazebo Turtlebot4 navigation task.

    The goal is to navigate a Turtlebot4 robot through a static world and reach a randomly sampled
    goal while avoiding obstacles. Observations consist of discretized laser-range bins,
    their bearing angles, the distance and orientation to the goal, and
    the previous action. Episodes terminate on goal reach or collision; timeouts
    are handled by Gymnasium's TimeLimit wrapper. Detailed description is given below.

    Observation Space
    -----------------
    1. min_ranges: Box(low=range_min, high=range_max, shape=(num_bins,), float32)
        Minimum distance reading within each of `num_bins` laser sectors.
        - range_min and range_max are obtained from the lidar sensor used.
        - num_bins by default it 20.
    2. min_ranges_angle: Box(low=angle_min, high=angle_max, shape=(num_bins,), float32)
        Corresponding bearing angles for each scan sector's/bin's minimum reading.
        - angle_min and angle_max are obtained from the lidar sensor used.
    3. dist_to_goal: Box(low=0.0, high=100.0, shape=(1,), float32)
        Euclidean distance [m] from the robot to the goal.
    4. orient_to_goal: Box(low=-pi, high=pi, shape=(1,), float32)
        Relative heading [rad] from robot's forward direction to the goal.
    5. action: Box(low=0.0, high=1.0, shape=(2,), float32)
        The last executed (linear, angular) action.

    Action Space
    ------------
    Continuous two-dimensional action:
        Box(low=[0.0, -1.0], high=[1.0, 1.0], shape=(2,), dtype=float32)

    Transition Dynamics
    -------------------
    - On `step(action)`, the action is converted to a ROS2 Twist message and
      published. The simulator propagates the state for `time_delta` seconds. Laser and odometry
      data are retrieved, processed into the observation dict, and the previous action is included.
    - Resetting the environment (`reset`) will:
        1. Pause and reset the Gazebo world (Moldels only, which is currently not supported).
        2. Sample or accept provided `start_pos` and `goal_pos` (x, y, yaw) options.
            - The internal `ScenarioGenerator` may be used to shuffle obstacle positions on reset.
        3. Teleport robot and obstacles to their poses and publish a goal marker.
        5. Propagate the state and return the initial observation and info.

    Reward Function
    ---------------
    The reward is designed in such a way that:
    - Encourages reaching the goal quickly, penalizes collisions heavily.
    - Provides a small shaping reward for staying away from obstacles and
      for forward motion with minimal rotation.

    Four components of the reward function:
    1. Target goal reached: +100
    2. Collision detected: -100
    3. Intermediate reward: linear_vel / 2 - |angular_v| / 2 - 0.001
    4. Obstacle clearance: (min(min_ranges) - 1)/2 if min(min_ranges) < 1.0 else 0.0

    Start State
    -----------
    - Robot is placed at `start_pos` (x, y, yaw) in the world.
    - Goal is placed at `goal_pos` (x, y, yaw).
    - Obstacles are distributed ensuring `obstacle_clearance` from both start
      and goal, if `shuffle_on_reset=True`.

    Episode Termination
    -------------------
    The episode ends if either of the following happens:
    1. Termination: `terminated = True` when:
        - Distance to goal < `goal_threshold`, or
        - Any laser reading < `collision_threshold`.
    2. Truncation: `truncated = True` when the step count exceeds Gymnasium's
      `max_episode_steps` (configured externally at registration).

    ```python
    >>> import gymnasium as gym
    >>> env = env = gym.make('Turtlebot4Env-v0', world_name='static_world')
    >>> env = FlattenObservation(env=env)
    >>> env
    <FlattenObservation<TimeLimit<OrderEnforcing<PassiveEnvChecker<Turtlebot4Env<Turtlebot4Env-v0>>>>>>
    ```
    """

    def __init__(
            self,
            world_name: str = 'static_world',
            robot_name: str = 'turtlebot4',
            map_path: Optional[Path] = None,
            yaml_path: Optional[Path] = None,
            sim_launch_path: Optional[Path] = None,
            robot_radius: float = 0.3,
            min_separation: float = 1.5,
            goal_sampling_bias: str = 'uniform',
            obstacle_prefix: str = 'obstacle',
            obstacle_clearance: float = 2.0,
            num_bins: int = 30,
            goal_threshold: float = 0.35,
            collision_threshold: float = 0.4,
            time_delta: float = 0.4,
            shuffle_on_reset: bool = True
    ):
        super(Turtlebot4Env, self).__init__()

        self.world_name = world_name
        self.robot_name = robot_name

        current_dir = Path(__file__).resolve().parent
        self.map_path = map_path or current_dir / 'maps' / f'{world_name}.pgm'
        self.yaml_path = yaml_path or current_dir / 'maps' / f'{world_name}.yaml'

        self.sim_launch_path = sim_launch_path
        self.robot_radius = robot_radius
        self.min_separation = min_separation
        self.goal_sampling_bias = goal_sampling_bias
        self.obstacle_clearance = obstacle_clearance
        self.obstacle_prefix = obstacle_prefix

        self.num_bins = num_bins

        self.time_delta = time_delta
        self.shuffle_on_reset = shuffle_on_reset
        self.goal_threshold = goal_threshold
        self.collision_threshold = collision_threshold

        self.sensors = Sensors(node_name=f'{self.robot_name}_sensors')
        self.ros_gz_pub = Publisher(node_name=f'{self.robot_name}_gz_pub')
        self.simulation_control = SimulationControl(
            world_name=self.world_name, node_name=f'{self.robot_name}_world_control'
        )

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.sensors)
        self.executor.add_node(self.ros_gz_pub)
        self.executor.add_node(self.simulation_control)

        self.executor.spin_once(timeout_sec=1.0)

        self.pose_converter = PoseConverter()
        self.twist_converter = TwistConverter()

        self.nav_scenario = ScenarioGenerator(
            map_path=self.map_path,
            yaml_path=self.yaml_path,
            robot_radius=self.robot_radius,
            min_separation=self.min_separation,
            obstacle_clearance=self.obstacle_clearance,
            seed=None
        )

        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )
        self.observation_space = self._build_observation_space()

        self._last_action = np.zeros(self.action_space.shape, dtype=np.float32)

        self._goal_pose: Optional[Pose] = None
        self._start_pose: Optional[Pose] = None

    def _build_observation_space(self) -> spaces.Dict:
        self.simulation_control.pause_unpause(pause=False)
        # Wait for scan and odometry to initialize
        while True:
            self.executor.spin_once(timeout_sec=0.1)
            range_min, range_max = self.sensors.get_range_min_max()
            angle_min, angle_max = self.sensors.get_angle_min_max()
            if None not in (range_min, range_max, angle_min, angle_max):
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

    def _get_info(self) -> Dict[str, Any]:
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
        pose_stamped = self.sensors.get_latest_pose_stamped()
        agent_pose = pose_stamped.pose

        # Extract positions
        agent_x = agent_pose.position.x
        agent_y = agent_pose.position.y
        goal_x = self._goal_pose.position.x
        goal_y = self._goal_pose.position.y

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

    def step(
            self,
            action: np.ndarray,
            debug: Optional[bool] = None
    ) -> Tuple[np.ndarray, float, bool, bool]:
        # Store action for inclusion in next observation
        self._last_action = action.copy()

        # Execute the action, propaget for time_delta
        twist_msg = self.twist_converter.from_dict({
            'linear': (float(action[0]), 0.0, 0.0),
            'angular': (0.0, 0.0, float(action[1]))
        })
        self.ros_gz_pub.pub_cmd_vel(twist_msg)

        observation, info = self._propagate_state(time_delta=self.time_delta)

        self.ros_gz_pub.pub_robot_path(pose_stamped=self.sensors.get_latest_pose_stamped())

        goal_reached = self._goal_reached(dist_to_goal=observation['dist_to_goal'].item())
        collision = self._collision(min_ranges=observation['min_ranges'])
        if goal_reached:
            self.sensors.get_logger().info('Goal reached!')
        elif collision:
            self.sensors.get_logger().info('Colission detected!')

        # MDP
        truncated = False
        terminated = goal_reached or collision
        reward = self._get_reward(
            action=action,
            min_ranges=observation['min_ranges'],
            dist_to_goal=observation['dist_to_goal']
        )

        if debug:
            self.ros_gz_pub.publish_observation(
                observation=observation,
                robot_pose=self.sensors.get_latest_pose_stamped(),
                goal_pose=self._goal_pose
            )

        return observation, reward, terminated, truncated, info

    def reset(
            self,
            seed: Optional[int] = None,
            options: Optional[dict] = None
    ) -> Tuple[Dict]:
        super().reset(seed=seed)

        self.simulation_control.reset_world()

        # Initialize last action to zeros
        self._last_action = np.zeros(self.action_space.shape, dtype=np.float32)
        twist_msg = self.twist_converter.from_dict({
            'linear': (float(self._last_action[0]), 0.0, 0.0),
            'angular': (0.0, 0.0, float(self._last_action[1]))
        })
        self.ros_gz_pub.pub_cmd_vel(twist_msg)

        self.ros_gz_pub.clear_path()

        options = options or {}
        start_pos = options.get('start_pos')  # (x, y, yaw)
        goal_pos = options.get('goal_pos')    # (x, y, yaw)
        if start_pos is None or goal_pos is None:
            start_xy, goal_xy = self.nav_scenario.generate_start_goal(
                max_attempts=100,
                goal_sampling_bias=self.goal_sampling_bias,
                eps=1e-5,
            )
            start_yaw = np.random.uniform(-np.pi, np.pi)
            goal_yaw = np.random.uniform(-np.pi, np.pi)
            start_pos = (*start_xy, start_yaw)
            goal_pos = (*goal_xy, goal_yaw)
        start_quat = euler2quat(ai=0.0, aj=0.0, ak=start_pos[2], axes='sxyz')
        goal_quat = euler2quat(ai=0.0, aj=0.0, ak=goal_pos[2], axes='sxyz')

        # Convert pos to pose
        self._start_pose = self.pose_converter.from_dict({
            'position': (start_pos[0], start_pos[1], 0.01),
            'orientation': (start_quat[1], start_quat[2], start_quat[3], start_quat[0])
        })
        self._goal_pose = self.pose_converter.from_dict({
            'position': (goal_pos[0], goal_pos[1], 0.01),
            'orientation': (goal_quat[1], goal_quat[2], goal_quat[3], goal_quat[0])
        })

        self.simulation_control.set_entity_pose(
            entity_name=self.robot_name,
            pose=self._start_pose
        )
        self.simulation_control.set_pose(
            pose=self._start_pose, frame_id='odom'
        )
        self.ros_gz_pub.pub_goal_marker(goal_pose=self._goal_pose)

        # Shuffle obstacles
        if self.shuffle_on_reset:
            self._shuffle_obstacles(
                start_pos=start_pos, goal_pos=goal_pos
            )

        observation, info = self._propagate_state(time_delta=self.time_delta)

        if options.get('debug'):
            self.ros_gz_pub.publish_observation(
                observation=observation,
                robot_pose=self.sensors.get_latest_pose_stamped(),
                goal_pose=self._goal_pose
            )

        return observation, info

    def _propagate_state(self, time_delta: float = 0.2) -> Tuple[Dict]:
        self.simulation_control.pause_unpause(pause=False)

        end_time = time.time() + time_delta
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=max(0, end_time - time.time()))

        self.simulation_control.pause_unpause(pause=True)

        observation = self._get_obs()
        info = {'distance_to_goal': observation['dist_to_goal'].item()}

        return observation, info

    def _shuffle_obstacles(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> None:
        # Get random obstacle locations
        obstacles = self.simulation_control.get_obstacles(starts_with=self.obstacle_prefix)
        obstacles_pos = self.nav_scenario.generate_obstacles(
            num_obstacles=len(obstacles), start_pos=start_pos, goal_pos=goal_pos
        )
        for obs_pos, obs_name in zip(obstacles_pos, obstacles[:len(obstacles_pos)]):
            # Convert to Pose
            obs_pose = self.pose_converter.from_dict({
                'position': (obs_pos[0], obs_pos[1], 0.01),
                'orientation': (0.0, 0.0, 0.0, 1.0)
            })
            self.simulation_control.set_entity_pose(
                entity_name=obs_name, pose=obs_pose
            )

    def _get_reward(
            self,
            action: np.ndarray,
            min_ranges: np.ndarray,
            dist_to_goal: float
    ) -> float:
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
            return True
        return False

    def _collision(self, min_ranges) -> bool:
        if min(min_ranges) < self.collision_threshold:
            return True
        return False

    def close(self) -> None:
        self.simulation_control.destroy_node()
        self.ros_gz_pub.destroy_node()
        self.sensors.destroy_node()
        rclpy.try_shutdown()


def main():
    import tb4_drl_navigation.envs  # noqa: F401

    rclpy.init(args=None)

    env = gym.make('Turtlebot4Env-v0', world_name='static_world')

    try:
        while input('Reset (y/n): ').lower() == 'y':
            obs, info = env.reset(options={'debug': True})
            min_ranges = obs.get('min_ranges', np.array([]))
            min_ranges_angles = obs.get('min_ranges_angle', np.array([]))

            print('orient_to_goal:', obs.get('orient_to_goal'), '\nInfo: ', info)
            print('Min range:', min(min_ranges))
            print('Min range angle:', min_ranges_angles[np.argmin(min_ranges)])
    except KeyboardInterrupt:
        pass
    finally:
        env.close()


if __name__ == '__main__':
    main()
