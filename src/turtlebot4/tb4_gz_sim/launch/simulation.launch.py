# Copyright (C) 2023 Open Source Robotics Foundation
# Copyright (C) 2023 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    sim_dir = get_package_share_directory('tb4_gz_sim')
    desc_dir = get_package_share_directory('tb4_description')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    launch_dir = os.path.join(sim_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    pose = {
        'x': LaunchConfiguration('x_pose', default='0.00'),
        'y': LaunchConfiguration('y_pose', default='0.00'),
        'z': LaunchConfiguration('z_pose', default='0.01'),
        'R': LaunchConfiguration('roll', default='0.00'),
        'P': LaunchConfiguration('pitch', default='0.00'),
        'Y': LaunchConfiguration('yaw', default='0.00'),
    }
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    slam_params_file = LaunchConfiguration('slam_params_file')
    do_slam = LaunchConfiguration('do_slam')
    do_localization = LaunchConfiguration('do_localization')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(desc_dir, 'rviz', 'config_drl.rviz'),
        description='Full path to the RVIZ config file to use',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start rviz',
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator',
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run the simulation in headless mode.',
        choices=['True', 'False'],
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher',
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_dir, 'worlds', 'static_world.sdf'),
        description='Full path to world model file to load',
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='turtlebot4', description='name of the robot'
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(desc_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro'),
        description='Full path to robot sdf file to spawn the robot in gazebo',
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(sim_dir, 'configs', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_do_slam_cmd = DeclareLaunchArgument(
        'do_slam',
        default_value='false',
        description='Option to do mapping.',
        choices=['true', 'false'],
    )

    declare_do_localization_cmd = DeclareLaunchArgument(
        'do_localization',
        default_value='true',
        description='Option to do localization using the robot_localization pkg.',
        choices=['true', 'false'],
    )

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             'robot_description': Command(['xacro', ' ', robot_sdf])}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
    )

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
    )

    gz_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_world': world,
                          'gz_headless_mode': headless}.items()
    )

    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_tb4.launch.py')
        ),
        launch_arguments={'namespace': namespace,
                          'use_simulator': use_simulator,
                          'use_sim_time': use_sim_time,
                          'robot_name': robot_name,
                          'x_pose': pose['x'],
                          'y_pose': pose['y'],
                          'z_pose': pose['z'],
                          'roll': pose['R'],
                          'pitch': pose['P'],
                          'yaw': pose['Y']}.items()
    )

    ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bridge.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': slam_params_file}.items(),
        condition=IfCondition(do_slam),
    )

    robot_localization_node = Node(
        condition=IfCondition(do_localization),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(sim_dir, 'configs', 'ekf.yaml'), {'use_sim_time': use_sim_time}]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_do_localization_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_do_slam_cmd)
    ld.add_action(gz_world)
    ld.add_action(robot_localization_node)
    ld.add_action(gz_robot)
    ld.add_action(ros_gz_bridge)
    ld.add_action(slam_toolbox_launch)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)

    return ld
