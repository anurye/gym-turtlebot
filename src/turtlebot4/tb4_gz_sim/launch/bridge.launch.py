# Copyright 2025 Ahmed Yesuf Nurye.
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    tb4_drl_pkg_share = get_package_share_directory('tb4_drl_navigation')
    sim_dir = get_package_share_directory('tb4_gz_sim')

    # world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_world_name = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(tb4_drl_pkg_share, 'worlds', 'static_world.sdf'),
        description='world model file name path',
    )
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    # Topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_ros_gz',
        namespace=namespace,
        parameters=[
            {
                'config_file': os.path.join(
                    sim_dir, 'configs', 'tb4_bridge.yaml'
                ),
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
    )

    camera_bridge_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_image',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=['/rgbd_camera/image'])

    camera_bridge_depth = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='bridge_gz_ros_camera_depth',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=['/rgbd_camera/depth_image'])

    # Services
    world_ctrl_srv = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            PythonExpression([
                "'/world/' + ('",
                LaunchConfiguration('world'),
                "'.split('/')[-1].split('.')[0]) + "
                "'/control@ros_gz_interfaces/srv/ControlWorld'"
            ])
        ],
        output='screen'
    )

    spawn_entity_srv = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            PythonExpression([
                "'/world/' + ('",
                LaunchConfiguration('world'),
                "'.split('/')[-1].split('.')[0]) + "
                "'/create@ros_gz_interfaces/srv/SpawnEntity'"
            ])
        ],
        output='screen'
    )

    set_entity_pose = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            PythonExpression([
                "'/world/' + ('",
                LaunchConfiguration('world'),
                "'.split('/')[-1].split('.')[0]) + "
                "'/set_pose@ros_gz_interfaces/srv/SetEntityPose'"
            ])
        ],
        output='screen'
    )

    delete_entity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            PythonExpression([
                "'/world/' + ('",
                LaunchConfiguration('world'),
                "'.split('/')[-1].split('.')[0]) + "
                "'/remove@ros_gz_interfaces/srv/DeleteEntity'"
            ])
        ],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_name)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(bridge)
    ld.add_action(camera_bridge_image)
    ld.add_action(camera_bridge_depth)
    ld.add_action(world_ctrl_srv)
    ld.add_action(spawn_entity_srv)
    ld.add_action(set_entity_pose)
    ld.add_action(delete_entity)

    return ld
