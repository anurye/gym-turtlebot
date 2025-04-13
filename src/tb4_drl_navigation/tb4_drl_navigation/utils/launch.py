"""
Utility functions for executing ros2 launch files.

Author: Ahmed Yesuf Nurye
Date: 2025-04-12
"""

import os
import shlex
import subprocess


class Launcher:
    def __init__(self, workspace_dir):
        self.workspace_dir = os.path.expanduser(workspace_dir)
        if not os.path.isdir(self.workspace_dir):
            raise ValueError(f'Workspace directory {self.workspace_dir} does not exist.')

    @classmethod
    def find_workspace(cls, start_dir=None):
        """Locate the ROS2 workspace directory by searching upwards from start_dir."""
        if start_dir is None:
            start_dir = os.getcwd()
        current_dir = os.path.abspath(start_dir)
        while True:
            src_dir = os.path.join(current_dir, 'src')
            # install_dir = os.path.join(current_dir, 'install')
            # build_dir = os.path.join(current_dir, 'build')
            if os.path.isdir(src_dir):
                return current_dir
            parent_dir = os.path.dirname(current_dir)
            if parent_dir == current_dir:
                raise FileNotFoundError('Could not find ROS2 workspace directory.')
            current_dir = parent_dir

    def build(self) -> None:
        """Build the ROS2 workspace."""
        try:
            subprocess.run(
                ['./build.sh'],
                cwd=self.workspace_dir,
                check=True
            )
        except subprocess.CalledProcessError as e:
            print(f'Failed to build: {str(e)}')
            raise

    def launch(self, package_name, launch_file, *launch_args, build_first=False):
        """Execute a ROS2 launch file."""
        if build_first:
            self.build()

        # Prepare launch command
        source_cmd = 'source install/setup.bash'
        launch_cmd = [
            'ros2', 'launch',
            shlex.quote(package_name),
            shlex.quote(launch_file)
        ] + [shlex.quote(arg) for arg in launch_args]
        launch_cmd_str = ' '.join(launch_cmd)
        full_cmd = f'{source_cmd} && {launch_cmd_str}'
        # print('Full launch cmd:', full_cmd)
        try:
            subprocess.run(
                full_cmd,
                shell=True,
                executable='/bin/bash',
                cwd=self.workspace_dir,
                check=True
            )
        except subprocess.CalledProcessError as e:
            print(f'Failed to launch: {e}')
            raise


if __name__ == '__main__':
    try:
        workspace = Launcher.find_workspace()
        launcher = Launcher(workspace)
        launcher.launch('tb4_gz_sim', 'simulation.launch.py')
    except Exception as e:
        print(f'Error: {e}')
