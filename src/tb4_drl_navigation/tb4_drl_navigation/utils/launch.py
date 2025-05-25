from pathlib import Path
import shlex
import subprocess


class Launcher:
    """Utility class for executing ros2 launch files."""

    def __init__(self, workspace_dir: Path):
        self.workspace_dir = workspace_dir.expanduser()
        if not self.workspace_dir.is_dir():
            raise ValueError(f'Workspace directory {self.workspace_dir} does not exist.')

    @classmethod
    def find_workspace(cls, start_dir=None):
        """Locate the ROS2 workspace directory by searching upwards from start_dir."""
        if start_dir is None:
            start_dir = Path(__file__).parent
        current_dir = start_dir.resolve()
        while True:
            src_dir = current_dir / 'src'
            install_dir = current_dir / 'install'
            build_dir = current_dir / 'build'
            if src_dir.is_dir() and install_dir.is_dir() and build_dir.is_dir():
                return current_dir
            parent_dir = current_dir.parent
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
        launcher.launch('tb4_gz_sim', 'simulation.launch.py', build_first=True)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
