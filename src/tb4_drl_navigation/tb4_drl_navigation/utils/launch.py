from pathlib import Path
import shlex
import subprocess
from typing import Optional


class Launcher:
    """Utility class for executing ros2 launch files."""

    workspace_dir: Path

    def __init__(self, workspace_dir: Path):
        expanded_dir = workspace_dir.expanduser()
        if not expanded_dir.is_dir():
            raise ValueError(f'Workspace directory {expanded_dir} does not exist.')
        self.workspace_dir = expanded_dir

    @classmethod
    def find_workspace(cls, start_dir: Optional[Path] = None):
        """Locate the ROS2 workspace directory by searching upwards from start_dir."""
        if start_dir is None:
            start_dir = Path(__file__).parent

        current_dir = start_dir.resolve()
        while True:
            candidate_src = current_dir / 'src'
            if candidate_src.is_dir():
                return current_dir

            parent_dir = current_dir.parent
            if parent_dir == current_dir:
                raise FileNotFoundError('Could not find ROS2 workspace directory.')
            current_dir = parent_dir

    def build(self) -> None:
        """Build the ROS2 workspace."""
        build_script = self.workspace_dir / 'build.sh'
        if not build_script.is_file():
            raise FileNotFoundError(f'Unable to locate build script at: {build_script}')

        try:
            subprocess.run(
                [str(build_script)],
                cwd=self.workspace_dir,
                check=True
            )
        except subprocess.CalledProcessError as e:
            print(f'Failed to build: {str(e)}')
            raise

    def launch(
            self,
            package_name: str,
            launch_file: str,
            *launch_args: str,
            build_first: bool = False
    ):
        """Execute a ROS2 launch file."""
        if build_first:
            self.build()

        # Source workspace install overlay and then execute ros2 launch
        source_command = 'source install/setup.bash'
        quoted_package = shlex.quote(package_name)
        quoted_launch_file = shlex.quote(launch_file)
        quoted_args: list[str] = [shlex.quote(arg) for arg in launch_args]

        ros2_launch_cmd = ['ros2', 'launch', quoted_package, quoted_launch_file] + quoted_args
        full_command = f"{source_command} && {' '.join(ros2_launch_cmd)}"
        print('Full launch cmd:', full_command)

        try:
            subprocess.run(
                full_command,
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
        launcher.launch(
            'tb4_gz_sim', 'simulation.launch.py', 'use_sim_time:=true', build_first=True
        )
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
