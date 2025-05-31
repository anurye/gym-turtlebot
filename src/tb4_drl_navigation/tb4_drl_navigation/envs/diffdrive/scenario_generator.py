import argparse
import logging
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np
from sklearn.neighbors import KDTree
import yaml


class ScenarioGenerator:
    """
    Generate randomized navigation scenarios from a static occupancy map.

    How it works:

      1. Load a occupancy map image and its YAML metadata (resolution, origin,
         thresholds).
      2. Process the map into “buffered free space” by eroding obstacles
         according to robot radius and clearance.
      3. Build a KD-tree of valid free cells.
      4. Provide `generate_start_goal()` to sample start/goal pairs subject to
         a minimum separation constraint and optional distance bias.
      5. Provide `generate_obstacles()` to place N obstacles at valid locations
         while respecting clearance from start/goal.

    Parameters
    ----------
    map_path : str
        Path to the occupancy map image.
    yaml_path : str
        Path to the corresponding YAML map metadata file.
    robot_radius : float, optional
        Robot's circular radius [m] used for map buffering (default: 0.3).
    min_separation : float, optional
        Minimum start-to-goal distance [m] (default: 2.0).
    obstacle_clearance : float, optional
        Clearance [m] required around any obstacle (default: 1.5).

    Examples
    --------
    >>> gen = ScenarioGenerator(
    ...     map_path='maps/static_world.pgm',
    ...     yaml_path='maps/static_world.yaml',
    ...     robot_radius=0.4,
    ...     min_separation=4.0,
    ...     obstacle_clearance=2.0
    ... )
    >>> start, goal = gen.generate_start_goal(
    ...     max_attempts=50, goal_sampling_bias='far', eps=1e-4
    ... )
    >>> obstacles = gen.generate_obstacles(
    ...     num_obstacles=10, start_pos=start, goal_pos=goal
    ... )

    """

    def __init__(
            self,
            map_path: Path,
            yaml_path: Path,
            robot_radius: float = 0.3,
            min_separation: float = 2.0,
            obstacle_clearance: float = 1.5,
            seed: Optional[int] = None,
    ):
        self.logger = logging.getLogger(self.__class__.__name__)
        self._validate_inputs(
            map_path, yaml_path, robot_radius, min_separation, obstacle_clearance
        )

        self.map_path = map_path
        self.yaml_path = yaml_path
        self.robot_radius = robot_radius
        self.min_separation = min_separation
        self.obstacle_clearance = obstacle_clearance

        self.seed = seed
        self._rng = np.random.default_rng(seed=self.seed)

        self.metadata = self._load_metadata()
        self.origin = self._parse_origin()
        self.map_img, self.processed_map = self._process_map()
        self.free_cells = self._get_free_cells()
        self.kdtree = KDTree(self.free_cells)

    @staticmethod
    def _validate_inputs(
        map_path: Path,
        yaml_path: Path,
        robot_radius: float,
        min_sep: float,
        obs_clear: float
    ) -> None:

        if not map_path.exists():
            raise FileNotFoundError(f'Map file {map_path} not found')
        if not yaml_path.exists():
            raise FileNotFoundError(f'Map metadata file {yaml_path} not found')
        if any(val <= 0 for val in [robot_radius, min_sep, obs_clear]):
            raise ValueError('All clearance/distance parameters must be positive')

    def _load_metadata(self) -> dict:
        """Load and validate map metadata."""
        with open(self.yaml_path) as f:
            metadata = yaml.safe_load(f)

        required_keys = {'resolution', 'origin', 'occupied_thresh', 'free_thresh'}
        if not required_keys.issubset(metadata):
            missing = required_keys - metadata.keys()
            raise ValueError(f'Missing required YAML keys: {missing}')

        return metadata

    def _parse_origin(self) -> Tuple[float, float, float]:
        """Convert origin to (x, y, yaw)."""
        # TODO: is there a slam pkg that provide origin as pos + q?
        origin = self.metadata['origin']
        if len(origin) == 3:
            return (origin[0], origin[1], origin[2])
        else:
            raise ValueError('Invalid origin format')

    def _process_map(self) -> Tuple[np.ndarray, np.ndarray]:
        """Process map image with morphological operations."""
        # Load map image
        map_img = cv2.imread(str(self.map_path), cv2.IMREAD_GRAYSCALE)
        if map_img is None:
            raise FileNotFoundError(f'Failed to load map image from {self.map_path}')

        # Apply thresholds to identify free space
        free_thresh = self.metadata['free_thresh']
        free_thresh_pixel = (1.0 - free_thresh) * 255
        free_mask = map_img > free_thresh_pixel

        # Check if any free cells are present
        if not np.any(free_mask):
            raise ValueError('No free cells found in the map.')

        # Convert to uint8 (0 and 255)
        free_mask_uint8 = np.where(free_mask, 255, 0).astype(np.uint8)

        # Calculate buffer in pixels
        resolution = self.metadata['resolution']
        buffer_distance = max(self.robot_radius, self.obstacle_clearance)
        buffer_pixels = int(np.ceil(buffer_distance / resolution))

        if buffer_pixels <= 0:
            self.logger.warning('Buffer pixels is zero. No erosion applied.')
            processed_map = free_mask.astype(int)
        else:
            # Create circular kernel for erosion
            kernel_size = buffer_pixels + 1 if buffer_pixels % 2 == 0 else buffer_pixels
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
            )

            # Erode the free areas to create buffer
            eroded_free = cv2.erode(free_mask_uint8, kernel, iterations=1)
            processed_map = (eroded_free == 255).astype(int)

        # Validate processed map
        if np.sum(processed_map) == 0:
            raise ValueError(
                'Processed map has no navigable areas. '
                'Check robot_radius and obstacle_clearance parameters.'
            )

        return map_img, processed_map

    def _get_free_cells(self) -> np.ndarray:
        """Get array of valid (row, col) positions."""
        return np.column_stack(np.where(self.processed_map == 1))

    def generate_start_goal(
            self,
            max_attempts: int = 100,
            goal_sampling_bias: str = 'uniform',
            eps: float = 1e-5,
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """Generate valid start and goal positions."""
        min_px_dist = self.min_separation / self.metadata['resolution']

        for _ in range(max_attempts):
            start_idx = self._rng.choice(len(self.free_cells))
            start_cell = self.free_cells[start_idx]

            distances, indices = self.kdtree.query(
                [start_cell], k=len(self.free_cells), return_distance=True
            )
            distances = distances.squeeze()
            indices = indices.squeeze()

            valid_mask = distances >= min_px_dist
            valid_indices = indices[valid_mask]
            valid_distances = distances[valid_mask]
            if not valid_indices.size > 0:
                continue

            if goal_sampling_bias == 'uniform':
                weights = np.ones_like(valid_distances)
            elif goal_sampling_bias == 'close':
                weights = 1.0 / (valid_distances + eps)
            elif goal_sampling_bias == 'far':
                weights = valid_distances
            else:
                raise ValueError(f'Invalid goal_sampling_bias: {goal_sampling_bias}')

            weights /= weights.sum()

            goal_cell = self.free_cells[self._rng.choice(valid_indices, p=weights)]
            return (
                self.map_to_world(start_cell),
                self.map_to_world(goal_cell),
            )

        raise RuntimeError(
            f'Failed to find valid start-goal pair in {max_attempts} attempts'
        )

    def generate_obstacles(
            self,
            num_obstacles: int,
            start_pos: Tuple[float, float],
            goal_pos: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        """Generate obstacle positions with clearance requirements."""
        resolution = self.metadata['resolution']
        min_px_clear = self.obstacle_clearance / resolution

        # Convert positions to map cells
        start_cell = self.world_to_map(start_pos)
        goal_cell = self.world_to_map(goal_pos)

        # Filter candidate cells
        start_distances = np.linalg.norm(self.free_cells - start_cell, axis=1)
        goal_distances = np.linalg.norm(self.free_cells - goal_cell, axis=1)
        valid_mask = (start_distances > min_px_clear) & (goal_distances > min_px_clear)
        candidates = self.free_cells[valid_mask]

        if len(candidates) < num_obstacles:
            self.logger.warning(
                f'Requested {num_obstacles} obstacles, only {len(candidates)} valid positions. '
                f'Placing {len(candidates)} obstacles.'
            )

        # KDTree for clearance checking
        kdtree = KDTree(candidates)
        obstacle_positions = []
        remaining_indices = np.arange(len(candidates))

        for _ in range(min(num_obstacles, len(candidates))):
            if len(remaining_indices) == 0:
                break
            # Randomly select from remaining candidates
            idx = self._rng.choice(remaining_indices)
            selected = candidates[idx]
            obstacle_positions.append(selected)

            # Remove nearby cells from consideration
            neighbors = kdtree.query_radius([selected], r=min_px_clear)
            remaining_indices = np.setdiff1d(remaining_indices, neighbors[0])

        return [self.map_to_world(cell) for cell in obstacle_positions]

    def map_to_world(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        """Convert map cell (row, col) to world coordinates (x, y)."""
        resolution = self.metadata['resolution']
        x_map = cell[1] * resolution  # Column to X
        y_map = cell[0] * resolution  # Row to Y

        # Apply origin transformation
        x = self.origin[0] + x_map * np.cos(self.origin[2]) - y_map * np.sin(self.origin[2])
        y = self.origin[1] + x_map * np.sin(self.origin[2]) + y_map * np.cos(self.origin[2])

        return x, y

    def world_to_map(self, pos: Tuple[float, float]) -> Tuple[int, int]:
        """Convert world coordinates (x, y) to map cell (row, col)."""
        resolution = self.metadata['resolution']
        dx = pos[0] - self.origin[0]
        dy = pos[1] - self.origin[1]

        # Inverse rotation
        rot_x = dx * np.cos(-self.origin[2]) - dy * np.sin(-self.origin[2])
        rot_y = dx * np.sin(-self.origin[2]) + dy * np.cos(-self.origin[2])

        col = int(rot_x / resolution)
        row = int(rot_y / resolution)

        return row, col

    def plot_debug(
            self,
            start_pos: Optional[Tuple[float, float]] = None,
            goal_pos: Optional[Tuple[float, float]] = None,
            obstacles: Optional[List[Tuple[float, float]]] = None
    ) -> None:
        """Visualize map with positions."""
        # Set up fonts
        plt.rcParams.update({
            'font.family': 'sans-serif',
            'font.sans-serif': ['DejaVu Sans', 'Liberation Sans', 'Arial'],
            'mathtext.fontset': 'cm',
            'font.size': 12,
            'axes.labelsize': 14,
            'axes.titlesize': 16,
            'legend.fontsize': 12,
            'xtick.labelsize': 12,
            'ytick.labelsize': 12,
            'text.usetex': False
        })

        plt.figure(figsize=(12, 12))

        # Extent
        extent = self._get_map_extent()
        # Original map background
        plt.imshow(
            self.map_img,
            cmap='binary',
            extent=extent,
            alpha=0.6,
            zorder=1,
        )

        # Processed free space overlay
        free_mask = np.ma.masked_where(self.processed_map == 0, self.processed_map)
        plt.imshow(
            free_mask,
            cmap='Greens',
            alpha=0.3,
            extent=extent,
            zorder=2,
        )

        # Clearance Zones
        clearance_kwargs = {
            'fill': True,
            'alpha': 0.2,
            'linestyle': '--',
            'linewidth': 2,
        }

        # Start clearance
        if start_pos:
            plt.gca().add_patch(
                plt.Circle(
                    start_pos,
                    self.obstacle_clearance,
                    color='red',
                    label='Start Clearance',
                    **clearance_kwargs,
                )
            )

        # Goal clearance
        if goal_pos:
            plt.gca().add_patch(
                plt.Circle(
                    goal_pos,
                    self.obstacle_clearance,
                    color='blue',
                    label='Goal Clearance',
                    **clearance_kwargs,
                )
            )

        # Obstacle clearances
        if obstacles:
            for idx, obstacle in enumerate(obstacles):
                plt.gca().add_patch(
                    plt.Circle(
                        obstacle,
                        self.obstacle_clearance,
                        color='orange',
                        label='Obstacle Clearance' if idx == 0 else None,
                        **clearance_kwargs,
                    )
                )

        # Position Markers
        marker_style = {
            's': 300,
            'edgecolors': 'black',
            'linewidths': 2,
            'zorder': 4,
        }

        if start_pos:
            plt.scatter(
                *start_pos,
                marker='D',
                c='lime',
                label='Start Position',
                **marker_style,
            )

        if goal_pos:
            plt.scatter(
                *goal_pos,
                marker='*',
                c='gold',
                label='Goal Position',
                **marker_style,
            )

        if obstacles:
            obs_x, obs_y = zip(*obstacles) if obstacles else ([], [])
            plt.scatter(
                obs_x,
                obs_y,
                marker='X',
                c='darkred',
                s=150,
                edgecolors='white',
                label='Obstacles',
                zorder=3,
            )

        # Add some final touches
        # plt.colorbar(label="Map Intensity", fraction=0.03, pad=0.01)
        plt.xlabel('World X [m]', fontsize=12)
        plt.ylabel('World Y [m]', fontsize=12)
        plt.title('Navigation Scenario Debug View', fontsize=14, pad=20)

        # Configure grid and background
        plt.grid(True, color='white', alpha=0.3, linestyle='--')
        plt.gca().set_facecolor('lightcyan')

        # Legend handling
        handles, labels = plt.gca().get_legend_handles_labels()
        unique_labels = dict(zip(labels, handles))
        plt.legend(
            unique_labels.values(),
            unique_labels.keys(),
            loc='upper left',
            bbox_to_anchor=(1.01, 1.0),
            fontsize=10,
            framealpha=0.95,
            facecolor='white',
            edgecolor='black',
            borderpad=1.5,
            title=r'Legend:',
            title_fontsize=11,
            handletextpad=2.0,
            handlelength=2.0,
            labelspacing=1.25,
            markerscale=0.8
        )
        # plt.subplots_adjust(right=0.78)

        plt.tight_layout()
        plt.show()

    def _get_map_extent(self) -> List[float]:
        """Calculate world coordinate extent."""
        height, width = self.map_img.shape
        bl = self.map_to_world((height - 1, 0))  # Bottom-left
        tr = self.map_to_world((0, width - 1))   # Top-right
        return [bl[0], tr[0], bl[1], tr[1]]


def main():
    logging.basicConfig(level=logging.INFO)

    current_dir = Path(__file__).parent
    default_map = current_dir / 'maps' / 'static_world.pgm'
    default_yaml = current_dir / 'maps' / 'static_world.yaml'

    parser = argparse.ArgumentParser(
        description='Load and process a static world map and its metadata YAML.'
    )
    parser.add_argument(
        '-m', '--map',
        type=Path,
        default=default_map,
        help=f'Path to the PGM map file (default: {default_map})'
    )
    parser.add_argument(
        '-y', '--yaml',
        type=Path,
        default=default_yaml,
        help=f'Path to the YAML metadata file (default: {default_yaml})'
    )

    args = parser.parse_args()

    try:
        nav_scenario = ScenarioGenerator(
            map_path=args.map,
            yaml_path=args.yaml,
            robot_radius=0.4,
            min_separation=2.0,
            obstacle_clearance=1.0,
            seed=None,
        )

        start, goal = nav_scenario.generate_start_goal(
            max_attempts=100,
            goal_sampling_bias='uniform'
        )
        obstacles = nav_scenario.generate_obstacles(
            num_obstacles=12, start_pos=start, goal_pos=goal
        )

        nav_scenario.plot_debug(
            start_pos=start, goal_pos=goal, obstacles=obstacles
        )

    except Exception as e:
        logging.error(f'Error: {e}')
        raise

    nav_scenario.logger.info(f'Start: {start}')
    nav_scenario.logger.info(f'Goal: {goal}')
    nav_scenario.logger.info(f'Obstacles: {obstacles}')


if __name__ == '__main__':
    main()
