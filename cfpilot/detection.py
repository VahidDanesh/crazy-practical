"""
Landing Pad Detection Module

Advanced landing pad detection using z-range sensor and peak detection algorithms.
"""

import math
import numpy as np
import logging
from collections import deque
from typing import List, Tuple, Optional, Dict, Any
from enum import Enum


class CellState(Enum):
    """States for grid map cells"""
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2
    ELEVATED = 3  # Landing pad or platform


class GridMap:
    """
    Grid-based environment discretization for spatial mapping and navigation.
    
    Discretizes the environment into a 2D grid of cells, each storing information
    about occupancy, height, and exploration status.
    """
    
    def __init__(self, bounds: Tuple[float, float, float, float], 
                 resolution: float = 0.1):
        """
        Initialize the grid map.
        
        Args:
            bounds: (min_x, max_x, min_y, max_y) boundaries of the environment
            resolution: Size of each grid cell in meters
        """
        self.logger = logging.getLogger(__name__)
        
        self.min_x, self.max_x, self.min_y, self.max_y = bounds
        self.resolution = resolution
        
        # Calculate grid dimensions
        self.width = int((self.max_x - self.min_x) / resolution) + 1
        self.height = int((self.max_y - self.min_y) / resolution) + 1
        
        # Initialize grid data structures
        self.occupancy_grid = np.full((self.height, self.width), CellState.UNKNOWN.value, dtype=int)
        self.height_grid = np.full((self.height, self.width), np.nan, dtype=float)
        self.confidence_grid = np.zeros((self.height, self.width), dtype=float)
        self.visit_count = np.zeros((self.height, self.width), dtype=int)
        
        self.logger.info(f"GridMap initialized: {self.width}x{self.height} cells, "
                        f"resolution={resolution}m, bounds={bounds}")
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid indices.
        
        Args:
            x, y: World coordinates in meters
            
        Returns:
            (row, col) grid indices
        """
        col = int((x - self.min_x) / self.resolution)
        row = int((y - self.min_y) / self.resolution)
        
        # Clamp to grid boundaries
        col = max(0, min(col, self.width - 1))
        row = max(0, min(row, self.height - 1))
        
        return row, col
    
    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """
        Convert grid indices to world coordinates (cell center).
        
        Args:
            row, col: Grid indices
            
        Returns:
            (x, y) world coordinates in meters
        """
        x = self.min_x + (col + 0.5) * self.resolution
        y = self.min_y + (row + 0.5) * self.resolution
        return x, y
    
    def update_cell(self, x: float, y: float, height: float, 
                   state: CellState = CellState.FREE) -> None:
        """
        Update a grid cell with new measurements.
        
        Args:
            x, y: World coordinates
            height: Measured height at this location
            state: Cell occupancy state
        """
        row, col = self.world_to_grid(x, y)
        
        # Update visit count
        self.visit_count[row, col] += 1
        
        # Update height with running average
        if np.isnan(self.height_grid[row, col]):
            self.height_grid[row, col] = height
        else:
            # Running average with more weight to recent measurements
            alpha = 0.3
            self.height_grid[row, col] = (1 - alpha) * self.height_grid[row, col] + alpha * height
        
        # Update occupancy state
        self.occupancy_grid[row, col] = state.value
        
        # Update confidence based on visit count
        self.confidence_grid[row, col] = min(1.0, self.visit_count[row, col] / 5.0)
    
    def get_cell_info(self, x: float, y: float) -> Dict[str, Any]:
        """
        Get information about a cell at world coordinates.
        
        Args:
            x, y: World coordinates
            
        Returns:
            Dictionary with cell information
        """
        row, col = self.world_to_grid(x, y)
        
        return {
            'position': (x, y),
            'grid_indices': (row, col),
            'state': CellState(self.occupancy_grid[row, col]),
            'height': self.height_grid[row, col],
            'confidence': self.confidence_grid[row, col],
            'visit_count': self.visit_count[row, col]
        }
    
    def find_free_cells(self, min_confidence: float = 0.5) -> List[Tuple[float, float]]:
        """
        Find all free cells with sufficient confidence.
        
        Args:
            min_confidence: Minimum confidence threshold
            
        Returns:
            List of (x, y) world coordinates of free cells
        """
        free_cells = []
        
        for row in range(self.height):
            for col in range(self.width):
                if (self.occupancy_grid[row, col] == CellState.FREE.value and
                    self.confidence_grid[row, col] >= min_confidence):
                    x, y = self.grid_to_world(row, col)
                    free_cells.append((x, y))
        
        return free_cells
    
    def find_unexplored_cells(self) -> List[Tuple[float, float]]:
        """
        Find cells that haven't been explored yet.
        
        Returns:
            List of (x, y) world coordinates of unexplored cells
        """
        unexplored = []
        
        for row in range(self.height):
            for col in range(self.width):
                if self.occupancy_grid[row, col] == CellState.UNKNOWN.value:
                    x, y = self.grid_to_world(row, col)
                    unexplored.append((x, y))
        
        return unexplored
    
    def find_elevated_regions(self, height_threshold: float = 0.05) -> List[Dict[str, Any]]:
        """
        Find regions with elevated surfaces (potential landing pads).
        
        Args:
            height_threshold: Minimum height difference to consider elevated
            
        Returns:
            List of elevated region information
        """
        elevated_regions = []
        
        # Calculate median height of known cells for baseline
        known_heights = self.height_grid[~np.isnan(self.height_grid)]
        if len(known_heights) < 3:
            return elevated_regions
        
        baseline_height = np.median(known_heights)
        
        for row in range(self.height):
            for col in range(self.width):
                height = self.height_grid[row, col]
                if (not np.isnan(height) and 
                    height > baseline_height + height_threshold and
                    self.confidence_grid[row, col] > 0.3):
                    
                    x, y = self.grid_to_world(row, col)
                    elevated_regions.append({
                        'position': (x, y),
                        'height': height,
                        'height_diff': height - baseline_height,
                        'confidence': self.confidence_grid[row, col]
                    })
        
        return elevated_regions
    
    def get_exploration_progress(self) -> Dict[str, Any]:
        """
        Get statistics about exploration progress.
        
        Returns:
            Dictionary with exploration statistics
        """
        total_cells = self.width * self.height
        explored_cells = np.sum(self.occupancy_grid != CellState.UNKNOWN.value)
        free_cells = np.sum(self.occupancy_grid == CellState.FREE.value)
        occupied_cells = np.sum(self.occupancy_grid == CellState.OCCUPIED.value)
        elevated_cells = np.sum(self.occupancy_grid == CellState.ELEVATED.value)
        
        return {
            'total_cells': total_cells,
            'explored_cells': int(explored_cells),
            'exploration_percentage': float(explored_cells / total_cells * 100),
            'free_cells': int(free_cells),
            'occupied_cells': int(occupied_cells),
            'elevated_cells': int(elevated_cells),
            'mean_confidence': float(np.mean(self.confidence_grid))
        }


class LandingPadDetector:
    """Detects landing pads using z-range sensor height measurements"""
    
    def __init__(self, grid_bounds: Optional[Tuple[float, float, float, float]] = None,
                 grid_resolution: float = 0.1):
        self.logger = logging.getLogger(__name__)
        
        # Detection parameters (configured via config)
        self.lag = 8                    
        self.threshold = 1.5            
        self.influence = 0.2            
        self.min_peak_height = 0.03     
        
        # Data storage
        self.height_data = deque(maxlen=1000)
        self.position_data = deque(maxlen=1000)
        self.peak_positions = []
        self.flight_direction = None
        
        # Detection state
        self.detection_active = False
        self.baseline_height = None
        self.running_mean = deque(maxlen=self.lag)
        self.running_std = deque(maxlen=self.lag)
        
        # Results
        self.calculated_center = None
        self.center_confidence = 0.0
        
        # Grid map integration
        if grid_bounds:
            self.grid_map = GridMap(grid_bounds, grid_resolution)
        else:
            # Default bounds - can be updated later
            self.grid_map = GridMap((-2.0, 2.0, -2.0, 2.0), grid_resolution)
        
        self.logger.info("Landing pad detector initialized with grid map")
    
    def configure_detection(self, params: Dict[str, Any]) -> None:
        """Configure detection parameters from config"""
        self.lag = params.get('lag', 8)
        self.threshold = params.get('threshold', 1.5)
        self.influence = params.get('influence', 0.2)
        self.min_peak_height = params.get('min_peak_height', 0.03)
        
        # Reset running statistics
        self.running_mean = deque(maxlen=self.lag)
        self.running_std = deque(maxlen=self.lag)
        
        self.logger.info(f"Detection configured: lag={self.lag}, threshold={self.threshold}, "
                        f"influence={self.influence}, min_height={self.min_peak_height}")
    
    def start_detection(self) -> None:
        """Start the landing pad detection process"""
        self.detection_active = True
        self.height_data.clear()
        self.position_data.clear()
        self.peak_positions.clear()
        self.baseline_height = None
        self.calculated_center = None
        self.center_confidence = 0.0
        
        self.logger.info("Landing pad detection started")
    
    def stop_detection(self) -> None:
        """Stop the detection process"""
        self.detection_active = False
        self.logger.info(f"Detection stopped. Found {len(self.peak_positions)} border points")
    
    def set_flight_direction(self, direction: str) -> None:
        """Set current flight direction for border classification"""
        self.flight_direction = direction
    
    def process_height_measurement(self, height: float, position: Tuple[float, float]) -> None:
        """
        Process a height measurement for landing pad detection.
        
        Args:
            height: Height measurement from z-range sensor (meters)
            position: Current (x, y) position of the drone
        """
        if not self.detection_active:
            return
        
        # Store data
        self.height_data.append(height)
        self.position_data.append(position)
        
        # Update grid map with measurement
        self.grid_map.update_cell(position[0], position[1], height, CellState.FREE)
        
        # Initialize baseline if not set
        if self.baseline_height is None and len(self.height_data) >= 5:
            self.baseline_height = np.median(list(self.height_data)[-5:])
            self.logger.info(f"Baseline height established: {self.baseline_height:.3f}m")
        
        # Need sufficient data for peak detection
        if len(self.height_data) < self.lag + 1:
            return
        
        # Apply z-score peak detection algorithm
        self._detect_peaks(height, position)
    
    def _detect_peaks(self, current_height: float, position: Tuple[float, float]) -> None:
        """Apply z-score based peak detection algorithm"""
        
        # Get recent data for running statistics
        recent_heights = list(self.height_data)[-self.lag:]
        
        # Update running mean and std
        if len(self.running_mean) == 0:
            # Initialize with recent data
            self.running_mean = deque(recent_heights, maxlen=self.lag)
            self.running_std = deque([np.std(recent_heights)] * len(recent_heights), maxlen=self.lag)
            return
        
        # Calculate current mean and std
        current_mean = np.mean(self.running_mean)
        current_std = np.mean(self.running_std) if self.running_std else 0.001
        
        # Z-score for peak detection
        if current_std > 0:
            z_score = abs(current_height - current_mean) / current_std
        else:
            z_score = 0
        
        # Check for significant peak (platform edge)
        if z_score > self.threshold:
            height_diff = abs(current_height - self.baseline_height) if self.baseline_height else 0
            
            # Verify it's a significant height change
            if height_diff > self.min_peak_height:
                # This looks like a platform edge
                self.peak_positions.append({
                    'position': position,
                    'height': current_height,
                    'height_diff': height_diff,
                    'direction': self.flight_direction,
                    'z_score': z_score
                })
                
                # Mark as elevated in grid map
                self.grid_map.update_cell(position[0], position[1], current_height, CellState.ELEVATED)
                
                self.logger.info(f"Platform edge detected at ({position[0]:.3f}, {position[1]:.3f}) "
                               f"height_diff={height_diff:.3f}m, z_score={z_score:.2f}")
                
                # Update running statistics with reduced influence
                influenced_value = self.influence * current_height + (1 - self.influence) * current_mean
                self.running_mean.append(influenced_value)
            else:
                # Normal update
                self.running_mean.append(current_height)
        else:
            # Normal update
            self.running_mean.append(current_height)
        
        # Update running standard deviation
        new_std = np.std(self.running_mean)
        self.running_std.append(new_std)
    
    def calculate_pad_center(self) -> Optional[Tuple[float, float]]:
        """
        Calculate landing pad center from detected border points.
        
        Returns:
            (x, y) center coordinates or None if insufficient data
        """
        if len(self.peak_positions) < 4:
            self.logger.warning(f"Insufficient border points for center calculation: {len(self.peak_positions)}")
            return None
        
        # Extract positions
        positions = [p['position'] for p in self.peak_positions]
        x_coords = [p[0] for p in positions]
        y_coords = [p[1] for p in positions]
        
        # Simple center calculation (centroid)
        center_x = np.mean(x_coords)
        center_y = np.mean(y_coords)
        
        # Calculate confidence based on point distribution
        distances = [math.sqrt((x - center_x)**2 + (y - center_y)**2) for x, y in positions]
        std_distance = np.std(distances)
        mean_distance = np.mean(distances)
        
        # Confidence: lower std relative to mean = higher confidence
        if mean_distance > 0:
            self.center_confidence = max(0, 1 - (std_distance / mean_distance))
        else:
            self.center_confidence = 0.0
        
        self.calculated_center = (center_x, center_y)
        
        self.logger.info(f"Calculated pad center: ({center_x:.3f}, {center_y:.3f}) "
                        f"confidence: {self.center_confidence:.2f}")
        
        return self.calculated_center
    
    def is_ready_for_landing(self) -> bool:
        """Check if we have sufficient data for confident landing"""
        if len(self.peak_positions) < 6:
            return False
        
        if self.calculated_center is None:
            return False
        
        # Check confidence threshold
        return self.center_confidence > 0.6
    
    def get_detection_statistics(self) -> Dict[str, Any]:
        """Get detection statistics for debugging"""
        stats = {
            'total_measurements': len(self.height_data),
            'total_border_points': len(self.peak_positions),
            'baseline_height': self.baseline_height,
            'calculated_center': self.calculated_center,
            'center_confidence': self.center_confidence,
            'detection_active': self.detection_active
        }
        
        # Add grid map statistics
        grid_stats = self.grid_map.get_exploration_progress()
        stats.update({f'grid_{k}': v for k, v in grid_stats.items()})
        
        return stats
    
    def get_grid_map(self) -> GridMap:
        """Get the grid map instance"""
        return self.grid_map
    
    def get_elevated_regions(self) -> List[Dict[str, Any]]:
        """Get elevated regions from grid map"""
        return self.grid_map.find_elevated_regions()
    
    def get_unexplored_areas(self) -> List[Tuple[float, float]]:
        """Get unexplored areas for search planning"""
        return self.grid_map.find_unexplored_cells()
    
    def export_grid_data(self) -> Dict[str, Any]:
        """Export grid map data for visualization or analysis"""
        return {
            'bounds': (self.grid_map.min_x, self.grid_map.max_x, 
                      self.grid_map.min_y, self.grid_map.max_y),
            'resolution': self.grid_map.resolution,
            'dimensions': (self.grid_map.width, self.grid_map.height),
            'occupancy_grid': self.grid_map.occupancy_grid.tolist(),
            'height_grid': np.where(np.isnan(self.grid_map.height_grid), 
                                   -1, self.grid_map.height_grid).tolist(),
            'confidence_grid': self.grid_map.confidence_grid.tolist(),
            'visit_count': self.grid_map.visit_count.tolist(),
            'exploration_progress': self.grid_map.get_exploration_progress()
        }


class SearchPattern:
    """Generates search patterns for systematic area coverage"""
    
    def __init__(self, grid_map: Optional[GridMap] = None):
        self.logger = logging.getLogger(__name__)
        self.grid_map = grid_map
    
    def set_grid_map(self, grid_map: GridMap) -> None:
        """Set the grid map for informed search planning"""
        self.grid_map = grid_map
    
    def generate_grid_pattern(self, center: Tuple[float, float], 
                            grid_size: float = 0.5, 
                            grid_spacing: float = 0.3) -> List[Tuple[float, float, str]]:
        """
        Generate a grid search pattern around a center point.
        
        Args:
            center: (x, y) center of search area
            grid_size: Size of search area (meters)
            grid_spacing: Distance between grid points (meters)
            
        Returns:
            List of (x, y, direction) waypoints
        """
        cx, cy = center
        half_size = grid_size / 2
        
        waypoints = []
        
        # Generate grid points
        x_points = np.arange(cx - half_size, cx + half_size + grid_spacing/2, grid_spacing)
        y_points = np.arange(cy - half_size, cy + half_size + grid_spacing/2, grid_spacing)
        
        # Create zigzag pattern for efficient coverage
        for i, y in enumerate(y_points):
            if i % 2 == 0:  # Even rows: left to right
                for x in x_points:
                    direction = 'forward' if i == 0 else 'right'
                    waypoints.append((x, y, direction))
            else:  # Odd rows: right to left
                for x in reversed(x_points):
                    direction = 'left'
                    waypoints.append((x, y, direction))
        
        self.logger.info(f"Generated {len(waypoints)} waypoints for grid search")
        return waypoints
    
    def generate_adaptive_pattern(self, center: Tuple[float, float], 
                                max_distance: float = 1.0,
                                prioritize_unexplored: bool = True) -> List[Tuple[float, float, str]]:
        """
        Generate an adaptive search pattern based on grid map information.
        
        Args:
            center: (x, y) center of search area
            max_distance: Maximum distance from center
            prioritize_unexplored: Whether to prioritize unexplored areas
            
        Returns:
            List of (x, y, direction) waypoints
        """
        if not self.grid_map:
            self.logger.warning("No grid map available, falling back to regular grid pattern")
            return self.generate_grid_pattern(center, max_distance * 2, 0.3)
        
        waypoints = []
        cx, cy = center
        
        # Get unexplored cells within range
        unexplored = self.grid_map.find_unexplored_cells()
        
        # Filter by distance
        candidates = []
        for x, y in unexplored:
            distance = math.sqrt((x - cx)**2 + (y - cy)**2)
            if distance <= max_distance:
                candidates.append((x, y, distance))
        
        # Sort by distance (closest first) or by priority
        if prioritize_unexplored:
            candidates.sort(key=lambda p: p[2])  # Sort by distance
        
        # Add waypoints with directions
        prev_x, prev_y = center
        for x, y, _ in candidates:
            # Determine direction based on movement
            dx = x - prev_x
            dy = y - prev_y
            
            if abs(dx) > abs(dy):
                direction = 'right' if dx > 0 else 'left'
            else:
                direction = 'forward' if dy > 0 else 'backward'
            
            waypoints.append((x, y, direction))
            prev_x, prev_y = x, y
        
        # If no unexplored areas, fall back to elevated regions
        if not waypoints:
            elevated = self.grid_map.find_elevated_regions()
            for region in elevated[:10]:  # Limit to top 10
                x, y = region['position']
                distance = math.sqrt((x - cx)**2 + (y - cy)**2)
                if distance <= max_distance:
                    waypoints.append((x, y, 'forward'))
        
        # If still no waypoints, generate a basic grid
        if not waypoints:
            waypoints = self.generate_grid_pattern(center, max_distance * 2, 0.3)
        
        self.logger.info(f"Generated {len(waypoints)} adaptive waypoints "
                        f"({len([w for w in waypoints if w[2] != 'forward'])} based on grid map)")
        return waypoints
    
    def generate_spiral_pattern(self, center: Tuple[float, float], 
                              max_radius: float = 0.8, 
                              spacing: float = 0.2) -> List[Tuple[float, float, str]]:
        """
        Generate a spiral search pattern.
        
        Args:
            center: (x, y) center of search area
            max_radius: Maximum spiral radius (meters)
            spacing: Distance between spiral turns (meters)
            
        Returns:
            List of (x, y, direction) waypoints
        """
        cx, cy = center
        waypoints = []
        
        # Start from center
        waypoints.append((cx, cy, 'forward'))
        
        # Generate spiral
        radius = spacing
        while radius <= max_radius:
            # Calculate number of points for this radius
            circumference = 2 * math.pi * radius
            num_points = max(8, int(circumference / (spacing * 0.5)))
            
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                x = cx + radius * math.cos(angle)
                y = cy + radius * math.sin(angle)
                
                direction = 'forward'  # Simplified direction
                waypoints.append((x, y, direction))
            
            radius += spacing
        
        self.logger.info(f"Generated {len(waypoints)} waypoints for spiral search")
        return waypoints
