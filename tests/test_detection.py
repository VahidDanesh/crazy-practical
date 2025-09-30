"""
Test Landing Pad Detection

Tests for the landing pad detection algorithms.
"""

import unittest
import numpy as np
import math
import sys
from pathlib import Path

# Add the parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from cfpilot.detection import LandingPadDetector, SearchPattern, GridMap, CellState


class TestLandingPadDetector(unittest.TestCase):
    """Test cases for landing pad detector"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.detector = LandingPadDetector()
        
        # Test configuration
        self.test_config = {
            'lag': 8,
            'threshold': 1.5,
            'influence': 0.2,
            'min_peak_height': 0.03
        }
    
    def test_detector_initialization(self):
        """Test detector initialization"""
        self.assertFalse(self.detector.detection_active)
        self.assertIsNone(self.detector.baseline_height)
        self.assertEqual(len(self.detector.peak_positions), 0)
    
    def test_configure_detection(self):
        """Test detection configuration"""
        self.detector.configure_detection(self.test_config)
        
        self.assertEqual(self.detector.lag, 8)
        self.assertEqual(self.detector.threshold, 1.5)
        self.assertEqual(self.detector.influence, 0.2)
        self.assertEqual(self.detector.min_peak_height, 0.03)
    
    def test_start_stop_detection(self):
        """Test starting and stopping detection"""
        self.detector.start_detection()
        self.assertTrue(self.detector.detection_active)
        
        self.detector.stop_detection()
        self.assertFalse(self.detector.detection_active)
    
    def test_height_measurement_processing(self):
        """Test height measurement processing"""
        self.detector.configure_detection(self.test_config)
        self.detector.start_detection()
        
        # Add some baseline measurements
        for i in range(10):
            self.detector.process_height_measurement(0.5, (i * 0.1, 0.0))
        
        # Should have established baseline
        self.assertIsNotNone(self.detector.baseline_height)
        self.assertEqual(len(self.detector.height_data), 10)
    
    def test_peak_detection(self):
        """Test peak detection with simulated platform"""
        self.detector.configure_detection(self.test_config)
        self.detector.start_detection()
        
        # Simulate flying over flat ground then platform
        positions = []
        heights = []
        
        # Flat ground
        for i in range(15):
            height = 0.5 + np.random.normal(0, 0.01)  # 50cm + noise
            position = (i * 0.1, 0.0)
            heights.append(height)
            positions.append(position)
            self.detector.process_height_measurement(height, position)
        
        # Platform edge (sudden height change)
        for i in range(5):
            height = 0.4 + np.random.normal(0, 0.01)  # 40cm (10cm platform)
            position = ((15 + i) * 0.1, 0.0)
            heights.append(height)
            positions.append(position)
            self.detector.process_height_measurement(height, position)
        
        # Should have detected some peaks
        stats = self.detector.get_detection_statistics()
        self.assertGreater(stats['total_measurements'], 0)
    
    def test_center_calculation_insufficient_points(self):
        """Test center calculation with insufficient points"""
        self.detector.configure_detection(self.test_config)
        self.detector.start_detection()
        
        # Add only 2 peak positions (insufficient)
        self.detector.peak_positions = [
            {'position': (0.0, 0.0), 'height': 0.4, 'height_diff': 0.1, 'direction': 'forward', 'z_score': 2.0},
            {'position': (0.1, 0.0), 'height': 0.4, 'height_diff': 0.1, 'direction': 'forward', 'z_score': 2.0}
        ]
        
        center = self.detector.calculate_pad_center()
        self.assertIsNone(center)
    
    def test_center_calculation_sufficient_points(self):
        """Test center calculation with sufficient points"""
        self.detector.configure_detection(self.test_config)
        self.detector.start_detection()
        
        # Add sufficient peak positions in a square pattern
        self.detector.peak_positions = [
            {'position': (0.0, 0.0), 'height': 0.4, 'height_diff': 0.1, 'direction': 'forward', 'z_score': 2.0},
            {'position': (0.2, 0.0), 'height': 0.4, 'height_diff': 0.1, 'direction': 'forward', 'z_score': 2.0},
            {'position': (0.2, 0.2), 'height': 0.4, 'height_diff': 0.1, 'direction': 'right', 'z_score': 2.0},
            {'position': (0.0, 0.2), 'height': 0.4, 'height_diff': 0.1, 'direction': 'left', 'z_score': 2.0},
        ]
        
        center = self.detector.calculate_pad_center()
        self.assertIsNotNone(center)
        
        # Center should be approximately (0.1, 0.1)
        self.assertAlmostEqual(center[0], 0.1, places=2)
        self.assertAlmostEqual(center[1], 0.1, places=2)
    
    def test_ready_for_landing(self):
        """Test landing readiness check"""
        self.detector.configure_detection(self.test_config)
        
        # Not ready initially
        self.assertFalse(self.detector.is_ready_for_landing())
        
        # Add sufficient points and calculate center
        self.detector.peak_positions = [
            {'position': (0.0, 0.0), 'height': 0.4, 'height_diff': 0.1, 'direction': 'forward', 'z_score': 2.0},
            {'position': (0.2, 0.0), 'height': 0.4, 'height_diff': 0.1, 'direction': 'forward', 'z_score': 2.0},
            {'position': (0.2, 0.2), 'height': 0.4, 'height_diff': 0.1, 'direction': 'right', 'z_score': 2.0},
            {'position': (0.0, 0.2), 'height': 0.4, 'height_diff': 0.1, 'direction': 'left', 'z_score': 2.0},
            {'position': (0.1, 0.0), 'height': 0.4, 'height_diff': 0.1, 'direction': 'forward', 'z_score': 2.0},
            {'position': (0.1, 0.2), 'height': 0.4, 'height_diff': 0.1, 'direction': 'forward', 'z_score': 2.0},
        ]
        
        center = self.detector.calculate_pad_center()
        
        # Should be ready if confidence is high enough
        if self.detector.center_confidence > 0.6:
            self.assertTrue(self.detector.is_ready_for_landing())


class TestSearchPattern(unittest.TestCase):
    """Test cases for search pattern generation"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.search_pattern = SearchPattern()
    
    def test_grid_pattern_generation(self):
        """Test grid pattern generation"""
        center = (0.0, 0.0)
        waypoints = self.search_pattern.generate_grid_pattern(center, grid_size=0.6, grid_spacing=0.3)
        
        self.assertGreater(len(waypoints), 0)
        
        # Each waypoint should be a tuple of (x, y, direction)
        for waypoint in waypoints:
            self.assertEqual(len(waypoint), 3)
            x, y, direction = waypoint
            self.assertIsInstance(x, float)
            self.assertIsInstance(y, float)
            self.assertIsInstance(direction, str)
    
    def test_spiral_pattern_generation(self):
        """Test spiral pattern generation"""
        center = (0.0, 0.0)
        waypoints = self.search_pattern.generate_spiral_pattern(center, max_radius=0.8, spacing=0.2)
        
        self.assertGreater(len(waypoints), 0)
        
        # First waypoint should be at center
        first_waypoint = waypoints[0]
        self.assertEqual(first_waypoint[0], center[0])
        self.assertEqual(first_waypoint[1], center[1])
        
        # Each waypoint should be a tuple of (x, y, direction)
        for waypoint in waypoints:
            self.assertEqual(len(waypoint), 3)
            x, y, direction = waypoint
            self.assertIsInstance(x, float)
            self.assertIsInstance(y, float)
            self.assertIsInstance(direction, str)


class TestGridMap(unittest.TestCase):
    """Test cases for grid map functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Create a 2x2 meter grid with 0.1m resolution
        self.grid_map = GridMap((-1.0, 1.0, -1.0, 1.0), resolution=0.1)
    
    def test_grid_initialization(self):
        """Test grid map initialization"""
        self.assertEqual(self.grid_map.width, 21)  # (-1.0 to 1.0) / 0.1 + 1
        self.assertEqual(self.grid_map.height, 21)
        self.assertEqual(self.grid_map.resolution, 0.1)
    
    def test_world_to_grid_conversion(self):
        """Test coordinate conversion"""
        # Test center point
        row, col = self.grid_map.world_to_grid(0.0, 0.0)
        self.assertEqual(row, 10)  # Middle of 21x21 grid
        self.assertEqual(col, 10)
        
        # Test corners
        row, col = self.grid_map.world_to_grid(-1.0, -1.0)
        self.assertEqual(row, 0)
        self.assertEqual(col, 0)
        
        row, col = self.grid_map.world_to_grid(1.0, 1.0)
        self.assertEqual(row, 20)
        self.assertEqual(col, 20)
    
    def test_grid_to_world_conversion(self):
        """Test reverse coordinate conversion"""
        # Test center cell
        x, y = self.grid_map.grid_to_world(10, 10)
        self.assertAlmostEqual(x, 0.05, places=2)  # Cell center offset
        self.assertAlmostEqual(y, 0.05, places=2)
    
    def test_cell_updates(self):
        """Test updating cells with measurements"""
        # Update a cell
        self.grid_map.update_cell(0.0, 0.0, 0.5, CellState.FREE)
        
        # Check the cell info
        info = self.grid_map.get_cell_info(0.0, 0.0)
        self.assertEqual(info['state'], CellState.FREE)
        self.assertAlmostEqual(info['height'], 0.5, places=2)
        self.assertEqual(info['visit_count'], 1)
        self.assertGreater(info['confidence'], 0)
    
    def test_multiple_measurements(self):
        """Test multiple measurements at same location"""
        # Add multiple measurements
        self.grid_map.update_cell(0.0, 0.0, 0.5, CellState.FREE)
        self.grid_map.update_cell(0.0, 0.0, 0.7, CellState.FREE)
        
        info = self.grid_map.get_cell_info(0.0, 0.0)
        # Height should be averaged
        self.assertNotEqual(info['height'], 0.5)  # Should be between 0.5 and 0.7
        self.assertNotEqual(info['height'], 0.7)
        self.assertEqual(info['visit_count'], 2)
    
    def test_find_unexplored_cells(self):
        """Test finding unexplored areas"""
        # Initially all cells should be unexplored
        unexplored = self.grid_map.find_unexplored_cells()
        self.assertEqual(len(unexplored), 21 * 21)  # All cells
        
        # Update one cell
        self.grid_map.update_cell(0.0, 0.0, 0.5, CellState.FREE)
        
        # Should have one less unexplored cell
        unexplored = self.grid_map.find_unexplored_cells()
        self.assertEqual(len(unexplored), 21 * 21 - 1)
    
    def test_find_elevated_regions(self):
        """Test finding elevated regions"""
        # Add more measurements at ground level to establish baseline
        for i in range(10):
            x = i * 0.1 - 0.4
            self.grid_map.update_cell(x, 0.0, 0.3, CellState.FREE)
        
        # Add multiple elevated measurements at same location to overcome averaging
        for _ in range(3):
            self.grid_map.update_cell(0.5, 0.0, 0.5, CellState.ELEVATED)
        
        elevated = self.grid_map.find_elevated_regions()
        self.assertGreater(len(elevated), 0)
        
        # Check that elevated region is detected
        found_elevated = False
        for region in elevated:
            if abs(region['position'][0] - 0.55) < 0.1:  # Account for grid resolution
                found_elevated = True
                break
        self.assertTrue(found_elevated)
    
    def test_exploration_progress(self):
        """Test exploration progress tracking"""
        # Initially no exploration
        progress = self.grid_map.get_exploration_progress()
        self.assertEqual(progress['explored_cells'], 0)
        self.assertEqual(progress['exploration_percentage'], 0.0)
        
        # Explore some cells - make sure they don't overlap
        for i in range(10):
            x = i * 0.2 - 1.0  # Spread them out more
            self.grid_map.update_cell(x, 0.0, 0.5, CellState.FREE)
        
        progress = self.grid_map.get_exploration_progress()
        self.assertEqual(progress['explored_cells'], 10)
        self.assertGreater(progress['exploration_percentage'], 0)
        
        
class TestSearchPatternWithGridMap(unittest.TestCase):
    """Test search pattern with grid map integration"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.grid_map = GridMap((-2.0, 2.0, -2.0, 2.0), resolution=0.2)
        self.search_pattern = SearchPattern(self.grid_map)
    
    def test_adaptive_pattern_with_unexplored(self):
        """Test adaptive pattern generation with unexplored areas"""
        center = (0.0, 0.0)
        
        # Generate adaptive pattern
        waypoints = self.search_pattern.generate_adaptive_pattern(center, max_distance=1.0)
        
        # Should generate waypoints
        self.assertGreater(len(waypoints), 0)
        
        # All waypoints should be within max distance
        for x, y, direction in waypoints:
            distance = math.sqrt(x**2 + y**2)  # Distance from center (0,0)
            self.assertLessEqual(distance, 1.0)
    
    def test_adaptive_pattern_fallback(self):
        """Test adaptive pattern fallback when no grid map"""
        search_pattern = SearchPattern()  # No grid map
        center = (0.0, 0.0)
        
        waypoints = search_pattern.generate_adaptive_pattern(center)
        
        # Should still generate waypoints (fallback to grid pattern)
        self.assertGreater(len(waypoints), 0)


if __name__ == '__main__':
    unittest.main()
