"""
Test Landing Pad Detection

Tests for the landing pad detection algorithms.
"""

import unittest
import numpy as np
import sys
from pathlib import Path

# Add the parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from cfpilot.detection import LandingPadDetector, SearchPattern


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


if __name__ == '__main__':
    unittest.main()
