"""
Test Flight Missions

Tests for different flight mission implementations.
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
from pathlib import Path

# Add the parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from cfpilot.controller import CrazyflieController
from cfpilot.missions import BasicFlightMission, SensorExplorationMission, LandingPadDetectionMission


class TestMissions(unittest.TestCase):
    """Test cases for flight missions"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.mock_config = {
            'connection': {
                'uri': 'radio://0/88/2M/E7E7E7E7F0',
                'cache_dir': './cache'
            },
            'flight': {
                'default_height': 0.5,
                'hover_duration': 2.0,
                'default_velocity': 0.3,
                'controller': 'MELLINGER'
            },
            'safety': {
                'max_flight_time': 30.0,
                'battery_threshold': 3.2
            },
            'logging': {
                'period_ms': 100
            },
            'obstacle_avoidance': {
                'min_safe_distance': 500,
                'ceiling_min_distance': 1500,
                'search_velocity': 0.15,
                'alternative_search_radius': 0.3
            },
            'peak_detection': {
                'active_profile': 'outdoor_robust',
                'outdoor_robust': {
                    'lag': 12,
                    'threshold': 2.5,
                    'influence': 0.4,
                    'min_peak_height': 0.06
                }
            }
        }
    
    @patch('cfpilot.controller.yaml.safe_load')
    @patch('cfpilot.controller.open')
    def test_mission_initialization(self, mock_open, mock_yaml_load):
        """Test mission initialization"""
        mock_yaml_load.return_value = self.mock_config
        
        controller = CrazyflieController()
        
        basic_mission = BasicFlightMission(controller)
        sensor_mission = SensorExplorationMission(controller)
        landing_mission = LandingPadDetectionMission(controller)
        
        self.assertEqual(basic_mission.controller, controller)
        self.assertEqual(sensor_mission.controller, controller)
        self.assertEqual(landing_mission.controller, controller)
    
    @patch('cfpilot.controller.yaml.safe_load')
    @patch('cfpilot.controller.open')
    def test_controller_safety_methods(self, mock_open, mock_yaml_load):
        """Test controller safety methods"""
        mock_yaml_load.return_value = self.mock_config
        
        controller = CrazyflieController()
        
        # Test path safety check with mock multiranger
        mock_multiranger = Mock()
        mock_multiranger.front = 1.0  # 1 meter clear
        mock_multiranger.back = 1.0
        mock_multiranger.left = 1.0
        mock_multiranger.right = 1.0
        mock_multiranger.up = 2.0
        
        # Should be safe
        result = controller.check_path_safety(mock_multiranger, 0.5, 0.5)
        self.assertTrue(result)
        
        # Test with obstacle too close
        mock_multiranger.front = 0.3  # 30cm - too close
        result = controller.check_path_safety(mock_multiranger, 0.5, 0.5)
        self.assertFalse(result)
    
    @patch('cfpilot.controller.yaml.safe_load')
    @patch('cfpilot.controller.open')
    def test_find_free_direction(self, mock_open, mock_yaml_load):
        """Test finding free direction"""
        mock_yaml_load.return_value = self.mock_config
        
        controller = CrazyflieController()
        
        # Mock multiranger with clear front
        controller.multiranger = Mock()
        controller.multiranger.front = 5.0  # 5 meters clear
        controller.multiranger.back = 0.5   # 0.5 meters
        controller.multiranger.left = 0.5   # 0.5 meters  
        controller.multiranger.right = 0.5  # 0.5 meters
        
        direction, distance = controller.find_free_direction(1.0)  # 1m threshold
        
        self.assertEqual(direction, 'front')
        self.assertEqual(distance, 5.0)
    
    @patch('cfpilot.controller.yaml.safe_load')
    @patch('cfpilot.controller.open')
    def test_controller_type_selection(self, mock_open, mock_yaml_load):
        """Test controller type selection"""
        mock_yaml_load.return_value = self.mock_config
        
        controller = CrazyflieController()
        controller_type = controller.get_controller_type()
        
        # Should return MELLINGER based on config
        from cfpilot.controller import PositionHlCommander
        self.assertEqual(controller_type, PositionHlCommander.CONTROLLER_MELLINGER)
        
        # Test PID controller
        controller.config['flight']['controller'] = 'PID'
        controller_type = controller.get_controller_type()
        self.assertEqual(controller_type, PositionHlCommander.CONTROLLER_PID)
    
    @patch('cfpilot.controller.yaml.safe_load')
    @patch('cfpilot.controller.open')
    def test_battery_safety_check(self, mock_open, mock_yaml_load):
        """Test battery safety checking"""
        mock_yaml_load.return_value = self.mock_config
        
        controller = CrazyflieController()
        
        # Test normal battery voltage
        data = {'pm.vbat': 3.8}  # Good voltage
        controller.log_battery_safety(data)
        self.assertFalse(controller.emergency_triggered)
        
        # Test low battery voltage
        data = {'pm.vbat': 3.0}  # Low voltage
        controller.log_battery_safety(data)
        self.assertTrue(controller.emergency_triggered)
    
    @patch('cfpilot.controller.yaml.safe_load')
    @patch('cfpilot.controller.open')
    def test_alternative_position_finding(self, mock_open, mock_yaml_load):
        """Test finding safe alternative positions"""
        mock_yaml_load.return_value = self.mock_config
        
        controller = CrazyflieController()
        
        # Mock multiranger that reports obstacles everywhere
        mock_multiranger = Mock()
        mock_multiranger.front = 0.3
        mock_multiranger.back = 0.3
        mock_multiranger.left = 0.3
        mock_multiranger.right = 0.3
        mock_multiranger.up = 0.8
        
        # Should return None when no safe alternatives
        alt_x, alt_y = controller.find_safe_alternative(mock_multiranger, 1.0, 1.0)
        self.assertIsNone(alt_x)
        self.assertIsNone(alt_y)


if __name__ == '__main__':
    unittest.main()
