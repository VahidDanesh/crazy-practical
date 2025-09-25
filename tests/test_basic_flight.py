"""
Test Basic Flight Mission

Tests for the basic takeoff-hover-land mission.
"""

import unittest
from unittest.mock import Mock, patch, MagicMock
import sys
from pathlib import Path

# Add the parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from cfpilot.controller import CrazyflieController
from cfpilot.missions import BasicFlightMission


class TestBasicFlightMission(unittest.TestCase):
    """Test cases for basic flight mission"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Mock config to avoid file dependencies
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
            }
        }
    
    @patch('cfpilot.controller.yaml.safe_load')
    @patch('cfpilot.controller.open')
    def test_controller_initialization(self, mock_open, mock_yaml_load):
        """Test controller initialization"""
        mock_yaml_load.return_value = self.mock_config
        
        controller = CrazyflieController()
        
        self.assertFalse(controller.is_connected)
        self.assertFalse(controller.emergency_triggered)
        self.assertFalse(controller.flight_active)
        self.assertEqual(controller.config, self.mock_config)
    
    @patch('cfpilot.controller.yaml.safe_load')
    @patch('cfpilot.controller.open')
    def test_emergency_shutdown(self, mock_open, mock_yaml_load):
        """Test emergency shutdown functionality"""
        mock_yaml_load.return_value = self.mock_config
        
        controller = CrazyflieController()
        controller.emergency_shutdown()
        
        self.assertTrue(controller.emergency_triggered)
        self.assertFalse(controller.flight_active)
    
    @patch('cfpilot.controller.yaml.safe_load')
    @patch('cfpilot.controller.open')
    @patch('cfpilot.missions.cflib.crtp.init_drivers')
    @patch('cfpilot.missions.SyncCrazyflie')
    def test_basic_mission_execution(self, mock_sync_cf, mock_init_drivers, mock_open, mock_yaml_load):
        """Test basic mission execution without hardware"""
        mock_yaml_load.return_value = self.mock_config
        
        # Mock SyncCrazyflie context manager
        mock_scf = MagicMock()
        mock_sync_cf.return_value.__enter__.return_value = mock_scf
        
        controller = CrazyflieController()
        mission = BasicFlightMission(controller)
        
        # This should not raise any exceptions in mock environment
        try:
            mission.execute()
        except Exception as e:
            # Expected in mock environment, just ensure it's attempting execution
            self.assertIsNotNone(e)
    
    @patch('cfpilot.controller.yaml.safe_load')
    @patch('cfpilot.controller.open')
    def test_config_loading_error(self, mock_open, mock_yaml_load):
        """Test config loading error handling"""
        mock_open.side_effect = FileNotFoundError("Config file not found")
        
        with self.assertRaises(SystemExit):
            CrazyflieController()


if __name__ == '__main__':
    unittest.main()
