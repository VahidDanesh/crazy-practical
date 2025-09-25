"""
Core Crazyflie Controller

Main controller class for autonomous Crazyflie missions with clean architecture
and comprehensive safety features.
"""

import time
import yaml
import signal
import sys
import logging
import csv
import threading
from pathlib import Path
from datetime import datetime
from queue import Queue, Empty
from typing import Optional, Tuple, Dict, Any

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

from .detection import LandingPadDetector, SearchPattern


class CrazyflieController:
    """Main controller for autonomous Crazyflie missions"""
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize the Crazyflie controller.
        
        Args:
            config_path: Path to configuration file. If None, uses default.
        """
        self._setup_logging()
        self.config = self._load_config(config_path)
        
        # Flight state
        self.is_connected = False
        self.emergency_triggered = False
        self.flight_active = False
        
        # Threading
        self.flight_thread = None
        self.command_queue = Queue()
        self.data_queue = Queue()
        
        # Data storage
        self.flight_data = []
        
        # Sensor interface
        self.multiranger = None
        
        # Landing pad detection
        self.landing_detector = LandingPadDetector()
        self.search_pattern = SearchPattern()
        
        # Safety
        self.safety_timer = None
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.logger.info("Crazyflie Controller initialized")
    
    def _setup_logging(self) -> None:
        """Setup logging configuration"""
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
    
    def _load_config(self, config_path: Optional[str]) -> Dict[str, Any]:
        """Load configuration from YAML file"""
        if config_path is None:
            script_dir = Path(__file__).parent
            config_path = script_dir / "config" / "flight_config.yaml"
        
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            self.logger.error(f"Config file not found: {config_path}")
            sys.exit(1)
        except yaml.YAMLError as e:
            self.logger.error(f"Error parsing config: {e}")
            sys.exit(1)
    
    def _signal_handler(self, signum: int, frame) -> None:
        """Handle shutdown signals"""
        self.logger.warning(f"Signal {signum} received - initiating emergency shutdown")
        self.emergency_shutdown()
    
    def emergency_shutdown(self) -> None:
        """Emergency shutdown procedure"""
        self.emergency_triggered = True
        self.flight_active = False
        if self.safety_timer:
            self.safety_timer.cancel()
        self.logger.warning("Emergency shutdown initiated")
    
    def check_path_safety(self, multiranger: Multiranger, target_x: float, target_y: float) -> bool:
        """
        Check if path to target position is safe from obstacles.
        
        Args:
            multiranger: Multiranger sensor object
            target_x: Target X position
            target_y: Target Y position
            
        Returns:
            True if path is safe, False if obstacles detected
        """
        try:
            # Get current sensor readings
            front_distance = multiranger.front
            back_distance = multiranger.back
            left_distance = multiranger.left
            right_distance = multiranger.right
            up_distance = multiranger.up
            
            # Safety thresholds from config (in mm)
            min_safe_distance = self.config['obstacle_avoidance']['min_safe_distance']
            ceiling_min_distance = self.config['obstacle_avoidance']['ceiling_min_distance']
            
            # Check immediate surroundings
            if (front_distance and front_distance < min_safe_distance or 
                back_distance and back_distance < min_safe_distance or
                left_distance and left_distance < min_safe_distance or 
                right_distance and right_distance < min_safe_distance):
                
                self.logger.warning(f"Obstacle too close: F:{front_distance}m B:{back_distance}m "
                                  f"L:{left_distance}m R:{right_distance}m")
                return False
            
            # Check ceiling clearance
            if up_distance and up_distance < ceiling_min_distance:
                self.logger.warning(f"Ceiling too close: {up_distance}m")
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error checking path safety: {e}")
            return False  # Fail safe
    
    def find_safe_alternative(self, multiranger: Multiranger, target_x: float, target_y: float) -> Tuple[Optional[float], Optional[float]]:
        """
        Find a safe alternative position near the target.
        
        Args:
            multiranger: Multiranger sensor object
            target_x: Original target X position
            target_y: Original target Y position
            
        Returns:
            Tuple of (alt_x, alt_y) or (None, None) if no safe alternative
        """
        try:
            radius = self.config['obstacle_avoidance']['alternative_search_radius']
            small_radius = radius * 0.7
            
            # Try positions in a circle around the target
            alternatives = [
                (target_x + radius, target_y),           # Right
                (target_x - radius, target_y),           # Left
                (target_x, target_y + radius),           # Forward
                (target_x, target_y - radius),           # Back
                (target_x + small_radius, target_y + small_radius), # Diagonal
                (target_x - small_radius, target_y - small_radius), # Diagonal
                (target_x + small_radius, target_y - small_radius), # Diagonal
                (target_x - small_radius, target_y + small_radius), # Diagonal
            ]
            
            for alt_x, alt_y in alternatives:
                if self.check_path_safety(multiranger, alt_x, alt_y):
                    return alt_x, alt_y
            
            return None, None
            
        except Exception as e:
            self.logger.error(f"Error finding safe alternative: {e}")
            return None, None
    
    def setup_logging(self, scf: SyncCrazyflie, log_type: str = "basic") -> bool:
        """
        Setup data logging for flight missions.
        
        Args:
            scf: SyncCrazyflie instance
            log_type: Type of logging ("basic" or "landing")
            
        Returns:
            True if logging setup successful, False otherwise
        """
        if log_type == "landing":
            return self._setup_landing_logging(scf)
        else:
            return self._setup_basic_logging(scf)
    
    def _setup_basic_logging(self, scf: SyncCrazyflie) -> bool:
        """Setup basic flight data logging"""
        log_config = LogConfig(name='FlightData', period_in_ms=self.config['logging']['period_ms'])
        
        essential_vars = ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 'pm.vbat']
        
        try:
            for var in essential_vars:
                log_config.add_variable(var, 'float')
            
            self.sync_logger = SyncLogger(scf, log_config)
            self.logger.info("Basic logging configured")
            return True
        except (KeyError, AttributeError) as e:
            self.logger.warning(f"Logging setup failed: {e}")
            return False
    
    def _setup_landing_logging(self, scf: SyncCrazyflie) -> bool:
        """Setup logging for landing pad detection"""
        log_config = LogConfig(name='LandingData', period_in_ms=50)
        
        landing_vars = ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 'pm.vbat']
        
        try:
            for var in landing_vars:
                log_config.add_variable(var, 'float')
            
            # Add z-range sensor for landing pad detection
            log_config.add_variable('range.zrange', 'uint16_t')
            
            self.sync_logger = SyncLogger(scf, log_config)
            self.logger.info("Landing detection logging configured with z-range sensor")
            return True
        except (KeyError, AttributeError) as e:
            self.logger.warning(f"Landing logging setup failed: {e}")
            return False
    
    def setup_crazyflie(self, scf: SyncCrazyflie) -> None:
        """
        Setup Crazyflie parameters and estimator.
        
        Args:
            scf: SyncCrazyflie instance
        """
        # Set initial position to zero and reset estimator
        scf.cf.param.set_value('kalman.initialX', '0')
        scf.cf.param.set_value('kalman.initialY', '0') 
        scf.cf.param.set_value('kalman.initialZ', '0')
        time.sleep(0.1)
        
        # Reset Kalman filter to apply initial position
        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(3)  # Extra time for estimator convergence
        
        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)
    
    def setup_safety_timer(self) -> None:
        """Setup safety timer for emergency shutdown"""
        self.safety_timer = threading.Timer(
            self.config['safety']['max_flight_time'], 
            self.emergency_shutdown
        )
        self.safety_timer.start()
    
    def log_battery_safety(self, data: Dict[str, Any]) -> None:
        """Check battery safety from log data"""
        battery_voltage = data.get('pm.vbat', 0)
        if battery_voltage < self.config['safety']['battery_threshold'] and battery_voltage > 0:
            self.logger.warning(f"Low battery: {battery_voltage:.2f}V")
            self.emergency_shutdown()
    
    def save_flight_data(self) -> None:
        """Save flight data to CSV"""
        if not self.flight_data:
            return

        script_dir = Path(__file__).parent
        log_dir = script_dir.parent / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = log_dir / f"flight_log_{timestamp}.csv"
        
        try:
            with open(log_file, 'w', newline='') as csvfile:
                if self.flight_data:
                    fieldnames = self.flight_data[0].keys()
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(self.flight_data)
            
            self.logger.info(f"Flight data saved to: {log_file}")
        except Exception as e:
            self.logger.error(f"Failed to save flight data: {e}")
    
    def print_sensor_status(self) -> None:
        """Print current sensor readings"""
        if self.multiranger:
            front = self.multiranger.front or "None"
            back = self.multiranger.back or "None"
            left = self.multiranger.left or "None"
            right = self.multiranger.right or "None"
            up = self.multiranger.up or "None"
            down = self.multiranger.down or "None"
            
            self.logger.info(f"Sensors: F:{front}m B:{back}m L:{left}m R:{right}m U:{up}m D:{down}m")
    
    def find_free_direction(self, threshold: float) -> Tuple[Optional[str], float]:
        """Find direction with most free space"""
        if not self.multiranger:
            return None, 0
            
        directions = {
            'front': self.multiranger.front,
            'back': self.multiranger.back,
            'left': self.multiranger.left,
            'right': self.multiranger.right
        }
        
        # Filter free directions (None means >8m)
        free_directions = {}
        for k, v in directions.items():
            if v is None or v > threshold:
                free_directions[k] = v or 8.0
        
        if not free_directions:
            return None, 0
        
        # Return direction with maximum distance
        best_direction = max(free_directions, key=free_directions.get)
        max_distance = free_directions[best_direction]
        
        return best_direction, max_distance
    
    def get_controller_type(self) -> int:
        """Get controller type from config"""
        return (PositionHlCommander.CONTROLLER_PID 
                if self.config['flight']['controller'] == 'PID' 
                else PositionHlCommander.CONTROLLER_MELLINGER)
    
    def cleanup(self) -> None:
        """Cleanup resources and save data"""
        if self.safety_timer:
            self.safety_timer.cancel()
        
        self.save_flight_data()
        self.flight_active = False
        self.logger.info("Controller cleanup completed")
