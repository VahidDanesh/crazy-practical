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
    
    def __init__(self, config_path: Optional[str] = None, enable_plotting: bool = False):
        """
        Initialize the Crazyflie controller.
        
        Args:
            config_path: Path to configuration file. If None, uses default.
            enable_plotting: Enable real-time 3D visualization
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
        
        # Logging
        self.sync_logger = None
        
        # Sensor interface
        self.multiranger = None
        
        # Landing pad detection
        self.landing_detector = LandingPadDetector()
        self.search_pattern = SearchPattern()
        
        # Plotting
        self.enable_plotting = enable_plotting
        self.plotter = None
        
        # Separate loggers for plotting (like original multiranger_pointcloud.py)
        self.position_logger = None
        self.measurement_logger = None
        
        
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
        self.logger.warning("❌ Emergency shutdown initiated")
    
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
    
    def setup_logging(self, scf: SyncCrazyflie) -> bool:
        """
        Setup data logging for flight missions.
        
        Args:
            scf: SyncCrazyflie instance
            
        Returns:
            True if logging setup successful, False otherwise
        """
        try:
            if self.enable_plotting:
                # Use dual loggers for full sensor data
                pos_config = LogConfig(name='Position', period_in_ms=self.config['logging']['period_ms'])
                pos_config.add_variable('stateEstimate.x', 'float')
                pos_config.add_variable('stateEstimate.y', 'float') 
                pos_config.add_variable('stateEstimate.z', 'float')
                pos_config.add_variable('pm.vbat', 'float')
                
                meas_config = LogConfig(name='Meas', period_in_ms=self.config['logging']['period_ms'])
                meas_config.add_variable('range.front', 'uint16_t')
                meas_config.add_variable('range.back', 'uint16_t')
                meas_config.add_variable('range.up', 'uint16_t')
                meas_config.add_variable('range.left', 'uint16_t')
                meas_config.add_variable('range.right', 'uint16_t')
                meas_config.add_variable('range.zrange', 'uint16_t')
                meas_config.add_variable('stabilizer.roll', 'float')
                meas_config.add_variable('stabilizer.pitch', 'float')
                meas_config.add_variable('stabilizer.yaw', 'float')
                
                self.position_logger = SyncLogger(scf, pos_config)
                self.measurement_logger = SyncLogger(scf, meas_config)
                self.logger.info("Dual logging configured for visualization")
            else:
                # Use single logger for basic missions
                log_config = LogConfig(name='Position', period_in_ms=self.config['logging']['period_ms'])
                log_config.add_variable('stateEstimate.x', 'float')
                log_config.add_variable('stateEstimate.y', 'float')
                log_config.add_variable('stateEstimate.z', 'float')
                log_config.add_variable('pm.vbat', 'float')
                log_config.add_variable('range.zrange', 'uint16_t')  # For landing detection
                
                self.sync_logger = SyncLogger(scf, log_config)
                self.logger.info("Basic logging configured")
            
            return True
            
        except (KeyError, AttributeError) as e:
            self.logger.warning(f"Logging setup failed: {e}")
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
        time.sleep(1)  # Extra time for estimator convergence
        
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
    
    def setup_plotter(self) -> None:
        """Setup 3D point cloud plotter if enabled"""
        if not self.enable_plotting:
            return
        
        try:
            from .visualization import PointCloudPlotter
            self.plotter = PointCloudPlotter()
            self.plotter.start()
            self.logger.info("3D visualization started")
        except ImportError:
            self.logger.warning("Visualization not available - install vispy and PyQt5")
            self.enable_plotting = False
    
    def update_plotter(self, pos_data: Dict[str, Any] = None, meas_data: Dict[str, Any] = None) -> None:
        """Update plotter with sensor and position data"""
        if not self.plotter:
            return
        
        # Update position if position data available
        if pos_data:
            x = pos_data.get('stateEstimate.x', 0)
            y = pos_data.get('stateEstimate.y', 0) 
            z = pos_data.get('stateEstimate.z', 0)
            self.plotter.update_position(x, y, z)
        
        # Update sensors if measurement data available  
        if meas_data:
            sensor_data = {
                'roll': meas_data.get('stabilizer.roll', 0),
                'pitch': meas_data.get('stabilizer.pitch', 0),
                'yaw': meas_data.get('stabilizer.yaw', 0),
                'front': meas_data.get('range.front', 8000),
                'back': meas_data.get('range.back', 8000),
                'left': meas_data.get('range.left', 8000),
                'right': meas_data.get('range.right', 8000),
                'up': meas_data.get('range.up', 8000),
                'down': meas_data.get('range.zrange', 8000)
            }
            self.plotter.update_sensors(sensor_data)
    
    def get_log_data(self) -> Dict[str, Any]:
        """Get data from active loggers"""
        combined_data = {}
        
        if self.enable_plotting and self.position_logger and self.measurement_logger:
            # Get data from both loggers
            try:
                for pos_entry in self.position_logger:
                    combined_data.update(pos_entry[1])
                    break
                for meas_entry in self.measurement_logger:
                    combined_data.update(meas_entry[1])
                    break
            except Exception as e:
                self.logger.warning(f"Error getting log data: {e}")
        elif self.sync_logger:
            # Get data from single logger
            try:
                for log_entry in self.sync_logger:
                    combined_data = log_entry[1]
                    break
            except Exception as e:
                self.logger.warning(f"Error getting log data: {e}")
        
        return combined_data
    
    def log_battery_safety(self, data: Dict[str, Any]) -> None:
        """Check battery safety from log data"""
        battery_voltage = data.get('pm.vbat', 0)
        if battery_voltage < self.config['safety']['battery_threshold'] and battery_voltage > 0:
            self.logger.warning(f"🔋 Low battery: {battery_voltage:.2f}V")
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
            
            self.logger.info(f"🛜 Flight data saved to: {log_file}")
        except Exception as e:
            self.logger.error(f"❌ Failed to save flight data: {e}")
    
    def print_sensor_status(self) -> None:
        """Print current sensor readings"""
        if self.multiranger:
            front = self.multiranger.front
            back = self.multiranger.back
            left = self.multiranger.left 
            right = self.multiranger.right
            up = self.multiranger.up 
            down = self.multiranger.down
            
            self.logger.info(f"Sensors: F:{front} m B:{back} m L:{left} m R:{right} m U:{up}m D:{down} m")
    
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
    
    def run_logging_loop(self, duration_seconds: float = None) -> None:
        """Simple unified logging loop that handles all complexity"""
        if not duration_seconds:
            duration_seconds = float('inf')  # Run indefinitely
        
        start_time = time.time()
        
        while ((time.time() - start_time < duration_seconds) and 
               not self.emergency_triggered):
            
            # Get data from active loggers
            data = self.get_log_data()
            if data:
                self.flight_data.append(data)
                self.log_battery_safety(data)
                # Update plotter with new data
                if self.enable_plotting:
                    self.update_plotter(pos_data=data, meas_data=data)
            
            # Process Qt events if needed
            if self.plotter:
                self.plotter.process_events()
            
            time.sleep(0.1)
    
    def start_loggers_context(self):
        """Return appropriate context manager for logging"""
        if self.enable_plotting and self.position_logger:
            # Dual logger context
            class DualLoggerContext:
                def __init__(self, pos_logger, meas_logger):
                    self.pos_logger = pos_logger
                    self.meas_logger = meas_logger
                
                def __enter__(self):
                    self.pos_logger.__enter__()
                    self.meas_logger.__enter__()
                    return self
                
                def __exit__(self, *args):
                    self.pos_logger.__exit__(*args)
                    self.meas_logger.__exit__(*args)
            
            return DualLoggerContext(self.position_logger, self.measurement_logger)
        elif self.sync_logger:
            # Single logger context
            return self.sync_logger
        else:
            # No logging context
            class NoLoggerContext:
                def __enter__(self): return self
                def __exit__(self, *args): pass
            return NoLoggerContext()
    
    def cleanup(self) -> None:
        """Cleanup resources and save data"""
        if self.safety_timer:
            self.safety_timer.cancel()
        
        if self.plotter:
            self.plotter.stop()
        
        # Reset loggers
        self.position_logger = None
        self.measurement_logger = None
        
        self.save_flight_data()
        self.flight_active = False
        self.logger.info("Controller cleanup completed")
