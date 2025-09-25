#!/usr/bin/env python3
"""
Modular Crazyflie Controller
Main controller class with threading support for autonomous missions

Features:
- Threaded flight control
- Modular mission components
- Real-time sensor monitoring
- Safety systems
- Data logging
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

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger


class CrazyflieController:
    """Main controller for autonomous Crazyflie missions"""
    
    def __init__(self, config_path=None):
        # Setup logging first
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Load configuration
        if config_path is None:
            script_dir = Path(__file__).parent
            config_path = script_dir.parent / "config" / "flight_config.yaml"
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
        
        # Safety
        self.safety_timer = None
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.logger.info("🚁 Crazyflie Controller initialized")
    
    def _load_config(self, config_path):
        """Load configuration from YAML file"""
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except FileNotFoundError:
            self.logger.error(f"Config file not found: {config_path}")
            sys.exit(1)
        except yaml.YAMLError as e:
            self.logger.error(f"Error parsing config: {e}")
            sys.exit(1)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        self.logger.warning(f"Signal {signum} received - initiating emergency shutdown")
        self.emergency_shutdown()
    
    def emergency_shutdown(self):
        """Emergency shutdown procedure"""
        self.emergency_triggered = True
        self.flight_active = False
        if self.safety_timer:
            self.safety_timer.cancel()
        self.logger.warning("🚨 Emergency shutdown initiated")
    
    def _setup_logging(self, cf):
        """Setup flight data logging"""
        log_config = LogConfig(name='FlightData', period_in_ms=self.config['logging']['period_ms'])
        
        # Add standard variables
        for var in self.config['logging']['variables']:
            log_config.add_variable(var, 'float')
        
        try:
            cf.log.add_config(log_config)
            log_config.data_received_cb.add_callback(self._log_data_callback)
            log_config.error_cb.add_callback(self._log_error_callback)
            log_config.start()
            self.logger.info("✅ Data logging started")
        except KeyError as e:
            self.logger.warning(f"Logging variable not found: {e}")
        except AttributeError:
            self.logger.error("Could not setup logging configuration")
    
    def _log_data_callback(self, timestamp, data, logconf):
        """Callback for flight data logging"""
        data['timestamp'] = timestamp
        self.flight_data.append(data.copy())
        
        # Put data in queue for real-time processing
        try:
            self.data_queue.put_nowait(data.copy())
        except:
            pass  # Queue full, skip
        
        # Safety checks
        battery_voltage = data.get('pm.vbat', 0)
        if battery_voltage < self.config['safety']['battery_threshold'] and battery_voltage > 0:
            self.logger.warning(f"⚠️ Low battery: {battery_voltage:.2f}V")
            self.emergency_shutdown()
    
    def _log_error_callback(self, logconf, msg):
        """Callback for logging errors"""
        self.logger.error(f"Logging error {logconf.name}: {msg}")
    
    def _connection_callbacks(self, scf):
        """Setup connection callbacks"""
        scf.cf.connected.add_callback(lambda uri: self._on_connected(uri))
        scf.cf.disconnected.add_callback(lambda uri: self._on_disconnected(uri))
        scf.cf.connection_failed.add_callback(lambda uri, msg: self._on_connection_error(uri, msg))
        scf.cf.connection_lost.add_callback(lambda uri, msg: self._on_connection_error(uri, msg))
    
    def _on_connected(self, uri):
        """Handle successful connection"""
        self.logger.info(f"✅ Connected to {uri}")
        self.is_connected = True
    
    def _on_disconnected(self, uri):
        """Handle disconnection"""
        self.logger.info(f"Disconnected from {uri}")
        self.is_connected = False
        self.flight_active = False
    
    def _on_connection_error(self, uri, msg):
        """Handle connection errors"""
        self.logger.error(f"🚨 Connection error {uri}: {msg}")
        self.emergency_shutdown()
    
    def _save_flight_data(self):
        """Save flight data to CSV"""
        if not self.flight_data:
            return
            
        log_file = Path(self.config['logging']['log_file'])
        log_file.parent.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = log_file.parent / f"flight_log_{timestamp}.csv"
        
        try:
            with open(log_file, 'w', newline='') as csvfile:
                if self.flight_data:
                    fieldnames = self.flight_data[0].keys()
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(self.flight_data)
            
            self.logger.info(f"📊 Flight data saved to: {log_file}")
        except Exception as e:
            self.logger.error(f"Failed to save flight data: {e}")
    
    def _flight_worker(self, mission_func, *args, **kwargs):
        """Worker thread for flight operations"""
        try:
            self.flight_active = True
            mission_func(*args, **kwargs)
        except Exception as e:
            self.logger.error(f"Flight mission failed: {e}")
            self.emergency_shutdown()
        finally:
            self.flight_active = False
            self._save_flight_data()
            self.logger.info("🏁 Flight worker thread completed")
    
    def start_mission(self, mission_func, *args, **kwargs):
        """Start a mission in a separate thread"""
        if self.flight_thread and self.flight_thread.is_alive():
            self.logger.warning("Mission already running")
            return False
        
        self.flight_thread = threading.Thread(
            target=self._flight_worker,
            args=(mission_func,) + args,
            kwargs=kwargs,
            daemon=True
        )
        self.flight_thread.start()
        return True
    
    def wait_for_mission_complete(self, timeout=None):
        """Wait for mission to complete"""
        if self.flight_thread:
            self.flight_thread.join(timeout)
            return not self.flight_thread.is_alive()
        return True
    
    # =============================================================================
    # MISSION COMPONENTS
    # =============================================================================
    
    def basic_flight_mission(self):
        """Basic takeoff-hover-land mission"""
        uri = self.config['connection']['uri']
        cache_dir = self.config['connection']['cache_dir']
        
        self.logger.info(f"🔗 Starting basic flight mission at {uri}")
        cflib.crtp.init_drivers()
        
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache_dir)) as scf:
                self._connection_callbacks(scf)
                self._setup_logging(scf.cf)
                
                # Reset and arm
                scf.cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                scf.cf.param.set_value('kalman.resetEstimation', '0')
                time.sleep(2)
                scf.cf.platform.send_arming_request(True)
                time.sleep(1.0)
                
                # Safety timer
                self.safety_timer = threading.Timer(
                    self.config['safety']['max_flight_time'], 
                    self.emergency_shutdown
                )
                self.safety_timer.start()
                
                controller_type = (PositionHlCommander.CONTROLLER_PID 
                                 if self.config['flight']['controller'] == 'PID' 
                                 else PositionHlCommander.CONTROLLER_MELLINGER)
                
                with PositionHlCommander(
                    scf,
                    x=0.0, y=0.0, z=0.0,
                    default_velocity=self.config['flight']['default_velocity'],
                    default_height=self.config['flight']['default_height'],
                    controller=controller_type
                ) as pc:
                    
                    self.logger.info("🚀 Taking off...")
                    time.sleep(2)
                    
                    if not self.emergency_triggered:
                        hover_height = self.config['flight']['hover_height']
                        self.logger.info(f"🔄 Hovering at {hover_height}m")
                        pc.go_to(0.0, 0.0, hover_height)
                        
                        # Hover duration
                        hover_duration = self.config['flight']['hover_duration']
                        start_time = time.time()
                        
                        while (time.time() - start_time < hover_duration 
                               and not self.emergency_triggered):
                            time.sleep(0.1)
                    
                    # Land
                    self.logger.info("🛬 Landing...")
                    velocity = (self.config['safety']['emergency_land_velocity'] 
                              if self.emergency_triggered 
                              else self.config['flight']['landing_velocity'])
                    pc.land(velocity=velocity)
                    time.sleep(2)
                
                if self.safety_timer:
                    self.safety_timer.cancel()
                
                self.logger.info("✅ Basic flight mission completed")
                
        except Exception as e:
            self.logger.error(f"Basic flight mission failed: {e}")
            raise
    
    def sensor_exploration_mission(self):
        """Sensor-based exploration mission"""
        uri = self.config['connection']['uri']
        cache_dir = self.config['connection']['cache_dir']
        
        self.logger.info(f"🔗 Starting sensor exploration mission at {uri}")
        cflib.crtp.init_drivers()
        
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache_dir)) as scf:
                self._connection_callbacks(scf)
                self._setup_logging(scf.cf)
                
                # Reset and arm
                scf.cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                scf.cf.param.set_value('kalman.resetEstimation', '0')
                time.sleep(2)
                scf.cf.platform.send_arming_request(True)
                time.sleep(1.0)
                
                # Safety timer
                self.safety_timer = threading.Timer(
                    self.config['safety']['max_flight_time'], 
                    self.emergency_shutdown
                )
                self.safety_timer.start()
                
                controller_type = (PositionHlCommander.CONTROLLER_PID 
                                 if self.config['flight']['controller'] == 'PID' 
                                 else PositionHlCommander.CONTROLLER_MELLINGER)
                
                with PositionHlCommander(
                    scf,
                    x=0.0, y=0.0, z=0.0,
                    default_velocity=self.config['flight']['default_velocity'],
                    default_height=self.config['flight']['default_height'],
                    controller=controller_type
                ) as pc:
                    
                    with Multiranger(scf) as multiranger:
                        self.multiranger = multiranger
                        
                        self.logger.info("🚀 Taking off...")
                        time.sleep(3)  # Wait for sensor stabilization
                        
                        # Exploration phase
                        moves_made = 0
                        max_moves = 5
                        obstacle_threshold = 0.8  # meters
                        move_distance = 0.3  # meters
                        
                        while moves_made < max_moves and not self.emergency_triggered:
                            # Print sensor status
                            self._print_sensor_status()
                            
                            # Find free direction
                            direction, distance = self._find_free_direction(obstacle_threshold)
                            
                            if direction:
                                self.logger.info(f"🎯 Moving {direction} ({distance:.2f}m free)")
                                self._move_in_direction(pc, direction, move_distance)
                                moves_made += 1
                                time.sleep(2)
                            else:
                                self.logger.warning("⚠️ No free directions - staying in place")
                                time.sleep(1)
                                moves_made += 1
                    
                    # Land
                    self.logger.info("🛬 Landing...")
                    velocity = (self.config['safety']['emergency_land_velocity'] 
                              if self.emergency_triggered 
                              else self.config['flight']['landing_velocity'])
                    pc.land(velocity=velocity)
                    time.sleep(2)
                
                if self.safety_timer:
                    self.safety_timer.cancel()
                
                self.logger.info("✅ Sensor exploration mission completed")
                
        except Exception as e:
            self.logger.error(f"Sensor exploration mission failed: {e}")
            raise
    
    # =============================================================================
    # HELPER METHODS
    # =============================================================================
    
    def _print_sensor_status(self):
        """Print current sensor readings"""
        if self.multiranger:
            front = self.multiranger.front or "None"
            back = self.multiranger.back or "None"
            left = self.multiranger.left or "None"
            right = self.multiranger.right or "None"
            up = self.multiranger.up or "None"
            down = self.multiranger.down or "None"
            
            self.logger.info(f"📡 Sensors: F:{front}m B:{back}m L:{left}m R:{right}m U:{up}m D:{down}m")
    
    def _find_free_direction(self, threshold):
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
    
    def _move_in_direction(self, pc, direction, distance):
        """Move in specified direction"""
        movements = {
            'front': lambda: pc.forward(distance),
            'back': lambda: pc.back(distance),
            'left': lambda: pc.left(distance),
            'right': lambda: pc.right(distance)
        }
        
        if direction in movements:
            movements[direction]()
            time.sleep(1)
            return True
        return False


# =============================================================================
# SIMPLE INTERFACE FUNCTIONS
# =============================================================================

def run_basic_flight():
    """Simple interface for basic flight"""
    controller = CrazyflieController()
    controller.start_mission(controller.basic_flight_mission)
    controller.wait_for_mission_complete()

def run_sensor_exploration():
    """Simple interface for sensor exploration"""
    controller = CrazyflieController()
    controller.start_mission(controller.sensor_exploration_mission)
    controller.wait_for_mission_complete()


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "sensor":
        run_sensor_exploration()
    else:
        run_basic_flight()
