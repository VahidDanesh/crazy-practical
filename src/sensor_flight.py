#!/usr/bin/env python3
"""
Sensor-Aware Flight Controller
Reads multiranger sensors and moves towards free space

Features:
- Multiranger sensor reading (front, back, left, right, up, down)
- Simple obstacle detection
- Movement towards free direction
- All safety features from basic_flight
"""

import time
import yaml
import signal
import sys
import logging
import csv
from pathlib import Path
from threading import Timer
from datetime import datetime

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger


class SensorFlightController:
    """Flight controller with sensor-based navigation"""
    
    def __init__(self, config_path=None):
        # Setup logging first
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Determine config path
        if config_path is None:
            script_dir = Path(__file__).parent
            config_path = script_dir.parent / "config" / "flight_config.yaml"
        
        self.config = self._load_config(config_path)
        self.is_connected = False
        self.emergency_triggered = False
        self.flight_data = []
        self.safety_timer = None
        
        # Movement parameters
        self.obstacle_threshold = 0.8  # meters - consider obstacle if closer
        self.move_distance = 0.3  # meters - how far to move
        self.multiranger = None
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.logger.info("🚁 Sensor Flight Controller initialized")
        
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
        """Handle Ctrl+C and other signals"""
        self.logger.warning(f"Signal {signum} received - initiating emergency shutdown")
        self.emergency_triggered = True
    
    def _setup_logging(self, cf):
        """Setup data logging including sensors"""
        log_config = LogConfig(name='FlightData', period_in_ms=self.config['logging']['period_ms'])
        
        # Add position and attitude
        for var in self.config['logging']['variables']:
            log_config.add_variable(var, 'float')
        
        # Add sensor readings
        sensor_vars = [
            'range.front', 'range.back', 'range.left', 'range.right', 
            'range.up', 'range.zrange'
        ]
        for var in sensor_vars:
            log_config.add_variable(var, 'uint16_t')
        
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
        """Callback for logging data"""
        # Add timestamp and store
        data['timestamp'] = timestamp
        self.flight_data.append(data.copy())
        
        # Safety checks
        battery_voltage = data.get('pm.vbat', 0)
        if battery_voltage < self.config['safety']['battery_threshold'] and battery_voltage > 0:
            self.logger.warning(f"⚠️ Low battery: {battery_voltage:.2f}V - Emergency landing!")
            self.emergency_triggered = True
    
    def _log_error_callback(self, logconf, msg):
        """Callback for logging errors"""
        self.logger.error(f"Logging error {logconf.name}: {msg}")
    
    def _save_flight_data(self):
        """Save flight data to CSV"""
        if not self.flight_data:
            return
            
        log_file = Path(self.config['logging']['log_file'])
        log_file.parent.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = log_file.parent / f"sensor_flight_{timestamp}.csv"
        
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
    
    def _find_free_direction(self):
        """Find direction with most free space"""
        if not self.multiranger:
            return None, 0
            
        directions = {
            'front': self.multiranger.front,
            'back': self.multiranger.back,
            'left': self.multiranger.left,
            'right': self.multiranger.right
        }
        
        # Filter out directions with obstacles (None means >8m, treat as free)
        free_directions = {}
        for k, v in directions.items():
            if v is None or v > self.obstacle_threshold:
                free_directions[k] = v or 8.0  # Treat None as 8m
        
        if not free_directions:
            return None, 0
        
        # Return direction with maximum distance
        best_direction = max(free_directions, key=free_directions.get)
        max_distance = free_directions[best_direction]
        
        return best_direction, max_distance
    
    def _move_in_direction(self, pc, direction):
        """Move in specified direction"""
        distance = self.move_distance
        
        movements = {
            'front': lambda: pc.forward(distance),
            'back': lambda: pc.back(distance),
            'left': lambda: pc.left(distance),
            'right': lambda: pc.right(distance)
        }
        
        if direction in movements:
            self.logger.info(f"🏃 Moving {direction} {distance}m")
            movements[direction]()
            time.sleep(1)  # Wait for movement completion
            return True
        return False
    
    def _connection_callbacks(self, scf):
        """Setup connection callbacks"""
        scf.cf.connected.add_callback(lambda uri: self.logger.info(f"✅ Connected to {uri}"))
        scf.cf.disconnected.add_callback(lambda uri: self.logger.info(f"Disconnected from {uri}"))
        scf.cf.connection_failed.add_callback(lambda uri, msg: self._handle_connection_error(uri, msg))
        scf.cf.connection_lost.add_callback(lambda uri, msg: self._handle_connection_error(uri, msg))
    
    def _handle_connection_error(self, uri, msg):
        """Handle connection errors"""
        self.logger.error(f"🚨 Connection issue {uri}: {msg}")
        self.emergency_triggered = True
    
    def execute_sensor_flight(self):
        """Execute sensor-aware flight sequence"""
        uri = self.config['connection']['uri']
        cache_dir = self.config['connection']['cache_dir']
        
        self.logger.info(f"🔗 Connecting to Crazyflie at {uri}")
        
        cflib.crtp.init_drivers()
        
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache_dir)) as scf:
                self._connection_callbacks(scf)
                self._setup_logging(scf.cf)
                
                # Reset Kalman filter
                scf.cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                scf.cf.param.set_value('kalman.resetEstimation', '0')
                time.sleep(2)
                
                # Arm and setup safety timer
                scf.cf.platform.send_arming_request(True)
                time.sleep(1.0)
                
                max_flight_time = self.config['safety']['max_flight_time']
                self.safety_timer = Timer(max_flight_time, lambda: setattr(self, 'emergency_triggered', True))
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
                        time.sleep(3)  # Wait for takeoff and sensor stabilization
                        
                        # Sensor exploration phase
                        moves_made = 0
                        max_moves = 5
                        
                        while moves_made < max_moves and not self.emergency_triggered:
                            self._print_sensor_status()
                            
                            direction, distance = self._find_free_direction()
                            
                            if direction:
                                self.logger.info(f"🎯 Found free space: {direction} ({distance:.2f}m)")
                                if self._move_in_direction(pc, direction):
                                    moves_made += 1
                                    time.sleep(2)  # Wait for movement and sensor update
                            else:
                                self.logger.warning("⚠️ No free directions found - staying in place")
                                time.sleep(1)
                                moves_made += 1
                    
                    if not self.emergency_triggered:
                        self.logger.info("🛬 Landing...")
                        pc.land(velocity=self.config['flight']['landing_velocity'])
                        time.sleep(2)
                    else:
                        self.logger.info("🚨 Emergency landing...")
                        pc.land(velocity=self.config['safety']['emergency_land_velocity'])
                        time.sleep(3)
                
                if self.safety_timer:
                    self.safety_timer.cancel()
                
                self.logger.info("✅ Sensor flight completed")
                
        except Exception as e:
            self.logger.error(f"Flight failed: {e}")
            if self.safety_timer:
                self.safety_timer.cancel()
        finally:
            self._save_flight_data()
            self.logger.info("🏁 Sensor flight controller shutdown complete")


def main():
    """Main entry point"""
    controller = SensorFlightController()
    
    try:
        controller.execute_sensor_flight()
    except KeyboardInterrupt:
        controller.logger.info("Flight interrupted by user")
    except Exception as e:
        controller.logger.error(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()
