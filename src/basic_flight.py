#!/usr/bin/env python3
"""
Basic Crazyflie Flight Controller
Takeoff -> Hover -> Land with Safety Features

Features:
- Emergency landing on connection loss
- Automatic timeout safety
- Battery monitoring
- Clean shutdown handling
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


class SafeFlightController:
    """Safe flight controller with comprehensive error handling"""
    
    def __init__(self, config_path=None):
        # Setup logging first
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Determine config path
        if config_path is None:
            # Try to find config relative to script location
            script_dir = Path(__file__).parent
            config_path = script_dir.parent / "config" / "flight_config.yaml"
        
        self.config = self._load_config(config_path)
        self.is_connected = False
        self.emergency_triggered = False
        self.flight_data = []
        self.safety_timer = None
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.logger.info("🚁 Safe Flight Controller initialized")
        
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
        """Setup data logging"""
        log_config = LogConfig(name='FlightData', period_in_ms=self.config['logging']['period_ms'])
        
        for var in self.config['logging']['variables']:
            log_config.add_variable(var, 'float')
        
        try:
            cf.log.add_config(log_config)
            log_config.data_received_cb.add_callback(self._log_data_callback)
            log_config.error_cb.add_callback(self._log_error_callback)
            log_config.start()
            self.logger.info("✅ Data logging started")
        except KeyError as e:
            self.logger.error(f"Logging variable not found: {e}")
        except AttributeError:
            self.logger.error("Could not setup logging configuration")
    
    def _log_data_callback(self, timestamp, data, logconf):
        """Callback for logging data"""
        # Add timestamp
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
        
        # Generate unique filename with timestamp
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
    
    def _connection_failed_callback(self, link_uri, msg):
        """Handle connection failure"""
        self.logger.error(f"🚨 Connection to {link_uri} failed: {msg}")
        self.is_connected = False
        self.emergency_triggered = True
    
    def _connection_lost_callback(self, link_uri, msg):
        """Handle connection loss"""
        self.logger.error(f"🚨 Connection to {link_uri} lost: {msg}")
        self.emergency_triggered = True
    
    def _disconnected_callback(self, link_uri):
        """Handle disconnection"""
        self.logger.info(f"Disconnected from {link_uri}")
        self.is_connected = False
    
    def _connected_callback(self, link_uri):
        """Handle successful connection"""
        self.logger.info(f"✅ Connected to {link_uri}")
        self.is_connected = True
    
    def _safety_timeout(self):
        """Safety timeout - force emergency landing"""
        self.logger.warning("⏰ Safety timeout reached - Emergency landing!")
        self.emergency_triggered = True
    
    def _emergency_land(self, pc):
        """Execute emergency landing"""
        try:
            self.logger.info("🚨 EMERGENCY LANDING INITIATED")
            pc.land(velocity=self.config['safety']['emergency_land_velocity'])
            time.sleep(3)  # Wait for landing completion
            self.logger.info("✅ Emergency landing completed")
        except Exception as e:
            self.logger.error(f"Emergency landing failed: {e}")
    
    def execute_basic_flight(self):
        """Execute basic takeoff-hover-land sequence"""
        uri = self.config['connection']['uri']
        cache_dir = self.config['connection']['cache_dir']
        
        self.logger.info(f"🔗 Connecting to Crazyflie at {uri}")
        
        # Initialize drivers
        cflib.crtp.init_drivers()
        
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache_dir)) as scf:
                # Setup callbacks
                scf.cf.connected.add_callback(self._connected_callback)
                scf.cf.disconnected.add_callback(self._disconnected_callback)
                scf.cf.connection_failed.add_callback(self._connection_failed_callback)
                scf.cf.connection_lost.add_callback(self._connection_lost_callback)
                
                # Setup logging
                self._setup_logging(scf.cf)
                
                # Reset Kalman filter
                scf.cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                scf.cf.param.set_value('kalman.resetEstimation', '0')
                time.sleep(2)
                
                # Arm the Crazyflie
                scf.cf.platform.send_arming_request(True)
                time.sleep(1.0)
                
                # Setup safety timer
                max_flight_time = self.config['safety']['max_flight_time']
                self.safety_timer = Timer(max_flight_time, self._safety_timeout)
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
                    # Takeoff happens automatically with PositionHlCommander
                    time.sleep(2)
                    
                    if not self.emergency_triggered:
                        hover_height = self.config['flight']['hover_height']
                        self.logger.info(f"🔄 Hovering at {hover_height}m")
                        pc.go_to(0.0, 0.0, hover_height)
                        
                        # Hover for specified duration
                        hover_duration = self.config['flight']['hover_duration']
                        start_time = time.time()
                        
                        while (time.time() - start_time < hover_duration 
                               and not self.emergency_triggered):
                            time.sleep(0.1)  # Check emergency status frequently
                        
                        if self.emergency_triggered:
                            self._emergency_land(pc)
                        else:
                            self.logger.info("🛬 Landing...")
                            pc.land(velocity=self.config['flight']['landing_velocity'])
                            time.sleep(2)
                    
                    else:
                        self._emergency_land(pc)
                
                # Cancel safety timer
                if self.safety_timer:
                    self.safety_timer.cancel()
                
                self.logger.info("✅ Flight sequence completed")
                
        except Exception as e:
            self.logger.error(f"Flight failed: {e}")
            if self.safety_timer:
                self.safety_timer.cancel()
        finally:
            # Save flight data
            self._save_flight_data()
            self.logger.info("🏁 Flight controller shutdown complete")


def main():
    """Main entry point"""
    controller = SafeFlightController()
    
    try:
        controller.execute_basic_flight()
    except KeyboardInterrupt:
        controller.logger.info("Flight interrupted by user")
    except Exception as e:
        controller.logger.error(f"Unexpected error: {e}")


if __name__ == "__main__":
    main()
