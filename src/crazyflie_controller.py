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
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
try:
    from .landing_pad_detector import LandingPadDetector, SearchPattern
    from .realtime_plotter import create_realtime_plotter
except ImportError:
    from landing_pad_detector import LandingPadDetector, SearchPattern
    from realtime_plotter import create_realtime_plotter


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
        
        # Landing pad detection
        self.landing_detector = LandingPadDetector()
        self.search_pattern = SearchPattern()
        
        # Real-time plotter for debugging
        self.realtime_plotter = None
        
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
    
    def _check_path_safety(self, multiranger, target_x: float, target_y: float, strict_mode=True) -> bool:
        """
        Check if path to target position is safe from obstacles
        
        Args:
            multiranger: Multiranger sensor object
            target_x: Target X position
            target_y: Target Y position
            strict_mode: If False, only check for very close obstacles (<200mm)
            
        Returns:
            bool: True if path is safe, False if obstacles detected
        """
        try:
            # Get current sensor readings - handle None values
            front_distance = multiranger.front
            back_distance = multiranger.back
            left_distance = multiranger.left
            right_distance = multiranger.right
            up_distance = multiranger.up
            
            # Safety thresholds from config (already in meters)
            MIN_SAFE_DISTANCE = self.config['obstacle_avoidance']['min_safe_distance']
            CEILING_MIN_DISTANCE = self.config['obstacle_avoidance']['ceiling_min_distance']
            
            # In non-strict mode, only avoid very close obstacles (emergency avoidance)
            if not strict_mode:
                MIN_SAFE_DISTANCE = 0.2  # 20cm emergency threshold
                CEILING_MIN_DISTANCE = 0.3  # 30cm emergency ceiling threshold
                
            # Check obstacles - distances from multiranger are already in meters
            distances = [front_distance, back_distance, left_distance, right_distance]
            direction_names = ['front', 'back', 'left', 'right']
            
            for i, distance in enumerate(distances):
                if distance is not None and distance < MIN_SAFE_DISTANCE:
                    if strict_mode:
                        self.logger.debug(f"🚫 Obstacle in {direction_names[i]} direction: {distance:.3f}m")
                        return False
                    else:
                        self.logger.warning(f"🚨 Emergency avoidance - obstacle in {direction_names[i]}: {distance:.3f}m")
                        return False
            
            # Check ceiling clearance - None means > 8m clearance
            if up_distance is not None and up_distance < CEILING_MIN_DISTANCE:
                if strict_mode:
                    self.logger.debug(f"🚫 Ceiling close: {up_distance:.3f}m")
                    return False
                else:
                    self.logger.warning(f"🚨 Emergency avoidance - ceiling too close: {up_distance:.3f}m")
                    return False
            
            # Path is safe
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Error checking path safety: {e}")
            return False  # Fail safe - assume unsafe if can't check
    
    def _find_safe_alternative(self, multiranger, target_x: float, target_y: float) -> tuple:
        """
        Find a safe alternative position near the target
        
        Args:
            multiranger: Multiranger sensor object
            target_x: Original target X position
            target_y: Original target Y position
            
        Returns:
            tuple: (alt_x, alt_y) or (None, None) if no safe alternative
        """
        try:
            # Get search radius from config
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
                if self._check_path_safety(multiranger, alt_x, alt_y):
                    return alt_x, alt_y
            
            # No safe alternative found
            return None, None
            
        except Exception as e:
            self.logger.error(f"❌ Error finding safe alternative: {e}")
            return None, None
    
    def _setup_sync_logging(self, scf):
        """Setup synchronous flight data logging"""
        log_config = LogConfig(name='FlightData', period_in_ms=self.config['logging']['period_ms'])
        
        # Add only essential variables to avoid size issues
        essential_vars = ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 'pm.vbat']
        
        try:
            for var in essential_vars:
                log_config.add_variable(var, 'float')
            
            self.sync_logger = SyncLogger(scf, log_config)
            self.logger.info("✅ Sync logging configured")
            return True
        except (KeyError, AttributeError) as e:
            self.logger.warning(f"Logging setup failed: {e}")
            self.logger.info("Continuing without logging...")
            return False
    
    def _setup_landing_logging(self, scf):
        """Setup logging specifically for landing pad detection"""
        log_config = LogConfig(name='LandingData', period_in_ms=50)  # Higher frequency for detection
        
        # Essential variables for landing detection including z-range sensor
        landing_vars = ['stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 'pm.vbat']
        
        try:
            for var in landing_vars:
                log_config.add_variable(var, 'float')
            
            # Add z-range sensor for height above ground measurement
            try:
                log_config.add_variable('range.zrange', 'uint16_t')
                self.logger.info("✅ Z-range sensor added to logging")
            except KeyError:
                self.logger.warning("Z-range sensor not available, using stateEstimate.z")
            
            self.sync_logger = SyncLogger(scf, log_config)
            self.logger.info("✅ Landing detection logging configured")
            return True
        except (KeyError, AttributeError) as e:
            self.logger.warning(f"Landing logging setup failed: {e}")
            self.logger.info("Continuing without logging...")
            return False
    
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
    
    def _log_battery_safety(self, data):
        """Check battery safety from log data"""
        battery_voltage = data.get('pm.vbat', 0)
        if battery_voltage < self.config['safety']['battery_threshold'] and battery_voltage > 0:
            self.logger.warning(f"⚠️ Low battery: {battery_voltage:.2f}V")
            self.emergency_shutdown()
    
    def _save_flight_data(self):
        """Save flight data to CSV"""
        if not self.flight_data:
            return

        # Determine the log folder one directory above this script
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
                # Setup sync logging
                logging_ok = self._setup_sync_logging(scf)
                
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
                scf.cf.platform.send_arming_request(True)
                time.sleep(1.0)
                
                # Safety timer
                self.safety_timer = threading.Timer(
                    self.config['safety']['max_flight_time'], 
                    self.emergency_shutdown
                )
                self.safety_timer.start()
                
                # Use MotionCommander for basic flight
                with MotionCommander(scf, default_height=self.config['flight']['default_height']) as mc:
                    if logging_ok:
                        with self.sync_logger:
                            self.logger.info("🚀 Taking off with MotionCommander...")
                            time.sleep(2)
                            
                            if not self.emergency_triggered:
                                hover_duration = self.config['flight']['hover_duration']
                                start_time = time.time()
                                
                                self.logger.info(f"🔄 Hovering for {hover_duration}s")
                                while (time.time() - start_time < hover_duration 
                                       and not self.emergency_triggered):
                                    # Process log data for safety
                                    for log_entry in self.sync_logger:
                                        self.flight_data.append(log_entry[1])
                                        self._log_battery_safety(log_entry[1])
                                        time.sleep(0.1)
                                        break
                    else:
                        self.logger.info("🚀 Taking off without logging...")
                        time.sleep(2)
                        
                        if not self.emergency_triggered:
                            hover_duration = self.config['flight']['hover_duration']
                            time.sleep(hover_duration)
                    
                    self.logger.info("🛬 Landing with MotionCommander...")
                    # MotionCommander handles landing automatically on exit
                
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
                # Setup sync logging
                logging_ok = self._setup_sync_logging(scf)
                
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
                scf.cf.platform.send_arming_request(True)
                time.sleep(1.0)
                
                # Safety timer
                self.safety_timer = threading.Timer(
                    self.config['safety']['max_flight_time'], 
                    self.emergency_shutdown
                )
                self.safety_timer.start()
                
                # Use MotionCommander for exploration
                with MotionCommander(scf, default_height=self.config['flight']['default_height']) as mc:
                    with Multiranger(scf) as multiranger:
                        self.multiranger = multiranger
                        
                        if logging_ok:
                            with self.sync_logger:
                                self.logger.info("🚀 Taking off for sensor exploration...")
                                time.sleep(3)  # Wait for sensor stabilization
                                
                                # Exploration phase
                                moves_made = 0
                                max_moves = 5
                                obstacle_threshold = 0.8  # meters
                                move_distance = 0.5  # meters
                                
                                # Start continuous logging in background
                                log_thread = threading.Thread(target=self._continuous_logging, daemon=True)
                                log_thread.start()
                                
                                while moves_made < max_moves and not self.emergency_triggered:
                                    self._print_sensor_status()
                                    direction, distance = self._find_free_direction(obstacle_threshold)
                                    
                                    if direction:
                                        self.logger.info(f"🎯 Moving {direction} ({distance:.2f}m free)")
                                        self._move_with_motion_commander(mc, direction, move_distance)
                                        moves_made += 1
                                        time.sleep(2)
                                    else:
                                        self.logger.warning("⚠️ No free directions - staying in place")
                                        time.sleep(1)
                                        moves_made += 1
                        else:
                            self.logger.info("🚀 Taking off for sensor exploration without logging...")
                            time.sleep(3)
                            
                            # Basic exploration without logging
                            moves_made = 0
                            max_moves = 3
                            while moves_made < max_moves and not self.emergency_triggered:
                                self._print_sensor_status()
                                mc.forward(0.3)
                                time.sleep(2)
                                moves_made += 1
                        
                        self.logger.info("🛬 Landing...")
                        # MotionCommander handles landing automatically on exit
                
                if self.safety_timer:
                    self.safety_timer.cancel()
                
                self.logger.info("✅ Sensor exploration mission completed")
                
        except Exception as e:
            self.logger.error(f"Sensor exploration mission failed: {e}")
            raise
    
    def landing_pad_detection_mission(self, enable_realtime_plot=True):
        """Landing pad detection and landing mission"""
        uri = self.config['connection']['uri']
        cache_dir = self.config['connection']['cache_dir']
        
        self.logger.info(f"🔗 Starting landing pad detection mission at {uri}")
        cflib.crtp.init_drivers()
        
        # Start real-time plotter for debugging
        if enable_realtime_plot:
            try:
                self.realtime_plotter = create_realtime_plotter()
                self.logger.info("📊 Data logger started for debugging")
            except Exception as e:
                self.logger.warning(f"Could not start data logger: {e}")
                self.realtime_plotter = None
        
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache_dir)) as scf:
                # Setup sync logging with z-range sensor
                logging_ok = self._setup_landing_logging(scf)
                
                # Reset and arm
                scf.cf.param.set_value('kalman.initialX', '0')
                scf.cf.param.set_value('kalman.initialY', '0') 
                scf.cf.param.set_value('kalman.initialZ', '0')
                time.sleep(0.1)
                
                scf.cf.param.set_value('kalman.resetEstimation', '1')
                time.sleep(0.1)
                scf.cf.param.set_value('kalman.resetEstimation', '0')
                time.sleep(3)
                scf.cf.platform.send_arming_request(True)
                time.sleep(1.0)
                
                # Safety timer
                self.safety_timer = threading.Timer(
                    self.config['safety']['max_flight_time'], 
                    self.emergency_shutdown
                )
                self.safety_timer.start()
                
                # Use PositionHlCommander for precise waypoint navigation
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
                        
                        if logging_ok:
                            with self.sync_logger:
                                self.logger.info("🚀 Taking off for landing pad detection...")
                                time.sleep(3)
                                
                                # Initialize detection with config parameters
                                profile = self.config['peak_detection']['active_profile']
                                peak_params = self.config['peak_detection'][profile]
                                self.landing_detector.configure_detection(peak_params)
                                self.landing_detector.start_detection()
                                                
                                # Generate more efficient search pattern for landing pad detection
                                # Use smaller search area and larger step size for faster coverage
                                self.search_pattern.search_area_size = 1.5  # Reduce from 2.0m to 1.5m
                                self.search_pattern.step_size = 0.4  # Increase from 0.3m to 0.4m
                                search_points = self.search_pattern.generate_grid_pattern(center=(0, 0))
                                self.logger.info(f"Generated {len(search_points)} search waypoints (1.5m area, 0.4m steps)")
                                
                                # Execute search pattern with obstacle avoidance
                                pad_detected = False
                                for i, (x, y, direction) in enumerate(search_points):
                                    if self.emergency_triggered:
                                        break
                                    
                                    # Set flight direction for border classification
                                    self.landing_detector.set_flight_direction(direction)
                                    
                                    self.logger.info(f"Moving to search point {i+1}/{len(search_points)}: "
                                                   f"({x:.2f}, {y:.2f}) direction: {direction}")
                                    
                                    # Check obstacle avoidance configuration
                                    enable_obstacle_avoidance = self.config['obstacle_avoidance'].get('enable_during_search', True)
                                    
                                    if enable_obstacle_avoidance:
                                        # Use relaxed obstacle avoidance (emergency only)
                                        safe_to_move = self._check_path_safety(multiranger, x, y, strict_mode=False)
                                        if not safe_to_move:
                                            self.logger.warning(f"🚨 Emergency obstacle detected! Skipping waypoint ({x:.2f}, {y:.2f})")
                                            continue
                                    else:
                                        # Obstacle avoidance disabled during search
                                        self.logger.debug(f"Obstacle avoidance disabled - moving to ({x:.2f}, {y:.2f})")
                                    
                                    # Move to search point with slow velocity for safety
                                    safe_velocity = self.config['obstacle_avoidance']['search_velocity']
                                    pc.go_to(x, y, velocity=safe_velocity)
                                    time.sleep(0.5)
                                    
                                    # Process height measurements during movement
                                    for log_entry in self.sync_logger:
                                        self.flight_data.append(log_entry[1])
                                        self._log_battery_safety(log_entry[1])
                                        
                                        # Get height measurement - prefer z-range sensor if available
                                        z_range_mm = log_entry[1].get('range.zrange', None)
                                        if z_range_mm is not None and z_range_mm > 0:
                                            # Z-range sensor returns mm, convert to meters for consistent units
                                            height = z_range_mm / 1000.0
                                        else:
                                            # Fallback to state estimator (already in meters)
                                            height = log_entry[1].get('stateEstimate.z', 0)
                                        
                                        position = (log_entry[1].get('stateEstimate.x', x),
                                                  log_entry[1].get('stateEstimate.y', y))
                                        
                                        # Process height for landing pad detection
                                        detection_result = self.landing_detector.process_height_measurement(height, position)
                                        
                                        # Send data to real-time plotter
                                        if self.realtime_plotter:
                                            stats = self.landing_detector.get_detection_statistics()
                                            self.realtime_plotter.add_data_point(
                                                height=height,
                                                position=position,
                                                is_detection=detection_result,
                                                baseline_height=self.landing_detector.baseline_height,
                                                stats=stats
                                            )
                                        
                                        break
                                    
                                    # Check if we have enough data for landing (reduced requirement)
                                    stats = self.landing_detector.get_detection_statistics()
                                    if stats['total_border_points'] >= 4:  # Reduced from 8 to 4 for faster detection
                                        center = self.landing_detector.calculate_pad_center()
                                        if center and self.landing_detector.is_ready_for_landing(min_confidence=0.3):  # Reduced confidence requirement
                                            self.logger.info(f"🎯 Landing pad detected at center: ({center[0]:.3f}, {center[1]:.3f}) after {i+1} waypoints")
                                            pad_detected = True
                                            break
                                
                                # Stop detection
                                self.landing_detector.stop_detection()
                                
                                # Land on detected pad or return to center
                                if pad_detected:
                                    center = self.landing_detector.calculated_center
                                    confidence = self.landing_detector.center_confidence
                                    
                                    self.logger.info(f"🛬 Landing on detected pad at ({center[0]:.3f}, {center[1]:.3f}) "
                                                   f"with confidence {confidence:.2f}")
                                    
                                    # Navigate to pad center
                                    pc.go_to(center[0], center[1], velocity=0.15)
                                    time.sleep(2)
                                    
                                    # Precision landing
                                    self.logger.info("🎯 Executing precision landing...")
                                    # PositionHlCommander will handle landing on exit
                                else:
                                    self.logger.warning("⚠️ Landing pad not detected - returning to start")
                                    pc.go_to(0, 0, velocity=0.2)
                                    time.sleep(1)
                        else:
                            self.logger.info("🚀 Taking off without detailed logging...")
                            time.sleep(3)
                            
                            # Basic search without logging using PositionHlCommander
                            current_pos = pc.get_position()
                            for i in range(3):
                                if self.emergency_triggered:
                                    break
                                pc.forward(0.5)
                                time.sleep(1)
                                pc.right(0.5) 
                                time.sleep(1)
                        
                        self.logger.info("🛬 Landing...")
                
                if self.safety_timer:
                    self.safety_timer.cancel()
                
                # Print detection statistics
                stats = self.landing_detector.get_detection_statistics()
                self.logger.info(f"Detection stats: {stats}")
                
                self.logger.info("✅ Landing pad detection mission completed")
                
        except Exception as e:
            self.logger.error(f"Landing pad detection mission failed: {e}")
            raise
        finally:
            # Stop real-time plotter
            if self.realtime_plotter:
                try:
                    # Save the plot before closing
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    plot_file = f"./logs/landing_detection_debug_{timestamp}.png"
                    self.realtime_plotter.save_plot(plot_file)
                    self.realtime_plotter.stop()
                    self.logger.info("📊 Data logger stopped and saved")
                except Exception as e:
                    self.logger.warning(f"Error stopping plotter: {e}")
    
    # =============================================================================
    # HELPER METHODS
    # =============================================================================
    
    def _print_sensor_status(self):
        """Print current sensor readings"""
        if self.multiranger:
            # Multiranger values are already in meters, handle None values
            front = f"{self.multiranger.front:.2f}" if self.multiranger.front is not None else ">8.0"
            back = f"{self.multiranger.back:.2f}" if self.multiranger.back is not None else ">8.0"
            left = f"{self.multiranger.left:.2f}" if self.multiranger.left is not None else ">8.0"
            right = f"{self.multiranger.right:.2f}" if self.multiranger.right is not None else ">8.0"
            up = f"{self.multiranger.up:.2f}" if self.multiranger.up is not None else ">8.0"
            down = f"{self.multiranger.down:.2f}" if self.multiranger.down is not None else ">8.0"
            
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
        
        # Filter free directions (None means >8m, values already in meters)
        free_directions = {}
        for k, v in directions.items():
            # Multiranger values are already in meters, None means > 8m
            distance_m = v if v is not None else 8.0
            if distance_m > threshold:
                free_directions[k] = distance_m
        
        if not free_directions:
            return None, 0
        
        # Return direction with maximum distance
        best_direction = max(free_directions, key=free_directions.get)
        max_distance = free_directions[best_direction]
        
        return best_direction, max_distance
    
    def _continuous_logging(self):
        """Continuous logging thread for SyncLogger"""
        try:
            for log_entry in self.sync_logger:
                if self.emergency_triggered:
                    break
                self.flight_data.append(log_entry[1])
                self._log_battery_safety(log_entry[1])
        except Exception as e:
            self.logger.warning(f"Logging thread error: {e}")
    
    def _move_with_motion_commander(self, mc, direction, distance):
        """Move in specified direction using MotionCommander"""
        movements = {
            'front': lambda: mc.forward(distance),
            'back': lambda: mc.back(distance),
            'left': lambda: mc.left(distance),
            'right': lambda: mc.right(distance)
        }
        
        if direction in movements:
            movements[direction]()
            return True
        return False
    
    def run_basic_flight(self):
        """Public interface for basic flight mission"""
        self.basic_flight_mission()
    
    def run_sensor_exploration(self):
        """Public interface for sensor exploration mission"""
        self.sensor_exploration_mission()
    
    def run_landing_pad_detection(self, enable_realtime_plot=True):
        """Public interface for landing pad detection mission"""
        self.landing_pad_detection_mission(enable_realtime_plot)


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

def run_landing_pad_detection():
    """Simple interface for landing pad detection"""
    controller = CrazyflieController()
    controller.start_mission(lambda: controller.landing_pad_detection_mission(enable_realtime_plot=True))
    controller.wait_for_mission_complete()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "sensor":
            run_sensor_exploration()
        elif sys.argv[1] == "landing":
            run_landing_pad_detection()
        else:
            run_basic_flight()
    else:
        run_basic_flight()
