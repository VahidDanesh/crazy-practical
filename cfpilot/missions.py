"""
Flight Missions Module

Different autonomous flight missions for the Crazyflie platform.
"""

import time
import threading
from typing import Optional

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

from .controller import CrazyflieController


class BaseMission:
    """Base class for all flight missions"""
    
    def __init__(self, controller: CrazyflieController):
        self.controller = controller
        self.logger = controller.logger
    
    def execute(self) -> None:
        """Execute the mission - to be implemented by subclasses"""
        raise NotImplementedError("Subclasses must implement execute method")


class BasicFlightMission(BaseMission):
    """Basic takeoff-hover-land mission"""
    
    def execute(self) -> None:
        """Execute basic flight mission"""
        uri = self.controller.config['connection']['uri']
        cache_dir = self.controller.config['connection']['cache_dir']
        
        self.logger.info(f"🛫 Starting basic flight mission at {uri}")
        cflib.crtp.init_drivers()
        
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache_dir)) as scf:
                # Setup everything through controller
                self.controller.setup_logging(scf)
                self.controller.setup_crazyflie(scf)
                self.controller.setup_safety_timer()
                self.controller.setup_plotter()
                
                # Simple flight with unified logging
                with MotionCommander(scf, default_height=self.controller.config['flight']['default_height']) as mc:
                    with self.controller.start_loggers_context():
                        self.logger.info("🚁 Taking off...")
                        time.sleep(2)
                        
                        if not self.controller.emergency_triggered:
                            hover_duration = self.controller.config['flight']['hover_duration']
                            self.logger.info(f"🕰️ Hovering for {hover_duration}s")
                            
                            # Simple unified logging loop
                            self.controller.run_logging_loop(hover_duration)
                        
                        self.logger.info("🛬 Landing...")
                
                self.controller.cleanup()
                self.logger.info("✅ Basic flight mission completed")
                
        except Exception as e:
            self.logger.error(f"❌ Basic flight mission failed: {e}")
            raise


class SensorExplorationMission(BaseMission):
    """Sensor-based exploration mission"""
    
    def execute(self) -> None:
        """Execute sensor exploration mission"""
        uri = self.controller.config['connection']['uri']
        cache_dir = self.controller.config['connection']['cache_dir']
        
        self.logger.info(f"🔍 Starting sensor exploration mission at {uri}")
        cflib.crtp.init_drivers()
        
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache_dir)) as scf:
                # Setup everything through controller
                self.controller.setup_logging(scf)
                self.controller.setup_crazyflie(scf)
                self.controller.setup_safety_timer()
                self.controller.setup_plotter()
                
                # Simple exploration with unified logging
                with MotionCommander(scf, default_height=self.controller.config['flight']['default_height']) as mc:
                    with Multiranger(scf) as multiranger:
                        with self.controller.start_loggers_context():
                            self.controller.multiranger = multiranger
                            self.logger.info("🚁 Taking off for sensor exploration...")
                            time.sleep(3)
                            
                            # Exploration phase
                            moves_made = 0
                            max_moves = 3
                            obstacle_threshold = 0.8
                            move_distance = 0.5
                            
                            while moves_made < max_moves and not self.controller.emergency_triggered:
                                self.controller.print_sensor_status()
                                direction, distance = self.controller.find_free_direction(obstacle_threshold)
                                
                                if direction:
                                    self.logger.info(f"➡️ Moving {direction} ({distance:.2f} m free)")
                                    self._move_with_motion_commander(mc, direction, move_distance)
                                    moves_made += 1
                                    time.sleep(2)
                                else:
                                    self.logger.warning("⚠️ No free directions - staying in place")
                                    time.sleep(1)
                                    moves_made += 1
                                
                                # Simple logging update
                                data = self.controller.get_log_data()
                                if data:
                                    self.controller.flight_data.append(data)
                                    self.controller.log_battery_safety(data)
                                
                                # Update visualization
                                if self.controller.plotter:
                                    self.controller.plotter.process_events()
                            
                            self.logger.info("🛬 Landing...")
                
                self.controller.cleanup()
                self.logger.info("✅ Sensor exploration mission completed")
                
        except Exception as e:
            self.logger.error(f"❌ Sensor exploration mission failed: {e}")
            raise
    
    def _move_with_motion_commander(self, mc: MotionCommander, direction: str, distance: float) -> bool:
    
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


class LandingPadDetectionMission(BaseMission):
    """Landing pad detection and landing mission"""
    
    def execute(self) -> None:
        """Execute landing pad detection mission"""
        uri = self.controller.config['connection']['uri']
        cache_dir = self.controller.config['connection']['cache_dir']
        
        self.logger.info(f"🎯 Starting landing pad detection mission at {uri}")
        cflib.crtp.init_drivers()
        
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache=cache_dir)) as scf:
                # Setup everything through controller
                self.controller.setup_logging(scf)  # Use full logging for detection
                self.controller.setup_crazyflie(scf)
                self.controller.setup_safety_timer()
                self.controller.setup_plotter()
                
                # Landing detection with unified logging
                controller_type = self.controller.get_controller_type()
                with PositionHlCommander(
                    scf, x=0.0, y=0.0, z=0.0,
                    default_velocity=self.controller.config['flight']['default_velocity'],
                    default_height=self.controller.config['flight']['default_height'],
                    controller=controller_type
                ) as pc:
                    with Multiranger(scf) as multiranger:
                        with self.controller.start_loggers_context():
                            self.controller.multiranger = multiranger
                            self._execute_detection_mission(pc, multiranger)
                            self.logger.info("🛬 Landing...")
                
                self.controller.cleanup()
                
                # Print detection statistics
                stats = self.controller.landing_detector.get_detection_statistics()
                self.logger.info(f"📊 Detection stats: {stats}")
                self.logger.info("✅ Landing pad detection mission completed")
                
        except Exception as e:
            self.logger.error(f"❌ Landing pad detection mission failed: {e}")
            raise
    
    def _execute_detection_mission(self, pc: PositionHlCommander, multiranger: Multiranger) -> None:
        """Execute the main detection mission with logging"""
        self.logger.info("🚁 Taking off for landing pad detection...")
        time.sleep(3)
        
        # Initialize detection with config parameters
        profile = self.controller.config['peak_detection']['active_profile']
        peak_params = self.controller.config['peak_detection'][profile]
        self.controller.landing_detector.configure_detection(peak_params)
        self.controller.landing_detector.start_detection()
                        
        # Generate search pattern
        search_points = self.controller.search_pattern.generate_grid_pattern(center=(0, 0))
        self.logger.info(f"🗺️ Generated {len(search_points)} search waypoints")
        
        # Execute search pattern with obstacle avoidance
        pad_detected = False
        for i, (x, y, direction) in enumerate(search_points):
            if self.controller.emergency_triggered:
                break
            
            # Set flight direction for border classification
            self.controller.landing_detector.set_flight_direction(direction)
            
            self.logger.info(f"➡️ Moving to search point {i+1}/{len(search_points)}: "
                           f"({x:.2f}, {y:.2f}) direction: {direction}")
            
            # Check for obstacles before moving
            safe_to_move = self.controller.check_path_safety(multiranger, x, y)
            
            if safe_to_move:
                # Move to search point with slow velocity for safety
                safe_velocity = self.controller.config['obstacle_avoidance']['search_velocity']
                pc.go_to(x, y, velocity=safe_velocity)
                time.sleep(0.5)
            else:
                self.logger.warning(f"🚧 Obstacle detected! Skipping waypoint ({x:.2f}, {y:.2f})")
                # Try alternative safe position
                alt_x, alt_y = self.controller.find_safe_alternative(multiranger, x, y)
                if alt_x is not None:
                    self.logger.info(f"🔄 Moving to safe alternative: ({alt_x:.2f}, {alt_y:.2f})")
                    pc.go_to(alt_x, alt_y, velocity=safe_velocity * 0.7)
                    time.sleep(0.5)
                else:
                    self.logger.warning("⚠️ No safe alternative found, staying at current position")
                    continue
            
            # Simple unified logging and detection
            data = self.controller.get_log_data()
            if data:
                self.controller.flight_data.append(data)
                self.controller.log_battery_safety(data)
                
                # Process height for landing pad detection
                height = data.get('range.zrange', 0) / 1000.0
                position = (data.get('stateEstimate.x', x), data.get('stateEstimate.y', y))
                self.controller.landing_detector.process_height_measurement(height, position)
            
            # Update visualization
            if self.controller.plotter:
                self.controller.plotter.process_events()
            
            # Check if we have enough data for landing
            stats = self.controller.landing_detector.get_detection_statistics()
            if stats['total_border_points'] >= 8:
                center = self.controller.landing_detector.calculate_pad_center()
                if center and self.controller.landing_detector.is_ready_for_landing():
                    self.logger.info(f"🎯 Landing pad detected at center: ({center[0]:.3f}, {center[1]:.3f})")
                    pad_detected = True
                    break
        
        # Stop detection
        self.controller.landing_detector.stop_detection()
        
        # Land on detected pad or return to center
        if pad_detected:
            center = self.controller.landing_detector.calculated_center
            confidence = self.controller.landing_detector.center_confidence
            
            self.logger.info(f"🛬 Landing on detected pad at ({center[0]:.3f}, {center[1]:.3f}) "
                           f"with confidence {confidence:.2f}")
            
            # Navigate to pad center
            pc.go_to(center[0], center[1], velocity=0.15)
            time.sleep(2)
            
            self.logger.info("🎯 Executing precision landing...")
        else:
            self.logger.warning("⚠️ Landing pad not detected - returning to start")
            pc.go_to(0, 0, velocity=0.2)
            time.sleep(1)
    
