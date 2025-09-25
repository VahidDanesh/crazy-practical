#!/usr/bin/env python3
"""
Simple Landing Pad Detection Mission
Clean implementation following drone.py style
"""

import time
import logging
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.position_hl_commander import PositionHlCommander
try:
    from .simple_landing_detector import SimpleLandingDetector, SimpleSearchPattern, determine_movement_direction
except ImportError:
    from simple_landing_detector import SimpleLandingDetector, SimpleSearchPattern, determine_movement_direction


class SimpleLandingMission:
    """Simple landing pad detection mission"""
    
    def __init__(self, uri, cruise_height=0.5):
        self.uri = uri
        self.cruise_height = cruise_height
        
        # Mission state
        self.current_position = [0.0, 0.0, 0.0]
        self.current_height = 0.0
        self.mission_complete = False
        
        # Landing pad detection
        self.detector = SimpleLandingDetector(height_threshold=0.03, min_detections=4)
        self.search_pattern = SimpleSearchPattern(area_size=1.5, step_size=0.4)
        
        # Logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        self.logger.info("🚁 Simple Landing Mission initialized")
    
    def run_mission(self):
        """Execute the complete mission"""
        self.logger.info(f"🔗 Starting mission at {self.uri}")
        cflib.crtp.init_drivers()
        
        try:
            with SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache='.cache')) as scf:
                self.setup_logging(scf)
                self.reset_estimator(scf)
                
                with PositionHlCommander(scf, default_height=self.cruise_height) as pc:
                    self.logger.info("🚀 Taking off...")
                    time.sleep(2)  # Wait for takeoff
                    
                    # Execute search mission
                    landing_pad_found = self.search_for_landing_pad(pc)
                    
                    if landing_pad_found:
                        center = self.detector.calculate_center()
                        self.land_on_pad(pc, center)
                    else:
                        self.logger.warning("⚠️ Landing pad not found")
                    
                    self.logger.info("🛬 Mission complete")
                    
        except Exception as e:
            self.logger.error(f"Mission failed: {e}")
            raise
    
    def search_for_landing_pad(self, position_commander):
        """
        Search for landing pad using grid pattern
        
        Returns:
            bool: True if landing pad found
        """
        self.logger.info("🔍 Starting landing pad search...")
        
        # Generate search waypoints
        waypoints = self.search_pattern.generate_grid_search()
        self.logger.info(f"Generated {len(waypoints)} search waypoints")
        
        # Start detection
        self.detector.start_detection()
        
        # Execute search pattern
        for i, (x, y, direction) in enumerate(waypoints):
            if self.mission_complete:
                break
            
            self.logger.info(f"Moving to waypoint {i+1}/{len(waypoints)}: ({x:.2f}, {y:.2f})")
            
            # Set movement direction for detection
            self.detector.set_direction(direction)
            
            # Move to waypoint
            position_commander.go_to(x, y, velocity=0.2)
            time.sleep(0.5)  # Allow time for height measurement
            
            # Check if we have enough detections
            if self.detector.is_ready_for_landing():
                stats = self.detector.get_statistics()
                self.logger.info(f"🎯 Landing pad detected! Stats: {stats}")
                return True
        
        # Search complete
        self.detector.stop_detection()
        stats = self.detector.get_statistics()
        self.logger.info(f"Search complete. Final stats: {stats}")
        
        return self.detector.is_ready_for_landing()
    
    def land_on_pad(self, position_commander, center):
        """
        Navigate to landing pad center and land
        
        Args:
            position_commander: Position commander instance
            center: (x, y) coordinates of landing pad center
        """
        if center is None:
            self.logger.warning("No landing pad center calculated")
            return
        
        self.logger.info(f"🎯 Navigating to landing pad center: ({center[0]:.3f}, {center[1]:.3f})")
        
        # Move to center
        position_commander.go_to(center[0], center[1], velocity=0.15)
        time.sleep(2)
        
        self.logger.info("🛬 Landing on pad...")
        # Landing handled automatically by PositionHlCommander context exit
    
    def setup_logging(self, scf):
        """Setup position and sensor logging"""
        # Position logging
        pos_log = LogConfig(name='Position', period_in_ms=50)
        pos_log.add_variable('stateEstimate.x', 'float')
        pos_log.add_variable('stateEstimate.y', 'float')
        pos_log.add_variable('stateEstimate.z', 'float')
        
        try:
            scf.cf.log.add_config(pos_log)
            pos_log.data_received_cb.add_callback(self.position_callback)
            pos_log.start()
            self.logger.info("✅ Position logging started")
        except Exception as e:
            self.logger.warning(f"Position logging failed: {e}")
        
        # Height sensor logging
        height_log = LogConfig(name='Height', period_in_ms=50)
        try:
            height_log.add_variable('range.zrange', 'uint16_t')
            scf.cf.log.add_config(height_log)
            height_log.data_received_cb.add_callback(self.height_callback)
            height_log.start()
            self.logger.info("✅ Height sensor logging started")
        except Exception as e:
            self.logger.warning(f"Height logging failed: {e}")
    
    def position_callback(self, timestamp, data, logconf):
        """Handle position data"""
        self.current_position = [
            data['stateEstimate.x'],
            data['stateEstimate.y'],
            data['stateEstimate.z']
        ]
    
    def height_callback(self, timestamp, data, logconf):
        """Handle height sensor data"""
        if 'range.zrange' in data and data['range.zrange'] > 0:
            # Convert mm to meters
            self.current_height = data['range.zrange'] / 1000.0
            
            # Process height for landing pad detection
            position = (self.current_position[0], self.current_position[1])
            self.detector.process_height(self.current_height, position)
    
    def reset_estimator(self, scf):
        """Reset Kalman filter"""
        scf.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        scf.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
        self.logger.info("✅ Kalman filter reset")


def main():
    """Main entry point"""
    # Configuration
    uri = "radio://0/88/2M/E7E7E7E7F0"
    
    # Create and run mission
    mission = SimpleLandingMission(uri)
    
    try:
        mission.run_mission()
    except KeyboardInterrupt:
        mission.logger.info("Mission interrupted by user")
    except Exception as e:
        mission.logger.error(f"Mission failed: {e}")


if __name__ == "__main__":
    main()
