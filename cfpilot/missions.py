"""
Flight Missions Module - Async API

Different autonomous flight missions for the Crazyflie platform using async API.
"""

import time
import threading
import logging
from typing import Optional

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

from .controller import CrazyflieController


class BaseMission:
    """Base class for all flight missions"""
    
    def __init__(self, controller: CrazyflieController):
        self.controller = controller
        self.logger = self.controller.logger
    
    def run(self, uri: str) -> None:
        """Run the mission"""
        cflib.crtp.init_drivers()
        
        try:
            self.controller.connect(uri)
            if self.controller.wait_for_connection():
                self.execute()
            else:
                self.logger.error("Failed to connect")
        finally:
            self.controller.disconnect()
            self.controller.cleanup()
    
    def execute(self) -> None:
        """Execute the mission - to be implemented by subclasses"""
        raise NotImplementedError("Subclasses must implement execute method")


class BasicFlightMission(BaseMission):
    """Basic takeoff-hover-land mission"""
    
    def execute(self) -> None:
        """Execute basic flight mission"""
        self.controller.setup_safety_timer()
        self.controller.setup_plotter()
        
        pc = PositionHlCommander(self.controller.cf, 
                               default_velocity=self.controller.config['flight']['default_velocity'],
                               default_height=self.controller.config['flight']['takeoff_height'])
        
        self.logger.info("Taking off...")
        pc.take_off(height=self.controller.config['flight']['takeoff_height'])
        time.sleep(2)
        
        # Hover for 5 seconds
        start_time = time.time()
        while time.time() - start_time < 5.0 and not self.controller.emergency_triggered:
            time.sleep(0.1)
        
        self.logger.info("Landing...")
        pc.land()
        time.sleep(2)


class SensorExplorationMission(BaseMission):
    """Mission for exploring environment with multiranger sensors"""
    
    def execute(self) -> None:
        """Execute sensor exploration mission"""
        self.controller.setup_safety_timer()
        self.controller.setup_plotter()
        
        pc = PositionHlCommander(self.controller.cf, 
                               default_velocity=self.controller.config['flight']['default_velocity'],
                               default_height=self.controller.config['flight']['takeoff_height'])
        
        multiranger = Multiranger(self.controller.cf)
        multiranger.start()
        
        # Add data callback
        def sensor_callback(timestamp, data, logconf_name):
            if 'range.front' in data:
                self.logger.info(f"Sensors - F:{data.get('range.front')} B:{data.get('range.back')} "
                               f"L:{data.get('range.left')} R:{data.get('range.right')}")
        
        self.controller.add_data_callback(sensor_callback)
        
        self.logger.info("Taking off...")
        pc.take_off(height=self.controller.config['flight']['takeoff_height'])
        time.sleep(2)
        
        # Movement pattern
        positions = [
            (0.5, 0, 0.5),
            (0.5, 0.5, 0.5),
            (0, 0.5, 0.5),
            (-0.5, 0.5, 0.5),
            (-0.5, 0, 0.5),
            (-0.5, -0.5, 0.5),
            (0, -0.5, 0.5),
            (0.5, -0.5, 0.5),
            (0, 0, 0.5)
        ]
        
        for i, (x, y, z) in enumerate(positions):
            if self.controller.emergency_triggered:
                break
                
            self.logger.info(f"Moving to position {i + 1}: ({x}, {y}, {z})")
            pc.go_to(x, y, z)
            time.sleep(3)
        
        self.logger.info("Landing...")
        pc.land()
        time.sleep(2)
        
        multiranger.stop()
        self.controller.remove_data_callback(sensor_callback)


class LandingPadDetectionMission(BaseMission):
    """Mission for searching and landing on a landing pad"""
    
    def execute(self) -> None:
        """Execute landing pad detection mission"""
        self.controller.setup_safety_timer()
        self.controller.setup_plotter()
        
        pc = PositionHlCommander(self.controller.cf, 
                               default_velocity=self.controller.config['flight']['default_velocity'],
                               default_height=self.controller.config['flight']['takeoff_height'])
        
        multiranger = Multiranger(self.controller.cf)
        multiranger.start()
        
        landing_pad_found = False
        
        # Add data callback for landing detection
        def detection_callback(timestamp, data, logconf_name):
            if self.controller.landing_detector.landing_pad_detected:
                self.logger.info("Landing pad detected!")
        
        self.controller.add_data_callback(detection_callback)
        
        self.logger.info("Taking off...")
        pc.take_off(height=self.controller.config['flight']['takeoff_height'])
        time.sleep(2)
        
        # Search pattern
        search_positions = [
            (0.5, 0.5, 1.0),
            (-0.5, 0.5, 1.0),
            (-0.5, -0.5, 1.0),
            (0.5, -0.5, 1.0),
            (1.0, 0, 1.0),
            (0, 1.0, 1.0),
            (-1.0, 0, 1.0),
            (0, -1.0, 1.0)
        ]
        
        for i, (x, y, z) in enumerate(search_positions):
            if self.controller.emergency_triggered:
                break
                
            self.logger.info(f"Searching position {i + 1}: ({x}, {y}, {z})")
            pc.go_to(x, y, z)
            time.sleep(4)
            
            # Check if landing pad detected
            if self.controller.landing_detector.landing_pad_detected:
                landing_position = self.controller.landing_detector.get_landing_position()
                if landing_position:
                    self.logger.info(f"Landing at detected position: {landing_position}")
                    pc.go_to(landing_position[0], landing_position[1], 0.3)
                    time.sleep(3)
                    pc.land()
                    landing_pad_found = True
                    break
        
        if not landing_pad_found:
            self.logger.info("Landing pad not found - landing at current position")
            pc.land()
        
        time.sleep(2)
        
        multiranger.stop()
        self.controller.remove_data_callback(detection_callback)