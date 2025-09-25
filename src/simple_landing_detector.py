#!/usr/bin/env python3
"""
Simple Landing Pad Detection
Clean implementation based on height changes detection
"""

import numpy as np
from collections import deque
import logging


class SimpleLandingDetector:
    """Simple landing pad detector using height threshold"""
    
    def __init__(self, height_threshold=0.06, min_detections=4):
        """
        Initialize detector
        
        Args:
            height_threshold: Minimum height increase to detect landing pad (meters)
            min_detections: Minimum detections needed for reliable center calculation
        """
        self.height_threshold = height_threshold
        self.min_detections = min_detections
        
        # State
        self.baseline_height = None
        self.detection_active = False
        
        # Store detected border points by direction
        self.detected_borders = {
            'north': [],  # Moving north (positive Y)
            'south': [],  # Moving south (negative Y)
            'east': [],   # Moving east (positive X)
            'west': []    # Moving west (negative X)
        }
        
        # Current movement direction
        self.current_direction = 'unknown'
        
        self.logger = logging.getLogger(__name__)
    
    def start_detection(self, baseline_height=None):
        """Start detection with optional baseline"""
        self.detection_active = True
        self.baseline_height = baseline_height
        self.clear_detections()
        self.logger.info(f"Detection started with baseline: {baseline_height}")
    
    def stop_detection(self):
        """Stop detection"""
        self.detection_active = False
        self.logger.info("Detection stopped")
    
    def clear_detections(self):
        """Clear all detected points"""
        for direction in self.detected_borders:
            self.detected_borders[direction].clear()
    
    def set_direction(self, direction):
        """Set current movement direction"""
        if direction in self.detected_borders:
            self.current_direction = direction
    
    def process_height(self, height, position):
        """
        Process height measurement at given position
        
        Args:
            height: Current height measurement (meters)
            position: Current (x, y) position tuple
            
        Returns:
            bool: True if landing pad detected at this position
        """
        if not self.detection_active:
            return False
        
        # Set baseline on first measurement
        if self.baseline_height is None:
            self.baseline_height = height
            self.logger.info(f"Baseline height set: {height:.3f}m")
            return False
        
        # Check if height indicates landing pad
        height_above_baseline = height - self.baseline_height
        
        if height_above_baseline > self.height_threshold:
            # Landing pad detected
            self.detected_borders[self.current_direction].append(position)
            self.logger.info(f"Border detected: {position} (direction: {self.current_direction}, height: +{height_above_baseline:.3f}m)")
            return True
        
        return False
    
    def get_detection_count(self):
        """Get total number of detections"""
        return sum(len(points) for points in self.detected_borders.values())
    
    def calculate_center(self):
        """
        Calculate landing pad center from detected border points
        
        Returns:
            tuple: (x, y) center coordinates or None if insufficient data
        """
        all_points = []
        for points in self.detected_borders.values():
            all_points.extend(points)
        
        if len(all_points) < self.min_detections:
            return None
        
        # Calculate center as median of all detected points
        x_coords = [p[0] for p in all_points]
        y_coords = [p[1] for p in all_points]
        
        center_x = np.median(x_coords)
        center_y = np.median(y_coords)
        
        self.logger.info(f"Landing pad center: ({center_x:.3f}, {center_y:.3f}) from {len(all_points)} points")
        return (center_x, center_y)
    
    def is_ready_for_landing(self):
        """Check if enough detections for reliable landing"""
        return self.get_detection_count() >= self.min_detections
    
    def get_statistics(self):
        """Get detection statistics"""
        return {
            'total_detections': self.get_detection_count(),
            'detections_by_direction': {k: len(v) for k, v in self.detected_borders.items()},
            'baseline_height': self.baseline_height,
            'detection_active': self.detection_active,
            'calculated_center': self.calculate_center()
        }


class SimpleSearchPattern:
    """Generate simple search patterns"""
    
    def __init__(self, area_size=1.5, step_size=0.3):
        self.area_size = area_size
        self.step_size = step_size
    
    def generate_grid_search(self, center=(0, 0)):
        """
        Generate simple grid search pattern
        
        Returns:
            List of (x, y, direction) waypoints
        """
        waypoints = []
        half_area = self.area_size / 2
        
        # Create grid points
        y_positions = np.arange(-half_area, half_area + self.step_size, self.step_size)
        
        for i, y in enumerate(y_positions):
            if i % 2 == 0:  # Even rows: west to east
                x_positions = np.arange(-half_area, half_area + self.step_size, self.step_size)
                direction = 'east'
            else:  # Odd rows: east to west
                x_positions = np.arange(half_area, -half_area - self.step_size, -self.step_size)
                direction = 'west'
            
            for x in x_positions:
                waypoints.append((center[0] + x, center[1] + y, direction))
        
        return waypoints


def determine_movement_direction(current_pos, next_pos):
    """
    Determine movement direction between two positions
    
    Args:
        current_pos: (x, y) current position
        next_pos: (x, y) next position
        
    Returns:
        str: Direction ('north', 'south', 'east', 'west')
    """
    dx = next_pos[0] - current_pos[0]
    dy = next_pos[1] - current_pos[1]
    
    if abs(dx) > abs(dy):
        return 'east' if dx > 0 else 'west'
    else:
        return 'north' if dy > 0 else 'south'
