#!/usr/bin/env python3
"""
Landing Pad Detection System
Enhanced peak detection for 10cm elevated landing pads

Features:
- Robust peak detection with noise filtering
- Multi-directional border collection
- Center calculation from border points
- Adaptive thresholding
"""

import numpy as np
import time
from collections import deque
from typing import List, Tuple, Optional, Dict
import logging


class PeakDetection:
    """Enhanced peak detection with noise filtering and adaptive thresholding"""
    
    def __init__(self, lag=10, threshold=2.5, influence=0.3, min_peak_height=0.06):
        self.lag = lag
        self.threshold = threshold
        self.influence = influence
        self.min_peak_height = min_peak_height  # Minimum height for valid peak
        
        # Data storage
        self.y = []
        self.signals = []
        self.filtered_y = []
        self.avg_filter = []
        self.std_filter = []
        
        # Enhancement features
        self.noise_buffer = deque(maxlen=7)  # Larger buffer for better noise reduction
        self.peak_detected = False
        self.last_peak_time = 0
        self.min_peak_interval = 0.3  # Reduced interval for better responsiveness
        
        # False positive reduction
        self.consecutive_high_readings = 0
        self.min_consecutive_readings = 3  # Need 3 consecutive high readings
        self.peak_confirmation_buffer = deque(maxlen=5)
        
        # Statistics for adaptive thresholding
        self.baseline_std = None
        self.baseline_mean = None
        
        self.logger = logging.getLogger(__name__)
    
    def _preprocess_value(self, new_value):
        """Apply noise reduction preprocessing with enhanced filtering"""
        # Add to noise buffer for median filtering
        self.noise_buffer.append(new_value)
        
        # Apply median filter to reduce noise
        if len(self.noise_buffer) >= 5:
            # Use median of recent values to reduce noise spikes
            filtered_value = np.median(list(self.noise_buffer))
        elif len(self.noise_buffer) >= 3:
            # Use smaller median for initial readings
            filtered_value = np.median(list(self.noise_buffer)[-3:])
        else:
            filtered_value = new_value
        
        return filtered_value
    
    def _check_upward_trend(self, current_idx, trend_window=3):
        """Check if we're in an upward trend over the last few readings"""
        if current_idx < trend_window:
            return True  # Not enough data, allow detection
        
        # Check if recent values show upward trend
        recent_values = self.y[current_idx-trend_window:current_idx+1]
        
        # Simple trend check: more increases than decreases
        increases = 0
        for i in range(1, len(recent_values)):
            if recent_values[i] > recent_values[i-1]:
                increases += 1
        
        # Require at least half of the changes to be increases
        return increases >= len(recent_values) // 2
    
    def _adaptive_threshold(self):
        """Calculate adaptive threshold based on signal characteristics"""
        if len(self.y) < self.lag * 2:
            return self.threshold
            
        # Calculate recent signal statistics
        recent_data = self.y[-self.lag*2:]
        current_std = np.std(recent_data)
        
        # Adapt threshold based on signal noise level
        if current_std > 0:
            adaptive_factor = min(max(current_std / 0.02, 0.5), 3.0)  # Scale factor
            return self.threshold * adaptive_factor
        else:
            return self.threshold
    
    def detect_peak(self, new_value):
        """Enhanced peak detection with preprocessing and adaptive thresholding"""
        current_time = time.time()
        
        # Preprocess the input value
        processed_value = self._preprocess_value(new_value)
        
        # Add to data
        self.y.append(processed_value)
        i = len(self.y) - 1
        
        # Extended initialization period for better baseline
        if i < self.lag * 2:  # Extended baseline period
            self.peak_detected = False
            return False
            
        if i == self.lag * 2:
            self.signals = [0] * len(self.y)
            self.filtered_y = self.y.copy()
            self.avg_filter = [0] * len(self.y)
            self.std_filter = [0] * len(self.y)
            
            # Use more data for better baseline estimation
            baseline_data = self.y[:self.lag * 2]
            self.avg_filter[i] = np.mean(baseline_data)
            self.std_filter[i] = np.std(baseline_data)
            
            # Set robust baseline statistics
            self.baseline_mean = self.avg_filter[i]
            self.baseline_std = self.std_filter[i]
            
            self.peak_detected = False
            return False
        
        # Extend arrays
        self.signals.append(0)
        self.filtered_y.append(0)
        self.avg_filter.append(0)
        self.std_filter.append(0)
        
        # Get adaptive threshold
        current_threshold = self._adaptive_threshold()
        
        # Enhanced peak detection logic with strict baseline validation
        baseline_reference = self.baseline_mean if self.baseline_mean is not None else self.avg_filter[i-1]
        
        # Check for actual height increase above baseline
        height_above_baseline = processed_value - baseline_reference
        statistical_deviation = abs(self.y[i] - self.avg_filter[i-1])
        threshold_value = current_threshold * self.std_filter[i-1]
        
        # First check: Must be significantly above baseline AND statistically significant
        baseline_exceeded = height_above_baseline > self.min_peak_height
        statistically_significant = statistical_deviation > threshold_value
        
        if baseline_exceeded and statistically_significant:
            # Check for positive trend (height increasing)
            if self.y[i] > self.avg_filter[i-1]:
                # Add trend validation - check if we're in an upward trend
                recent_trend = self._check_upward_trend(i)
                
                if recent_trend:
                    # Consecutive high readings for confirmation
                    self.consecutive_high_readings += 1
                    self.peak_confirmation_buffer.append(processed_value)
                    
                    # Time-based and consecutive reading validation
                    if (current_time - self.last_peak_time > self.min_peak_interval and 
                        self.consecutive_high_readings >= self.min_consecutive_readings):
                        
                        # Final validation with buffer median above baseline
                        buffer_median = np.median(list(self.peak_confirmation_buffer))
                        if buffer_median - baseline_reference > self.min_peak_height:
                            self.signals[i] = 1
                            self.peak_detected = True
                            self.last_peak_time = current_time
                            self.logger.info(f"Peak confirmed: height={processed_value:.3f}, "
                                           f"baseline={baseline_reference:.3f}, "
                                           f"above_baseline={height_above_baseline:.3f}, "
                                           f"consecutive={self.consecutive_high_readings}")
                        else:
                            self.signals[i] = 0
                            self.peak_detected = False
                    else:
                        self.signals[i] = 0
                        self.peak_detected = False
                else:
                    # No upward trend - reset consecutive counter
                    self.consecutive_high_readings = 0
                    self.signals[i] = 0
                    self.peak_detected = False
            else:
                # Negative deviation - reset consecutive counter
                self.consecutive_high_readings = 0
                self.signals[i] = -1
                self.peak_detected = False
                
            # Update filtered value with influence
            self.filtered_y[i] = (self.influence * self.y[i] + 
                                (1 - self.influence) * self.filtered_y[i-1])
        else:
            # Not above baseline or not statistically significant - reset consecutive counter
            self.consecutive_high_readings = max(0, self.consecutive_high_readings - 1)
            self.signals[i] = 0
            self.filtered_y[i] = self.y[i]
            self.peak_detected = False
        
        # Update moving average and std
        start_idx = max(0, i - self.lag)
        self.avg_filter[i] = np.mean(self.filtered_y[start_idx:i])
        self.std_filter[i] = np.std(self.filtered_y[start_idx:i])
        
        return self.peak_detected
    
    def reset(self):
        """Reset the detector for new detection session"""
        self.y.clear()
        self.signals.clear()
        self.filtered_y.clear()
        self.avg_filter.clear()
        self.std_filter.clear()
        self.noise_buffer.clear()
        self.peak_detected = False
        self.last_peak_time = 0
        self.baseline_std = None
        self.baseline_mean = None
        self.consecutive_high_readings = 0
        self.peak_confirmation_buffer.clear()


class LandingPadDetector:
    """Main landing pad detection system"""
    
    def __init__(self, pad_height_threshold=0.08, min_border_points=4):
        self.pad_height_threshold = pad_height_threshold  # 8cm for 10cm pad
        self.min_border_points = min_border_points
        
        # Peak detector
        self.peak_detector = PeakDetection(lag=8, threshold=2.0, influence=0.2)
        
        # Border collection for different flight directions
        self.border_points = {
            'north': [],  # Flying north (positive Y)
            'south': [],  # Flying south (negative Y) 
            'east': [],   # Flying east (positive X)
            'west': [],   # Flying west (negative X)
            'all': []     # All detected points
        }
        
        # Detection state
        self.baseline_height = None
        self.current_direction = 'unknown'
        self.on_pad = False
        self.detection_active = False
        
        # Spatial validation for false positive reduction
        self.recent_detections = deque(maxlen=10)  # Store recent detection positions
        self.min_spatial_consistency = 0.5  # Minimum distance consistency in meters
        
        # Center calculation
        self.calculated_center = None
        self.center_confidence = 0.0
        
        self.logger = logging.getLogger(__name__)
    
    def start_detection(self, baseline_height=None):
        """Start detection session"""
        self.detection_active = True
        self.baseline_height = baseline_height
        self.peak_detector.reset()
        self.clear_border_points()
        self.logger.info(f"Landing pad detection started (baseline: {baseline_height})")
    
    def configure_detection(self, params: dict):
        """Configure peak detection parameters from config"""
        self.peak_detector = PeakDetection(
            lag=params.get('lag', 10),
            threshold=params.get('threshold', 2.5),
            influence=params.get('influence', 0.3),
            min_peak_height=params.get('min_peak_height', 0.06)
        )
        self.pad_height_threshold = params.get('min_peak_height', 0.06)
        self.logger.info(f"Detection configured: {params}")
    
    def stop_detection(self):
        """Stop detection session"""
        self.detection_active = False
        self.logger.info("Landing pad detection stopped")
    
    def clear_border_points(self):
        """Clear all collected border points"""
        for direction in self.border_points:
            self.border_points[direction].clear()
        self.calculated_center = None
        self.center_confidence = 0.0
        self.recent_detections.clear()
    
    def set_flight_direction(self, direction: str):
        """Set current flight direction for border point classification"""
        valid_directions = ['north', 'south', 'east', 'west', 'unknown']
        if direction in valid_directions:
            self.current_direction = direction
        else:
            self.logger.warning(f"Invalid direction: {direction}")
    
    def process_height_measurement(self, height: float, position: Tuple[float, float]) -> bool:
        """
        Process height measurement and detect landing pad
        
        Args:
            height: Current height measurement from z-range sensor
            position: Current (x, y) position
            
        Returns:
            bool: True if peak detected
        """
        if not self.detection_active:
            return False
        
        # Set baseline if not set
        if self.baseline_height is None:
            self.baseline_height = height
            self.logger.info(f"Baseline height set to: {height:.3f}m")
            return False
        
        # Calculate relative height
        relative_height = height - self.baseline_height
        
        # Simple threshold detection for simulation and real flight
        # This detects when we're over an elevated surface
        threshold_detected = relative_height > self.pad_height_threshold
        
        # Debug logging
        if threshold_detected:
            self.logger.debug(f"Threshold detection: pos=({position[0]:.2f}, {position[1]:.2f}) "
                            f"height={height:.3f} relative={relative_height:.3f}")
        
        # Also use peak detection for additional robustness
        peak_detected = self.peak_detector.detect_peak(relative_height)
        
        # Detect pad if either method triggers
        pad_detected = threshold_detected or peak_detected
        
        if pad_detected:
            # Spatial validation to reduce false positives
            if self._validate_spatial_consistency(position):
                # Store border point with direction classification
                border_point = (position[0], position[1], relative_height)
                self.border_points[self.current_direction].append(border_point)
                self.border_points['all'].append(border_point)
                self.recent_detections.append(position)
                
                detection_method = "peak" if peak_detected else "threshold"
                self.logger.info(f"Border point detected ({detection_method}): {border_point} (direction: {self.current_direction})")
                return True
            else:
                self.logger.debug(f"Detection rejected due to spatial inconsistency at {position}")
                return False
        
        return False
    
    def _validate_spatial_consistency(self, position: Tuple[float, float]) -> bool:
        """Validate that detection is spatially consistent with recent detections"""
        if len(self.recent_detections) < 2:
            return True  # Not enough data for validation
        
        # Check if position is within reasonable distance of recent detections
        current_pos = np.array(position)
        recent_positions = np.array(self.recent_detections)
        
        # Calculate distances to recent detections
        distances = np.linalg.norm(recent_positions - current_pos, axis=1)
        min_distance = np.min(distances)
        
        # Allow detection if it's close to recent detections OR if it's the first in a new area
        # This allows for detecting landing pad borders while rejecting random noise
        if len(self.border_points['all']) == 0:
            return True  # First detection always allowed
        elif min_distance < self.min_spatial_consistency:
            return True  # Close to recent detections
        elif len(self.recent_detections) >= 5:
            # If we have enough detections, be more strict
            median_distance = np.median(distances[-5:])  # Last 5 detections
            return median_distance < self.min_spatial_consistency * 1.5
        else:
            return True  # Allow during initial detection phase
    
    def calculate_pad_center(self) -> Optional[Tuple[float, float]]:
        """
        Calculate landing pad center from collected border points
        
        Returns:
            Tuple[float, float]: (x, y) center coordinates or None
        """
        all_points = self.border_points['all']
        
        if len(all_points) < self.min_border_points:
            self.logger.warning(f"Insufficient border points: {len(all_points)} < {self.min_border_points}")
            return None
        
        # Extract x, y coordinates
        x_coords = [point[0] for point in all_points]
        y_coords = [point[1] for point in all_points]
        
        # Calculate center using median (more robust than mean)
        center_x = np.median(x_coords)
        center_y = np.median(y_coords)
        
        # Calculate confidence based on point distribution
        x_std = np.std(x_coords)
        y_std = np.std(y_coords)
        max_std = max(x_std, y_std)
        
        # Confidence decreases with higher standard deviation
        self.center_confidence = max(0.0, 1.0 - (max_std / 0.3))  # 30cm max spread
        
        self.calculated_center = (center_x, center_y)
        
        self.logger.info(f"Pad center calculated: ({center_x:.3f}, {center_y:.3f}) "
                        f"confidence: {self.center_confidence:.2f}")
        
        return self.calculated_center
    
    def get_detection_statistics(self) -> Dict:
        """Get current detection statistics"""
        stats = {
            'total_border_points': len(self.border_points['all']),
            'points_by_direction': {
                direction: len(points) 
                for direction, points in self.border_points.items() 
                if direction != 'all'
            },
            'calculated_center': self.calculated_center,
            'center_confidence': self.center_confidence,
            'detection_active': self.detection_active,
            'baseline_height': self.baseline_height
        }
        return stats
    
    def is_ready_for_landing(self, min_confidence=0.6) -> bool:
        """Check if enough data collected for confident landing"""
        return (self.calculated_center is not None and 
                self.center_confidence >= min_confidence and
                len(self.border_points['all']) >= self.min_border_points)


class SearchPattern:
    """Generate systematic search patterns for landing pad detection"""
    
    def __init__(self, search_area_size=2.0, step_size=0.3):
        self.search_area_size = search_area_size  # meters
        self.step_size = step_size  # meters between search points
        self.logger = logging.getLogger(__name__)
    
    def generate_grid_pattern(self, center=(0, 0)) -> List[Tuple[float, float, str]]:
        """
        Generate grid search pattern
        
        Returns:
            List of (x, y, direction) tuples
        """
        pattern = []
        half_size = self.search_area_size / 2
        
        # Create grid points
        x_points = np.arange(-half_size, half_size + self.step_size, self.step_size)
        y_points = np.arange(-half_size, half_size + self.step_size, self.step_size)
        
        # Generate serpentine pattern (more efficient)
        for i, y in enumerate(y_points):
            if i % 2 == 0:  # Even rows: left to right
                x_sequence = x_points
                direction_sequence = ['east'] * len(x_points)
            else:  # Odd rows: right to left
                x_sequence = x_points[::-1]
                direction_sequence = ['west'] * len(x_points)
            
            for j, x in enumerate(x_sequence):
                # Add movement direction
                if j < len(direction_sequence):
                    direction = direction_sequence[j]
                else:
                    direction = 'unknown'
                    
                pattern.append((center[0] + x, center[1] + y, direction))
        
        # Add north-south movements between rows
        enhanced_pattern = []
        for i, point in enumerate(pattern):
            enhanced_pattern.append(point)
            
            # Add north/south transition between rows
            if i < len(pattern) - 1:
                next_point = pattern[i + 1]
                if abs(point[1] - next_point[1]) > self.step_size * 0.5:  # Row transition
                    direction = 'north' if next_point[1] > point[1] else 'south'
                    enhanced_pattern.append((point[0], next_point[1], direction))
        
        self.logger.info(f"Generated search pattern with {len(enhanced_pattern)} waypoints")
        return enhanced_pattern
    
    def generate_spiral_pattern(self, center=(0, 0)) -> List[Tuple[float, float, str]]:
        """Generate spiral search pattern (alternative)"""
        pattern = []
        x, y = center
        
        # Spiral parameters
        direction_cycle = ['east', 'north', 'west', 'south']
        direction_idx = 0
        step_count = 1
        steps_in_direction = 0
        steps_before_turn = 1
        
        pattern.append((x, y, 'unknown'))
        
        while abs(x - center[0]) <= self.search_area_size/2 and abs(y - center[1]) <= self.search_area_size/2:
            # Move in current direction
            current_direction = direction_cycle[direction_idx]
            
            if current_direction == 'east':
                x += self.step_size
            elif current_direction == 'north':
                y += self.step_size
            elif current_direction == 'west':
                x -= self.step_size
            elif current_direction == 'south':
                y -= self.step_size
            
            pattern.append((x, y, current_direction))
            steps_in_direction += 1
            
            # Check if we need to turn
            if steps_in_direction >= steps_before_turn:
                direction_idx = (direction_idx + 1) % 4
                steps_in_direction = 0
                
                # Increase steps before next turn (every 2 turns)
                if direction_idx % 2 == 0:
                    steps_before_turn += 1
        
        self.logger.info(f"Generated spiral pattern with {len(pattern)} waypoints")
        return pattern
