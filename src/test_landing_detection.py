#!/usr/bin/env python3
"""
Test script for landing pad detection
Tests the detection algorithms with simulated data
"""

import numpy as np
import matplotlib.pyplot as plt
from landing_pad_detector import PeakDetection, LandingPadDetector, SearchPattern


def test_peak_detection():
    """Test the robust peak detection algorithm"""
    print("🧪 Testing Robust Peak Detection...")
    
    # Create detector with more sensitive settings for simulation
    detector = PeakDetection(lag=10, threshold=1.0, influence=0.1, min_peak_height=0.08)
    
    # Simulate height data: ground (0.4m) -> landing pad (0.5m) -> ground (0.4m)
    time_steps = 100
    ground_height = 0.4
    pad_height = 0.5
    noise_level = 0.005  # 5mm noise
    
    # Generate test signal
    heights = []
    signals = []
    
    for i in range(time_steps):
        # Create step function with noise
        if 30 <= i <= 60:  # Pad detection region
            true_height = pad_height
        else:
            true_height = ground_height
        
        # Add noise
        noisy_height = true_height + np.random.normal(0, noise_level)
        heights.append(noisy_height)
        
        # Test detection
        peak_detected = detector.detect_peak(noisy_height)
        signals.append(1 if peak_detected else 0)
        
        if peak_detected:
            print(f"  Peak detected at step {i}, height: {noisy_height:.4f}m")
    
    # Plot results
    plt.figure(figsize=(12, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(heights, 'b-', label='Height measurements', alpha=0.7)
    plt.axhline(y=ground_height, color='g', linestyle='--', alpha=0.5, label='Ground level')
    plt.axhline(y=pad_height, color='r', linestyle='--', alpha=0.5, label='Pad level')
    plt.ylabel('Height (m)')
    plt.title('Simulated Height Measurements')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.subplot(2, 1, 2)
    plt.plot(signals, 'ro-', markersize=4, label='Peak detections')
    plt.ylabel('Peak Detected')
    plt.xlabel('Time Steps')
    plt.title('Peak Detection Results')
    plt.ylim(-0.1, 1.1)
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('/home/vahid/repos/crazyflie/crazy-practical/logs/peak_detection_test.png')
    plt.show()
    
    # Statistics
    detections_in_pad_region = sum(signals[30:61])
    detections_outside_pad = sum(signals[:30]) + sum(signals[61:])
    
    print(f"  Detections in pad region (30-60): {detections_in_pad_region}")
    print(f"  False detections outside pad: {detections_outside_pad}")
    print(f"  Success rate: {detections_in_pad_region > 0 and detections_outside_pad < 3}")
    
    return detections_in_pad_region > 0 and detections_outside_pad < 3


def test_landing_pad_detector():
    """Test the complete landing pad detection system"""
    print("\n🧪 Testing Landing Pad Detector...")
    
    # Create detector with appropriate threshold for 10cm pad elevation
    detector = LandingPadDetector(pad_height_threshold=0.07, min_border_points=4)
    
    # Start detection
    detector.start_detection(baseline_height=0.4)
    
    # Simulate flight over landing pad
    # Create a 30cm x 30cm landing pad at position (1.0, 1.5)
    pad_center = (1.0, 1.5)
    pad_size = 0.3  # 30cm
    ground_height = 0.4
    pad_height = 0.5
    
    # Simulate grid search pattern with better coverage
    search_pattern = SearchPattern(search_area_size=1.5, step_size=0.1)
    search_points = search_pattern.generate_grid_pattern(center=(1.0, 1.5))
    
    detected_points = []
    points_over_pad = 0
    points_detected_over_pad = 0
    
    print(f"  Simulating flight over {len(search_points)} search points...")
    
    for i, (x, y, direction) in enumerate(search_points):
        detector.set_flight_direction(direction)
        
        # Check if current position is over the landing pad
        dx = abs(x - pad_center[0])
        dy = abs(y - pad_center[1])
        over_pad = dx <= pad_size/2 and dy <= pad_size/2
        
        if over_pad:
            # Over the landing pad
            points_over_pad += 1
            height = pad_height + np.random.normal(0, 0.005)  # Add noise
        else:
            # Over ground
            height = ground_height + np.random.normal(0, 0.005)
        
        # Process height measurement
        detected = detector.process_height_measurement(height, (x, y))
        if detected:
            detected_points.append((x, y))
            if over_pad:
                points_detected_over_pad += 1
                print(f"    ✅ Border point detected at ({x:.2f}, {y:.2f}) direction: {direction} height: {height:.3f}")
            else:
                print(f"    ❌ FALSE POSITIVE at ({x:.2f}, {y:.2f}) direction: {direction} height: {height:.3f}")
    
    print(f"  Summary: {points_over_pad} points over pad, {points_detected_over_pad} detected correctly")
    
    # Calculate center
    center = detector.calculate_pad_center()
    stats = detector.get_detection_statistics()
    
    print(f"  Detection Results:")
    print(f"    Total border points: {stats['total_border_points']}")
    print(f"    Points by direction: {stats['points_by_direction']}")
    print(f"    Calculated center: {center}")
    print(f"    True center: {pad_center}")
    print(f"    Center confidence: {stats['center_confidence']:.2f}")
    print(f"    Ready for landing: {detector.is_ready_for_landing()}")
    
    # Calculate accuracy
    if center:
        center_error = np.sqrt((center[0] - pad_center[0])**2 + (center[1] - pad_center[1])**2)
        print(f"    Center error: {center_error:.3f}m")
        accuracy_good = center_error < 0.15  # Within 15cm
        confidence_good = stats['center_confidence'] > 0.2  # Lower threshold for test
    else:
        accuracy_good = False
        confidence_good = False
        print(f"    Center error: No center calculated")
    
    # Visualize results
    plt.figure(figsize=(10, 8))
    
    # Plot search pattern
    search_x = [point[0] for point in search_points]
    search_y = [point[1] for point in search_points]
    plt.plot(search_x, search_y, 'b.', alpha=0.5, markersize=4, label='Search points')
    
    # Plot landing pad
    pad_rect = plt.Rectangle((pad_center[0] - pad_size/2, pad_center[1] - pad_size/2), 
                           pad_size, pad_size, fill=False, edgecolor='red', linewidth=2, label='True landing pad')
    plt.gca().add_patch(pad_rect)
    
    # Plot detected border points
    if detected_points:
        det_x = [point[0] for point in detected_points]
        det_y = [point[1] for point in detected_points]
        plt.plot(det_x, det_y, 'go', markersize=8, label='Detected border points')
    
    # Plot calculated center
    if center:
        plt.plot(center[0], center[1], 'r*', markersize=15, label='Calculated center')
    
    # Plot true center
    plt.plot(pad_center[0], pad_center[1], 'k+', markersize=15, label='True center')
    
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Landing Pad Detection Test Results')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.savefig('/home/vahid/repos/crazyflie/crazy-practical/logs/landing_detection_test.png')
    plt.show()
    
    detector.stop_detection()
    
    return accuracy_good and confidence_good and stats['total_border_points'] >= 4


def test_search_patterns():
    """Test search pattern generation"""
    print("\n🧪 Testing Search Patterns...")
    
    search_pattern = SearchPattern(search_area_size=2.0, step_size=0.3)
    
    # Test grid pattern
    grid_points = search_pattern.generate_grid_pattern(center=(0, 0))
    print(f"  Grid pattern: {len(grid_points)} points")
    
    # Test spiral pattern  
    spiral_points = search_pattern.generate_spiral_pattern(center=(0, 0))
    print(f"  Spiral pattern: {len(spiral_points)} points")
    
    # Visualize patterns
    plt.figure(figsize=(12, 5))
    
    plt.subplot(1, 2, 1)
    grid_x = [p[0] for p in grid_points]
    grid_y = [p[1] for p in grid_points]
    plt.plot(grid_x, grid_y, 'bo-', markersize=4, linewidth=1)
    plt.title('Grid Search Pattern')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    plt.subplot(1, 2, 2)
    spiral_x = [p[0] for p in spiral_points]
    spiral_y = [p[1] for p in spiral_points]
    plt.plot(spiral_x, spiral_y, 'ro-', markersize=4, linewidth=1)
    plt.title('Spiral Search Pattern')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    plt.tight_layout()
    plt.savefig('/home/vahid/repos/crazyflie/crazy-practical/logs/search_patterns_test.png')
    plt.show()
    
    return len(grid_points) > 0 and len(spiral_points) > 0


def main():
    """Run all tests"""
    print("🚀 Starting Landing Pad Detection Tests...\n")
    
    # Run tests
    test1_pass = test_peak_detection()
    test2_pass = test_landing_pad_detector()
    test3_pass = test_search_patterns()
    
    # Summary
    print(f"\n📊 Test Results Summary:")
    print(f"  Peak Detection Test: {'✅ PASS' if test1_pass else '❌ FAIL'}")
    print(f"  Landing Pad Detector Test: {'✅ PASS' if test2_pass else '❌ FAIL'}")
    print(f"  Search Patterns Test: {'✅ PASS' if test3_pass else '❌ FAIL'}")
    
    all_tests_pass = test1_pass and test2_pass and test3_pass
    print(f"\n🎯 Overall Result: {'✅ ALL TESTS PASS' if all_tests_pass else '❌ SOME TESTS FAILED'}")
    
    if all_tests_pass:
        print("\n🚁 Landing pad detection system is ready for flight testing!")
    else:
        print("\n⚠️  Please review failed tests before flight testing.")


if __name__ == "__main__":
    main()
