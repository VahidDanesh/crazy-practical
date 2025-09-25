#!/usr/bin/env python3
"""
Test script for the Crazyflie Controller
Tests initialization and basic functionality without actual flight
"""

import sys
import logging
from pathlib import Path

# Add src directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from crazyflie_controller import CrazyflieController

def test_controller_initialization():
    """Test controller initialization"""
    print("🧪 Testing Controller Initialization...")
    
    try:
        controller = CrazyflieController()
        print("✅ Controller initialized successfully")
        
        # Test configuration loading
        print(f"✅ Config loaded: URI={controller.config['connection']['uri']}")
        print(f"✅ Default height: {controller.config['flight']['default_height']}m")
        print(f"✅ Controller type: {controller.config['flight']['controller']}")
        
        # Test landing detector initialization
        print(f"✅ Landing detector initialized: {type(controller.landing_detector).__name__}")
        print(f"✅ Search pattern initialized: {type(controller.search_pattern).__name__}")
        
        return True
        
    except Exception as e:
        print(f"❌ Controller initialization failed: {e}")
        return False

def test_search_pattern_generation():
    """Test search pattern generation"""
    print("\n🧪 Testing Search Pattern Generation...")
    
    try:
        controller = CrazyflieController()
        
        # Test grid pattern generation
        grid_points = controller.search_pattern.generate_grid_pattern(center=(0, 0))
        print(f"✅ Grid pattern generated: {len(grid_points)} waypoints")
        
        # Test spiral pattern generation  
        spiral_points = controller.search_pattern.generate_spiral_pattern(center=(0, 0))
        print(f"✅ Spiral pattern generated: {len(spiral_points)} waypoints")
        
        # Verify point format
        if grid_points:
            sample_point = grid_points[0]
            print(f"✅ Point format: ({sample_point[0]:.2f}, {sample_point[1]:.2f}, '{sample_point[2]}')")
        
        return True
        
    except Exception as e:
        print(f"❌ Search pattern generation failed: {e}")
        return False

def test_landing_detector_configuration():
    """Test landing detector configuration"""
    print("\n🧪 Testing Landing Detector Configuration...")
    
    try:
        controller = CrazyflieController()
        
        # Test configuration loading
        profile = controller.config['peak_detection']['active_profile']
        peak_params = controller.config['peak_detection'][profile]
        print(f"✅ Active profile: {profile}")
        print(f"✅ Peak detection params: {peak_params}")
        
        # Test detector configuration
        controller.landing_detector.configure_detection(peak_params)
        print("✅ Landing detector configured successfully")
        
        # Test detection start/stop
        controller.landing_detector.start_detection()
        print("✅ Landing detection started")
        
        controller.landing_detector.stop_detection()
        print("✅ Landing detection stopped")
        
        return True
        
    except Exception as e:
        print(f"❌ Landing detector configuration failed: {e}")
        return False

def test_safety_features():
    """Test safety feature configuration"""
    print("\n🧪 Testing Safety Features...")
    
    try:
        controller = CrazyflieController()
        
        # Test safety configuration
        safety_config = controller.config['safety']
        print(f"✅ Max flight time: {safety_config['max_flight_time']}s")
        print(f"✅ Battery threshold: {safety_config['battery_threshold']}V")
        print(f"✅ Connection timeout: {safety_config['connection_timeout']}s")
        
        # Test obstacle avoidance configuration
        obstacle_config = controller.config['obstacle_avoidance']
        print(f"✅ Min safe distance: {obstacle_config['min_safe_distance']}m")
        print(f"✅ Search velocity: {obstacle_config['search_velocity']}m/s")
        
        return True
        
    except Exception as e:
        print(f"❌ Safety features test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("🚀 Starting Crazyflie Controller Tests...\n")
    
    # Setup logging to see any issues
    logging.basicConfig(level=logging.WARNING)  # Reduce noise during testing
    
    # Run tests
    test1_pass = test_controller_initialization()
    test2_pass = test_search_pattern_generation()
    test3_pass = test_landing_detector_configuration()
    test4_pass = test_safety_features()
    
    # Summary
    print(f"\n📊 Test Results Summary:")
    print(f"  Controller Initialization: {'✅ PASS' if test1_pass else '❌ FAIL'}")
    print(f"  Search Pattern Generation: {'✅ PASS' if test2_pass else '❌ FAIL'}")
    print(f"  Landing Detector Config: {'✅ PASS' if test3_pass else '❌ FAIL'}")
    print(f"  Safety Features: {'✅ PASS' if test4_pass else '❌ FAIL'}")
    
    all_tests_pass = test1_pass and test2_pass and test3_pass and test4_pass
    print(f"\n🎯 Overall Result: {'✅ ALL TESTS PASS' if all_tests_pass else '❌ SOME TESTS FAILED'}")
    
    if all_tests_pass:
        print("\n🚁 Crazyflie Controller is ready for waypoint navigation and landing pad detection!")
        print("\nTo run the landing pad detection mission:")
        print("  python src/crazyflie_controller.py landing")
    else:
        print("\n⚠️  Please review failed tests before running missions.")

if __name__ == "__main__":
    main()
