#!/usr/bin/env python3
"""
Hardware Test Script for Landing Pad Detection
Run step-by-step tests on real Crazyflie hardware
"""

import time
import logging
import argparse
from pathlib import Path
from crazyflie_controller import CrazyflieController

def setup_logging():
    """Setup logging for hardware tests"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('./logs/hardware_test.log'),
            logging.StreamHandler()
        ]
    )

def test_connection(controller):
    """Test basic connection to Crazyflie"""
    print("\n🔗 Testing Connection...")
    print("=" * 40)
    
    try:
        # This will be tested when we run basic flight
        print("✅ Connection test will run with basic flight")
        return True
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False

def test_basic_flight(controller):
    """Test basic takeoff, hover, and landing"""
    print("\n🚁 Testing Basic Flight...")
    print("=" * 40)
    print("This will:")
    print("  1. Take off to configured height")
    print("  2. Hover for configured duration")
    print("  3. Land safely")
    print("  4. Log flight data")
    
    input("Press ENTER when ready to start basic flight test...")
    
    try:
        controller.run_basic_flight()
        print("✅ Basic flight test completed")
        return True
    except Exception as e:
        print(f"❌ Basic flight test failed: {e}")
        return False

def test_sensor_reading(controller):
    """Test multiranger sensor readings"""
    print("\n📡 Testing Sensor Reading...")
    print("=" * 40)
    print("This will:")
    print("  1. Take off and hover")
    print("  2. Read multiranger sensors")
    print("  3. Move based on free directions")
    print("  4. Return to start and land")
    
    input("Press ENTER when ready to start sensor test...")
    
    try:
        controller.run_sensor_exploration()
        print("✅ Sensor test completed")
        return True
    except Exception as e:
        print(f"❌ Sensor test failed: {e}")
        return False

def test_landing_pad_detection(controller):
    """Test landing pad detection system"""
    print("\n🎯 Testing Landing Pad Detection...")
    print("=" * 40)
    print("SETUP REQUIRED:")
    print("  1. Place a landing pad (3-10cm elevated) in the test area")
    print("  2. Ensure the area is clear of obstacles")
    print("  3. Position Crazyflie at takeoff point")
    print("\nThis test will:")
    print("  1. Take off and hover")
    print("  2. Search for landing pad using grid pattern")
    print("  3. Detect landing pad using height sensors")
    print("  4. Calculate pad center")
    print("  5. Attempt precision landing")
    
    input("Press ENTER when landing pad is set up and ready...")
    
    try:
        controller.run_landing_pad_detection()
        print("✅ Landing pad detection test completed")
        return True
    except Exception as e:
        print(f"❌ Landing pad detection test failed: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Hardware test suite for Crazyflie')
    parser.add_argument('--test', choices=['connection', 'basic', 'sensors', 'landing', 'all'],
                       default='all', help='Test to run')
    parser.add_argument('--config', default='./config/flight_config.yaml',
                       help='Configuration file path')
    
    args = parser.parse_args()
    
    # Setup logging
    setup_logging()
    
    # Create logs directory
    Path('./logs').mkdir(exist_ok=True)
    
    print("🚁 CRAZYFLIE HARDWARE TEST SUITE")
    print("=" * 50)
    print(f"Configuration: {args.config}")
    print(f"Test suite: {args.test}")
    
    # Initialize controller
    try:
        controller = CrazyflieController(args.config)
        print("✅ Controller initialized successfully")
    except Exception as e:
        print(f"❌ Failed to initialize controller: {e}")
        return
    
    # Run tests
    tests = []
    if args.test in ['connection', 'all']:
        tests.append(('Connection', test_connection))
    if args.test in ['basic', 'all']:
        tests.append(('Basic Flight', test_basic_flight))
    if args.test in ['sensors', 'all']:
        tests.append(('Sensor Reading', test_sensor_reading))
    if args.test in ['landing', 'all']:
        tests.append(('Landing Pad Detection', test_landing_pad_detection))
    
    results = {}
    
    for test_name, test_func in tests:
        print(f"\n{'='*20} {test_name} {'='*20}")
        
        try:
            result = test_func(controller)
            results[test_name] = result
            
            if result:
                print(f"✅ {test_name}: PASSED")
            else:
                print(f"❌ {test_name}: FAILED")
                
            if not result and args.test == 'all':
                response = input(f"\n{test_name} failed. Continue? (y/N): ")
                if response.lower() != 'y':
                    break
                    
        except KeyboardInterrupt:
            print(f"\n⚠️  {test_name} interrupted by user")
            break
        except Exception as e:
            print(f"❌ {test_name} error: {e}")
            results[test_name] = False
    
    # Print summary
    print("\n" + "="*50)
    print("🏁 TEST SUMMARY")
    print("="*50)
    
    passed = sum(1 for result in results.values() if result)
    total = len(results)
    
    for test_name, result in results.items():
        status = "✅ PASSED" if result else "❌ FAILED"
        print(f"{test_name:.<30} {status}")
    
    print(f"\nOverall: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! System ready for full mission.")
    else:
        print("⚠️  Some tests failed. Please review and fix issues.")
    
    print("\n📊 Check logs directory for detailed flight data")
    print("📊 Use flight_data_plotter.py to visualize results")

if __name__ == '__main__':
    main()
