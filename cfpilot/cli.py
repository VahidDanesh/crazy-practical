"""
Command Line Interface for CFPilot

Provides command-line access to different flight missions.
"""

import argparse
import sys
import logging
from typing import Optional

from .controller import CrazyflieController
from .missions import BasicFlightMission, SensorExplorationMission, LandingPadDetectionMission


def setup_logging(level: str = "INFO") -> None:
    """Setup logging configuration"""
    numeric_level = getattr(logging, level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError(f'Invalid log level: {level}')
    
    logging.basicConfig(
        level=numeric_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


def run_basic_flight(config_path: Optional[str] = None) -> int:
    """Run basic flight mission"""
    try:
        controller = CrazyflieController(config_path)
        mission = BasicFlightMission(controller)
        mission.execute()
        return 0
    except KeyboardInterrupt:
        logging.info("Flight interrupted by user")
        return 1
    except Exception as e:
        logging.error(f"Flight failed: {e}")
        return 1


def run_sensor_exploration(config_path: Optional[str] = None) -> int:
    """Run sensor exploration mission"""
    try:
        controller = CrazyflieController(config_path)
        mission = SensorExplorationMission(controller)
        mission.execute()
        return 0
    except KeyboardInterrupt:
        logging.info("Flight interrupted by user")
        return 1
    except Exception as e:
        logging.error(f"Flight failed: {e}")
        return 1


def run_landing_pad_detection(config_path: Optional[str] = None) -> int:
    """Run landing pad detection mission"""
    try:
        controller = CrazyflieController(config_path)
        mission = LandingPadDetectionMission(controller)
        mission.execute()
        return 0
    except KeyboardInterrupt:
        logging.info("Flight interrupted by user")
        return 1
    except Exception as e:
        logging.error(f"Flight failed: {e}")
        return 1


def main() -> None:
    """Main CLI entry point"""
    parser = argparse.ArgumentParser(
        description="CFPilot - Autonomous Crazyflie Flight System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  cfpilot basic                    # Basic takeoff-hover-land
  cfpilot sensor                   # Sensor-based exploration
  cfpilot landing                  # Landing pad detection
  cfpilot basic --config my.yaml  # Use custom config file
        """
    )
    
    parser.add_argument(
        'mission',
        choices=['basic', 'sensor', 'landing'],
        help='Mission type to execute'
    )
    
    parser.add_argument(
        '--config', '-c',
        type=str,
        help='Path to configuration file'
    )
    
    parser.add_argument(
        '--log-level', '-l',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
        default='INFO',
        help='Set logging level'
    )
    
    parser.add_argument(
        '--version',
        action='version',
        version='CFPilot 1.0.0'
    )
    
    args = parser.parse_args()
    
    # Setup logging
    setup_logging(args.log_level)
    
    # Execute mission
    if args.mission == 'basic':
        exit_code = run_basic_flight(args.config)
    elif args.mission == 'sensor':
        exit_code = run_sensor_exploration(args.config)
    elif args.mission == 'landing':
        exit_code = run_landing_pad_detection(args.config)
    else:
        parser.error(f"Unknown mission: {args.mission}")
    
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
