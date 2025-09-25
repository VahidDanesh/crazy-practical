# CFPilot - Crazyflie Autonomous Flight System

An advanced autonomous flight system for the Crazyflie platform with comprehensive safety features, obstacle avoidance, and landing pad detection capabilities.

## Installation

### From Source

1. **Clone and install:**
   ```bash
   git clone https://github.com/VahidDanesh/cfpilot.git
   cd cfpilot
   pip install -e .
   ```

2. **Or install with development dependencies:**
   ```bash
   pip install -e ".[dev]"
   ```

### Quick Start

1. **Configure your Crazyflie URI in:**
   ```bash
   cfpilot/config/flight_config.yaml
   ```

2. **Run different flight missions:**
   ```bash
   # Basic flight test
   cfpilot basic
   
   # Sensor exploration
   cfpilot sensor
   
   # Landing pad detection and autonomous landing
   cfpilot landing
   
   # Use custom config
   cfpilot basic --config my_config.yaml
   
   # Set log level
   cfpilot landing --log-level DEBUG
   ```

## Flight Missions

### 1. Basic Flight (`cfpilot basic`)
- Simple takeoff, hover, and landing
- Position hold using PositionHlCommander
- Comprehensive safety features

### 2. Sensor Exploration (`cfpilot sensor`)
- Multiranger sensor-based navigation
- Obstacle detection and avoidance
- Automatic free-space exploration

### 3. Landing Pad Detection (`cfpilot landing`)
- **Waypoint-based search pattern** using PositionHlCommander
- **Real-time landing pad detection** using z-range sensor
- **Peak detection algorithm** to identify platform edges
- **Precision landing** on detected platforms
- **Obstacle avoidance** during search

## Safety Features

- Emergency landing on connection loss
- Automatic timeout protection  
- Battery monitoring with auto-landing
- Ctrl+C emergency shutdown
- Real-time obstacle detection
- Flight data logging with CSV export

## Landing Pad Detection System

The landing pad detection uses advanced algorithms to identify elevated platforms:

- **Z-range sensor** for height measurements
- **Peak detection** with configurable sensitivity
- **Waypoint navigation** for systematic search
- **Center calculation** from detected border points
- **Confidence scoring** for landing decisions

### Detection Parameters (configurable in `config/flight_config.yaml`):
- `lag`: Running average window size
- `threshold`: Peak detection sensitivity
- `influence`: Baseline adaptation rate
- `min_peak_height`: Minimum platform height (meters)

## Development

### Running Tests

```bash
# Run all tests
python -m pytest tests/

# Run specific test file
python -m pytest tests/test_detection.py

# Run with coverage
python -m pytest tests/ --cov=cfpilot
```

### Code Formatting

```bash
# Format code
black cfpilot/ tests/

# Check formatting
black --check cfpilot/ tests/

# Sort imports
isort cfpilot/ tests/
```

## Project Structure

```
crazy-practical/
├── cfpilot/                         # Main package
│   ├── __init__.py                  # Package initialization
│   ├── controller.py                # Core controller class
│   ├── detection.py                 # Landing pad detection
│   ├── missions.py                  # Flight missions
│   ├── cli.py                       # Command line interface
│   └── config/
│       └── flight_config.yaml       # Default configuration
├── tests/                           # Test suite
│   ├── __init__.py
│   ├── test_basic_flight.py
│   ├── test_detection.py
│   └── test_missions.py
├── logs/                            # Flight data logs
├── setup.py                         # Package setup
├── pyproject.toml                   # Build configuration
├── requirements.txt                 # Dependencies
└── README.md                        # This file
```

## Package Architecture

- **`controller.py`**: Core CrazyflieController class with clean, modular design
- **`detection.py`**: LandingPadDetector and SearchPattern classes for autonomous landing
- **`missions.py`**: Modular mission classes (BasicFlightMission, SensorExplorationMission, etc.)
- **`cli.py`**: Command-line interface for easy mission execution
- **`tests/`**: Comprehensive test suite with unit tests for all components