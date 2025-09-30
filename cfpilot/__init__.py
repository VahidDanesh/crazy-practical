"""
CFPilot - Crazyflie Autonomous Flight Package

An advanced autonomous flight system for the Crazyflie platform with comprehensive 
safety features, obstacle avoidance, and landing pad detection capabilities.
"""

__version__ = "1.0.0"
__author__ = "Crazyflie Autonomous Team"

from .controller import CrazyflieController
from .detection import LandingPadDetector, SearchPattern, GridMap, CellState
from .missions import BasicFlightMission, SensorExplorationMission, LandingPadDetectionMission

try:
    from .visualization import PointCloudPlotter, GridMapVisualizer
    VISUALIZATION_AVAILABLE = True
    __all__ = [
        "CrazyflieController", "LandingPadDetector", "SearchPattern", "GridMap", "CellState",
        "BasicFlightMission", "SensorExplorationMission", "LandingPadDetectionMission",
        "PointCloudPlotter", "GridMapVisualizer"
    ]
except ImportError as e:
    # Visualization not available - create placeholder classes
    VISUALIZATION_AVAILABLE = False
    
    class GridMapVisualizer:
        def __init__(self):
            print("Warning: Visualization dependencies not available")
        def plot_grid_map(self, *args, **kwargs):
            print("Visualization not available - install matplotlib and vispy")
        def save_grid_data(self, *args, **kwargs):
            print("Visualization not available - install matplotlib")
    
    __all__ = [
        "CrazyflieController", "LandingPadDetector", "SearchPattern", "GridMap", "CellState",
        "BasicFlightMission", "SensorExplorationMission", "LandingPadDetectionMission",
        "GridMapVisualizer"
    ]
