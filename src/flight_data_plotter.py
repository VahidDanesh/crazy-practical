#!/usr/bin/env python3
"""
Flight Data Visualization Tool
Plots flight data from Crazyflie landing pad detection missions
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import argparse
from datetime import datetime
import json

class FlightDataPlotter:
    """Visualize flight data and landing pad detection results"""
    
    def __init__(self, log_file: str):
        self.log_file = Path(log_file)
        self.data = None
        self.detection_events = []
        
    def load_data(self):
        """Load flight data from CSV log file"""
        try:
            self.data = pd.read_csv(self.log_file)
            print(f"✅ Loaded {len(self.data)} data points from {self.log_file}")
            print(f"📊 Columns: {list(self.data.columns)}")
            
            # Convert timestamp if exists
            if 'timestamp' in self.data.columns:
                self.data['timestamp'] = pd.to_datetime(self.data['timestamp'])
                self.data['time_elapsed'] = (self.data['timestamp'] - self.data['timestamp'].iloc[0]).dt.total_seconds()
            else:
                # Create time index based on logging frequency
                self.data['time_elapsed'] = np.arange(len(self.data)) * 0.01  # 10ms logging
                
        except Exception as e:
            print(f"❌ Error loading data: {e}")
            return False
        return True
    
    def plot_flight_trajectory(self, save_path: str = None):
        """Plot 3D flight trajectory"""
        if self.data is None:
            print("❌ No data loaded")
            return
            
        fig = plt.figure(figsize=(15, 10))
        
        # 3D trajectory plot
        ax1 = fig.add_subplot(221, projection='3d')
        
        x = self.data.get('stateEstimate.x', np.zeros(len(self.data)))
        y = self.data.get('stateEstimate.y', np.zeros(len(self.data)))
        z = self.data.get('stateEstimate.z', np.zeros(len(self.data)))
        
        # Color trajectory by time
        colors = plt.cm.viridis(np.linspace(0, 1, len(self.data)))
        ax1.scatter(x, y, z, c=colors, s=1, alpha=0.6)
        ax1.plot(x, y, z, 'b-', alpha=0.3, linewidth=0.5)
        
        # Mark start and end
        ax1.scatter(x.iloc[0], y.iloc[0], z.iloc[0], c='green', s=100, marker='o', label='Start')
        ax1.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], c='red', s=100, marker='x', label='End')
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Flight Trajectory')
        ax1.legend()
        
        # 2D top view
        ax2 = fig.add_subplot(222)
        ax2.plot(x, y, 'b-', alpha=0.7, linewidth=1)
        ax2.scatter(x.iloc[0], y.iloc[0], c='green', s=100, marker='o', label='Start')
        ax2.scatter(x.iloc[-1], y.iloc[-1], c='red', s=100, marker='x', label='End')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_title('Flight Path (Top View)')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        ax2.axis('equal')
        
        # Height over time
        ax3 = fig.add_subplot(223)
        ax3.plot(self.data['time_elapsed'], z, 'g-', linewidth=1, label='Height')
        
        # Add z-range sensor if available
        if 'range.zrange' in self.data.columns:
            zrange = self.data['range.zrange'] / 1000.0  # Convert mm to m
            ax3.plot(self.data['time_elapsed'], zrange, 'r-', linewidth=1, alpha=0.7, label='Z-Range Sensor')
        
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Height (m)')
        ax3.set_title('Height vs Time')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        
        # Battery voltage
        ax4 = fig.add_subplot(224)
        if 'pm.vbat' in self.data.columns:
            ax4.plot(self.data['time_elapsed'], self.data['pm.vbat'], 'orange', linewidth=1)
            ax4.axhline(y=3.2, color='red', linestyle='--', alpha=0.7, label='Low Battery')
            ax4.set_xlabel('Time (s)')
            ax4.set_ylabel('Battery (V)')
            ax4.set_title('Battery Voltage')
            ax4.grid(True, alpha=0.3)
            ax4.legend()
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"📊 Flight trajectory saved to {save_path}")
        
        plt.show()
    
    def plot_landing_pad_detection(self, save_path: str = None):
        """Plot height data and detected landing pad events"""
        if self.data is None:
            print("❌ No data loaded")
            return
            
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(15, 12))
        
        time = self.data['time_elapsed']
        
        # Raw height vs z-range
        ax1.plot(time, self.data.get('stateEstimate.z', np.zeros(len(self.data))), 
                'b-', linewidth=1, label='State Estimate Z', alpha=0.8)
        
        if 'range.zrange' in self.data.columns:
            zrange = self.data['range.zrange'] / 1000.0  # Convert mm to m
            ax1.plot(time, zrange, 'r-', linewidth=1, label='Z-Range Sensor', alpha=0.8)
            
            # Calculate relative height (assuming baseline at start)
            baseline = np.median(zrange[:50])  # First 50 readings as baseline
            relative_height = zrange - baseline
            
            # Mark potential landing pad detections (height > 5cm above baseline)
            pad_threshold = 0.05
            pad_detections = relative_height > pad_threshold
            
            if np.any(pad_detections):
                detection_times = time[pad_detections]
                detection_heights = zrange[pad_detections]
                ax1.scatter(detection_times, detection_heights, 
                           c='yellow', s=30, alpha=0.8, label='Potential Pad Detections', zorder=5)
        
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Height (m)')
        ax1.set_title('Height Measurements During Flight')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Relative height analysis
        if 'range.zrange' in self.data.columns:
            ax2.plot(time, relative_height, 'purple', linewidth=1, label='Height Above Baseline')
            ax2.axhline(y=pad_threshold, color='red', linestyle='--', alpha=0.7, 
                       label=f'Detection Threshold ({pad_threshold*100:.0f}cm)')
            ax2.fill_between(time, 0, relative_height, where=(relative_height > pad_threshold), 
                           color='yellow', alpha=0.3, label='Above Threshold')
            
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Relative Height (m)')
            ax2.set_title('Landing Pad Detection Analysis')
            ax2.grid(True, alpha=0.3)
            ax2.legend()
        
        # Position during detection
        x = self.data.get('stateEstimate.x', np.zeros(len(self.data)))
        y = self.data.get('stateEstimate.y', np.zeros(len(self.data)))
        
        ax3.plot(x, y, 'b-', alpha=0.6, linewidth=1, label='Flight Path')
        ax3.scatter(x.iloc[0], y.iloc[0], c='green', s=100, marker='o', label='Start')
        ax3.scatter(x.iloc[-1], y.iloc[-1], c='red', s=100, marker='x', label='End')
        
        # Mark detection positions
        if 'range.zrange' in self.data.columns and np.any(pad_detections):
            detection_x = x[pad_detections]
            detection_y = y[pad_detections]
            ax3.scatter(detection_x, detection_y, c='yellow', s=50, alpha=0.8, 
                       label='Detection Positions', zorder=5)
        
        ax3.set_xlabel('X (m)')
        ax3.set_ylabel('Y (m)')
        ax3.set_title('Detection Positions (Top View)')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        ax3.axis('equal')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"📊 Landing pad analysis saved to {save_path}")
        
        plt.show()
    
    def generate_report(self, save_path: str = None):
        """Generate flight statistics report"""
        if self.data is None:
            print("❌ No data loaded")
            return
            
        report = {
            'flight_summary': {
                'duration': float(self.data['time_elapsed'].iloc[-1]),
                'data_points': len(self.data),
                'logging_frequency': len(self.data) / self.data['time_elapsed'].iloc[-1]
            },
            'position_stats': {
                'max_x': float(self.data.get('stateEstimate.x', pd.Series([0])).max()),
                'min_x': float(self.data.get('stateEstimate.x', pd.Series([0])).min()),
                'max_y': float(self.data.get('stateEstimate.y', pd.Series([0])).max()),
                'min_y': float(self.data.get('stateEstimate.y', pd.Series([0])).min()),
                'max_height': float(self.data.get('stateEstimate.z', pd.Series([0])).max()),
                'min_height': float(self.data.get('stateEstimate.z', pd.Series([0])).min())
            }
        }
        
        # Battery analysis
        if 'pm.vbat' in self.data.columns:
            report['battery'] = {
                'start_voltage': float(self.data['pm.vbat'].iloc[0]),
                'end_voltage': float(self.data['pm.vbat'].iloc[-1]),
                'min_voltage': float(self.data['pm.vbat'].min()),
                'voltage_drop': float(self.data['pm.vbat'].iloc[0] - self.data['pm.vbat'].iloc[-1])
            }
        
        # Landing pad detection analysis
        if 'range.zrange' in self.data.columns:
            zrange = self.data['range.zrange'] / 1000.0
            baseline = np.median(zrange[:50])
            relative_height = zrange - baseline
            pad_detections = np.sum(relative_height > 0.05)
            
            report['landing_pad_detection'] = {
                'baseline_height': float(baseline),
                'max_relative_height': float(relative_height.max()),
                'detection_events': int(pad_detections),
                'detection_percentage': float(pad_detections / len(self.data) * 100)
            }
        
        print("\n📊 FLIGHT REPORT")
        print("=" * 50)
        print(f"Flight Duration: {report['flight_summary']['duration']:.1f}s")
        print(f"Data Points: {report['flight_summary']['data_points']}")
        print(f"Logging Rate: {report['flight_summary']['logging_frequency']:.1f} Hz")
        
        if 'battery' in report:
            print(f"\nBattery: {report['battery']['start_voltage']:.2f}V → {report['battery']['end_voltage']:.2f}V")
            print(f"Voltage Drop: {report['battery']['voltage_drop']:.2f}V")
        
        if 'landing_pad_detection' in report:
            print(f"\nLanding Pad Detection:")
            print(f"  Baseline Height: {report['landing_pad_detection']['baseline_height']:.3f}m")
            print(f"  Max Height Above Baseline: {report['landing_pad_detection']['max_relative_height']:.3f}m")
            print(f"  Detection Events: {report['landing_pad_detection']['detection_events']}")
            print(f"  Detection Rate: {report['landing_pad_detection']['detection_percentage']:.1f}%")
        
        if save_path:
            with open(save_path, 'w') as f:
                json.dump(report, f, indent=2)
            print(f"\n📄 Report saved to {save_path}")
        
        return report

def main():
    parser = argparse.ArgumentParser(description='Plot Crazyflie flight data')
    parser.add_argument('log_file', help='Path to CSV log file')
    parser.add_argument('--trajectory', action='store_true', help='Plot flight trajectory')
    parser.add_argument('--detection', action='store_true', help='Plot landing pad detection analysis')
    parser.add_argument('--report', action='store_true', help='Generate flight report')
    parser.add_argument('--save', help='Save plots to directory')
    
    args = parser.parse_args()
    
    plotter = FlightDataPlotter(args.log_file)
    
    if not plotter.load_data():
        return
    
    save_dir = Path(args.save) if args.save else None
    if save_dir:
        save_dir.mkdir(exist_ok=True)
    
    if args.trajectory or not any([args.trajectory, args.detection, args.report]):
        save_path = save_dir / 'flight_trajectory.png' if save_dir else None
        plotter.plot_flight_trajectory(save_path)
    
    if args.detection:
        save_path = save_dir / 'landing_detection.png' if save_dir else None
        plotter.plot_landing_pad_detection(save_path)
    
    if args.report:
        save_path = save_dir / 'flight_report.json' if save_dir else None
        plotter.generate_report(save_path)

if __name__ == '__main__':
    main()
