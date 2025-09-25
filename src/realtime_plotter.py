#!/usr/bin/env python3
"""
Real-time plotter for landing pad detection debugging
Shows height measurements, baseline, and detection events in real-time
"""

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import threading
import time
from queue import Queue, Empty


class RealtimePlotter:
    """Real-time plotter for landing pad detection debugging"""
    
    def __init__(self, max_points=200):
        self.max_points = max_points
        
        # Data storage
        self.times = deque(maxlen=max_points)
        self.heights = deque(maxlen=max_points)
        self.positions_x = deque(maxlen=max_points)
        self.positions_y = deque(maxlen=max_points)
        self.detections = deque(maxlen=max_points)
        self.baseline = deque(maxlen=max_points)
        
        # Data queue for thread-safe updates
        self.data_queue = Queue()
        
        # Setup matplotlib
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(12, 10))
        self.fig.suptitle('Real-time Landing Pad Detection Debug', fontsize=14)
        
        # Height plot
        self.line_height, = self.ax1.plot([], [], 'b-', label='Height', linewidth=2)
        self.line_baseline, = self.ax1.plot([], [], 'g--', label='Baseline', linewidth=1)
        self.scatter_detections = self.ax1.scatter([], [], c='red', s=50, label='Detections', zorder=5)
        self.ax1.set_ylabel('Height (m)')
        self.ax1.set_title('Height Measurements & Detection')
        self.ax1.legend()
        self.ax1.grid(True, alpha=0.3)
        
        # 2D position plot
        self.scatter_positions = self.ax2.scatter([], [], c='blue', s=20, alpha=0.6, label='Flight path')
        self.scatter_pad_detections = self.ax2.scatter([], [], c='red', s=100, marker='x', label='Pad detections')
        self.ax2.set_xlabel('X Position (m)')
        self.ax2.set_ylabel('Y Position (m)')
        self.ax2.set_title('Flight Path & Landing Pad Detections')
        self.ax2.legend()
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_aspect('equal')
        
        # Statistics plot
        self.ax3.text(0.1, 0.8, 'Detection Statistics:', fontsize=12, weight='bold')
        self.stats_text = self.ax3.text(0.1, 0.6, '', fontsize=10, family='monospace')
        self.ax3.set_xlim(0, 1)
        self.ax3.set_ylim(0, 1)
        self.ax3.axis('off')
        
        # Animation
        self.start_time = time.time()
        self.is_running = True
        
    def add_data_point(self, height, position, is_detection=False, baseline_height=None, stats=None):
        """Add a new data point (thread-safe)"""
        try:
            self.data_queue.put_nowait({
                'height': height,
                'position': position,
                'is_detection': is_detection,
                'baseline_height': baseline_height,
                'stats': stats,
                'timestamp': time.time()
            })
        except:
            pass  # Queue full, skip
    
    def update_plots(self):
        """Update plots with new data"""
        # Process all queued data
        new_data = []
        try:
            while True:
                data = self.data_queue.get_nowait()
                new_data.append(data)
        except Empty:
            pass
        
        # Add new data points
        for data in new_data:
            current_time = data['timestamp'] - self.start_time
            
            self.times.append(current_time)
            self.heights.append(data['height'])
            self.positions_x.append(data['position'][0])
            self.positions_y.append(data['position'][1])
            self.detections.append(data['is_detection'])
            
            if data['baseline_height'] is not None:
                self.baseline.append(data['baseline_height'])
            else:
                self.baseline.append(self.baseline[-1] if self.baseline else data['height'])
        
        if not new_data:
            return
        
        # Update height plot
        if self.times:
            times_array = np.array(self.times)
            heights_array = np.array(self.heights)
            baseline_array = np.array(self.baseline)
            
            self.line_height.set_data(times_array, heights_array)
            self.line_baseline.set_data(times_array, baseline_array)
            
            # Update detection markers
            detection_times = times_array[np.array(self.detections)]
            detection_heights = heights_array[np.array(self.detections)]
            
            self.ax1.collections.clear()  # Clear old scatter
            if len(detection_times) > 0:
                self.ax1.scatter(detection_times, detection_heights, c='red', s=50, label='Detections', zorder=5)
            
            # Auto-scale height plot
            self.ax1.relim()
            self.ax1.autoscale_view()
        
        # Update position plot
        if self.positions_x:
            pos_x = np.array(self.positions_x)
            pos_y = np.array(self.positions_y)
            detections_array = np.array(self.detections)
            
            # Flight path
            self.ax2.collections.clear()  # Clear old scatter
            self.ax2.scatter(pos_x, pos_y, c='blue', s=20, alpha=0.6, label='Flight path')
            
            # Detection positions
            if np.any(detections_array):
                det_x = pos_x[detections_array]
                det_y = pos_y[detections_array]
                self.ax2.scatter(det_x, det_y, c='red', s=100, marker='x', label='Pad detections')
            
            # Auto-scale position plot
            self.ax2.relim()
            self.ax2.autoscale_view()
        
        # Update statistics
        if new_data and new_data[-1]['stats']:
            stats = new_data[-1]['stats']
            stats_text = (
                f"Total border points: {stats.get('total_border_points', 0)}\n"
                f"Detection active: {stats.get('detection_active', False)}\n"
                f"Baseline height: {stats.get('baseline_height', 'None'):.3f}m\n"
                f"Center confidence: {stats.get('center_confidence', 0):.2f}\n"
                f"Calculated center: {stats.get('calculated_center', 'None')}\n"
                f"Points by direction:\n"
            )
            
            if 'points_by_direction' in stats:
                for direction, count in stats['points_by_direction'].items():
                    stats_text += f"  {direction}: {count}\n"
            
            self.stats_text.set_text(stats_text)
        
        # Redraw (no canvas operations needed for Agg backend)
        pass
    
    def start_logging(self):
        """Start data collection (no animation with Agg backend)"""
        self.is_running = True
    
    def stop(self):
        """Stop the animation and close plots"""
        self.is_running = False
        plt.close(self.fig)
    
    def save_plot(self, filename):
        """Save the current plot with final data"""
        # Final update before saving
        self.update_plots()
        self.fig.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"Plot saved to {filename}")


def create_realtime_plotter():
    """Create a data logger that saves plots at the end"""
    plotter = RealtimePlotter()
    plotter.start_logging()
    return plotter


if __name__ == "__main__":
    # Test the plotter with simulated data
    print("Testing real-time plotter...")
    
    plotter = RealtimePlotter()
    plotter.start_animation()
    
    # Simulate flight data
    import time
    
    try:
        for i in range(100):
            # Simulate flight path
            t = i * 0.1
            x = 0.5 * np.sin(t * 0.5)
            y = 0.5 * np.cos(t * 0.5)
            
            # Simulate height with landing pad
            if 0.8 < x < 1.2 and 0.8 < y < 1.2:
                height = 0.6  # On landing pad
                is_detection = np.random.random() > 0.7
            else:
                height = 0.5 + 0.02 * np.random.random()  # Ground level with noise
                is_detection = False
            
            baseline = 0.5
            
            stats = {
                'total_border_points': max(0, i - 50),
                'detection_active': True,
                'baseline_height': baseline,
                'center_confidence': min(1.0, i / 100.0),
                'calculated_center': (1.0, 1.0) if i > 50 else None,
                'points_by_direction': {'north': i//4, 'south': i//5, 'east': i//6, 'west': i//7}
            }
            
            plotter.add_data_point(height, (x, y), is_detection, baseline, stats)
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("Stopping plotter...")
    finally:
        plotter.stop()
