#!/usr/bin/env python3
"""
Analyze flight height data to debug landing pad detection
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def analyze_height_data(csv_file):
    """Analyze height data from flight log"""
    
    # Load data
    data = pd.read_csv(csv_file)
    print(f"Loaded {len(data)} data points")
    
    # Extract height data
    if 'range.zrange' in data.columns:
        z_range_mm = data['range.zrange'].values
        z_range_m = z_range_mm / 1000.0  # Convert mm to m
        print(f"Z-range data available: {len(z_range_mm)} points")
    else:
        print("No Z-range data found")
        return
    
    state_z = data['stateEstimate.z'].values if 'stateEstimate.z' in data.columns else None
    
    # Statistics
    print(f"\nZ-range sensor statistics:")
    print(f"  Min: {np.min(z_range_m):.3f}m ({np.min(z_range_mm)}mm)")
    print(f"  Max: {np.max(z_range_m):.3f}m ({np.max(z_range_mm)}mm)")
    print(f"  Mean: {np.mean(z_range_m):.3f}m ({np.mean(z_range_mm):.0f}mm)")
    print(f"  Std: {np.std(z_range_m):.3f}m ({np.std(z_range_mm):.0f}mm)")
    print(f"  Range: {np.max(z_range_m) - np.min(z_range_m):.3f}m")
    
    if state_z is not None:
        print(f"\nState estimator Z statistics:")
        print(f"  Min: {np.min(state_z):.3f}m")
        print(f"  Max: {np.max(state_z):.3f}m") 
        print(f"  Mean: {np.mean(state_z):.3f}m")
        print(f"  Std: {np.std(state_z):.3f}m")
        print(f"  Range: {np.max(state_z) - np.min(state_z):.3f}m")
    
    # Check for landing pad detection threshold
    print(f"\nLanding pad detection analysis:")
    baseline = np.min(z_range_m)  # Use minimum as baseline
    threshold = 0.06  # 6cm from config
    above_threshold = z_range_m > (baseline + threshold)
    
    print(f"  Baseline (min height): {baseline:.3f}m")
    print(f"  Detection threshold: {threshold:.3f}m")
    print(f"  Target height for detection: {baseline + threshold:.3f}m")
    print(f"  Points above threshold: {np.sum(above_threshold)}/{len(z_range_m)}")
    
    if np.sum(above_threshold) > 0:
        print(f"  Max height above baseline: {np.max(z_range_m) - baseline:.3f}m")
    else:
        print(f"  No points detected above threshold!")
        print(f"  Actual max above baseline: {np.max(z_range_m) - baseline:.3f}m")
    
    # Plot analysis
    plt.figure(figsize=(15, 10))
    
    # Height over time
    plt.subplot(2, 2, 1)
    plt.plot(z_range_m, 'b-', label='Z-range (m)', alpha=0.7)
    if state_z is not None:
        plt.plot(state_z, 'r-', label='State Est. Z', alpha=0.7)
    plt.axhline(y=baseline, color='g', linestyle='--', label=f'Baseline ({baseline:.3f}m)')
    plt.axhline(y=baseline + threshold, color='orange', linestyle='--', label=f'Detection threshold')
    plt.ylabel('Height (m)')
    plt.title('Height Measurements Over Time')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Height histogram
    plt.subplot(2, 2, 2)
    plt.hist(z_range_m, bins=30, alpha=0.7, label='Z-range')
    if state_z is not None:
        plt.hist(state_z, bins=30, alpha=0.7, label='State Est.')
    plt.axvline(x=baseline, color='g', linestyle='--', label='Baseline')
    plt.axvline(x=baseline + threshold, color='orange', linestyle='--', label='Threshold')
    plt.xlabel('Height (m)')
    plt.ylabel('Count')
    plt.title('Height Distribution')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Position plot
    if 'stateEstimate.x' in data.columns and 'stateEstimate.y' in data.columns:
        plt.subplot(2, 2, 3)
        x = data['stateEstimate.x'].values
        y = data['stateEstimate.y'].values
        
        # Color by height
        scatter = plt.scatter(x, y, c=z_range_m, cmap='viridis', s=10, alpha=0.7)
        plt.colorbar(scatter, label='Height (m)')
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Flight Path (colored by height)')
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
    
    # Height variation analysis
    plt.subplot(2, 2, 4)
    # Calculate rolling statistics
    window = min(10, len(z_range_m) // 4)
    if window > 1:
        rolling_mean = pd.Series(z_range_m).rolling(window).mean()
        rolling_std = pd.Series(z_range_m).rolling(window).std()
        
        plt.plot(rolling_mean, 'b-', label=f'Rolling mean (window={window})')
        plt.fill_between(range(len(rolling_mean)), 
                        rolling_mean - rolling_std, 
                        rolling_mean + rolling_std, 
                        alpha=0.3, label='±1 std')
        plt.axhline(y=baseline + threshold, color='orange', linestyle='--', label='Detection threshold')
        plt.ylabel('Height (m)')
        plt.xlabel('Sample')
        plt.title('Height Variation Analysis')
        plt.legend()
        plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plot
    output_file = csv_file.replace('.csv', '_height_analysis.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"\nAnalysis plot saved to: {output_file}")
    
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python height_analyzer.py <flight_log.csv>")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    analyze_height_data(csv_file)
