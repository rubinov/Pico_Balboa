#!/usr/bin/env python3
"""
Example: Analyzing Balboa Robot Data
Demonstrates how to load and plot data collected from the robot
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def load_balboa_data(filename):
    """Load data from CSV file"""
    df = pd.read_csv(filename)
    
    # Convert timestamp to seconds (from microseconds)
    df['Time_s'] = df['Time_us'] / 1e6
    
    # Make time relative to start
    df['Time_s'] = df['Time_s'] - df['Time_s'].iloc[0]
    
    return df

def plot_encoders(df):
    """Plot encoder data"""
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8))
    fig.suptitle('Encoder Analysis', fontsize=14, fontweight='bold')
    
    # Plot encoder positions
    ax1.plot(df['Time_s'], df['left_enc'], label='Left Encoder', linewidth=2)
    ax1.plot(df['Time_s'], df['right_enc'], label='Right Encoder', linewidth=2)
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.set_ylabel('Encoder Counts')
    ax1.set_title('Encoder Position')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot encoder velocities (derivative)
    dt = df['Time_s'].diff().mean()
    df['left_vel'] = df['left_enc'].diff() / dt
    df['right_vel'] = df['right_enc'].diff() / dt
    
    ax2.plot(df['Time_s'], df['left_vel'], label='Left Velocity', linewidth=2)
    ax2.plot(df['Time_s'], df['right_vel'], label='Right Velocity', linewidth=2)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_ylabel('Velocity (counts/s)')
    ax2.set_title('Encoder Velocity')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot motor command
    ax3.plot(df['Time_s'], df['motor_cmd'], label='Motor Command', 
             linewidth=2, color='red', alpha=0.7)
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Motor Speed')
    ax3.set_title('Motor Command')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_imu(df):
    """Plot IMU data"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('IMU Analysis', fontsize=14, fontweight='bold')
    
    # Plot accelerometer
    ax1.plot(df['Time_s'], df['ax'], label='X-axis', linewidth=2)
    ax1.plot(df['Time_s'], df['ay'], label='Y-axis', linewidth=2)
    ax1.plot(df['Time_s'], df['az'], label='Z-axis', linewidth=2)
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax1.set_ylabel('Acceleration (raw)')
    ax1.set_title('Accelerometer')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot gyroscope
    ax2.plot(df['Time_s'], df['gx'], label='X-axis', linewidth=2)
    ax2.plot(df['Time_s'], df['gy'], label='Y-axis', linewidth=2)
    ax2.plot(df['Time_s'], df['gz'], label='Z-axis', linewidth=2)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angular Velocity (raw)')
    ax2.set_title('Gyroscope')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_encoder_symmetry(df):
    """Check encoder symmetry (for debugging)"""
    fig, ax = plt.subplots(figsize=(10, 6))
    
    ax.plot(df['left_enc'], df['right_enc'], 'o-', markersize=2, linewidth=1)
    
    # Add diagonal line (perfect symmetry)
    min_val = min(df['left_enc'].min(), df['right_enc'].min())
    max_val = max(df['left_enc'].max(), df['right_enc'].max())
    ax.plot([min_val, max_val], [min_val, max_val], 'r--', 
            label='Perfect symmetry', linewidth=2)
    
    ax.set_xlabel('Left Encoder')
    ax.set_ylabel('Right Encoder')
    ax.set_title('Encoder Symmetry Check\n(Should follow diagonal for straight motion)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    return fig

def analyze_test(filename):
    """Complete analysis of a test run"""
    print(f"\n{'='*60}")
    print(f"Analyzing: {filename}")
    print(f"{'='*60}\n")
    
    # Load data
    df = load_balboa_data(filename)
    
    # Print statistics
    print(f"Duration: {df['Time_s'].iloc[-1]:.2f} seconds")
    print(f"Samples: {len(df)}")
    print(f"Sample rate: {len(df) / df['Time_s'].iloc[-1]:.1f} Hz")
    print()
    
    print("Encoder Summary:")
    print(f"  Left:  Start={df['left_enc'].iloc[0]:6d}, End={df['left_enc'].iloc[-1]:6d}, "
          f"Range={df['left_enc'].max()-df['left_enc'].min():6d}")
    print(f"  Right: Start={df['right_enc'].iloc[0]:6d}, End={df['right_enc'].iloc[-1]:6d}, "
          f"Range={df['right_enc'].max()-df['right_enc'].min():6d}")
    print()
    
    # Check for symmetry
    final_diff = abs(df['left_enc'].iloc[-1] - df['right_enc'].iloc[-1])
    print(f"Final encoder difference: {final_diff} counts")
    if final_diff < 50:
        print("  ✓ Good symmetry (forward/reverse test)")
    else:
        print(f"  ⚠ Asymmetric ({final_diff} counts difference)")
    print()
    
    # Plot results
    print("Generating plots...")
    plot_encoders(df)
    plot_imu(df)
    plot_encoder_symmetry(df)
    
    plt.show()
    
    return df

# Example usage
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        # Default filename pattern
        filename = "balboa_data_20241128_120000.csv"
        print(f"Usage: python {sys.argv[0]} <data_file.csv>")
        print(f"Example: python {sys.argv[0]} {filename}")
        print()
        
        # Try to find a recent file
        import glob
        files = sorted(glob.glob("balboa_data_*.csv"))
        if files:
            filename = files[-1]
            print(f"Using most recent file: {filename}\n")
        else:
            print("No data files found!")
            sys.exit(1)
    
    df = analyze_test(filename)
    
    # Additional analysis examples
    print("\nAdditional Analysis:")
    print(f"Max motor speed: {df['motor_cmd'].abs().max()}")
    print(f"Max left encoder velocity: {df['left_vel'].abs().max():.1f} counts/s")
    print(f"Max right encoder velocity: {df['right_vel'].abs().max():.1f} counts/s")
