#!/usr/bin/env python3
"""
Balboa Robot Data Analysis GUI
Date: November 30, 2024 at 18:30 UTC

Analyzes recorded data from Pico Balboa robot to help tune control parameters.
Data structure (11 fields):
  - timestamp_us: Time in microseconds
  - ax: Accelerometer X (raw counts)
  - ay: Accelerometer Z (raw counts)
  - az: Gyroscope Y (raw counts)
  - gx: Left motor command (actual)
  - gy: Right motor command (actual)
  - gz: Angle from vertical (degrees)
  - left_enc: Left encoder counts
  - right_enc: Right encoder counts
  - motor_cmd: Base motor command (motorSpeed)
  - trajectory_error: Trajectory error (risingAngleOffset, degrees)

Hardware specs:
  - Encoders: 620 counts/revolution
  - Wheels: 80mm diameter
  - Distance per count: 0.405 mm/count
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import csv

class BalboaAnalyzer:
    def __init__(self, root):
        self.root = root
        self.root.title("Balboa Robot Data Analyzer")
        self.root.geometry("1400x900")
        
        # Data storage
        self.data = None
        self.params = {
            'ANGLE_RESPONSE': 11,
            'ANGLE_RATE_RATIO': 140,
            'DISTANCE_DIFF_RESPONSE': 0,
            'GEAR_RATIO': 76,
            'gy_zero': 0
        }
        
        # Hardware specs
        self.COUNTS_PER_REV = 620
        self.WHEEL_DIAMETER_MM = 80
        self.MM_PER_COUNT = (np.pi * self.WHEEL_DIAMETER_MM) / self.COUNTS_PER_REV
        
        self.setup_ui()
        
    def setup_ui(self):
        # Top frame for file loading and parameters
        top_frame = ttk.Frame(self.root, padding="10")
        top_frame.pack(side=tk.TOP, fill=tk.X)
        
        # File loading
        ttk.Button(top_frame, text="Load CSV File", 
                  command=self.load_file).pack(side=tk.LEFT, padx=5)
        
        self.file_label = ttk.Label(top_frame, text="No file loaded", 
                                    foreground="red")
        self.file_label.pack(side=tk.LEFT, padx=10)
        
        # Parameter entry frame
        param_frame = ttk.LabelFrame(top_frame, text="Control Parameters", 
                                     padding="5")
        param_frame.pack(side=tk.LEFT, padx=20)
        
        # Parameter entries
        params_to_show = [
            ('ANGLE_RESPONSE', 'Angle Response'),
            ('ANGLE_RATE_RATIO', 'Angle Rate Ratio (ms)'),
            ('DISTANCE_DIFF_RESPONSE', 'Distance Diff Response'),
            ('GEAR_RATIO', 'Gear Ratio'),
            ('gy_zero', 'Gyro Y Zero')
        ]
        
        self.param_entries = {}
        for i, (key, label) in enumerate(params_to_show):
            ttk.Label(param_frame, text=f"{label}:").grid(row=i, column=0, 
                                                          sticky=tk.W, padx=2)
            entry = ttk.Entry(param_frame, width=10)
            entry.insert(0, str(self.params[key]))
            entry.grid(row=i, column=1, padx=2)
            self.param_entries[key] = entry
        
        ttk.Button(param_frame, text="Update", 
                  command=self.update_params).grid(row=len(params_to_show), 
                                                  column=0, columnspan=2, pady=5)
        
        # Analysis buttons
        button_frame = ttk.Frame(top_frame)
        button_frame.pack(side=tk.RIGHT, padx=10)
        
        ttk.Button(button_frame, text="Time Series", 
                  command=self.plot_time_series).pack(pady=2)
        ttk.Button(button_frame, text="Verify Calculations", 
                  command=self.plot_verification).pack(pady=2)
        ttk.Button(button_frame, text="Tuning Analysis", 
                  command=self.plot_tuning_analysis).pack(pady=2)
        ttk.Button(button_frame, text="Phase Portrait", 
                  command=self.plot_phase_portrait).pack(pady=2)
        
        # Main plot area with notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Info/Guidance tab
        self.create_guidance_tab()
        
    def create_guidance_tab(self):
        """Create tab with tuning guidance"""
        guide_frame = ttk.Frame(self.notebook)
        self.notebook.add(guide_frame, text="Tuning Guide")
        
        # Text widget with scrollbar
        text_frame = ttk.Frame(guide_frame)
        text_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        scrollbar = ttk.Scrollbar(text_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        text = tk.Text(text_frame, wrap=tk.WORD, width=80, height=40,
                      yscrollcommand=scrollbar.set)
        text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=text.yview)
        
        guide_text = """
BALBOA TUNING GUIDE

═══════════════════════════════════════════════════════════════════

PARAMETER OVERVIEW:

1. ANGLE_RATE_RATIO = 140 ms (0.14 seconds)
   - Physical time constant of inverted pendulum
   - Relates angular velocity to angle
   - Formula: trajectory_error = angleRate × (ANGLE_RATE_RATIO/1000) + angle
   - Typical range: 100-180 ms
   - Pololu default: 140 ms

2. ANGLE_RESPONSE = 11
   - Control gain (how aggressively to correct trajectory error)
   - Higher = more aggressive response
   - Formula: motorSpeed += (ANGLE_RESPONSE × trajectory_error) / 100 / GEAR_RATIO
   - Typical range: 5-20
   - Pololu default: 11

3. DISTANCE_DIFF_RESPONSE = 0 (currently disabled)
   - Differential correction to prevent spinning
   - Negative value corrects rotation
   - Formula: left_correction = (distanceDiff × DISTANCE_DIFF_RESPONSE) / 100
   - Typical range: -75 to -25
   - Pololu default: -50
   - Start with 0, add only if robot spins

4. GEAR_RATIO = 76
   - Scales motor commands to physical motion
   - Based on encoder resolution (620 counts/rev)
   - Adjusted from Pololu (111) for your hardware
   - Should not change unless hardware changes

═══════════════════════════════════════════════════════════════════

TUNING PROCEDURE:

PHASE 1: BASIC BALANCING (Start Here)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Goal: Get robot to balance, even if unstable

Current Settings:
  ANGLE_RESPONSE = 11
  ANGLE_RATE_RATIO = 140
  DISTANCE_DIFF_RESPONSE = 0

Test: Run RUN command, observe behavior

WHAT TO LOOK FOR:

1. Robot Falls Immediately Forward
   Problem: Not enough correction
   Solution: Increase ANGLE_RESPONSE
   Try: 15, then 20 if needed
   
2. Robot Falls Immediately Backward
   Problem: Too much correction
   Solution: Decrease ANGLE_RESPONSE
   Try: 8, then 5 if needed
   
3. Robot Oscillates With Growing Amplitude
   Problem: Control too aggressive (unstable)
   Solution: Decrease ANGLE_RESPONSE by ~20%
   Try: 9, then 7
   
4. Robot Oscillates With Constant Amplitude
   Good sign! Ready for fine tuning
   Continue to Phase 2

PHASE 2: OSCILLATION TUNING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Goal: Reduce oscillations, improve stability

Look at angle vs time plot:

1. Fast Oscillations (< 0.5 seconds period)
   Problem: Too much weight on angleRate
   Solution: Decrease ANGLE_RATE_RATIO
   Try: 120, then 100
   Effect: Reduces derivative action
   
2. Slow Oscillations (> 2 seconds period)
   Problem: Not enough weight on angleRate
   Solution: Increase ANGLE_RATE_RATIO
   Try: 160, then 180
   Effect: Increases derivative damping
   
3. Oscillations Decay Slowly
   Problem: Low damping
   Solution: Slightly increase ANGLE_RATE_RATIO
   Or slightly decrease ANGLE_RESPONSE

PHASE 3: DRIFT/SPIN CORRECTION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Goal: Stop robot from spinning or drifting

Prerequisites: Robot balances reasonably well

Test: Record data, check if left_enc - right_enc grows linearly

1. Robot Spins Slowly in Place
   Problem: Unequal motor performance
   Solution: Enable DISTANCE_DIFF_RESPONSE
   Try: -25, then -50
   Monitor: Plot differential correction
   
2. Robot Spins Too Fast / Oscillates Left-Right
   Problem: Differential correction too strong
   Solution: Reduce magnitude
   Try: -25, then -10
   
3. Robot Stays Straight
   Great! Keep DISTANCE_DIFF_RESPONSE = 0

PHASE 4: FINE TUNING
━━━━━━━━━━━━━━━━━━━

Goal: Optimize performance

1. Minimize Steady-State Angle Error
   If angle settles away from 0°:
   - Check gyro drift compensation (0.999 factor)
   - May indicate mechanical imbalance
   
2. Minimize Motor Command Variation
   If motor_cmd oscillates excessively:
   - Slightly reduce ANGLE_RESPONSE
   - Creates smoother control
   
3. Improve Disturbance Rejection
   Push robot gently, should recover quickly:
   - If too slow: Increase ANGLE_RESPONSE
   - If overshoots: Decrease ANGLE_RESPONSE

═══════════════════════════════════════════════════════════════════

WHAT TO LOOK FOR IN PLOTS:

TIME SERIES PLOT:
1. Angle: Should settle near 0° with minimal oscillation
2. Trajectory Error: Should stay near 0° when balanced
3. Motor Commands: Should be smooth, not chattery
4. Differential: Should be small (< 20 units) if enabled

PHASE PORTRAIT (Angle vs Angular Rate):
1. Should spiral inward to origin (0, 0)
2. Tight spiral = well-damped
3. Wide circular motion = underdamped
4. Diverging spiral = unstable

VERIFICATION PLOTS:
1. Trajectory error should match calculation
2. Motor command average should match base motor_cmd
3. Gyro integration should be smooth

═══════════════════════════════════════════════════════════════════

COMMON PROBLEMS:

1. "Robot balances but keeps spinning"
   → Enable DISTANCE_DIFF_RESPONSE = -50
   → Check encoder counts difference

2. "Robot oscillates forever"
   → Reduce ANGLE_RESPONSE by 20%
   → Try ANGLE_RATE_RATIO ± 20

3. "Robot falls forward when trying to balance"
   → Increase ANGLE_RESPONSE
   → Check that angle is correctly calibrated

4. "Control seems random/noisy"
   → Check gyro calibration (gy_zero)
   → Verify sensor connections
   → Check for electromagnetic interference

5. "Robot won't stand up"
   → Not a tuning issue, check stand-up sequence
   → Verify motors have enough power
   → Check trigger angle threshold (60°)

═══════════════════════════════════════════════════════════════════

ADVANCED: PARAMETER RELATIONSHIPS

Trajectory Error Formation:
  trajectory_error = angleRate × τ + angle
  where τ = ANGLE_RATE_RATIO / 1000

Physical Meaning:
  τ = sqrt(height / gravity) ≈ 0.14s for 80mm height
  This is NOT arbitrary - it's pendulum physics!

Control Law:
  Δu = (ANGLE_RESPONSE × trajectory_error) / 100 / GEAR_RATIO
  Δu = (11 × trajectory_error) / 7600
  Δu ≈ trajectory_error × 0.00145

For trajectory_error = 10°:
  Motor increases by 0.0145 per cycle (10ms)
  Over 100 cycles (1 second): increases by 1.45 units

Stability Considerations:
- Higher ANGLE_RESPONSE → faster response, risk of instability
- Higher ANGLE_RATE_RATIO → more damping, may be sluggish
- Balance between speed and stability

═══════════════════════════════════════════════════════════════════

QUICK START RECIPE:

1. Load data file (from robot's READ command)
2. Check Control Parameters match robot's parameter dump
3. Click "Time Series" to see basic behavior
4. Click "Tuning Analysis" for detailed diagnosis
5. Adjust parameters based on guidance above
6. Update parameters on robot with ANGLE command
7. Test again and iterate

Remember: 
- Make ONE change at a time
- Record data after each change
- Small adjustments (10-20%) are usually sufficient
- Pololu defaults (11, 140, -50) are good starting points

═══════════════════════════════════════════════════════════════════
"""
        text.insert('1.0', guide_text)
        text.config(state=tk.DISABLED)
        
    def load_file(self):
        """Load CSV data file"""
        filename = filedialog.askopenfilename(
            title="Select Balboa Data File",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if not filename:
            return
            
        try:
            # Read CSV file
            with open(filename, 'r') as f:
                reader = csv.reader(f)
                header = next(reader)
                rows = list(reader)
            
            # Expected fields
            expected = ['timestamp_us', 'ax', 'ay', 'az', 'gx', 'gy', 'gz',
                       'left_enc', 'right_enc', 'motor_cmd', 'trajectory_error']
            
            if header != expected:
                messagebox.showwarning("Header Mismatch",
                    f"Expected fields:\n{expected}\n\nGot:\n{header}\n\n"
                    "Will attempt to load anyway...")
            
            # Convert to numpy array
            data_array = np.array(rows, dtype=float)
            
            # Create structured data dictionary
            self.data = {
                'timestamp_us': data_array[:, 0],
                'ax': data_array[:, 1],  # Accel X
                'ay': data_array[:, 2],  # Accel Z
                'az': data_array[:, 3],  # Gyro Y (raw)
                'gx': data_array[:, 4],  # motor_cmd_left
                'gy': data_array[:, 5],  # motor_cmd_right
                'gz': data_array[:, 6],  # angle
                'left_enc': data_array[:, 7],
                'right_enc': data_array[:, 8],
                'motor_cmd': data_array[:, 9],  # base motor command
                'trajectory_error': data_array[:, 10]
            }
            
            # Calculate time in seconds
            self.data['time'] = (self.data['timestamp_us'] - 
                                self.data['timestamp_us'][0]) / 1e6
            
            # Calculate derived quantities
            self.calculate_derived_quantities()
            
            self.file_label.config(
                text=f"Loaded: {filename.split('/')[-1]} ({len(rows)} samples)",
                foreground="green"
            )
            
            # Auto-detect parameters from data if possible
            self.auto_detect_parameters()
            
        except Exception as e:
            messagebox.showerror("Load Error", f"Error loading file:\n{str(e)}")
            
    def calculate_derived_quantities(self):
        """Calculate additional useful quantities"""
        if self.data is None:
            return
        
        # Angular velocity from raw gyro
        gy_zero = self.params['gy_zero']
        self.data['angleRate'] = (self.data['az'] - gy_zero) / 32.768  # deg/s
        
        # Average motor command
        self.data['motor_cmd_avg'] = (self.data['gx'] + self.data['gy']) / 2
        
        # Differential correction
        self.data['differential'] = self.data['gx'] - self.data['gy']
        
        # Encoder distances
        self.data['distance_left'] = np.cumsum(np.diff(self.data['left_enc'], 
                                                       prepend=0)) * self.MM_PER_COUNT
        self.data['distance_right'] = np.cumsum(np.diff(self.data['right_enc'], 
                                                        prepend=0)) * self.MM_PER_COUNT
        self.data['distance_diff'] = self.data['distance_left'] - self.data['distance_right']
        
    def auto_detect_parameters(self):
        """Try to detect parameters from data"""
        # This would parse the parameter dump if it was included in the file
        # For now, just use defaults
        pass
        
    def update_params(self):
        """Update parameters from entry fields"""
        try:
            for key, entry in self.param_entries.items():
                self.params[key] = float(entry.get())
            
            # Recalculate derived quantities with new parameters
            if self.data is not None:
                self.calculate_derived_quantities()
            
            messagebox.showinfo("Success", "Parameters updated successfully!")
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid parameter value:\n{str(e)}")
            
    def plot_time_series(self):
        """Plot main time series data"""
        if self.data is None:
            messagebox.showwarning("No Data", "Please load a data file first")
            return
        
        # Create new tab
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Time Series")
        self.notebook.select(tab)
        
        # Create figure
        fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('Balboa Control System Time Series', fontsize=14, fontweight='bold')
        
        time = self.data['time']
        
        # 1. Angle
        axes[0].plot(time, self.data['gz'], 'b-', linewidth=2, label='Angle')
        axes[0].axhline(0, color='k', linestyle='--', alpha=0.3)
        axes[0].set_ylabel('Angle (deg)', fontweight='bold')
        axes[0].set_title('Estimated Angle from Vertical')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        
        # 2. Angular Rate
        axes[1].plot(time, self.data['angleRate'], 'g-', linewidth=2, 
                    label='Angular Rate')
        axes[1].axhline(0, color='k', linestyle='--', alpha=0.3)
        axes[1].set_ylabel('Rate (deg/s)', fontweight='bold')
        axes[1].set_title('Angular Velocity')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        
        # 3. Trajectory Error
        axes[2].plot(time, self.data['trajectory_error'], 'r-', linewidth=2,
                    label='Trajectory Error')
        axes[2].axhline(0, color='k', linestyle='--', alpha=0.3)
        axes[2].set_ylabel('Error (deg)', fontweight='bold')
        axes[2].set_title('Trajectory Error (Control Input)')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend()
        
        # 4. Motor Commands
        axes[3].plot(time, self.data['gx'], 'b-', alpha=0.7, label='Left Motor')
        axes[3].plot(time, self.data['gy'], 'r-', alpha=0.7, label='Right Motor')
        axes[3].plot(time, self.data['motor_cmd'], 'k--', linewidth=2, 
                    label='Base Command')
        axes[3].set_ylabel('Motor Command', fontweight='bold')
        axes[3].set_title('Motor Commands (Actual vs Base)')
        axes[3].grid(True, alpha=0.3)
        axes[3].legend()
        
        # 5. Encoder Positions
        axes[4].plot(time, self.data['left_enc'], 'b-', alpha=0.7, label='Left')
        axes[4].plot(time, self.data['right_enc'], 'r-', alpha=0.7, label='Right')
        axes[4].set_ylabel('Counts', fontweight='bold')
        axes[4].set_xlabel('Time (s)', fontweight='bold')
        axes[4].set_title('Encoder Positions')
        axes[4].grid(True, alpha=0.3)
        axes[4].legend()
        
        plt.tight_layout()
        
        # Embed in tab
        canvas = FigureCanvasTkAgg(fig, tab)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add toolbar
        toolbar = NavigationToolbar2Tk(canvas, tab)
        toolbar.update()
        
    def plot_verification(self):
        """Plot verification of calculations"""
        if self.data is None:
            messagebox.showwarning("No Data", "Please load a data file first")
            return
        
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Verification")
        self.notebook.select(tab)
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle('Calculation Verification', fontsize=14, fontweight='bold')
        
        time = self.data['time']
        
        # 1. Verify trajectory error calculation
        ANGLE_RATE_RATIO = self.params['ANGLE_RATE_RATIO']
        trajectory_calc = (self.data['angleRate'] * (ANGLE_RATE_RATIO / 1000) + 
                          self.data['gz'])
        
        axes[0, 0].plot(time, self.data['trajectory_error'], 'b-', 
                       linewidth=2, label='Recorded', alpha=0.7)
        axes[0, 0].plot(time, trajectory_calc, 'r--', 
                       linewidth=2, label='Calculated', alpha=0.7)
        axes[0, 0].set_ylabel('Trajectory Error (deg)')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_title('Trajectory Error Verification')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        error = np.abs(self.data['trajectory_error'] - trajectory_calc)
        axes[0, 0].text(0.02, 0.98, f'Mean Error: {np.mean(error):.3f}°',
                       transform=axes[0, 0].transAxes, 
                       verticalalignment='top',
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # 2. Verify motor command average
        axes[0, 1].plot(time, self.data['motor_cmd'], 'b-', 
                       linewidth=2, label='Base Command', alpha=0.7)
        axes[0, 1].plot(time, self.data['motor_cmd_avg'], 'r--',
                       linewidth=2, label='Avg(Left,Right)', alpha=0.7)
        axes[0, 1].set_ylabel('Motor Command')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_title('Motor Command Consistency')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        cmd_diff = np.abs(self.data['motor_cmd'] - self.data['motor_cmd_avg'])
        axes[0, 1].text(0.02, 0.98, f'Mean Diff: {np.mean(cmd_diff):.2f}',
                       transform=axes[0, 1].transAxes,
                       verticalalignment='top',
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # 3. Differential correction
        DISTANCE_DIFF_RESPONSE = self.params['DISTANCE_DIFF_RESPONSE']
        if DISTANCE_DIFF_RESPONSE != 0:
            # Calculate from encoders
            distanceLeft_cum = np.cumsum(np.diff(self.data['left_enc'], prepend=0))
            distanceRight_cum = np.cumsum(np.diff(self.data['right_enc'], prepend=0))
            distanceDiff = distanceLeft_cum - distanceRight_cum
            diff_correction_calc = (distanceDiff * DISTANCE_DIFF_RESPONSE) / 100
            
            # From actual motors
            diff_correction_actual = self.data['differential'] / 2
            
            axes[1, 0].plot(time, diff_correction_calc, 'b-',
                           label='From Encoders', alpha=0.7)
            axes[1, 0].plot(time, diff_correction_actual, 'r--',
                           label='From Motors', alpha=0.7)
            axes[1, 0].set_ylabel('Differential Correction')
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_title('Differential Correction Verification')
            axes[1, 0].legend()
            axes[1, 0].grid(True, alpha=0.3)
        else:
            axes[1, 0].text(0.5, 0.5, 'DISTANCE_DIFF_RESPONSE = 0\n(Disabled)',
                           ha='center', va='center',
                           transform=axes[1, 0].transAxes, fontsize=14)
            axes[1, 0].set_title('Differential Correction (Disabled)')
        
        # 4. Gyro integration verification
        # Integrate from raw gyro
        dt = np.mean(np.diff(time))
        angle_integrated = np.zeros_like(self.data['angleRate'])
        angle_integrated[0] = self.data['gz'][0]
        for i in range(1, len(angle_integrated)):
            angle_integrated[i] = (angle_integrated[i-1] + 
                                   self.data['angleRate'][i-1] * dt) * 0.999
        
        axes[1, 1].plot(time, self.data['gz'], 'b-',
                       linewidth=2, label='Recorded', alpha=0.7)
        axes[1, 1].plot(time, angle_integrated, 'r--',
                       linewidth=2, label='Re-integrated', alpha=0.7)
        axes[1, 1].set_ylabel('Angle (deg)')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_title('Gyro Integration Verification')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        integ_error = np.mean(np.abs(self.data['gz'] - angle_integrated))
        axes[1, 1].text(0.02, 0.98, f'Mean Error: {integ_error:.3f}°',
                       transform=axes[1, 1].transAxes,
                       verticalalignment='top',
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        
        canvas = FigureCanvasTkAgg(fig, tab)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        toolbar = NavigationToolbar2Tk(canvas, tab)
        toolbar.update()
        
    def plot_tuning_analysis(self):
        """Plot diagnostic information for tuning"""
        if self.data is None:
            messagebox.showwarning("No Data", "Please load a data file first")
            return
        
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Tuning Analysis")
        self.notebook.select(tab)
        
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        fig.suptitle('Control Tuning Diagnostics', fontsize=14, fontweight='bold')
        
        time = self.data['time']
        
        # 1. Angle error statistics
        angle_mean = np.mean(self.data['gz'])
        angle_std = np.std(self.data['gz'])
        angle_max = np.max(np.abs(self.data['gz']))
        
        axes[0, 0].plot(time, self.data['gz'], 'b-', linewidth=2)
        axes[0, 0].axhline(0, color='k', linestyle='--', alpha=0.3)
        axes[0, 0].axhline(angle_mean, color='r', linestyle='--', 
                          alpha=0.5, label=f'Mean: {angle_mean:.2f}°')
        axes[0, 0].fill_between(time, angle_mean - angle_std, 
                                angle_mean + angle_std,
                                alpha=0.2, color='r', 
                                label=f'Std: ±{angle_std:.2f}°')
        axes[0, 0].set_ylabel('Angle (deg)')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_title('Angle Statistics')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # Add diagnosis
        diagnosis = ""
        if angle_std > 10:
            diagnosis = "HIGH OSCILLATION\nReduce ANGLE_RESPONSE"
        elif angle_std < 2:
            diagnosis = "WELL DAMPED\nGood tuning!"
        else:
            diagnosis = "MODERATE OSCILLATION\nTune ANGLE_RATE_RATIO"
        
        axes[0, 0].text(0.98, 0.98, diagnosis,
                       transform=axes[0, 0].transAxes,
                       verticalalignment='top', horizontalalignment='right',
                       bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.7),
                       fontsize=10, fontweight='bold')
        
        # 2. Oscillation frequency analysis
        # Simple peak detection
        from scipy import signal
        if len(self.data['gz']) > 100:
            peaks, _ = signal.find_peaks(self.data['gz'], height=2)
            if len(peaks) > 1:
                periods = np.diff(time[peaks])
                avg_period = np.mean(periods)
                freq = 1 / avg_period if avg_period > 0 else 0
                
                axes[0, 1].plot(time, self.data['gz'], 'b-', alpha=0.5)
                axes[0, 1].plot(time[peaks], self.data['gz'][peaks], 'ro', 
                               label=f'Peaks (Period: {avg_period:.2f}s)')
                axes[0, 1].set_ylabel('Angle (deg)')
                axes[0, 1].set_xlabel('Time (s)')
                axes[0, 1].set_title('Oscillation Analysis')
                axes[0, 1].legend()
                axes[0, 1].grid(True, alpha=0.3)
                
                # Diagnosis
                if avg_period < 0.5:
                    diagnosis = "FAST OSCILLATION\nDecrease ANGLE_RATE_RATIO"
                elif avg_period > 2.0:
                    diagnosis = "SLOW OSCILLATION\nIncrease ANGLE_RATE_RATIO"
                else:
                    diagnosis = "NORMAL FREQUENCY\nGood balance"
                
                axes[0, 1].text(0.98, 0.98, diagnosis,
                               transform=axes[0, 1].transAxes,
                               verticalalignment='top', horizontalalignment='right',
                               bbox=dict(boxstyle='round', facecolor='cyan', alpha=0.7),
                               fontsize=10, fontweight='bold')
            else:
                axes[0, 1].plot(time, self.data['gz'], 'b-')
                axes[0, 1].text(0.5, 0.5, 'Not enough oscillations\nto analyze',
                               ha='center', va='center',
                               transform=axes[0, 1].transAxes, fontsize=12)
        else:
            axes[0, 1].text(0.5, 0.5, 'Insufficient data',
                           ha='center', va='center',
                           transform=axes[0, 1].transAxes, fontsize=12)
        
        # 3. Motor command smoothness
        motor_cmd_changes = np.abs(np.diff(self.data['motor_cmd']))
        axes[1, 0].plot(time[1:], motor_cmd_changes, 'g-', linewidth=1)
        axes[1, 0].axhline(np.mean(motor_cmd_changes), color='r', 
                          linestyle='--', label=f'Mean: {np.mean(motor_cmd_changes):.2f}')
        axes[1, 0].set_ylabel('|ΔMotor Command|')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_title('Motor Command Smoothness')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        if np.mean(motor_cmd_changes) > 10:
            diagnosis = "CHATTERING\nControl too aggressive"
        else:
            diagnosis = "SMOOTH\nGood control"
        
        axes[1, 0].text(0.98, 0.98, diagnosis,
                       transform=axes[1, 0].transAxes,
                       verticalalignment='top', horizontalalignment='right',
                       bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7),
                       fontsize=10, fontweight='bold')
        
        # 4. Control effort
        axes[1, 1].hist(self.data['motor_cmd'], bins=50, alpha=0.7, color='blue')
        axes[1, 1].axvline(np.mean(self.data['motor_cmd']), color='r',
                          linestyle='--', linewidth=2,
                          label=f'Mean: {np.mean(self.data['motor_cmd']):.1f}')
        axes[1, 1].set_xlabel('Motor Command')
        axes[1, 1].set_ylabel('Count')
        axes[1, 1].set_title('Control Effort Distribution')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        # 5. Encoder difference (spinning detection)
        axes[2, 0].plot(time, self.data['distance_diff'], 'purple', linewidth=2)
        axes[2, 0].set_ylabel('Distance Difference (mm)')
        axes[2, 0].set_xlabel('Time (s)')
        axes[2, 0].set_title('Wheel Distance Difference (Spin Detection)')
        axes[2, 0].grid(True, alpha=0.3)
        
        # Check if spinning
        final_diff = self.data['distance_diff'][-1]
        spin_rate = final_diff / time[-1] if time[-1] > 0 else 0
        
        if abs(spin_rate) > 50:  # mm/s threshold
            diagnosis = f"SPINNING!\n{spin_rate:.1f} mm/s\nEnable DISTANCE_DIFF_RESPONSE"
        elif abs(spin_rate) > 10:
            diagnosis = f"SLOW DRIFT\n{spin_rate:.1f} mm/s\nConsider correction"
        else:
            diagnosis = f"STRAIGHT\n{spin_rate:.1f} mm/s\nGood!"
        
        axes[2, 0].text(0.98, 0.98, diagnosis,
                       transform=axes[2, 0].transAxes,
                       verticalalignment='top', horizontalalignment='right',
                       bbox=dict(boxstyle='round', facecolor='orange', alpha=0.7),
                       fontsize=10, fontweight='bold')
        
        # 6. Differential correction (if enabled)
        axes[2, 1].plot(time, self.data['differential'], 'brown', linewidth=2)
        axes[2, 1].axhline(0, color='k', linestyle='--', alpha=0.3)
        axes[2, 1].set_ylabel('Left - Right Motor')
        axes[2, 1].set_xlabel('Time (s)')
        axes[2, 1].set_title('Differential Correction Applied')
        axes[2, 1].grid(True, alpha=0.3)
        
        if self.params['DISTANCE_DIFF_RESPONSE'] == 0:
            axes[2, 1].text(0.5, 0.5, 'DISABLED\nDISTANCE_DIFF_RESPONSE = 0',
                           ha='center', va='center',
                           transform=axes[2, 1].transAxes, 
                           fontsize=12, fontweight='bold')
        
        plt.tight_layout()
        
        canvas = FigureCanvasTkAgg(fig, tab)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        toolbar = NavigationToolbar2Tk(canvas, tab)
        toolbar.update()
        
    def plot_phase_portrait(self):
        """Plot phase portrait (angle vs angular rate)"""
        if self.data is None:
            messagebox.showwarning("No Data", "Please load a data file first")
            return
        
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Phase Portrait")
        self.notebook.select(tab)
        
        fig, ax = plt.subplots(1, 1, figsize=(10, 10))
        fig.suptitle('Phase Portrait: Angle vs Angular Rate', 
                    fontsize=14, fontweight='bold')
        
        # Color by time
        scatter = ax.scatter(self.data['gz'], self.data['angleRate'],
                           c=self.data['time'], cmap='viridis',
                           s=20, alpha=0.6)
        
        # Mark start and end
        ax.plot(self.data['gz'][0], self.data['angleRate'][0], 
               'go', markersize=15, label='Start', zorder=5)
        ax.plot(self.data['gz'][-1], self.data['angleRate'][-1],
               'ro', markersize=15, label='End', zorder=5)
        
        # Target (origin)
        ax.plot(0, 0, 'k*', markersize=20, label='Target', zorder=5)
        
        # Add trajectory lines
        ax.plot(self.data['gz'], self.data['angleRate'], 
               'k-', alpha=0.2, linewidth=0.5)
        
        ax.set_xlabel('Angle (degrees)', fontsize=12, fontweight='bold')
        ax.set_ylabel('Angular Rate (deg/s)', fontsize=12, fontweight='bold')
        ax.set_title('System Trajectory in State Space')
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color='k', linestyle='--', alpha=0.5)
        ax.axvline(0, color='k', linestyle='--', alpha=0.5)
        ax.legend()
        
        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Time (s)', fontsize=12)
        
        # Add interpretation text
        interpretation = """
PHASE PORTRAIT INTERPRETATION:

Ideal behavior: Spiral inward to origin (0,0)

✓ Tight spiral = Well-damped, stable
✓ Reaches origin = Perfect balance

✗ Wide circles = Underdamped (oscillating)
✗ Diverging = Unstable (falling)
✗ Doesn't reach origin = Steady-state error
"""
        
        ax.text(1.15, 0.5, interpretation,
               transform=ax.transAxes,
               verticalalignment='center',
               bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8),
               fontsize=10, family='monospace')
        
        plt.tight_layout()
        
        canvas = FigureCanvasTkAgg(fig, tab)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        toolbar = NavigationToolbar2Tk(canvas, tab)
        toolbar.update()

def main():
    root = tk.Tk()
    app = BalboaAnalyzer(root)
    root.mainloop()

if __name__ == "__main__":
    main()