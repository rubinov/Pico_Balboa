#!/usr/bin/env python3
"""
Balboa Data Analyzer GUI
Interactive data analysis tool for Balboa robot data
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import numpy as np

class DataAnalyzer:
    def __init__(self, root):
        self.root = root
        self.root.title("Balboa Data Analyzer")
        self.root.geometry("1200x800")
        
        self.df = None
        self.current_file = None
        
        self.create_widgets()
        
    def create_widgets(self):
        # ============ CONTROL FRAME ============
        control_frame = ttk.Frame(self.root, padding=10)
        control_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        
        ttk.Button(control_frame, text="📂 Open CSV File", 
                  command=self.load_file, width=20).grid(row=0, column=0, padx=5)
        
        self.file_label = ttk.Label(control_frame, text="No file loaded", foreground="gray")
        self.file_label.grid(row=0, column=1, padx=10, sticky="w")
        
        # ============ PLOT SELECTION FRAME ============
        plot_frame = ttk.LabelFrame(self.root, text="Plots", padding=10)
        plot_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        
        ttk.Button(plot_frame, text="📊 Encoders", 
                  command=self.plot_encoders, width=15).grid(row=0, column=0, padx=5, pady=5)
        
        ttk.Button(plot_frame, text="📈 Gyro", 
                  command=self.plot_imu, width=15).grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Button(plot_frame, text="🎯 Symmetry", 
                  command=self.plot_symmetry, width=15).grid(row=0, column=2, padx=5, pady=5)
        
        ttk.Button(plot_frame, text="📋 Statistics", 
                  command=self.show_stats, width=15).grid(row=0, column=3, padx=5, pady=5)
        
        ttk.Button(plot_frame, text="🔄 All Plots", 
                  command=self.plot_all, width=15).grid(row=0, column=4, padx=5, pady=5)
        
        # ============ STATS FRAME ============
        stats_frame = ttk.LabelFrame(self.root, text="Statistics", padding=10)
        stats_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        
        self.stats_text = tk.Text(stats_frame, height=8, width=100, font=('Courier', 9))
        self.stats_text.grid(row=0, column=0, sticky="ew")
        
        # ============ PLOT CANVAS FRAME ============
        self.canvas_frame = ttk.Frame(self.root)
        self.canvas_frame.grid(row=3, column=0, sticky="nsew", padx=10, pady=5)
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(3, weight=1)
        
    def load_file(self):
        """Load CSV file"""
        filename = filedialog.askopenfilename(
            title="Select Balboa Data File",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if not filename:
            return
        
        try:
            # Load data
            self.df = pd.read_csv(filename)
            
            # Check for required columns
            required = ['Time_us', 'AX', 'AY', 'AZ', 'GX', 'GY', 'GZ', 
                       'LeftEnc', 'RightEnc', 'MotorCmd']
            
            # Handle case differences
            self.df.columns = [col.lower() for col in self.df.columns]
            
            # Convert time to seconds
            if 'time_us' in self.df.columns:
                self.df['time_s'] = self.df['time_us'] / 1e6
                self.df['time_s'] = self.df['time_s'] - self.df['time_s'].iloc[0]
            
            # Calculate velocities
            if 'leftenc' in self.df.columns and 'rightenc' in self.df.columns:
                dt = self.df['time_s'].diff().mean()
                self.df['left_vel'] = self.df['leftenc'].diff() / dt
                self.df['right_vel'] = self.df['rightenc'].diff() / dt
            
            self.current_file = filename
            self.file_label.config(text=f"Loaded: {filename.split('/')[-1]}", foreground="green")
            
            # Show statistics
            self.show_stats()
            
            messagebox.showinfo("Success", f"Loaded {len(self.df)} samples")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load file: {e}")
            self.file_label.config(text="Error loading file", foreground="red")
    
    def clear_canvas(self):
        """Clear previous plots"""
        for widget in self.canvas_frame.winfo_children():
            widget.destroy()
    
    def show_stats(self):
        """Display data statistics"""
        if self.df is None:
            messagebox.showwarning("No Data", "Please load a file first")
            return
        
        self.stats_text.delete(1.0, tk.END)
        
        stats = f"""
DATA STATISTICS
{'='*80}
File: {self.current_file.split('/')[-1] if self.current_file else 'N/A'}
Samples: {len(self.df)}
Duration: {self.df['time_s'].iloc[-1]:.2f} seconds
Sample Rate: {len(self.df) / self.df['time_s'].iloc[-1]:.1f} Hz

ENCODERS
{'='*80}
Left  - Start: {self.df['leftenc'].iloc[0]:7d}  End: {self.df['leftenc'].iloc[-1]:7d}  Range: {self.df['leftenc'].iloc[-1] - self.df['leftenc'].iloc[0]:7d}
Right - Start: {self.df['rightenc'].iloc[0]:7d}  End: {self.df['rightenc'].iloc[-1]:7d}  Range: {self.df['rightenc'].iloc[-1] - self.df['rightenc'].iloc[0]:7d}
Difference (L-R): {(self.df['leftenc'].iloc[-1] - self.df['leftenc'].iloc[0]) - (self.df['rightenc'].iloc[-1] - self.df['rightenc'].iloc[0]):7d}

MOTORS
{'='*80}
Max Command: {self.df['motorcmd'].max():7d}
Min Command: {self.df['motorcmd'].min():7d}
        """
        
        self.stats_text.insert(1.0, stats)
    
    def plot_encoders(self):
        """Plot encoder data with commanded vs observed speed"""
        if self.df is None:
            messagebox.showwarning("No Data", "Please load a file first")
            return
        
        self.clear_canvas()
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        fig.suptitle('Encoder Analysis', fontsize=14, fontweight='bold')
        
        # Plot encoder positions
        ax1.plot(self.df['time_s'], self.df['leftenc'], label='Left', linewidth=2)
        ax1.plot(self.df['time_s'], self.df['rightenc'], label='Right', linewidth=2)
        ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax1.set_ylabel('Encoder Counts')
        ax1.set_title('Position')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot commanded vs observed speed
        # Average the left and right velocities for observed speed
        observed_speed = (self.df['left_vel'] + self.df['right_vel']) / 2
        
        ax2.plot(self.df['time_s'], self.df['motorcmd'], label='Commanded Speed', 
                linewidth=2, color='red', alpha=0.7)
        ax2.plot(self.df['time_s'], observed_speed, label='Observed Speed', 
                linewidth=2, color='blue', alpha=0.7)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (counts/s)')
        ax2.set_title('Commanded vs Observed Speed')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Embed in tkinter
        canvas = FigureCanvasTkAgg(fig, master=self.canvas_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        toolbar = NavigationToolbar2Tk(canvas, self.canvas_frame)
        toolbar.update()
    
    def plot_imu(self):
        """Plot gyro data with derivative and integral"""
        if self.df is None:
            messagebox.showwarning("No Data", "Please load a file first")
            return
        
        self.clear_canvas()
        
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
        fig.suptitle('Pitch Gyro Analysis', fontsize=14, fontweight='bold')
        
        # Calculate derivative and integral
        dt = self.df['time_s'].diff().mean()
        gyro_derivative = self.df['gy'].diff() / dt
        gyro_integral = (self.df['gy'] * dt).cumsum()
        
        # Plot gyro
        ax1.plot(self.df['time_s'], self.df['gy'], label='Pitch Gyro (GY)', linewidth=2)
        ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax1.set_ylabel('Angular Rate (raw)')
        ax1.set_title('Pitch Gyro (calibrated)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot derivative
        ax2.plot(self.df['time_s'], gyro_derivative, label='Gyro Derivative', 
                linewidth=2, color='orange')
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax2.set_ylabel('Angular Acceleration (raw/s)')
        ax2.set_title('Pitch Gyro Derivative')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot integral (angle)
        ax3.plot(self.df['time_s'], gyro_integral, label='Gyro Integral (Angle)', 
                linewidth=2, color='green')
        ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Integrated Angle (raw*s)')
        ax3.set_title('Pitch Gyro Integral')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Embed in tkinter
        canvas = FigureCanvasTkAgg(fig, master=self.canvas_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        toolbar = NavigationToolbar2Tk(canvas, self.canvas_frame)
        toolbar.update()
    
    def plot_symmetry(self):
        """Plot encoder symmetry check"""
        if self.df is None:
            messagebox.showwarning("No Data", "Please load a file first")
            return
        
        self.clear_canvas()
        
        fig, ax = plt.subplots(figsize=(8, 8))
        
        ax.scatter(self.df['leftenc'], self.df['rightenc'], alpha=0.5, s=10)
        
        # Add diagonal reference line
        min_val = min(self.df['leftenc'].min(), self.df['rightenc'].min())
        max_val = max(self.df['leftenc'].max(), self.df['rightenc'].max())
        ax.plot([min_val, max_val], [min_val, max_val], 'r--', label='Perfect symmetry')
        
        ax.set_xlabel('Left Encoder')
        ax.set_ylabel('Right Encoder')
        ax.set_title('Encoder Symmetry Check')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        plt.tight_layout()
        
        # Embed in tkinter
        canvas = FigureCanvasTkAgg(fig, master=self.canvas_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        toolbar = NavigationToolbar2Tk(canvas, self.canvas_frame)
        toolbar.update()
    
    def plot_all(self):
        """Create all plots in separate windows"""
        if self.df is None:
            messagebox.showwarning("No Data", "Please load a file first")
            return
        
        # Calculate observed speed
        observed_speed = (self.df['left_vel'] + self.df['right_vel']) / 2
        
        # Create encoders plot
        fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        fig1.suptitle('Encoder Analysis', fontsize=14, fontweight='bold')
        
        ax1.plot(self.df['time_s'], self.df['leftenc'], label='Left', linewidth=2)
        ax1.plot(self.df['time_s'], self.df['rightenc'], label='Right', linewidth=2)
        ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax1.set_ylabel('Encoder Counts')
        ax1.set_title('Position')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        ax2.plot(self.df['time_s'], self.df['motorcmd'], label='Commanded Speed', 
                linewidth=2, color='red', alpha=0.7)
        ax2.plot(self.df['time_s'], observed_speed, label='Observed Speed', 
                linewidth=2, color='blue', alpha=0.7)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (counts/s)')
        ax2.set_title('Commanded vs Observed Speed')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Calculate gyro derivative and integral
        dt = self.df['time_s'].diff().mean()
        gyro_derivative = self.df['gy'].diff() / dt
        gyro_integral = (self.df['gy'] * dt).cumsum()
        
        # Create gyro plot
        fig2, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
        fig2.suptitle('Pitch Gyro Analysis', fontsize=14, fontweight='bold')
        
        ax1.plot(self.df['time_s'], self.df['gy'], label='Pitch Gyro (GY)', linewidth=2)
        ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax1.set_ylabel('Angular Rate (raw)')
        ax1.set_title('Pitch Gyro (calibrated)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        ax2.plot(self.df['time_s'], gyro_derivative, label='Gyro Derivative', 
                linewidth=2, color='orange')
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax2.set_ylabel('Angular Acceleration (raw/s)')
        ax2.set_title('Pitch Gyro Derivative')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        ax3.plot(self.df['time_s'], gyro_integral, label='Gyro Integral (Angle)', 
                linewidth=2, color='green')
        ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Integrated Angle (raw*s)')
        ax3.set_title('Pitch Gyro Integral')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Create symmetry plot
        fig3, ax = plt.subplots(figsize=(8, 8))
        ax.scatter(self.df['leftenc'], self.df['rightenc'], alpha=0.5, s=10)
        min_val = min(self.df['leftenc'].min(), self.df['rightenc'].min())
        max_val = max(self.df['leftenc'].max(), self.df['rightenc'].max())
        ax.plot([min_val, max_val], [min_val, max_val], 'r--', label='Perfect symmetry')
        ax.set_xlabel('Left Encoder')
        ax.set_ylabel('Right Encoder')
        ax.set_title('Encoder Symmetry Check')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        plt.tight_layout()
        
        plt.show()

def main():
    root = tk.Tk()
    app = DataAnalyzer(root)
    root.mainloop()

if __name__ == "__main__":
    main()