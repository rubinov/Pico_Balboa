#!/usr/bin/env python3
"""
Balboa Robot Controller GUI
Controls Balboa robot via Bluetooth serial connection
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime

class BalboaController:
    def __init__(self, root):
        self.root = root
        self.root.title("Balboa Robot Controller")
        self.root.geometry("900x700")
        
        # Serial connection
        self.ser = None
        self.connected = False
        self.reading = False
        self.receive_thread = None
        
        # Data buffer
        self.received_data = []
        
        # Create GUI
        self.create_widgets()
        
        # Auto-connect on startup
        self.root.after(500, self.auto_connect)
        
    def create_widgets(self):
        # ============ CONNECTION FRAME ============
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10, pady=5)
        
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, padx=5)
        self.port_var = tk.StringVar(value="COM12")
        self.port_entry = ttk.Entry(conn_frame, textvariable=self.port_var, width=10)
        self.port_entry.grid(row=0, column=1, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="● Disconnected", foreground="red")
        self.status_label.grid(row=0, column=3, padx=10)
        
        # List available ports button
        ttk.Button(conn_frame, text="List Ports", command=self.list_ports).grid(row=0, column=4, padx=5)
        
        # ============ SEQUENCE FRAME ============
        seq_frame = ttk.LabelFrame(self.root, text="Sequence Definition", padding=10)
        seq_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        
        # Headers
        ttk.Label(seq_frame, text="Step", font=('Arial', 9, 'bold')).grid(row=0, column=0, padx=5, pady=5)
        ttk.Label(seq_frame, text="Duration (ms)", font=('Arial', 9, 'bold')).grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(seq_frame, text="Left Speed", font=('Arial', 9, 'bold')).grid(row=0, column=2, padx=5, pady=5)
        ttk.Label(seq_frame, text="Right Speed", font=('Arial', 9, 'bold')).grid(row=0, column=3, padx=5, pady=5)
        
        # Create 5 step entries
        self.step_entries = []
        for i in range(5):
            step_num = ttk.Label(seq_frame, text=f"{i}")
            step_num.grid(row=i+1, column=0, padx=5, pady=3)
            
            duration = ttk.Entry(seq_frame, width=12)
            duration.grid(row=i+1, column=1, padx=5, pady=3)
            duration.insert(0, "2000" if i < 2 else "")  # Pre-fill first two steps
            
            left_speed = ttk.Entry(seq_frame, width=12)
            left_speed.grid(row=i+1, column=2, padx=5, pady=3)
            if i == 0:
                left_speed.insert(0, "100")  # Pre-fill step 0
            elif i == 1:
                left_speed.insert(0, "-100")  # Pre-fill step 1
            
            right_speed = ttk.Entry(seq_frame, width=12)
            right_speed.grid(row=i+1, column=3, padx=5, pady=3)
            if i == 0:
                right_speed.insert(0, "100")
            elif i == 1:
                right_speed.insert(0, "-100")
            
            self.step_entries.append({
                'duration': duration,
                'left': left_speed,
                'right': right_speed
            })
        
        # Clear button
        ttk.Button(seq_frame, text="Clear All", command=self.clear_steps).grid(row=6, column=0, columnspan=4, pady=10)
        
        # Quick presets
        preset_frame = ttk.Frame(seq_frame)
        preset_frame.grid(row=7, column=0, columnspan=4, pady=5)
        ttk.Label(preset_frame, text="Presets:").pack(side=tk.LEFT, padx=5)
        ttk.Button(preset_frame, text="Forward/Reverse", command=self.preset_forward_reverse).pack(side=tk.LEFT, padx=2)
        ttk.Button(preset_frame, text="Spin Test", command=self.preset_spin).pack(side=tk.LEFT, padx=2)
        
        # ============ CONTROL FRAME ============
        control_frame = ttk.LabelFrame(self.root, text="Control", padding=10)
        control_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        
        ttk.Button(control_frame, text="▶ RUN Sequence", command=self.run_sequence, 
                  style="Accent.TButton", width=20).grid(row=0, column=0, padx=5, pady=5)
        
        ttk.Button(control_frame, text="📊 READ Data", command=self.read_data,
                  width=20).grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Button(control_frame, text="📋 SHOW_SEQ", command=self.show_sequence,
                  width=20).grid(row=0, column=2, padx=5, pady=5)
        
        # ============ FILE SAVE FRAME ============
        file_frame = ttk.LabelFrame(self.root, text="Data File", padding=10)
        file_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=5)
        
        ttk.Label(file_frame, text="Filename:").grid(row=0, column=0, padx=5)
        self.filename_var = tk.StringVar(value=f"balboa_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        ttk.Entry(file_frame, textvariable=self.filename_var, width=40).grid(row=0, column=1, padx=5)
        ttk.Button(file_frame, text="Browse...", command=self.browse_file).grid(row=0, column=2, padx=5)
        
        # ============ CONSOLE FRAME ============
        console_frame = ttk.LabelFrame(self.root, text="Console Output", padding=10)
        console_frame.grid(row=1, column=1, rowspan=3, sticky="nsew", padx=10, pady=5)
        
        self.console = scrolledtext.ScrolledText(console_frame, width=50, height=25, 
                                                  font=('Consolas', 9))
        self.console.grid(row=0, column=0, sticky="nsew")
        
        # Clear console button
        ttk.Button(console_frame, text="Clear Console", 
                  command=lambda: self.console.delete(1.0, tk.END)).grid(row=1, column=0, pady=5)
        
        # Configure grid weights for resizing
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(1, weight=1)
        console_frame.rowconfigure(0, weight=1)
        console_frame.columnconfigure(0, weight=1)
        
    def log(self, message, tag=None):
        """Add message to console with timestamp"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        full_message = f"[{timestamp}] {message}\n"
        self.console.insert(tk.END, full_message, tag)
        self.console.see(tk.END)
        self.console.update()
        
    def auto_connect(self):
        """Automatically connect to COM12 on startup"""
        self.log("Attempting auto-connect to COM12...")
        self.connect_serial()
        
    def toggle_connection(self):
        """Toggle serial connection"""
        if self.connected:
            self.disconnect_serial()
        else:
            self.connect_serial()
    
    def connect_serial(self):
        """Connect to serial port"""
        port = self.port_var.get()
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(0.5)  # Give it time to initialize
            self.connected = True
            self.status_label.config(text="● Connected", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.log(f"✓ Connected to {port} at 115200 baud")
            
            # Start receive thread
            self.reading = True
            self.receive_thread = threading.Thread(target=self.receive_data, daemon=True)
            self.receive_thread.start()
            
        except serial.SerialException as e:
            self.log(f"✗ Connection failed: {e}")
            messagebox.showerror("Connection Error", f"Failed to connect to {port}\n{e}")
            self.connected = False
            
    def disconnect_serial(self):
        """Disconnect from serial port"""
        self.reading = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False
        self.status_label.config(text="● Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")
        self.log("Disconnected")
        
    def receive_data(self):
        """Background thread to receive serial data"""
        while self.reading and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.log(line)
                        self.received_data.append(line)
            except Exception as e:
                if self.reading:  # Only log if we're supposed to be reading
                    self.log(f"Receive error: {e}")
            time.sleep(0.01)
    
    def send_command(self, command):
        """Send command to robot"""
        if not self.connected or not self.ser:
            self.log("✗ Not connected!")
            messagebox.showwarning("Not Connected", "Please connect to the robot first.")
            return False
            
        try:
            self.ser.write(f"{command}\n".encode())
            self.log(f"→ {command}")
            return True
        except Exception as e:
            self.log(f"✗ Send error: {e}")
            return False
    
    def clear_steps(self):
        """Clear all step entries"""
        for step in self.step_entries:
            step['duration'].delete(0, tk.END)
            step['left'].delete(0, tk.END)
            step['right'].delete(0, tk.END)
        self.log("Cleared all steps")
    
    def preset_forward_reverse(self):
        """Load forward/reverse test preset"""
        presets = [
            (2000, 100, 100),
            (500, 0, 0),
            (2000, -100, -100),
            (500, 0, 0),
            ("", "", "")
        ]
        for i, (dur, left, right) in enumerate(presets):
            self.step_entries[i]['duration'].delete(0, tk.END)
            self.step_entries[i]['left'].delete(0, tk.END)
            self.step_entries[i]['right'].delete(0, tk.END)
            if dur:
                self.step_entries[i]['duration'].insert(0, str(dur))
                self.step_entries[i]['left'].insert(0, str(left))
                self.step_entries[i]['right'].insert(0, str(right))
        self.log("Loaded Forward/Reverse preset")
    
    def preset_spin(self):
        """Load spin test preset"""
        presets = [
            (2000, 100, -100),  # Spin right
            (500, 0, 0),
            (2000, -100, 100),  # Spin left
            (500, 0, 0),
            ("", "", "")
        ]
        for i, (dur, left, right) in enumerate(presets):
            self.step_entries[i]['duration'].delete(0, tk.END)
            self.step_entries[i]['left'].delete(0, tk.END)
            self.step_entries[i]['right'].delete(0, tk.END)
            if dur:
                self.step_entries[i]['duration'].insert(0, str(dur))
                self.step_entries[i]['left'].insert(0, str(left))
                self.step_entries[i]['right'].insert(0, str(right))
        self.log("Loaded Spin Test preset")
    
    def run_sequence(self):
        """Send sequence to robot and run"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to the robot first.")
            return
        
        # Clear received data buffer
        self.received_data = []
        
        # Collect steps
        steps = []
        for i, step in enumerate(self.step_entries):
            dur = step['duration'].get().strip()
            left = step['left'].get().strip()
            right = step['right'].get().strip()
            
            if dur and left and right:  # Only add if all fields filled
                try:
                    dur_int = int(dur)
                    left_int = int(left)
                    right_int = int(right)
                    steps.append((i, dur_int, left_int, right_int))
                except ValueError:
                    messagebox.showerror("Invalid Input", 
                                       f"Step {i}: Please enter valid integers")
                    return
        
        if not steps:
            messagebox.showwarning("No Steps", "Please define at least one step.")
            return
        
        # Send steps
        self.log(f"Sending {len(steps)} steps to robot...")
        for step_num, duration, left, right in steps:
            cmd = f"STEP {step_num}, {duration}, {left}, {right}"
            if not self.send_command(cmd):
                return
            time.sleep(0.05)  # Small delay between commands
        
        # Send RUN command
        time.sleep(0.1)
        self.send_command("RUN")
        self.log("✓ Sequence started!")
    
    def show_sequence(self):
        """Request robot to display current sequence"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to the robot first.")
            return
        
        self.log("Requesting sequence display...")
        self.send_command("SHOW_SEQ")
        
    def read_data(self):
        """Read data from robot and save to file"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Please connect to the robot first.")
            return
        
        filename = self.filename_var.get()
        if not filename:
            messagebox.showwarning("No Filename", "Please enter a filename.")
            return
        
        # Clear previous data
        self.received_data = []
        
        # Send READ command
        self.log("Requesting data from robot...")
        if not self.send_command("READ"):
            return
        
        # Wait for data (in a separate thread to not block GUI)
        def wait_and_save():
            self.log("Waiting for data... (timeout 10s)")
            start_time = time.time()
            data_started = False
            
            # Wait for DATA_START
            while time.time() - start_time < 10:
                for line in self.received_data:
                    if "DATA_START" in line:
                        data_started = True
                        break
                if data_started:
                    break
                time.sleep(0.1)
            
            if not data_started:
                self.log("✗ No data received (timeout)")
                return
            
            # Wait for DATA_END
            self.log("Receiving data...")
            while time.time() - start_time < 30:  # Longer timeout for data
                for line in self.received_data:
                    if "DATA_END" in line:
                        self.save_data_to_file(filename)
                        return
                time.sleep(0.1)
            
            self.log("✗ Data transfer incomplete (timeout)")
        
        threading.Thread(target=wait_and_save, daemon=True).start()
    
    def save_data_to_file(self, filename):
        """Save received data to file"""
        try:
            # Find DATA_START and DATA_END
            start_idx = -1
            end_idx = -1
            
            for i, line in enumerate(self.received_data):
                if "DATA_START" in line:
                    start_idx = i
                if "DATA_END" in line:
                    end_idx = i
                    break
            
            if start_idx == -1 or end_idx == -1:
                self.log("✗ Could not find data markers")
                return
            
            # Extract data lines
            data_lines = self.received_data[start_idx+1:end_idx]
            
            if not data_lines:
                self.log("✗ No data found")
                return
            
            # Write to file
            with open(filename, 'w') as f:
                for line in data_lines:
                    f.write(line + '\n')
            
            self.log(f"✓ Saved {len(data_lines)} lines to {filename}")
            messagebox.showinfo("Success", f"Data saved to {filename}\n{len(data_lines)} lines written")
            
        except Exception as e:
            self.log(f"✗ Error saving file: {e}")
            messagebox.showerror("Save Error", f"Failed to save file:\n{e}")
    
    def browse_file(self):
        """Open file dialog to choose save location"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("Text files", "*.txt"), ("All files", "*.*")],
            initialfile=self.filename_var.get()
        )
        if filename:
            self.filename_var.set(filename)
    
    def list_ports(self):
        """List available COM ports"""
        ports = serial.tools.list_ports.comports()
        if ports:
            port_list = "\n".join([f"{p.device}: {p.description}" for p in ports])
            self.log(f"Available ports:\n{port_list}")
            messagebox.showinfo("Available Ports", port_list)
        else:
            self.log("No COM ports found")
            messagebox.showinfo("Available Ports", "No COM ports found")

def main():
    root = tk.Tk()
    
    # Set theme (try to use modern theme if available)
    try:
        root.tk.call("source", "azure.tcl")
        root.tk.call("set_theme", "light")
    except:
        pass  # Fall back to default theme
    
    app = BalboaController(root)
    
    # Handle window close
    def on_closing():
        if app.connected:
            app.disconnect_serial()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()