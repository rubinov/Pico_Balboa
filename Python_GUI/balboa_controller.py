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
import subprocess
import sys
import os

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
        ttk.Button(conn_frame, text="List", command=self.list_ports).grid(row=0, column=4, padx=5)
        
        # ============ SEQUENCE FRAME ============
        seq_frame = ttk.LabelFrame(self.root, text="Sequence Definition", padding=10)
        seq_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        
        # Headers
        ttk.Label(seq_frame, text="Step", font=('Arial', 9, 'bold')).grid(row=0, column=0, padx=5, pady=5)
        ttk.Label(seq_frame, text="Command", font=('Arial', 9, 'bold')).grid(row=0, column=1, padx=5, pady=5)
        ttk.Label(seq_frame, text="Param 1", font=('Arial', 9, 'bold')).grid(row=0, column=2, padx=5, pady=5)
        ttk.Label(seq_frame, text="Param 2", font=('Arial', 9, 'bold')).grid(row=0, column=3, padx=5, pady=5)
        ttk.Label(seq_frame, text="Param 3", font=('Arial', 9, 'bold')).grid(row=0, column=4, padx=5, pady=5)
        
        # Create 8 step entries
        self.step_entries = []
        commands = ["STEP", "INIT", "STAND", "MOVE", "STOP"]
        
        for i in range(8):
            step_num = ttk.Label(seq_frame, text=f"{i}")
            step_num.grid(row=i+1, column=0, padx=5, pady=3)
            
            cmd_type = ttk.Combobox(seq_frame, values=commands, width=10, state='readonly')
            cmd_type.grid(row=i+1, column=1, padx=5, pady=3)
            
            param1 = ttk.Entry(seq_frame, width=12)
            param1.grid(row=i+1, column=2, padx=5, pady=3)
            
            param2 = ttk.Entry(seq_frame, width=12)
            param2.grid(row=i+1, column=3, padx=5, pady=3)
            
            param3 = ttk.Entry(seq_frame, width=12)
            param3.grid(row=i+1, column=4, padx=5, pady=3)
            
            # Set default sequence
            if i == 0:  # INIT 1000
                cmd_type.set("INIT")
                param1.insert(0, "1000")
            elif i == 1:  # STAND -150, 150, 8000
                cmd_type.set("STAND")
                param1.insert(0, "-150") # Phase 1 Speed
                param2.insert(0, "150")  # Phase 2 Speed
                param3.insert(0, "8000") # Trigger
            elif i == 2:  # STEP 10000,0,0
                cmd_type.set("STEP")
                param1.insert(0, "10000")
                param2.insert(0, "0")
                param3.insert(0, "0")
            elif i == 3:  # STOP
                cmd_type.set("STOP")
            else:
                cmd_type.set("")
            
            cmd_type.bind('<<ComboboxSelected>>', lambda e, idx=i: self.update_param_labels(idx))
            
            self.step_entries.append({
                'cmd': cmd_type,
                'p1': param1,
                'p2': param2,
                'p3': param3
            })
        
        # Clear button
        ttk.Button(seq_frame, text="Clear", command=self.clear_steps).grid(row=9, column=0, columnspan=5, pady=10)
        
        # Quick presets
        preset_frame = ttk.Frame(seq_frame)
        preset_frame.grid(row=10, column=0, columnspan=5, pady=5)
        ttk.Label(preset_frame, text="Presets:").pack(side=tk.LEFT, padx=5)
        ttk.Button(preset_frame, text="Cal+Test", command=self.preset_calib_test).pack(side=tk.LEFT, padx=2)
        ttk.Button(preset_frame, text="Stand", command=self.preset_stand_test).pack(side=tk.LEFT, padx=2)
        
        # ============ CONTROL FRAME ============
        control_frame = ttk.LabelFrame(self.root, text="Control", padding=10)
        control_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=5)
        
        ttk.Button(control_frame, text="Upload", command=self.upload_sequence, 
                  width=15).grid(row=0, column=0, padx=5, pady=5)
        
        ttk.Button(control_frame, text="RUN", command=self.run_sequence, 
                  width=15).grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Button(control_frame, text="Read", command=self.read_data,
                  width=15).grid(row=0, column=2, padx=5, pady=5)
        
        ttk.Button(control_frame, text="Show", command=self.show_sequence,
                  width=15).grid(row=0, column=3, padx=5, pady=5)
        
        ttk.Button(control_frame, text="Analyze", command=self.launch_analyzer,
                  width=15).grid(row=1, column=0, padx=5, pady=5, columnspan=2)
        
        # ============ FILE SAVE FRAME ============
        file_frame = ttk.LabelFrame(self.root, text="Data File", padding=10)
        file_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=5)
        
        ttk.Label(file_frame, text="Filename:").grid(row=0, column=0, padx=5)
        self.filename_var = tk.StringVar(value=f"balboa_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        ttk.Entry(file_frame, textvariable=self.filename_var, width=40).grid(row=0, column=1, padx=5)
        ttk.Button(file_frame, text="Browse", command=self.browse_file).grid(row=0, column=2, padx=5)
        
        # ============ PID FRAME ============
        pid_frame = ttk.LabelFrame(self.root, text="PID Gains", padding=10)
        pid_frame.grid(row=4, column=0, sticky="ew", padx=10, pady=5)
        
        ttk.Label(pid_frame, text="P:").grid(row=0, column=0, padx=5)
        self.pid_p_var = tk.StringVar(value="0.05")
        ttk.Entry(pid_frame, textvariable=self.pid_p_var, width=10).grid(row=0, column=1, padx=5)
        
        ttk.Label(pid_frame, text="I:").grid(row=0, column=2, padx=5)
        self.pid_i_var = tk.StringVar(value="0.0")
        ttk.Entry(pid_frame, textvariable=self.pid_i_var, width=10).grid(row=0, column=3, padx=5)
        
        ttk.Label(pid_frame, text="D:").grid(row=0, column=4, padx=5)
        self.pid_d_var = tk.StringVar(value="0.50")
        ttk.Entry(pid_frame, textvariable=self.pid_d_var, width=10).grid(row=0, column=5, padx=5)
        
        ttk.Button(pid_frame, text="Set", command=self.set_pid, width=10).grid(row=0, column=6, padx=10)
        
        # ============ CONSOLE FRAME ============
        console_frame = ttk.LabelFrame(self.root, text="Console Output", padding=10)
        console_frame.grid(row=1, column=1, rowspan=4, sticky="nsew", padx=10, pady=5)
        
        self.console = scrolledtext.ScrolledText(console_frame, width=70, height=30, 
                                                  font=('Consolas', 9))
        self.console.grid(row=0, column=0, sticky="nsew")
        
        # Clear console button
        ttk.Button(console_frame, text="Clear", 
                  command=lambda: self.console.delete(1.0, tk.END)).grid(row=1, column=0, pady=5)
        
        # Configure grid weights for resizing
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=2)  # Console column wider
        self.root.rowconfigure(1, weight=1)
        console_frame.rowconfigure(0, weight=1)
        console_frame.columnconfigure(0, weight=1)
        
    def update_param_labels(self, step_idx):
        """Update parameter label hints based on command type"""
        # This is just a visual helper - actual validation happens on send
        pass
    
    def log(self, message, tag=None):
        """Add message to console with timestamp"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        full_message = f"[{timestamp}] {message}\n"
        self.console.insert(tk.END, full_message, tag)
        self.console.see(tk.END)
        
    def auto_connect(self):
        """Try to auto-connect on startup"""
        try:
            port = self.port_var.get()
            if port:
                self.log(f"Attempting auto-connect to {port}...")
                self.connect(port)
        except Exception as e:
            self.log(f"Auto-connect failed: {e}")
    
    def toggle_connection(self):
        """Connect or disconnect from serial port"""
        if self.connected:
            self.disconnect()
        else:
            port = self.port_var.get()
            if port:
                self.connect(port)
            else:
                messagebox.showerror("Error", "Please enter a COM port")
    
    def connect(self, port):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(port, 115200, timeout=1)
            self.connected = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="● Connected", foreground="green")
            self.log(f"✓ Connected to {port}")
            
            # Start receive thread
            self.reading = True
            self.receive_thread = threading.Thread(target=self.receive_data, daemon=True)
            self.receive_thread.start()
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {e}")
            self.log(f"✗ Connection failed: {e}")
    
    def disconnect(self):
        """Disconnect from serial port"""
        self.reading = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="● Disconnected", foreground="red")
        self.log("Disconnected")
    
    def ensure_connected(self):
        """Ensure connection is active, try to reconnect if needed"""
        if self.connected:
            # Check if connection is actually alive
            try:
                if self.ser and self.ser.is_open:
                    return True
            except:
                self.connected = False
        
        # Try to reconnect
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Please enter a COM port")
            return False
        
        self.log(f"→ Auto-reconnecting to {port}...")
        try:
            # Small delay to let BT recover
            time.sleep(0.5)
            self.connect(port)
            self.log(f"✓ Auto-reconnect successful")
            return True
        except Exception as e:
            self.log(f"✗ Auto-reconnect failed: {e}")
            messagebox.showerror("Connection Error", 
                               f"Cannot connect to {port}.\n\n"
                               "The robot may still be busy running a sequence.\n"
                               "Wait a moment and try again.\n\n"
                               "Troubleshooting:\n"
                               "- Robot is powered on\n"
                               "- Bluetooth is paired\n"
                               "- Correct COM port selected")
            return False
    
    def receive_data(self):
        """Background thread to receive data from serial"""
        while self.reading and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.root.after(0, lambda l=line: self.log(l))
                        # Store data lines for saving
                        self.received_data.append(line)
                time.sleep(0.01)
            except Exception as e:
                # Connection lost - mark as disconnected
                self.root.after(0, lambda: self.log(f"⚠ Connection lost: {e}"))
                self.root.after(0, lambda: self.log(f"→ Connection will auto-reconnect when needed"))
                self.connected = False
                self.root.after(0, lambda: self.connect_btn.config(text="Connect"))
                self.root.after(0, lambda: self.status_label.config(text="● Disconnected", foreground="orange"))
                break
    
    def send_command(self, cmd):
        """Send command to robot"""
        if not self.connected or not self.ser:
            messagebox.showwarning("Not Connected", "Please connect first")
            return False
        
        try:
            self.ser.write(f"{cmd}\n".encode('utf-8'))
            self.log(f"→ Sent: {cmd}")
            return True
        except Exception as e:
            messagebox.showerror("Send Error", f"Failed to send: {e}")
            self.log(f"✗ Send failed: {e}")
            return False
    
    def list_ports(self):
        """List available serial ports"""
        ports = serial.tools.list_ports.comports()
        if ports:
            port_list = "\n".join([f"{p.device}: {p.description}" for p in ports])
            messagebox.showinfo("Available Ports", port_list)
        else:
            messagebox.showinfo("Available Ports", "No serial ports found")
    
    def browse_file(self):
        """Browse for save file location"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=self.filename_var.get()
        )
        if filename:
            self.filename_var.set(filename)
    
    def clear_steps(self):
        """Clear all step entries"""
        for step in self.step_entries:
            step['cmd'].set("")
            step['p1'].delete(0, tk.END)
            step['p2'].delete(0, tk.END)
            step['p3'].delete(0, tk.END)
        self.log("Cleared all steps")
    
    def preset_calib_test(self):
        """Load calibrate and test preset"""
        presets = [
            ("INIT", "1000", "", ""),
            ("STEP", "2000", "100", "100"),
            ("STEP", "500", "0", "0"),
            ("STEP", "2000", "-100", "-100"),
            ("STOP", "", "", ""),
            ("", "", "", ""),
            ("", "", "", ""),
            ("", "", "", "")
        ]
        for i, (cmd, p1, p2, p3) in enumerate(presets):
            self.step_entries[i]['cmd'].set(cmd)
            self.step_entries[i]['p1'].delete(0, tk.END)
            self.step_entries[i]['p2'].delete(0, tk.END)
            self.step_entries[i]['p3'].delete(0, tk.END)
            if p1:
                self.step_entries[i]['p1'].insert(0, p1)
            if p2:
                self.step_entries[i]['p2'].insert(0, p2)
            if p3:
                self.step_entries[i]['p3'].insert(0, p3)
        self.log("Loaded Calibrate+Test preset")
    
    def preset_stand_test(self):
        """Load stand test preset"""
        presets = [
            ("INIT", "1000", "", ""),
            ("STAND", "-150", "150", "8000"),  # UPDATED PARAMETERS
            ("STEP", "3000", "0", "0"),
            ("STOP", "", "", ""),
            ("", "", "", ""),
            ("", "", "", ""),
            ("", "", "", ""),
            ("", "", "", "")
        ]
        for i, (cmd, p1, p2, p3) in enumerate(presets):
            self.step_entries[i]['cmd'].set(cmd)
            self.step_entries[i]['p1'].delete(0, tk.END)
            self.step_entries[i]['p2'].delete(0, tk.END)
            self.step_entries[i]['p3'].delete(0, tk.END)
            if p1:
                self.step_entries[i]['p1'].insert(0, p1)
            if p2:
                self.step_entries[i]['p2'].insert(0, p2)
            if p3:
                self.step_entries[i]['p3'].insert(0, p3)
        self.log("Loaded Stand Test preset")
    
    def upload_sequence(self):
        """Send sequence to robot (but don't run)"""
        if not self.ensure_connected():
            return
        
        # Collect and send commands
        commands = []
        for i, step in enumerate(self.step_entries):
            cmd_type = step['cmd'].get().strip()
            if not cmd_type:
                continue
            
            p1 = step['p1'].get().strip()
            p2 = step['p2'].get().strip()
            p3 = step['p3'].get().strip()
            
            try:
                if cmd_type == "STEP":
                    if not (p1 and p2 and p3):
                        messagebox.showerror("Invalid Input", 
                                           f"Step {i}: STEP requires duration, left, right")
                        return
                    dur = int(p1)
                    left = int(p2)
                    right = int(p3)
                    commands.append(f"STEP {i}, {dur}, {left}, {right}")
                    
                elif cmd_type == "INIT":
                    dur = int(p1) if p1 else 1000
                    commands.append(f"INIT {i}, {dur}")
                    
                elif cmd_type == "STAND":
                    # UPDATED: Now reads s1, s2, and trigger
                    if not (p1 and p2):
                         messagebox.showerror("Invalid Input", 
                                           f"Step {i}: STAND requires speed1, speed2")
                         return
                    
                    s1 = int(p1)
                    s2 = int(p2)
                    trig = int(p3) if p3 else 8000
                    commands.append(f"STAND {i}, {s1}, {s2}, {trig}")
                    
                elif cmd_type == "MOVE":
                    if not (p1 and p2):
                        messagebox.showerror("Invalid Input", 
                                           f"Step {i}: MOVE requires left and right counts")
                        return
                    left = int(p1)
                    right = int(p2)
                    commands.append(f"MOVE {i}, {left}, {right}")
                    
                elif cmd_type == "STOP":
                    commands.append(f"STOP {i}")
                    
            except ValueError as e:
                messagebox.showerror("Invalid Input", 
                                   f"Step {i}: Invalid number format - {e}")
                return
        
        if not commands:
            messagebox.showwarning("No Commands", "Please define at least one command.")
            return
        
        # Send commands
        self.log(f"Uploading {len(commands)} commands to robot...")
        for cmd in commands:
            if not self.send_command(cmd):
                return
            time.sleep(0.05)
        
        self.log("✓ Sequence uploaded (use RUN to execute)")
        return True
    
    def run_sequence(self):
        """Send sequence to robot and run"""
        if not self.ensure_connected():
            return
        
        # Clear received data buffer
        self.received_data = []
        
        # Upload the sequence
        if not self.upload_sequence():
            return
        
        # Send RUN command
        time.sleep(0.1)
        self.send_command("RUN")
        self.log("✓ Sequence started!")
    
    def show_sequence(self):
        """Request robot to display current sequence"""
        if not self.ensure_connected():
            return
        
        self.log("Requesting sequence display...")
        self.send_command("SHOW_SEQ")
    
    def set_pid(self):
        """Send PID gains to robot"""
        if not self.ensure_connected():
            return
        
        try:
            p = float(self.pid_p_var.get())
            i = float(self.pid_i_var.get())
            d = float(self.pid_d_var.get())
            
            cmd = f"PID {p}, {i}, {d}"
            self.send_command(cmd)
            self.log(f"✓ PID gains set: P={p}, I={i}, D={d}")
            
        except ValueError:
            messagebox.showerror("Invalid Input", "Please enter valid numbers for P, I, D")
            self.log("✗ Invalid PID values")
        
    def read_data(self):
        """Read data from robot and save to file"""
        if not self.ensure_connected():
            return
        
        self.received_data = []
        self.log("Requesting data download...")
        self.send_command("READ")
        
        # Wait for data in background
        self.root.after(100, self.check_data_complete)
        
    def check_data_complete(self):
        """Check if data download is complete"""
        # Look for DATA_END marker
        if any("DATA_END" in line for line in self.received_data):
            self.save_data()
        else:
            # Check again in 100ms
            self.root.after(100, self.check_data_complete)
    
    def save_data(self):
        """Save received data to file"""
        # Extract data between DATA_START and DATA_END
        data_lines = []
        capturing = False
        
        for line in self.received_data:
            if "DATA_START" in line:
                capturing = True
                continue
            if "DATA_END" in line:
                capturing = False
                break
            if capturing:
                data_lines.append(line)
        
        if not data_lines:
            self.log("⚠ No data received")
            return
        
        # Save to file
        filename = self.filename_var.get()
        try:
            with open(filename, 'w') as f:
                for line in data_lines:
                    f.write(line + '\n')
            
            self.log(f"✓ Saved {len(data_lines)} lines to {filename}")
            messagebox.showinfo("Success", f"Data saved to {filename}")
            
            # Update filename for next save
            self.filename_var.set(f"balboa_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
            
        except Exception as e:
            self.log(f"✗ Save failed: {e}")
            messagebox.showerror("Save Error", f"Failed to save: {e}")
    
    def launch_analyzer(self):
        """Launch the data analyzer GUI"""
        try:
            # Get path to analyzer script (same directory as this script)
            script_dir = os.path.dirname(os.path.abspath(__file__))
            analyzer_path = os.path.join(script_dir, "analyze_balboa_gui.py")
            
            # Launch analyzer in separate process
            if sys.platform == 'win32':
                subprocess.Popen([sys.executable, analyzer_path], 
                               creationflags=subprocess.CREATE_NEW_CONSOLE)
            else:
                subprocess.Popen([sys.executable, analyzer_path])
            
            self.log("✓ Launched data analyzer")
        except Exception as e:
            self.log(f"✗ Failed to launch analyzer: {e}")
            messagebox.showerror("Error", f"Failed to launch analyzer: {e}")

def main():
    root = tk.Tk()
    app = BalboaController(root)
    root.mainloop()

if __name__ == "__main__":
    main()