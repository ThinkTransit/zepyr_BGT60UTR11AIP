#!/usr/bin/env python3
"""
BGT60 Radar GUI Interface
PyQt6 application for interfacing with BGT60 radar over USB serial
"""

import sys
import serial
import time
import numpy as np
import re
import matplotlib
matplotlib.use('QtAgg')  # Use QtAgg backend for PyQt6
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QPushButton, QTextEdit, QLabel, 
                           QSpinBox, QComboBox, QProgressBar, QTabWidget,
                           QGridLayout, QLineEdit, QScrollArea)
from PyQt6.QtCore import QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QFont, QTextCursor

class SerialWorker(QThread):
    """Worker thread for serial communication"""
    data_received = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    
    def __init__(self, port, baudrate=115200):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        
    def connect(self):
        """Connect to serial port"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            return True
        except Exception as e:
            self.error_occurred.emit(f"Connection failed: {str(e)}")
            return False
            
    def disconnect(self):
        """Disconnect from serial port"""
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            
    def send_command(self, command):
        """Send command to radar"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(f"{command}\n".encode())
                return True
            except Exception as e:
                self.error_occurred.emit(f"Send failed: {str(e)}")
                return False
        return False
        
    def read_response(self, timeout=5):
        """Read response from radar with timeout"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return []
            
        lines = []
        start_time = time.time()
        
        # Give a short delay for command to process
        time.sleep(0.1)
        
        while (time.time() - start_time) < timeout:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        lines.append(line)
                        # Look for shell prompt or command completion
                        if "uart:~$" in line or line.endswith("$"):
                            break
                else:
                    # No data waiting, small delay
                    time.sleep(0.05)
            except Exception as e:
                self.error_occurred.emit(f"Read failed: {str(e)}")
                break
                
        return lines

class RadarPlotWidget(QWidget):
    """Custom widget for radar data plotting"""
    
    def __init__(self):
        super().__init__()
        self.figure = Figure(figsize=(12, 8))
        self.canvas = FigureCanvas(self.figure)
        
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        
        # Create subplots
        self.ax_raw = self.figure.add_subplot(221)
        self.ax_fft_mag = self.figure.add_subplot(222)
        self.ax_fft_phase = self.figure.add_subplot(223)
        self.ax_distance = self.figure.add_subplot(224)
        
        self.figure.tight_layout(pad=1.5, h_pad=2.5)
        
    def plot_raw_data(self, samples):
        """Plot raw sample data"""
        self.ax_raw.clear()
        self.ax_raw.plot(samples)
        self.ax_raw.set_title('Raw Radar Samples')
        self.ax_raw.set_xlabel('Sample Index')
        self.ax_raw.set_ylabel('Amplitude')
        self.ax_raw.grid(True)
        
    def plot_fft_data(self, fft_real, fft_imag):
        """Plot FFT magnitude and phase"""
        # Calculate magnitude and phase
        magnitude = np.sqrt(np.array(fft_real)**2 + np.array(fft_imag)**2)
        phase = np.arctan2(np.array(fft_imag), np.array(fft_real))
        
        # Plot magnitude
        self.ax_fft_mag.clear()
        self.ax_fft_mag.plot(magnitude[:len(magnitude)//2])  # Plot first half
        self.ax_fft_mag.set_title('FFT Magnitude')
        self.ax_fft_mag.set_xlabel('Frequency Bin')
        self.ax_fft_mag.set_ylabel('Magnitude')
        self.ax_fft_mag.grid(True)
        
        # Plot phase
        self.ax_fft_phase.clear()
        self.ax_fft_phase.plot(phase[:len(phase)//2])
        self.ax_fft_phase.set_title('FFT Phase')
        self.ax_fft_phase.set_xlabel('Frequency Bin')
        self.ax_fft_phase.set_ylabel('Phase (radians)')
        self.ax_fft_phase.grid(True)
        
    def plot_distance_profile(self, magnitude, skip_bins=25):
        """Plot distance profile with correct distance calculation"""
        self.ax_distance.clear()
        
        # Use the same parameters as the firmware
        SPEED_OF_LIGHT = 299792458.0  # m/s
        BANDWIDTH_HZ = 5000000000.0   # 5 GHz
        FFT_SIZE = 1024
        NUM_SAMPLES = 512
        
        # Calculate range bin length (same as firmware)
        range_bin_length = SPEED_OF_LIGHT / (2.0 * BANDWIDTH_HZ * FFT_SIZE / NUM_SAMPLES)
        
        # Convert bins to distance in meters, starting from skip_bins
        # This matches the firmware calculation: distance = range_bin_length * (relative_peak_bin + skip)
        max_bins = len(magnitude) // 2  # Only first half of FFT
        bin_indices = np.arange(max_bins)
        distances = bin_indices * range_bin_length
        
        self.ax_distance.plot(distances, magnitude[:max_bins])
        self.ax_distance.set_title('Distance Profile')
        self.ax_distance.set_xlabel('Distance (m)')
        self.ax_distance.set_ylabel('Magnitude')
        self.ax_distance.grid(True)
        
        # Find peak starting from skip_bins (same as firmware)
        search_start = min(skip_bins, max_bins - 1)
        search_magnitude = magnitude[search_start:max_bins]
        
        if len(search_magnitude) > 0:
            relative_peak_idx = np.argmax(search_magnitude)
            absolute_peak_idx = search_start + relative_peak_idx
            
            # Calculate distance same as firmware: range_bin_length * (relative_peak_bin + skip)
            peak_distance = range_bin_length * (relative_peak_idx + skip_bins)
            
            self.ax_distance.axvline(x=peak_distance, color='red', linestyle='--', 
                                    label=f'Peak: {peak_distance:.3f}m (bin {absolute_peak_idx})')
            self.ax_distance.legend()
            
            # Also mark the actual search start
            search_distance = range_bin_length * skip_bins
            self.ax_distance.axvline(x=search_distance, color='orange', linestyle=':', alpha=0.5,
                                    label=f'Search start: {search_distance:.3f}m (skip={skip_bins})')
            self.ax_distance.legend()
        
    def refresh_plot(self):
        """Refresh the plot canvas"""
        self.figure.tight_layout(pad=1.5, h_pad=2.5)
        self.canvas.draw()

class RadarMainWindow(QMainWindow):
    """Main application window"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BGT60 Radar Interface")
        self.setGeometry(100, 100, 1200, 800)
        
        # Serial connection
        self.serial_worker = None
        
        # Data storage
        self.raw_samples = []
        self.fft_real = []
        self.fft_imag = []
        
        self.init_ui()
        
    def init_ui(self):
        """Initialize user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Create main layout
        main_layout = QVBoxLayout(central_widget)
        
        # Connection panel
        conn_layout = QHBoxLayout()
        
        self.port_combo = QComboBox()
        self.port_combo.addItems(["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"])
        self.port_combo.setEditable(True)
        
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["115200"])
        self.baudrate_combo.setCurrentText("115200")
        
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        
        self.status_label = QLabel("Disconnected")
        
        conn_layout.addWidget(QLabel("Port:"))
        conn_layout.addWidget(self.port_combo)
        conn_layout.addWidget(QLabel("Baud:"))
        conn_layout.addWidget(self.baudrate_combo)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(QLabel("Status:"))
        conn_layout.addWidget(self.status_label)
        conn_layout.addStretch()
        
        main_layout.addLayout(conn_layout)
        
        # Create tabs
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)
        
        # Control tab (with integrated plots)
        self.create_control_tab()
        
        # Terminal tab
        self.create_terminal_tab()
        
    def create_control_tab(self):
        """Create simplified control panel tab"""
        control_widget = QWidget()
        layout = QVBoxLayout(control_widget)
        
        
        # Main controls in a horizontal layout
        main_controls = QHBoxLayout()
        
        # ONE SHOT button - does everything
        self.one_shot_btn = QPushButton("🎯 ONE SHOT\n(Init + Measure + Plot)")
        self.one_shot_btn.clicked.connect(self.one_shot_measurement)
        self.one_shot_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 20px;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        main_controls.addWidget(self.one_shot_btn)
        
        # Continuous measurement button
        self.continuous_btn = QPushButton("📡 Continuous\n(20 measurements)")
        self.continuous_btn.clicked.connect(self.continuous_measurement)
        self.continuous_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-size: 14px;
                font-weight: bold;
                padding: 20px;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
        """)
        main_controls.addWidget(self.continuous_btn)
        
        layout.addLayout(main_controls)
        
        # Settings row
        settings_layout = QHBoxLayout()
        settings_layout.addWidget(QLabel("Skip Bins:"))
        self.skip_spinbox = QSpinBox()
        self.skip_spinbox.setRange(0, 100)
        self.skip_spinbox.setValue(25)
        settings_layout.addWidget(self.skip_spinbox)
        
        # Gain control
        settings_layout.addWidget(QLabel("Gain:"))
        self.gain_combo = QComboBox()
        self.gain_combo.addItems(["0 dB", "5 dB", "10 dB", "15 dB", "20 dB", "25 dB", "30 dB"])
        self.gain_combo.setCurrentText("20 dB")  # Default gain
        self.gain_combo.currentTextChanged.connect(self.set_gain)
        settings_layout.addWidget(self.gain_combo)
        
        # High Pass Filter control
        settings_layout.addWidget(QLabel("HPF:"))
        self.hpf_combo = QComboBox()
        self.hpf_combo.addItems(["20 kHz", "40 kHz", "80 kHz", "140 kHz", "160 kHz"])
        self.hpf_combo.setCurrentText("20 kHz")  # Default HPF
        self.hpf_combo.currentTextChanged.connect(self.set_hpf)
        settings_layout.addWidget(self.hpf_combo)
        
        self.status_btn = QPushButton("Status")
        self.status_btn.clicked.connect(self.get_status)
        settings_layout.addWidget(self.status_btn)
        
        settings_layout.addStretch()
        layout.addLayout(settings_layout)
        
        # Progress bar
        self.progress = QProgressBar()
        layout.addWidget(self.progress)
        
        # Add the plot widget directly to this tab
        self.plot_widget = RadarPlotWidget()
        layout.addWidget(self.plot_widget)
        
        self.tabs.addTab(control_widget, "Radar Control & Plots")
        
    def create_terminal_tab(self):
        """Create terminal tab"""
        terminal_widget = QWidget()
        layout = QVBoxLayout(terminal_widget)
        
        self.terminal_output = QTextEdit()
        self.terminal_output.setFont(QFont("Courier", 10))
        self.terminal_output.setReadOnly(True)
        layout.addWidget(self.terminal_output)
        
        # Command input
        cmd_layout = QHBoxLayout()
        self.cmd_input = QLineEdit()
        self.cmd_input.returnPressed.connect(self.send_custom_command)
        
        self.send_btn = QPushButton("Send")
        self.send_btn.clicked.connect(self.send_custom_command)
        
        cmd_layout.addWidget(QLabel("Command:"))
        cmd_layout.addWidget(self.cmd_input)
        cmd_layout.addWidget(self.send_btn)
        
        layout.addLayout(cmd_layout)
        
        self.tabs.addTab(terminal_widget, "Terminal")
        
    def toggle_connection(self):
        """Toggle serial connection"""
        if self.serial_worker is None:
            # Connect
            port = self.port_combo.currentText()
            baudrate = int(self.baudrate_combo.currentText())
            self.serial_worker = SerialWorker(port, baudrate)
            
            try:
                if self.serial_worker.connect():
                    # Connect error signal
                    self.serial_worker.error_occurred.connect(self.log_message)
                    
                    self.status_label.setText("Connected")
                    self.connect_btn.setText("Disconnect")
                    self.log_message(f"Connected to {port} @ {baudrate} baud")
                    
                    # Initialize radar after connection
                    self.log_message("Initializing radar...")
                    self.send_command("radar init")
                    self.log_message("Radar initialized with default settings")
                else:
                    self.serial_worker = None
                    self.status_label.setText("Connection Failed")
                    self.log_message("Failed to open serial port")
            except Exception as e:
                self.serial_worker = None
                self.status_label.setText("Connection Failed")
                self.log_message(f"Connection error: {str(e)}")
        else:
            # Disconnect
            try:
                self.serial_worker.disconnect()
            except Exception as e:
                self.log_message(f"Disconnect error: {str(e)}")
            finally:
                self.serial_worker = None
                self.status_label.setText("Disconnected")
                self.connect_btn.setText("Connect")
                self.log_message("Disconnected")
            
    def send_command(self, command):
        """Send command and return response"""
        if not self.serial_worker:
            self.log_message("Error: Not connected")
            return []
            
        self.log_message(f"> {command}")
        
        try:
            if self.serial_worker.send_command(command):
                response = self.serial_worker.read_response()
                for line in response:
                    self.log_message(line)
                return response
            else:
                self.log_message("Error: Failed to send command")
                return []
        except Exception as e:
            self.log_message(f"Error sending command: {str(e)}")
            return []
            
    def log_message(self, message):
        """Log message to terminal"""
        # Remove ANSI escape sequences (color codes)
        clean_message = re.sub(r'\x1b\[[0-9;]*m', '', message)
        # Remove other control characters
        clean_message = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', clean_message)
        
        self.terminal_output.append(clean_message)
        self.terminal_output.moveCursor(QTextCursor.MoveOperation.End)
        
    def one_shot_measurement(self):
        """Complete workflow: init, measure, export, and plot"""
        if not self.serial_worker:
            self.log_message("Error: Not connected to radar")
            return
            
        try:
            self.progress.setValue(0)
            self.one_shot_btn.setEnabled(False)
            self.log_message("=== Starting One Shot Measurement ===")
            
            # Step 1: Apply UI configuration settings (20%)
            self.progress.setValue(20)
            self.log_message("Step 1/5: Applying configuration settings...")
            
            # Set skip value
            skip_value = self.skip_spinbox.value()
            self.log_message(f"  Setting skip to {skip_value}...")
            self.send_command(f"radar config skip {skip_value}")
            
            # Set gain
            gain_text = self.gain_combo.currentText()
            gain_db = int(gain_text.split()[0])
            self.log_message(f"  Setting gain to {gain_db} dB...")
            self.send_command(f"radar config gain {gain_db}")
            
            # Set HPF
            hpf_text = self.hpf_combo.currentText()
            hpf_khz = int(hpf_text.split()[0])
            self.log_message(f"  Setting HPF to {hpf_khz} kHz...")
            self.send_command(f"radar config hpf {hpf_khz}")
            
            # Step 2: Take measurement (40%)
            self.progress.setValue(40)
            self.log_message("Step 2/5: Taking measurement...")
            self.send_command("radar measure")
            
            # Step 3: Export data (60%)
            self.progress.setValue(60)
            self.log_message("Step 3/5: Exporting data...")
            
            # Export raw data
            raw_response = self.send_command("radar export raw")
            self.parse_raw_data(raw_response)
            
            # Export FFT data
            fft_response = self.send_command("radar export fft")
            self.parse_fft_data(fft_response)
            
            # Step 4: Plot data (100%)
            self.progress.setValue(100)
            self.log_message("Step 4/5: Plotting data...")
            self.plot_data()
            
            self.log_message("=== One Shot Measurement Complete! ===")
            
        except Exception as e:
            self.log_message(f"Error during one shot measurement: {str(e)}")
        finally:
            self.one_shot_btn.setEnabled(True)
            # Reset progress after a short delay
            import threading
            def reset_progress():
                time.sleep(2)
                self.progress.setValue(0)
            threading.Thread(target=reset_progress).start()
        
    def init_radar(self):
        """Initialize radar"""
        self.log_message("Initializing radar...")
        try:
            self.send_command("radar init")
        except Exception as e:
            self.log_message(f"Error initializing radar: {str(e)}")
        
    def get_status(self):
        """Get radar status"""
        self.log_message("Getting radar status...")
        try:
            self.send_command("radar status")
        except Exception as e:
            self.log_message(f"Error getting status: {str(e)}")
        
    def set_skip(self):
        """Set skip value"""
        skip_value = self.skip_spinbox.value()
        self.log_message(f"Setting skip value to {skip_value}...")
        try:
            self.send_command(f"radar config skip {skip_value}")
        except Exception as e:
            self.log_message(f"Error setting skip: {str(e)}")
    
    def set_gain(self):
        """Set gain value"""
        gain_text = self.gain_combo.currentText()
        gain_db = int(gain_text.split()[0])  # Extract numeric value
        self.log_message(f"Setting gain to {gain_db} dB...")
        try:
            response = self.send_command(f"radar config gain {gain_db}")
            # Note: Currently the firmware doesn't support direct gain control
            # It will show instructions for manual register control
        except Exception as e:
            self.log_message(f"Error setting gain: {str(e)}")
    
    def set_hpf(self):
        """Set High Pass Filter frequency"""
        hpf_text = self.hpf_combo.currentText()
        hpf_khz = int(hpf_text.split()[0])  # Extract numeric value
        self.log_message(f"Setting HPF to {hpf_khz} kHz...")
        try:
            response = self.send_command(f"radar config hpf {hpf_khz}")
        except Exception as e:
            self.log_message(f"Error setting HPF: {str(e)}")
        
    def single_measurement(self):
        """Take single measurement"""
        self.log_message("Taking single measurement...")
        try:
            self.send_command("radar measure")
        except Exception as e:
            self.log_message(f"Error taking measurement: {str(e)}")
        
    def continuous_measurement(self):
        """Take continuous measurements"""
        self.log_message("Starting continuous measurements...")
        try:
            self.send_command("radar continuous 1000 20")
        except Exception as e:
            self.log_message(f"Error starting continuous: {str(e)}")
        
    def export_raw_data(self):
        """Export raw sample data"""
        self.log_message("Exporting raw data...")
        try:
            response = self.send_command("radar export raw")
            self.parse_raw_data(response)
        except Exception as e:
            self.log_message(f"Error exporting raw data: {str(e)}")
        
    def export_fft_data(self):
        """Export FFT data"""
        self.log_message("Exporting FFT data...")
        try:
            response = self.send_command("radar export fft")
            self.parse_fft_data(response)
        except Exception as e:
            self.log_message(f"Error exporting FFT data: {str(e)}")
        
    def parse_raw_data(self, response):
        """Parse raw data response (supports both old and new formats)"""
        self.raw_samples = []
        in_data_section = False
        
        self.log_message(f"Raw data response has {len(response)} lines")
        
        for i, line in enumerate(response):
            line = line.strip()
            if line == "RAW_START":
                in_data_section = True
                self.log_message("Found RAW_START")
                continue
            elif line == "RAW_END":
                in_data_section = False
                self.log_message("Found RAW_END")
                break
            elif line.startswith("SAMPLES:"):
                num_samples = int(line.split(":")[1])
                self.log_message(f"Expecting {num_samples} samples")
                continue
                
            if in_data_section:
                # Parse batched data: "val1,val2,val3,..." format (new)
                if "," in line:
                    values = line.split(",")
                    for val_str in values:
                        try:
                            self.raw_samples.append(int(val_str.strip()))
                        except ValueError:
                            self.log_message(f"Failed to parse batch value: '{val_str}'")
                # Single value per line (old format)
                elif line.isdigit():
                    self.raw_samples.append(int(line))
                elif line and not line.startswith("uart:"):
                    self.log_message(f"Unrecognized raw data line {i}: '{line}'")
                
        self.log_message(f"Parsed {len(self.raw_samples)} raw samples")
        
    def parse_fft_data(self, response):
        """Parse FFT data response (supports both old and new formats)"""
        self.fft_real = []
        self.fft_imag = []
        in_data_section = False
        
        self.log_message(f"FFT data response has {len(response)} lines")
        
        for i, line in enumerate(response):
            line = line.strip()
            if line == "FFT_START":
                in_data_section = True
                self.log_message("Found FFT_START")
                continue
            elif line == "FFT_END":
                in_data_section = False
                self.log_message("Found FFT_END")
                break
            elif line.startswith("SIZE:"):
                fft_size = int(line.split(":")[1])
                self.log_message(f"Expecting {fft_size} FFT points")
                continue
                
            if in_data_section:
                # Parse batched FFT data: "real1,imag1;real2,imag2;..." format (new)
                if ";" in line:
                    pairs = line.split(";")
                    for pair in pairs:
                        if "," in pair:
                            try:
                                real_str, imag_str = pair.split(",")
                                real_val = float(real_str) / 1000.0
                                imag_val = float(imag_str) / 1000.0
                                self.fft_real.append(real_val)
                                self.fft_imag.append(imag_val)
                            except ValueError:
                                self.log_message(f"Failed to parse batch FFT pair: '{pair}'")
                # Single complex pair per line (old format)
                elif "," in line:
                    try:
                        real_str, imag_str = line.split(",")
                        real_val = float(real_str) / 1000.0
                        imag_val = float(imag_str) / 1000.0
                        self.fft_real.append(real_val)
                        self.fft_imag.append(imag_val)
                    except ValueError:
                        self.log_message(f"Failed to parse FFT pair: '{line}'")
                elif line and not line.startswith("uart:"):
                    self.log_message(f"Unrecognized FFT data line {i}: '{line}'")
                    
        self.log_message(f"Parsed {len(self.fft_real)} FFT points")
        
    def plot_data(self):
        """Plot the exported data"""
        if not self.raw_samples and not self.fft_real:
            self.log_message("No data to plot. Export data first.")
            return
            
        try:
            if self.raw_samples:
                self.log_message(f"Plotting {len(self.raw_samples)} raw samples...")
                self.plot_widget.plot_raw_data(self.raw_samples)
                
            if self.fft_real and self.fft_imag:
                self.log_message(f"Plotting FFT data ({len(self.fft_real)} points)...")
                self.plot_widget.plot_fft_data(self.fft_real, self.fft_imag)
                
                # Calculate magnitude for distance profile
                magnitude = np.sqrt(np.array(self.fft_real)**2 + np.array(self.fft_imag)**2)
                # Use the current skip value from the spinbox
                skip_value = self.skip_spinbox.value()
                self.plot_widget.plot_distance_profile(magnitude, skip_value)
                
            self.plot_widget.refresh_plot()
            self.log_message("✓ Plots updated successfully!")
            
        except Exception as e:
            self.log_message(f"Error plotting data: {str(e)}")
        
    def read_register(self):
        """Read register value"""
        addr = self.reg_addr_input.text().strip()
        if not addr:
            self.log_message("Error: Please enter register address")
            return
        
        self.send_command(f"radar register read {addr}")
        
    def write_register(self):
        """Write register value"""
        addr = self.reg_write_addr_input.text().strip()
        value = self.reg_write_val_input.text().strip()
        
        if not addr or not value:
            self.log_message("Error: Please enter both address and value")
            return
            
        self.send_command(f"radar register write {addr} {value}")
        
    def dump_registers(self):
        """Dump register values"""
        self.send_command("radar register dump 0x00 16")
        
    def send_custom_command(self):
        """Send custom command from terminal"""
        command = self.cmd_input.text().strip()
        if command:
            self.send_command(command)
            self.cmd_input.clear()

def main():
    app = QApplication(sys.argv)
    window = RadarMainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()