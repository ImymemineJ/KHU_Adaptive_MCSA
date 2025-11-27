import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

# ==============================================================================
# Configuration
# ==============================================================================
SERIAL_PORT = '/dev/tty.usbmodem1303'  # Change this to your STM32 COM port
BAUD_RATE = 460800    # Must match STM32 setting
FFT_LEN = 512
DATA_LEN = FFT_LEN // 2
FS = 4096.0           # Sampling Frequency

# ==============================================================================
# Serial Reader
# ==============================================================================
class SerialReader:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.ser.reset_input_buffer() # Clear any stale data
        self.data = np.zeros(DATA_LEN)
        self.running = True
        self.lock = threading.Lock()
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0
        self.temperature = 0.0
        
    def read_loop(self):
        print(f"Listening on {self.ser.port} at {self.ser.baudrate}...")
        while self.running:
            try:
                # 1. Sync: Look for 0xFFFFFFFF Header
                # Read one byte at a time until we find the sequence
                sync_buf = b''
                while len(sync_buf) < 4:
                    byte = self.ser.read(1)
                    if not byte: continue # Timeout or no data
                    
                    sync_buf += byte
                    
                    # Check if the last 4 bytes match the header
                    if len(sync_buf) >= 4:
                        if sync_buf[-4:] == b'\xff\xff\xff\xff':
                            break # Found header!
                        
                        # Keep only last 3 bytes to append next one
                        sync_buf = sync_buf[-3:]
                
                # 2. Read Data Payload (FFT_LEN/2 floats + 1 float for Temp + 5 floats for Features)
                # FFT_LEN = 512 -> 256 floats for FFT + 1 float for Temp + 5 floats for Features = 262 floats
                # 262 * 4 = 1048 bytes
                raw_data = self.ser.read((DATA_LEN + 1 + 5) * 4)
                
                if len(raw_data) == (DATA_LEN + 1 + 5) * 4:
                    # Unpack binary data to float array
                    floats = struct.unpack(f'{DATA_LEN + 1 + 5}f', raw_data)
                    
                    fft_data = floats[:DATA_LEN]
                    temp_val = floats[DATA_LEN]
                    features = floats[DATA_LEN+1:] # F1~F5
                    
                    # Basic sanity check: Magnitude shouldn't be infinite or NaN
                    if np.all(np.isfinite(fft_data)):
                        with self.lock:
                            self.data = np.array(fft_data)
                            self.temperature = temp_val
                            self.features = features
                            self.frame_count += 1
                    else:
                        print("Warning: Received NaN/Inf data, skipping frame.")
                        self.ser.reset_input_buffer() # Resync
                else:
                    print("Warning: Incomplete frame received.")
                    
            except Exception as e:
                print(f"Serial Error: {e}")
                time.sleep(0.1)

    def stop(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()

# ==============================================================================
# Visualization
# ==============================================================================
def main():
    try:
        reader = SerialReader(SERIAL_PORT, BAUD_RATE)
        thread = threading.Thread(target=reader.read_loop)
        thread.daemon = True
        thread.start()
        
        # Setup Plot
        fig, ax = plt.subplots()
        plt.subplots_adjust(right=0.75) # Make room for text box
        
        x_axis = np.linspace(0, FS/2, DATA_LEN)
        line, = ax.plot(x_axis, np.zeros(DATA_LEN))
        
        ax.set_ylim(0, 500000) # Initial Y-axis limit
        ax.set_xlim(0, FS/2)
        ax.set_title("Real-Time FFT Spectrum")
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("Magnitude")
        ax.grid(True)
        
        # Text Box for Features
        text_box = fig.text(0.78, 0.5, '', fontsize=10, verticalalignment='center',
                            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # FPS Calculation Variables
        fps_start_time = time.time()
        fps_frame_count = 0
        current_fps = 0.0
        
        def update(frame):
            nonlocal fps_start_time, fps_frame_count, current_fps
            
            with reader.lock:
                y_data = reader.data.copy()
                total_frames = reader.frame_count
                temp = reader.temperature
                feats = getattr(reader, 'features', [0.0]*5)
            
            # Calculate FPS based on reader's frame count
            now = time.time()
            dt = now - fps_start_time
            if dt >= 0.5:
                current_fps = (total_frames - fps_frame_count) / dt
                print(f"FPS: {current_fps:.1f}, Temp: {temp:.1f} C")
                fps_frame_count = total_frames
                fps_start_time = now
            
            # Ignore DC component (Index 0) for better visualization
            if len(y_data) > 0:
                y_data[0] = 0 
            
            line.set_ydata(y_data)
            
            # Dynamic Y-axis adjustment (Auto-scale)
            max_val = np.max(y_data) if len(y_data) > 0 else 0
            if max_val > 0:
                current_ylim = ax.get_ylim()[1]
                if max_val > current_ylim:
                    ax.set_ylim(0, max_val * 1.2)
                elif max_val < current_ylim * 0.5 and max_val > 10:
                    ax.set_ylim(0, max_val * 1.2)
            
            ax.set_title(f"Real-Time FFT Spectrum (FPS: {current_fps:.1f})")
            
            # Update Text Box
            info_text = (f"Temp: {temp:.1f} C\n\n"
                         f"Features:\n"
                         f"F1 (Peak): {feats[0]:.0f}\n"
                         f"F2 (Side): {feats[1]:.0f}\n"
                         f"F3 (Harm): {feats[2]:.0f}\n"
                         f"F4 (RMS):  {feats[3]:.0f}\n"
                         f"F5 (THD):  {feats[4]*100:.2f}%")
            text_box.set_text(info_text)
                
            return line, text_box

        ani = FuncAnimation(fig, update, interval=30, blit=False)
        plt.show()
        
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}.")
        print("Check if the port is correct and not used by another program (e.g., Tera Term).")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        if 'reader' in locals():
            reader.stop()

if __name__ == "__main__":
    main()
