import serial
import threading
import time

class SerialManager:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
        except Exception as e:
            print(f"[WARN] Cannot connect to serial {e}")
            self.ser = None

        self.buffer = "" 
        self.running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _reader_loop(self):
        while self.running:
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode().strip()
                    self.buffer = line
                    print(f"[Serial IN] {line}")
                except:
                    print(f"[Serial ERR] {e}")
            time.sleep(0.05)

    def get_latest_data(self):
        return self.buffer
    
    def send_command(self, command: str):
        if self.ser:
            self.ser.write((command + '\n').encode())

    def shutdown(self):
        self.running = False
        if self.ser:
            self.ser.close()