import serial
import time

port = '/dev/serial0'
baudrate = 115200
ser_timeout = 1

try:
    ser = serial.Serial(port, baudrate, timeout=ser_timeout)
    print(f"Succesfully connected to port: {port}")

    while True:
        if ser.in_waiting > 0:
            received = ser.read(ser.in_waiting)
            print(f"Received uart msg: {received}")
        time.sleep(0.1)

except serial.SerialException as e:
    print(f"Serial exception: {e}")

except KeyboardInterrupt:
    print(f"Closing program...")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Successfully closed serial port")