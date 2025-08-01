import serial
import time

port = '/dev/serial0'
baudrate = 115200
timeout = 1

try:
    ser = serial.Serial(port, baudrate, timeout=timeout)
    print(f"Serial port opened successfully.\nUsing port: {port}")

    test_data = b'\x01\x02\x03\x04'

    while True:
        print(f"Sending: {test_data}")
        ser.write(test_data)

        time.sleep(0.1)  # small delay before reading

        received = ser.read(len(test_data))
        print(f"Received: {received}")

        if received == test_data:
            print("Loopback successful ✅")
        else:
            print("Loopback mismatch ❌")

        time.sleep(1)  # wait before next iteration

except serial.SerialException as e:
    print(f"SerialException: {e}")

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")
