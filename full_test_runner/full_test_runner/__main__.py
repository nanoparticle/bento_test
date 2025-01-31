import serial
import can
import math
import time

STLINK_DEBUG_UART = "COM6"
USB_RS485_ADAPTER = "COM7"
USB_CANBUS_ADAPTER = "COM12"

if __name__ == "__main__":
    time.sleep(5)
    s = serial.Serial("COM4", 115200)
    increment_deg = 90
    delay_s = 0.08
    try:
        position = 0
        # while True:
        #     s.write(f"M{position}\n".encode())
        #     position += math.radians(increment_deg) * 10
        #     time.sleep(delay_s)

        d = 0.25
        while True:
            s.write(f"M{0}\n".encode())
            time.sleep(d)
            s.write(f"M{math.radians(45) * 10}\n".encode())
            time.sleep(d)

    except KeyboardInterrupt:
        s.close()
    
    s.close()