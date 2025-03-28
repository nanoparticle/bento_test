import serial
import math
import time
import struct

STLINK_DEBUG_UART = "COM6"
USB_RS485_ADAPTER = "COM7"
USB_CANBUS_ADAPTER = "COM12"

class CANAdapterDriver:
    MESSAGE_NUM_BYTES: int = 2 # MUST NOT BE MORE THAN 8
    DEVICE_CAN_ID: int = 0x0AB # MUST NOT BE MORE THAN 3 THREE HEX CHARACTERS
    def __init__(self, port: str):
        self._port = port
        self._serial_instance = serial.Serial(port, 115200)
        self._serial_instance.write("C\r".encode()) # Close channel, settings can only be changed when channel is closed
        self._serial_instance.read_all() # Flush input buffer
        self._serial_instance.write("S8\r".encode()) # Nominal 1Mbps
        self._serial_instance.write("Y5\r".encode()) # Data 5Mbps
        self._serial_instance.write("M0\r".encode()) # Set to normal mode (I believe this enables ACK)
        self._serial_instance.write("A1\r".encode()) # Enable automatic retransmission
        self._serial_instance.write("O\r".encode()) # Open channel
        self._serial_instance.flush() # Flush output buffer
        self._serial_instance.read_all() # Flush input buffer

    def read_and_print_blocking(self):
        while True:
            temp = self._serial_instance.read_all()
            if temp is not None:
                message = temp.decode()
                if message: print(repr(temp))

    def send_message_wait_for_reply_timeout(self, message: int, timeout_s: float):
        id = f"{self.DEVICE_CAN_ID:03X}"[-3:]
        message_bytes = message.to_bytes(self.MESSAGE_NUM_BYTES, "big")
        self._serial_instance.write(f"b{id}{str(self.MESSAGE_NUM_BYTES)}{message_bytes.hex()}\r".encode())
        end = time.monotonic() + timeout_s
        buffer = ""
        while time.monotonic() < end:
            temp = self._serial_instance.read_all()
            if temp is not None: buffer += temp.decode()
            if '\r' in buffer:
                print(int(buffer[5 : 5 + (2 * self.MESSAGE_NUM_BYTES)], 16))
                return int(buffer[5 : 5 + (2 * self.MESSAGE_NUM_BYTES)], 16) == ((message + 1) % pow(2, 8 * self.MESSAGE_NUM_BYTES))
        return False
    

class RS485AdapterDriver:
    MESSAGE_NUM_BYTES: int = 2
    def __init__(self, port: str):
        self._port = port
        self._serial_instance = serial.Serial(port, 3000000) # should be 3000000 but there is some clock issue
        self._serial_instance.read_all() # Flush input buffer
        
    def read_and_print_blocking(self):
        while True:
            temp = self._serial_instance.read_all()
            if temp is not None:
                message = temp.decode()
                if message: print(repr(temp))

    def send_message_wait_for_reply_timeout(self, message: int, timeout_s: float):
        message_bytes = f"{message:04X}\0".encode()
        self._serial_instance.write(message_bytes)
        end = time.monotonic() + timeout_s
        buffer = ""
        while time.monotonic() < end:
            temp = self._serial_instance.read_all()
            if temp is not None: buffer += temp.decode()
            if '\n' in buffer:
                print(buffer)
                print(int(buffer.split('\n')[0], 16))
                return int(buffer.split('\n')[0], 16) == ((message + 1) % pow(2, 8 * self.MESSAGE_NUM_BYTES))
        return False
        

if __name__ == "__main__":
    # can = CANAdapterDriver(USB_CANBUS_ADAPTER)
    # print(can.send_message_wait_for_reply_timeout(65535, 1.0))
    rs485 = RS485AdapterDriver(USB_RS485_ADAPTER)
    print(rs485.send_message_wait_for_reply_timeout(0, 1.0))
    exit()


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