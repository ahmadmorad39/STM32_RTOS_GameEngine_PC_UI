# serial_reader.py
import serial
import json

class SerialReader:
    def __init__(self, port="COM3", baudrate=115200):
        self.ser = serial.Serial(port, baudrate=baudrate, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE,
                            bytesize=7)

    def read_data(self):
        try:
            line = self.ser.readline()
            if line:
                # Try to decode and parse the JSON safely
                try:
                    print(f"line = {line}")
                    data = json.loads(line.decode('utf-8', errors='ignore').strip())
                    return data
                except (UnicodeDecodeError, json.JSONDecodeError) as e:
                    print(f"Error decoding or parsing data: {e}, Raw data: {line}")
                    return None
        except serial.SerialException as e:
            print(f"Error reading serial data: {e}")
            return None

