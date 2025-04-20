# serial_reader.py
import serial
import json

class SerialReader:
    def __init__(self, port="COM3", baudrate=115200):
        self.ser = serial.Serial(port, baudrate=baudrate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS)

    def read_data(self):
        try:
            line = self.ser.readline()
            if line:
                line_str = line.decode('utf-8', errors='ignore').strip()
                print(f"line = {line_str}")
                # Only try to parse if it looks like JSON
                if line_str.startswith('{') or line_str.startswith('['):
                    try:
                        data = json.loads(line_str)
                        return data
                    except json.JSONDecodeError as e:
                        print(f"JSON decode error: {e}, Raw data: {line_str}")
                        return None
                else:
                    print(f"Non-JSON message received: {line_str}")
                    return None
        except serial.SerialException as e:
            print(f"Error reading serial data: {e}")
            return None

    def send_command(self, command_str: str):
        if not self.ser.is_open:
            self.ser.open()

        # Convert to 10-byte ASCII string with padding if necessary
        command_bytes = command_str.encode('ascii')[:10].ljust(10, b' ')

        self.ser.write(command_bytes)
        print(f"Sent command: {command_bytes}")

        # Define valid commands, exactly 10 bytes
        known = {
            b'snake_game': "snake game",
            b'door_lock ': "Door lock",
            b'dashboard ' : "dashboard "
        }

        if command_bytes in known:
            print(f"-> Triggered: {known[command_bytes]}")
        else:
            print("-> Sent 10 bytes, unknown command")


