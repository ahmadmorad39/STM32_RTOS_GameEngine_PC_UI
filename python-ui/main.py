# main.py
from serial_reader import SerialReader
from game_ui import GameUI


def main():
    serial = SerialReader(port="COM3", baudrate=115200)
    gui = GameUI()

    while gui.running:
        data = serial.read_data()
        gui.draw(data)
        gui.handle_events()

    print("Exiting...")


if __name__ == "__main__":
    main()
