from projects.snake_game import SnakeGame
from projects.door_lock import DoorLockUI
from serial_reader import SerialReader


def launch_project(name, parent_dashboard=None):
    serial = SerialReader(port="COM3", baudrate=115200)
    serial.send_command(name)
    if name == "snake_game":
        snakegame_instance = SnakeGame(serial, parent_dashboard)
        snakegame_instance.run()
    elif name == "door_lock ":
        DoorLockUI(serial, parent_dashboard).run()
