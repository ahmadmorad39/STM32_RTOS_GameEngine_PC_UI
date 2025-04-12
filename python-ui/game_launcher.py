# game_launcher.py
from projects.snake_game import SnakeGame  # You can map more here later

def launch_project(name):
    if name == "Snake Game":
        snakegame_instance = SnakeGame()
        snakegame_instance.run()
