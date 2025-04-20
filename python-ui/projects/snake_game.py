# game_ui.py
import pygame
import random
from serial_reader import SerialReader
# Constants
ADC_MAX = 4095
SCREEN_WIDTH = 780
SCREEN_HEIGHT = 720
ADC_CENTER = 2400
DEADZONE = 300
GRID_SIZE = 20


def scale_adc_to_screen(adc_value, screen_size, adc_max=ADC_MAX, adc_center=ADC_CENTER):
    # Scale the ADC value to screen coordinates with deadzone
    scaled = (adc_value - adc_center) / (adc_max / 2)
    if abs(scaled) < DEADZONE / (adc_max / 2):
        return 0
    return int(scaled * (screen_size / 2))

class SnakeGame:
    def __init__(self, serial, parent_dashboard=None, width=SCREEN_WIDTH, height=SCREEN_HEIGHT):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Snake Game with STM32")
        self.clock = pygame.time.Clock()
        self.running = True
        self.serial = serial
        self.parent_dashboard = parent_dashboard  # Save it

        # Snake properties
        self.snake = [(width // 2, height // 2)]
        self.snake_length = 1
        self.snake_direction = "RIGHT"
        self.speed = 5
        self.move_counter = 0

        # Food
        self.food_pos = self.generate_food()
        self.score = 0

        # Fonts
        self.font = pygame.font.SysFont(None, 36)

    def run(self):
        while self.running:
            data = self.serial.read_data()
            self.draw(data)
            self.handle_events()

        pygame.quit()

        if self.parent_dashboard:
            self.serial.send_command("dashboard")
            self.parent_dashboard.show()

    def generate_food(self):
        x = random.randrange(GRID_SIZE, self.screen.get_width() - GRID_SIZE, GRID_SIZE)
        y = random.randrange(GRID_SIZE, self.screen.get_height() - GRID_SIZE, GRID_SIZE)
        return (x, y)

    def draw(self, game_data):
        self.screen.fill((0, 0, 0))  # Black background

        # Process joystick input
        if game_data:
            x_input = scale_adc_to_screen(game_data.get("x", ADC_CENTER), self.screen.get_width())
            y_input = scale_adc_to_screen(game_data.get("y", ADC_CENTER), self.screen.get_height())

            # Determine direction based on joystick
            if abs(x_input) > abs(y_input):
                if x_input > 0 and self.snake_direction != "LEFT":
                    self.snake_direction = "RIGHT"
                elif x_input < 0 and self.snake_direction != "RIGHT":
                    self.snake_direction = "LEFT"
            else:
                if y_input > 0 and self.snake_direction != "UP":
                    self.snake_direction = "DOWN"
                elif y_input < 0 and self.snake_direction != "DOWN":
                    self.snake_direction = "UP"

        # Move snake
        self.move_counter += 1
        if self.move_counter >= self.speed:
            self.move_counter = 0
            self.move_snake()

        # Check collision with food
        head_x, head_y = self.snake[0]
        food_x, food_y = self.food_pos

        if (abs(head_x - food_x) < GRID_SIZE and
                abs(head_y - food_y) < GRID_SIZE):
            self.snake_length += 1
            self.food_pos = self.generate_food()
            self.score += 10
            # Increase speed every 5 food items
            if self.score % 50 == 0 and self.speed > 1:
                self.speed -= 1

        # Draw food
        pygame.draw.rect(self.screen, (255, 0, 0),
                         (self.food_pos[0], self.food_pos[1], GRID_SIZE, GRID_SIZE))

        # Draw snake
        for segment in self.snake:
            pygame.draw.rect(self.screen, (0, 255, 0),
                             (segment[0], segment[1], GRID_SIZE, GRID_SIZE))

        # Draw head with different color
        pygame.draw.rect(self.screen, (0, 200, 0),
                         (self.snake[0][0], self.snake[0][1], GRID_SIZE, GRID_SIZE))

        # Draw score
        score_text = self.font.render(f"Score: {self.score}", True, (255, 255, 255))
        self.screen.blit(score_text, (10, 10))

        # Check for game over
        if self.check_collision():
            game_over_text = self.font.render("GAME OVER! Press R to restart", True, (255, 0, 0))
            self.screen.blit(game_over_text, (self.screen.get_width() // 2 - 180, self.screen.get_height() // 2))

        pygame.display.flip()
        self.clock.tick(60)

    def move_snake(self):
        if len(self.snake) < self.snake_length:
            # Add a new segment
            self.snake.append(self.snake[-1])

        for i in range(len(self.snake) - 1, 0, -1):
            self.snake[i] = (self.snake[i - 1][0], self.snake[i - 1][1])

        # Move head
        head_x, head_y = self.snake[0]
        if self.snake_direction == "RIGHT":
            self.snake[0] = (head_x + GRID_SIZE, head_y)
        elif self.snake_direction == "LEFT":
            self.snake[0] = (head_x - GRID_SIZE, head_y)
        elif self.snake_direction == "UP":
            self.snake[0] = (head_x, head_y - GRID_SIZE)
        elif self.snake_direction == "DOWN":
            self.snake[0] = (head_x, head_y + GRID_SIZE)

    def check_collision(self):
        head_x, head_y = self.snake[0]

        # Check wall collision
        if (head_x < 0 or head_x >= self.screen.get_width() or
                head_y < 0 or head_y >= self.screen.get_height()):
            return True

        # Check self collision (skip head)
        for segment in self.snake[1:]:
            if head_x == segment[0] and head_y == segment[1]:
                return True

        return False

    def reset_game(self):
        self.snake = [(self.screen.get_width() // 2, self.screen.get_height() // 2)]
        self.snake_length = 1
        self.snake_direction = "RIGHT"
        self.food_pos = self.generate_food()
        self.score = 0
        self.speed = 5

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r and self.check_collision():
                    self.reset_game()