import pygame
from serial_reader import SerialReader
import time

# Constants
SCREEN_WIDTH = 480
SCREEN_HEIGHT = 320
BG_COLOR = (30, 30, 30)
TEXT_COLOR = (255, 255, 255)
LOCKED_COLOR = (255, 0, 0)
UNLOCKED_COLOR = (0, 255, 0)
PROMPT_COLOR = (200, 200, 0)  # Yellow prompt color
PROMPT_TIMEOUT = 5  # Seconds without a new scan to show prompt again
RELOCK_COUNTDOWN = 2  # Seconds before relock

class DoorLockUI:
    def __init__(self, serial,parent_dashboard=None, width=SCREEN_WIDTH, height=SCREEN_HEIGHT):
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("RFID Door Lock")
        self.clock = pygame.time.Clock()
        self.running = True
        self.serial = serial
        self.parent_dashboard = parent_dashboard

        # Door lock state
        self.lock_status = "Unknown"
        self.last_uid = "None"
        self.color = TEXT_COLOR
        self.last_scan_time = 0

        # Relock countdown
        self.unlock_time = None

        # Font setup
        self.font_large = pygame.font.SysFont(None, 48)
        self.font_small = pygame.font.SysFont(None, 32)
        self.font_prompt = pygame.font.SysFont(None, 28)

    def run(self):
        while self.running:
            data = self.serial.read_data()
            self.update(data)
            self.handle_events()
            self.draw()

        pygame.quit()

        if self.parent_dashboard:
            self.serial.send_command("dashboard ")
            self.parent_dashboard.show()

    def update(self, data):
        if data:
            uid = data.get("rfid")
            status = data.get("status")
            relock_signal = data.get("relock")  # New

            if uid:
                self.last_uid = uid
                self.last_scan_time = time.time()

            if status:
                self.lock_status = status.upper()
                self.color = UNLOCKED_COLOR if self.lock_status == "UNLOCKED" else LOCKED_COLOR

                if self.lock_status == "LOCKED":
                    self.unlock_time = None  # Stop countdown

            if relock_signal:
                self.unlock_time = time.time()
                self.last_scan_time = time.time()

    def draw(self):
        self.screen.fill(BG_COLOR)

        current_time = time.time()
        show_prompt = (current_time - self.last_scan_time) > PROMPT_TIMEOUT

        if show_prompt:
            prompt_text = self.font_prompt.render("Please scan your RFID card...", True, PROMPT_COLOR)
            self.screen.blit(prompt_text, (40, 40))
        else:
            # Lock Status
            status_text = self.font_large.render(f"Status: {self.lock_status}", True, self.color)
            self.screen.blit(status_text, (40, 60))

            # UID
            uid_text = self.font_small.render(f"Last UID: {self.last_uid}", True, TEXT_COLOR)
            self.screen.blit(uid_text, (40, 140))

            # Countdown timer if relock message was received
            if self.unlock_time:
                elapsed = current_time - self.unlock_time
                remaining = max(0.0, RELOCK_COUNTDOWN - elapsed)
                if remaining > 0:
                    countdown_text = self.font_prompt.render(f"Relocking in: {remaining:.1f}s", True, PROMPT_COLOR)
                    self.screen.blit(countdown_text, (40, 200))

        pygame.display.flip()
        self.clock.tick(30)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
