"""
A simple simulator that performs the display logic and nothing else.
"""


import logging

from fastestalgo import constants
import pygame
from mdpalgo.interface.panel import Panel
from mdpalgo.map.grid import Grid
from mdpalgo.robot.robot import Robot, RobotMovement
import mdpalgo.interface
import mdpalgo.images
from imagerec.helpers import get_path_to

# Set the HEIGHT and WIDTH of the screen
WINDOW_SIZE = (600, 960)
PANEL_POSITION = (420, 120)
GRID_BLOCK_SIZE = 18
GRID_FROM_SCREEN_TOP_LEFT = (20, 20)

class SimplySimulator:

    def __init__(self, grid_size_x: int, grid_size_y: int):
        # Initialize pygame
        self.root = pygame
        self.root.init()
        self.root.display.set_caption("MDP Fastest Path Simulator")
        self.screen = pygame.display.set_mode(WINDOW_SIZE, pygame.SCALED)
        self.screen.fill(constants.GRAY)

        # This is the margin around the left and top of the grid on screen
        # display
        self.grid_from_screen_top_left = GRID_FROM_SCREEN_TOP_LEFT
        # Initialise 20 by 20 Grid
        self.grid = Grid(grid_size_x, grid_size_y, GRID_BLOCK_SIZE, self.grid_from_screen_top_left)
        self.redraw_grid()

        # Initialise side panel with buttons
        self.panel = Panel()
        self.panel_from_screen_top_left = PANEL_POSITION
        self.screen.blit(self.panel.get_surface(), self.panel_from_screen_top_left)

        # Used to manage how fast the screen updates
        self.clock = pygame.time.Clock()

        # Set up the car
        image_path = get_path_to(mdpalgo.interface).joinpath("car.png")
        car_image = pygame.image.load(image_path)
        self.car = Robot(self, self.screen, self.grid, constants.ROBOT_W, constants.ROBOT_H,
                         constants.ROBOT_STARTING_X, constants.ROBOT_STARTING_Y, constants.ROBOT_STARTING_ANGLE,
                         car_image)
        # Draw the car
        self.car.draw_car()

    def redraw_grid(self):
        grid_surface = self.grid.get_updated_grid_surface()
        self.screen.blit(grid_surface, self.grid_from_screen_top_left)

    def run(self):
        while True:
            for event in pygame.event.get():
                self.handle_event(event)

            self.root.display.flip()
            self.clock.tick(constants.FPS)

    def handle_event(self, event: pygame.event.Event):
        if event.type == pygame.QUIT:
            # Be IDLE friendly. If you forget this line, the program will 'hang' on exit.
            self.root.quit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            # User clicks the mouse. Get the position
            pos = pygame.mouse.get_pos()
            if self.is_pos_clicked_within_grid(pos):
                self.grid.grid_clicked(pos[0], pos[1])
                self.redraw_grid()
                self.car.draw_car()  # Redraw the car

            else:  # otherwise, area clicked is outside of grid
                self.check_button_clicked(pos)

    def is_pos_clicked_within_grid(self, pos):
        grid_from_screen_left = self.grid_from_screen_top_left[0]
        grid_from_screen_top = self.grid_from_screen_top_left[1]

        grid_pixel_size_x, grid_pixel_size_y = self.grid.get_total_pixel_size()
        if grid_from_screen_left < pos[0] < grid_from_screen_left + grid_pixel_size_x and \
           grid_from_screen_top < pos[1] < grid_from_screen_top + grid_pixel_size_y:
            return True
        return False

    def reprint_screen_and_buttons(self):
        self.screen.fill(constants.GRAY)
        self.panel.redraw_buttons()

    def get_screen_pixel_from_panel_pixel(self, panel_x, panel_y):
        x = self.panel_from_screen_top_left[0] + panel_x
        y = self.panel_from_screen_top_left[1] + panel_y
        return x, y

    def check_button_clicked(self, pos):
        map_button_func_to_robot_movement = {
                "FORWARD": RobotMovement.FORWARD,
                "BACKWARD": RobotMovement.BACKWARD,
                "FORWARD_RIGHT": RobotMovement.FORWARD_RIGHT,
                "FORWARD_LEFT": RobotMovement.FORWARD_LEFT,
                "BACKWARD_RIGHT": RobotMovement.BACKWARD_RIGHT,
                "BACKWARD_LEFT": RobotMovement.BACKWARD_LEFT,
            }
        for button in self.panel.buttons:
            panel_x, panel_y, l, h = button.get_xy_and_lh()
            x, y = self.get_screen_pixel_from_panel_pixel(panel_x, panel_y)
            if (x < pos[0] < (l + x)) and (y < pos[1] < (h + y)):
                button_func = self.panel.get_button_clicked(button)
                if button_func == "RESET":
                    print("Reset button pressed.")
                    self.reset_button_clicked()
                elif button_func == "CONNECT":
                    print("Connect button pressed.")
                elif button_func == "DISCONNECT":
                    print("Disconnect button pressed.")

                # for testing purposes
                elif button_func in map_button_func_to_robot_movement:
                    self.car.perform_move(map_button_func_to_robot_movement[button_func])
                elif button_func == "START":
                    self.start_button_clicked()
                else:
                    return

    def start_button_clicked(self):
        print("START button clicked!")

    def reset_button_clicked(self):
        self.grid.reset_data()
        self.redraw_grid()
        self.car.reset()

if __name__ == "__main__":
    # Set info logging mode
    logging.basicConfig(level=logging.INFO)

    x = SimplySimulator(17, 45)
    x.run()
