import time
from math import degrees
from pygame.math import Vector2
from mdpalgo import constants
import pygame
from mdpalgo.constants import BUFFER
from mdpalgo.map.configuration import Pose
from mdpalgo.map.grid import Grid
from enum import Enum

# This sets the margin between each Cell
MARGIN = 2
ONE_CELL = 20 + MARGIN
THREE_CELL = 3 * ONE_CELL
dt = 0.2     # Max without messing up is 0.8


# dt = round(self.clock.get_time() / 1000, 2)

class BorderException(Exception):
    pass


class ObstacleException(Exception):
    pass


class ObstacleTurnException(Exception):
    pass

class CheckingException(Exception):
    pass

class RobotMovement(Enum):
    FORWARD = "F"
    BACKWARD = "B"
    FORWARD_RIGHT = "FR"
    FORWARD_LEFT = "FL"
    BACKWARD_RIGHT = "BR"
    BACKWARD_LEFT = "BL"

class Robot(object):

    def __init__(self, simulator, screen, grid: Grid, robot_w, robot_h, grid_x, grid_y, angle, car_image):
        self.robot_w = robot_w
        self.robot_h = robot_h
        # actual pixel width and height of robot (inclusive of margin)
        self.screen_width = grid.get_block_size() * robot_w / 10 + (robot_w / 10 * MARGIN) + MARGIN
        self.screen_height = grid.get_block_size() * robot_h / 10 + (robot_h / 10 * MARGIN) + MARGIN
        # NOTE: grid_x and grid_y are the grid coordinates of the middle square of the 3x3 car area
        self.grid_x = grid_x
        self.grid_y = grid_y
        self.simulator = simulator
        self.screen = screen
        self.grid = grid
        # the position of the middle of the car with respect to the grid
        self.pixel_pos = Vector2(self.grid.grid_to_pixel([self.grid_x, self.grid_y])[0],
                                 self.grid.grid_to_pixel([self.grid_x, self.grid_y])[1])
        # self.pixel_pos = Vector2(grid.grid_to_pixel([1,1])[0],grid.grid_to_pixel([1,1])[1])
        self.angle = angle
        self.car_image = car_image
        self.car_rect = pygame.Rect(self.pixel_pos[0] - (0.5 * self.screen_width),
                                    self.pixel_pos[1] - (0.5 * self.screen_height),
                                    self.screen_width, self.screen_height)

        self.speed = 10
        self.velocity = Vector2(0.0, 0.0)
        self.steering = 0.0

        self.optimized_target_locations = None

    def get_pixel_pos(self):
        return self.pixel_pos

    def get_grid_pos(self):
        return (self.grid_x, self.grid_y)

    def get_angle_of_rotation(self):
        return self.angle

    def get_robot_pose(self) -> Pose:
        return Pose([self.grid_x, self.grid_y, self.angle])

    def draw_car(self):
        if constants.HEADLESS:
            return
        rotated = pygame.transform.rotate(self.car_image, self.angle)
        rect = rotated.get_rect()
        rect.center = self.car_rect.center
        self.screen.blit(rotated, rect)
        pygame.draw.rect(self.screen, constants.RED, self.car_rect, 1)

        # Refresh screen by frame rate
        now = pygame.time.get_ticks() / 1000
        if now - self.simulator.startTime > 1 / constants.FPS:
            self.simulator.startTime = now
            self.simulator.root.display.flip()

    def redraw_car(self):
        if constants.HEADLESS:
            return
        # Need to redraw over everything (grid_surface, grid and car)
        self.simulator.reprint_screen_and_buttons()
        self.simulator.redraw_grid()
        self.draw_car()

    def check_movement_complete(self, final_pixel_pos):
        if constants.IS_CHECKING:
            return True
        return abs(self.get_pixel_pos()[0] - final_pixel_pos[0]) < 4 and abs(
            self.get_pixel_pos()[1] - final_pixel_pos[1]) < 4

    def perform_move(self, move: RobotMovement):
        if move == RobotMovement.FORWARD:
            self.move_forward()
        elif move == RobotMovement.BACKWARD:
            self.move_backward()
        elif move == RobotMovement.FORWARD_RIGHT:
            self.move_forward_steer_right()
        elif move == RobotMovement.FORWARD_LEFT:
            self.move_forward_steer_left()
        elif move == RobotMovement.BACKWARD_RIGHT:
            self.move_backward_steer_right()
        elif move == RobotMovement.BACKWARD_LEFT:
            self.move_backward_steer_left()
        else:
            raise NotImplementedError(f"Robot is not implemented to perform {move} yet")

    # TODO: define possible movements (for turning motions picture steering wheel direction)
    # ALL MOTIONS take place in minimal unit.
    # for 1: is by 10 (one grid)
    # for 2-5: is 30 by 30 (3x3 grid) area plus rotation
    # 1. straight (one grid) - forward, backwards
    # 2. forward right/clockwise pi/2 turn
    # 3. forward left/anticlockwise pi/2 turn
    # 4. backward right/anticlockwise pi/2 turn
    # 5. backward left/clockwise pi/2 turn
    def move_forward(self):
        # print("MOVE FORWARD FACING", self.angle)
        initial_pixel_pos = self.get_pixel_pos()
        # Set position to stop moving
        if self.angle == constants.NORTH:  # CAR FACING NORTH
            final_pixel_pos = Vector2(initial_pixel_pos[0], initial_pixel_pos[1] - ONE_CELL)
        elif self.angle == constants.SOUTH:  # CAR FACING SOUTH
            final_pixel_pos = Vector2(initial_pixel_pos[0], initial_pixel_pos[1] + ONE_CELL)
        elif self.angle == constants.EAST:  # CAR FACING EAST
            final_pixel_pos = Vector2(initial_pixel_pos[0] + ONE_CELL, initial_pixel_pos[1])
        elif self.angle == constants.WEST:  # CAR FACING WEST
            final_pixel_pos = Vector2(initial_pixel_pos[0] - ONE_CELL, initial_pixel_pos[1])
        else:
            final_pixel_pos = initial_pixel_pos  # car will not move
        final_angle = self.angle
        final_grid_pos = self.grid.pixel_to_grid(final_pixel_pos)

        # Set velocity of car
        self.velocity += (0, -self.speed)
        while not self.check_movement_complete(final_pixel_pos):
            if constants.HEADLESS:
                break
            self.pixel_pos += self.velocity.rotate(-self.angle) * dt
            self.car_rect = pygame.Rect(self.pixel_pos[0] - (0.5 * self.screen_width),
                                        self.pixel_pos[1] - (0.5 * self.screen_height),
                                        self.screen_width, self.screen_height)
            self.redraw_car()

        # Reset velocity to 0
        self.velocity -= (0, -self.speed)
        self.correct_coords_and_angle(final_angle, final_pixel_pos)

        self.check_if_target_reached(final_pixel_pos, final_angle)

    def move_backward(self):
        # print("MOVE BACKWARD FACING", self.angle)
        initial_pixel_pos = self.get_pixel_pos()
        # Set position to stop moving
        if self.angle == constants.NORTH:  # CAR FACING NORTH
            final_pixel_pos = Vector2(initial_pixel_pos[0], initial_pixel_pos[1] + ONE_CELL)
        elif self.angle == constants.SOUTH:  # CAR FACING SOUTH
            final_pixel_pos = Vector2(initial_pixel_pos[0], initial_pixel_pos[1] - ONE_CELL)
        elif self.angle == constants.EAST:  # CAR FACING EAST
            final_pixel_pos = Vector2(initial_pixel_pos[0] - ONE_CELL, initial_pixel_pos[1])
        elif self.angle == constants.WEST:  # CAR FACING WEST
            final_pixel_pos = Vector2(initial_pixel_pos[0] + ONE_CELL, initial_pixel_pos[1])
        else:
            final_pixel_pos = initial_pixel_pos  # car will not move
        final_angle = self.angle
        final_grid_pos = self.grid.pixel_to_grid(final_pixel_pos)

        # Set velocity of car
        self.velocity += (0, self.speed)
        while not self.check_movement_complete(final_pixel_pos):
            if constants.HEADLESS:
                break
            self.pixel_pos += self.velocity.rotate(-self.angle) * dt
            self.car_rect = pygame.Rect(self.pixel_pos[0] - (0.5 * self.screen_width),
                                        self.pixel_pos[1] - (0.5 * self.screen_height),
                                        self.screen_width, self.screen_height)
            self.redraw_car()
        # Reset velocity to 0
        self.velocity -= (0, self.speed)
        self.correct_coords_and_angle(final_angle, final_pixel_pos)

        self.check_if_target_reached(final_pixel_pos, final_angle)
        return True

    def move_forward_steer_right(self):
        # print("STEERING RIGHT FORWARD FACING", self.angle)
        # Pause to simulate time taken for wheels to full rotate
        # time.sleep(constants.STEERING_TIME_DELAY)

        initial_pixel_pos = self.get_pixel_pos()
        initial_angle = self.angle
        # Set position to stop moving
        if self.angle == constants.NORTH:  # CAR FACING NORTH
            final_pixel_pos = Vector2(initial_pixel_pos[0] + THREE_CELL, initial_pixel_pos[1] - THREE_CELL)
            final_angle = constants.EAST
        elif self.angle == constants.SOUTH:  # CAR FACING SOUTH
            final_pixel_pos = Vector2(initial_pixel_pos[0] - THREE_CELL, initial_pixel_pos[1] + THREE_CELL)
            final_angle = constants.WEST
        elif self.angle == constants.EAST:  # CAR FACING EAST
            final_pixel_pos = Vector2(initial_pixel_pos[0] + THREE_CELL, initial_pixel_pos[1] + THREE_CELL)
            final_angle = constants.SOUTH
        elif self.angle == constants.WEST:  # CAR FACING WEST
            final_pixel_pos = Vector2(initial_pixel_pos[0] - THREE_CELL, initial_pixel_pos[1] - THREE_CELL)
            final_angle = constants.NORTH
        else:
            final_pixel_pos = initial_pixel_pos  # car will not move
            final_angle = initial_angle
        final_grid_pos = self.grid.pixel_to_grid(final_pixel_pos)

        # Set velocity of car
        self.velocity += (0, -self.speed)
        while not self.check_if_turned(initial_angle, final_pixel_pos):
            if constants.HEADLESS:
                break
            turning_radius = THREE_CELL
            angular_velocity = self.velocity.y / turning_radius

            self.pixel_pos += self.velocity.rotate(-self.angle) * dt
            self.angle += degrees(angular_velocity) * dt

            self.car_rect = pygame.Rect(self.pixel_pos[0] - (0.5 * self.screen_width),
                                        self.pixel_pos[1] - (0.5 * self.screen_height),
                                        self.screen_width, self.screen_height)
            self.redraw_car()

        # Reset velocity to 0 and do corrections for angle and coordinates
        self.velocity -= (0, -self.speed)
        self.correct_coords_and_angle(final_angle, final_pixel_pos)

        self.check_if_target_reached(final_pixel_pos, final_angle)
        return True

    def move_forward_steer_left(self):
        initial_pixel_pos = self.get_pixel_pos()
        initial_angle = self.angle
        # Set position to stop moving
        if self.angle == constants.NORTH:  # CAR FACING NORTH
            final_pixel_pos = Vector2(initial_pixel_pos[0] - THREE_CELL, initial_pixel_pos[1] - THREE_CELL)
            final_angle = constants.WEST
        elif self.angle == constants.SOUTH:  # CAR FACING SOUTH
            final_pixel_pos = Vector2(initial_pixel_pos[0] + THREE_CELL, initial_pixel_pos[1] + THREE_CELL)
            final_angle = constants.EAST
        elif self.angle == constants.EAST:  # CAR FACING EAST
            final_pixel_pos = Vector2(initial_pixel_pos[0] + THREE_CELL, initial_pixel_pos[1] - THREE_CELL)
            final_angle = constants.NORTH
        elif self.angle == constants.WEST:  # CAR FACING WEST
            final_pixel_pos = Vector2(initial_pixel_pos[0] - THREE_CELL, initial_pixel_pos[1] + THREE_CELL)
            final_angle = constants.SOUTH
        else:
            final_pixel_pos = initial_pixel_pos  # car will not move
            final_angle = initial_angle
        final_grid_pos = self.grid.pixel_to_grid(final_pixel_pos)

        # Set velocity of car
        self.velocity += (0, -self.speed)
        while not self.check_if_turned(initial_angle, final_pixel_pos):
            if constants.HEADLESS:
                break
            turning_radius = THREE_CELL
            angular_velocity = self.velocity.y / turning_radius

            self.pixel_pos += self.velocity.rotate(-self.angle) * dt
            self.angle -= degrees(angular_velocity) * dt

            self.car_rect = pygame.Rect(self.pixel_pos[0] - (0.5 * self.screen_width),
                                        self.pixel_pos[1] - (0.5 * self.screen_height),
                                        self.screen_width, self.screen_height)
            self.redraw_car()

        # Reset velocity to 0 and do corrections for angle and coordinates
        self.velocity -= (0, -self.speed)
        self.correct_coords_and_angle(final_angle, final_pixel_pos)

        self.check_if_target_reached(final_pixel_pos, final_angle)
        return True

    def move_backward_steer_right(self):
        # print("STEERING RIGHT BACKWARD FACING", self.angle)
        # Pause to simulate time taken for wheels to full rotate
        # time.sleep(constants.STEERING_TIME_DELAY)

        initial_pixel_pos = self.get_pixel_pos()
        initial_angle = self.angle
        # Set position to stop moving
        if self.angle == constants.NORTH:  # CAR FACING NORTH
            final_pixel_pos = Vector2(initial_pixel_pos[0] + THREE_CELL, initial_pixel_pos[1] + THREE_CELL)
            final_angle = constants.WEST
        elif self.angle == constants.SOUTH:  # CAR FACING SOUTH
            final_pixel_pos = Vector2(initial_pixel_pos[0] - THREE_CELL, initial_pixel_pos[1] - THREE_CELL)
            final_angle = constants.EAST
        elif self.angle == constants.EAST:  # CAR FACING EAST
            final_pixel_pos = Vector2(initial_pixel_pos[0] - THREE_CELL, initial_pixel_pos[1] + THREE_CELL)
            final_angle = constants.NORTH
        elif self.angle == constants.WEST:  # CAR FACING WEST
            final_pixel_pos = Vector2(initial_pixel_pos[0] + THREE_CELL, initial_pixel_pos[1] - THREE_CELL)
            final_angle = constants.SOUTH
        else:
            final_pixel_pos = initial_pixel_pos  # car will not move
            final_angle = initial_angle
        final_grid_pos = self.grid.pixel_to_grid(final_pixel_pos)

        # Set velocity of car
        self.velocity += (0, -self.speed)
        while not self.check_if_turned(initial_angle, final_pixel_pos):
            if constants.HEADLESS:
                break
            turning_radius = THREE_CELL
            angular_velocity = self.velocity.y / turning_radius

            self.pixel_pos -= self.velocity.rotate(-self.angle) * dt
            self.angle -= degrees(angular_velocity) * dt

            self.car_rect = pygame.Rect(self.pixel_pos[0] - (0.5 * self.screen_width),
                                        self.pixel_pos[1] - (0.5 * self.screen_height),
                                        self.screen_width, self.screen_height)
            self.redraw_car()

        # Reset velocity to 0 and do corrections for angle and coordinates
        self.velocity -= (0, -self.speed)
        self.correct_coords_and_angle(final_angle, final_pixel_pos)

        self.check_if_target_reached(final_pixel_pos, final_angle)
        return True

    def move_backward_steer_left(self):
        # print("STEERING LEFT BACKWARD FACING", self.angle)

        initial_pixel_pos = self.get_pixel_pos()
        initial_angle = self.angle
        # Set position to stop moving
        if self.angle == constants.NORTH:  # CAR FACING NORTH
            final_pixel_pos = Vector2(initial_pixel_pos[0] - THREE_CELL, initial_pixel_pos[1] + THREE_CELL)
            final_angle = constants.EAST
        elif self.angle == constants.SOUTH:  # CAR FACING SOUTH
            final_pixel_pos = Vector2(initial_pixel_pos[0] + THREE_CELL, initial_pixel_pos[1] - THREE_CELL)
            final_angle = constants.WEST
        elif self.angle == constants.EAST:  # CAR FACING EAST
            final_pixel_pos = Vector2(initial_pixel_pos[0] - THREE_CELL, initial_pixel_pos[1] - THREE_CELL)
            final_angle = constants.SOUTH
        elif self.angle == constants.WEST:  # CAR FACING WEST
            final_pixel_pos = Vector2(initial_pixel_pos[0] + THREE_CELL, initial_pixel_pos[1] + THREE_CELL)
            final_angle = constants.NORTH
        else:
            final_pixel_pos = initial_pixel_pos  # car will not move
            final_angle = initial_angle
        final_grid_pos = self.grid.pixel_to_grid(final_pixel_pos)

        # Set velocity of car
        self.velocity += (0, -self.speed)
        while not self.check_if_turned(initial_angle, final_pixel_pos):
            if constants.HEADLESS:
                break
            turning_radius = THREE_CELL
            angular_velocity = self.velocity.y / turning_radius

            self.pixel_pos -= self.velocity.rotate(-self.angle) * dt
            self.angle += degrees(angular_velocity) * dt

            self.car_rect = pygame.Rect(self.pixel_pos[0] - (0.5 * self.screen_width),
                                        self.pixel_pos[1] - (0.5 * self.screen_height),
                                        self.screen_width, self.screen_height)
            self.redraw_car()

        # Reset velocity to 0 and do corrections for angle and coordinates
        self.velocity -= (0, -self.speed)
        self.correct_coords_and_angle(final_angle, final_pixel_pos)

        self.check_if_target_reached(final_pixel_pos, final_angle)
        return True

    def update_robot(self, arglist):
        final_angle = arglist[0]
        final_pixel_pos = arglist[1]
        self.angle = final_angle
        self.pixel_pos = final_pixel_pos
        self.grid_x, self.grid_y = self.grid.pixel_to_grid(self.pixel_pos)
        self.car_rect = pygame.Rect(self.pixel_pos[0] - (0.5 * self.screen_width),
                                    self.pixel_pos[1] - (0.5 * self.screen_height),
                                    self.screen_width, self.screen_height)

    def correct_coords_and_angle(self, final_angle, final_pixel_pos):
        self.update_robot([final_angle, final_pixel_pos])
        if constants.IS_CHECKING:
            return
        self.redraw_car()

    def check_if_turned(self, initial_angle, final_pixel_pos):
        if constants.IS_CHECKING:
            return True
        # Set position to stop moving
        return self.check_movement_complete(final_pixel_pos) and abs(self.angle - initial_angle) > 80

    # TODO: for now, it is strictly right in front of the image, 4 grids away (counting from the centre of car)
    def check_if_target_reached(self, final_pixel_pos, final_angle):
        target_locations = self.optimized_target_locations
        for target_loc in target_locations:
            target_grid_x = target_loc[0]
            target_grid_y = target_loc[1]
            target_direction = target_loc[2]

            final_grid_pos = self.grid.pixel_to_grid(final_pixel_pos)
            final_grid_x = final_grid_pos[0]
            final_grid_y = final_grid_pos[1]
            # print("Checking for obstacles visited:", target_loc)

            if not constants.IS_CHECKING:
                # Check if in target grid
                if (final_grid_x == target_grid_x) and (final_grid_y == target_grid_y) and (
                        final_angle == target_direction):

                    # Repaint grid and car
                    self.redraw_car()
                    print("--Obstacle was visited!")

    def reset(self):
        self.angle = constants.ROBOT_STARTING_ANGLE
        self.grid_x = constants.ROBOT_STARTING_X
        self.grid_y = constants.ROBOT_STARTING_Y
        self.pixel_pos = Vector2(self.grid.grid_to_pixel([self.grid_x, self.grid_y])[0],
                                 self.grid.grid_to_pixel([self.grid_x, self.grid_y])[1])
        self.car_rect = pygame.Rect(self.pixel_pos[0] - (0.5 * self.screen_width),
                                    self.pixel_pos[1] - (0.5 * self.screen_height),
                                    self.screen_width, self.screen_height)
        self.redraw_car()


