"""
Explanation on coordinate system:
    * the display screen and also (row, column) system use a coordinate system with
        + origin: top left
        + x-axis: pointing right
        + y-axis: pointing down
    * the grid object uses a coordinate system with
        + origin: bottom left
        + x-axis: pointing right
        + y-axis: pointing up
"""


import logging

from mdpalgo import constants
import pygame
from mdpalgo.map.cell import Cell, CellStatus
import numpy as np

# This sets the margin between each Cell
MARGIN = 2

# This is the margin around the top and left of the grid on screen display
OUTER_MARGIN = 120

COLOR_DICT = {
    CellStatus.EMPTY: constants.WHITE,
    CellStatus.START: constants.BLUE,
    CellStatus.BOUNDARY: constants.LIGHT_RED,
    CellStatus.OBS: constants.BLACK,
    CellStatus.VISITED_OBS: constants.GREEN,
    CellStatus.PATH: constants.GRAY,
}

class Grid(object):

    def __init__(self, grid_column: int, grid_row: int, block_size):
        self.size_x = grid_column
        self.size_y = grid_row
        self.max_x = self.size_x - 1
        self.max_y = self.size_y - 1
        self.start_zone_size = 4
        self.block_size = block_size # size in cm of 1 square cell
        self.cells = np.empty((self.size_y, self.size_x), dtype=Cell)
        self.initialize_cells()
        self.optimized_target_locations = None
        self.obstacle_cells = {}
        self.reset_data()
        self.obstacle_statuses = [CellStatus.OBS, CellStatus.VISITED_OBS]

    def initialize_cells(self):
        for x in range(self.size_x):
            for y in range(self.size_y):
                self.cells[x][y] = Cell(x, y)

        self.set_start_zone()

    def set_start_zone(self):
        for x in range(self.start_zone_size):
            for y in range(self.start_zone_size):
                self.cells[x][y].set_starting_area_status()


    def reset_data(self):
        """Reset data in all grid cells"""
        self.obstacle_cells.clear()
        for x in range(self.size_x):
            for y in range(self.size_y):
                self.cells[x][y].remove_obstacle()
                self.cells[x][y].set_empty_status()

        self.set_start_zone()

    def reset(self, screen):
        """Reset both grid data and reflect this on the UI"""
        self.reset_data()
        self.update_grid(screen)

    def get_block_size(self):
        return self.block_size

    def get_cells(self):
        return self.cells

    def get_cell_by_row_column(self, row, column) -> Cell:
        x, y = self.get_xy_from_row_column(row, column)
        return self.cells[x][y]

    def get_xy_from_row_column(self, row, column) -> tuple:
        x = column
        y = self.max_y - row
        return (x, y)

    def get_row_column_from_xy(self, x, y) -> tuple:
        column = x
        row = self.max_y - y
        return (row, column)

    def get_cell_by_xycoords(self, x, y) -> Cell:
        return self.cells[x][y]

    def get_obstacle_cells(self):
        return self.obstacle_cells

    def get_target_locations(self):
        target_locations = []
        for obstacle_cell in self.obstacle_cells.values():
            # Get target grid positions and NSEW direction that car's centre has to reach for image rec
            target_grid_x, target_grid_y = obstacle_cell.get_xcoord(), obstacle_cell.get_ycoord()
            obstacle_direction = obstacle_cell.get_obstacle_direction()
            target_direction = constants.NORTH
            if obstacle_direction == constants.NORTH:
                target_direction = constants.SOUTH
                target_grid_x, target_grid_y = obstacle_cell.get_xcoord(), obstacle_cell.get_ycoord() + 4
            elif obstacle_direction == constants.SOUTH:
                target_direction = constants.NORTH
                target_grid_x, target_grid_y = obstacle_cell.get_xcoord(), obstacle_cell.get_ycoord() - 4
            elif obstacle_direction == constants.EAST:
                target_direction = constants.WEST
                target_grid_x, target_grid_y = obstacle_cell.get_xcoord() + 4, obstacle_cell.get_ycoord()
            elif obstacle_direction == constants.WEST:
                target_direction = constants.EAST
                target_grid_x, target_grid_y = obstacle_cell.get_xcoord() - 4, obstacle_cell.get_ycoord()

            target_loc = (target_grid_x, target_grid_y, target_direction, obstacle_cell)
            target_locations.append(target_loc)

        return target_locations

    def get_optimized_target_locations(self, fastest_path):
        optimized_fastest_path = fastest_path
        i = 1
        previous_target = fastest_path[0]
        # If there are no obstacles
        if len(optimized_fastest_path) <= 1:
            return optimized_fastest_path
        for target in fastest_path[1:]:
            target_x = target[0]
            target_y = target[1]

            neighbour_left, neighbour_right = self.get_potential_target_cells(target)

            # Calculate manhattan dists
            left_dist = abs(previous_target[0] - neighbour_left[0]) + abs(previous_target[1] - neighbour_left[1])
            right_dist = abs(previous_target[0] - neighbour_right[0]) + abs(previous_target[1] - neighbour_right[1])
            centre_dist = abs(previous_target[0] - target_x) + abs(previous_target[1] - target_y)
            if constants.CENTER_ON_OBS:
                new_optimized_target = target
            else:
                if centre_dist <= left_dist and centre_dist <= right_dist:
                    new_optimized_target = target
                elif left_dist < centre_dist and left_dist < right_dist:
                    new_optimized_target = neighbour_left
                elif right_dist < centre_dist and right_dist < left_dist:
                    new_optimized_target = neighbour_right
                else:
                    new_optimized_target = target

            # Change the optimized target
            optimized_fastest_path[i] = new_optimized_target
            previous_target = new_optimized_target
            i += 1
        return optimized_fastest_path

    def get_potential_target_cells(self, target):
        target_x = target[0]
        target_y = target[1]
        target_direction = target[2]
        obstacle_cell = target[3]

        # Get the 2 other neighbour potential target cells
        if target_direction == constants.NORTH:
            neighbour_left = (target_x - 1, target_y, target_direction, obstacle_cell)
            neighbour_right = (target_x + 1, target_y, target_direction, obstacle_cell)
        elif target_direction == constants.SOUTH:
            neighbour_left = (target_x + 1, target_y, target_direction, obstacle_cell)
            neighbour_right = (target_x - 1, target_y, target_direction, obstacle_cell)
        elif target_direction == constants.EAST:
            neighbour_left = (target_x, target_y + 1, target_direction, obstacle_cell)
            neighbour_right = (target_x, target_y - 1, target_direction, obstacle_cell)
        elif target_direction == constants.WEST:
            neighbour_left = (target_x, target_y - 1, target_direction, obstacle_cell)
            neighbour_right = (target_x, target_y + 1, target_direction, obstacle_cell)
        return neighbour_left, neighbour_right

    def create_obstacle(self, arglist):
        grid_x, grid_y, dir = arglist[0], arglist[1], arglist[2]
        # Set that location to one
        cell = self.get_cell_by_xycoords(grid_x, grid_y)
        cell.create_obstacle(dir)

        self.add_cell_to_obstacle_list(cell)
        self.set_obstacle_boundary_cells_around(cell)

    def get_virtual_map(self):
        # Get virtual map contains just the cell status
        def get_cell_status(cell: Cell):
            return cell.get_cell_status()

        return np.vectorize(get_cell_status)(self.cells)

    def is_obstacle_status(self, cell_status: CellStatus):
        return cell_status in self.obstacle_statuses

    def get_obstacle_key_from_xy_coords(self, x, y) -> str:
        return f"{x}-{y}"

    def remove_cell_from_obstacle_list(self, cell: Cell):
        obs_key_to_remove = self.get_obstacle_key_from_xy_coords(cell.get_xcoord(), cell.get_ycoord())
        self.obstacle_cells.pop(obs_key_to_remove)

    def add_cell_to_obstacle_list(self, cell: Cell):
        self.obstacle_cells[cell.get_obstacle().get_obstacle_id()] = cell

    def grid_clicked(self, pixel_x, pixel_y):
        # Change the x/y screen coordinates to grid coordinates
        x_grid, y_grid = self.pixel_to_grid((pixel_x, pixel_y))

        selected_cell = self.get_cell_by_xycoords(x_grid, y_grid)
        previous_status = selected_cell.get_cell_status()
        selected_cell.cell_clicked()
        current_status = selected_cell.get_cell_status()

        logging.info("Clicked (x,y): (" + str(pixel_x) + "," + str(pixel_y) + "); Grid coordinates: " + str(selected_cell.get_xcoord()) + " " + str(selected_cell.get_ycoord())
                     + "; Direction: " + str(selected_cell.get_obstacle_direction()))

        if self.is_obstacle_status(previous_status) and self.is_obstacle_status(current_status):
            return

        elif self.is_obstacle_status(previous_status) and not self.is_obstacle_status(current_status):
            self.remove_cell_from_obstacle_list(selected_cell)
            self.unset_obstacle_boundary_cells(selected_cell)
            for remaining_obstacle_cell in self.obstacle_cells.values():
                self.set_obstacle_boundary_cells_around(remaining_obstacle_cell)

        elif not self.is_obstacle_status(previous_status) and self.is_obstacle_status(current_status):
            self.add_cell_to_obstacle_list(selected_cell)
            self.set_obstacle_boundary_cells_around(selected_cell)

        else:
            raise Exception("Cell status does not update properly. On click, cell does not become obstacle or get removed from being an obstacle.")

    def get_boundary_cells_coords(self, cell: Cell):
        """Return a list of coordinates [x_coord, y_coord] of the cells
        surrounding a given cell, within a 2 cell radius (1 for diagonal)."""
        x, y = cell.get_xcoord(), cell.get_ycoord()
        boundary_cells = [
                            [x - 1, y + 2], [x, y + 2], [x + 1, y + 2],
            [x - 2, y + 1], [x - 1, y + 1], [x, y + 1], [x + 1, y + 1], [x + 2, y + 1],
            [x - 2, y    ], [x - 1, y    ],             [x + 1, y    ], [x + 2, y    ],
            [x - 2, y - 1], [x - 1, y - 1], [x, y - 1], [x + 1, y - 1], [x + 2, y - 1],
                            [x - 1, y - 2], [x, y - 2], [x + 1, y - 2],
        ]
        return boundary_cells

    def set_obstacle_boundary_cells_around(self, obstacle_cell: Cell):
        """For an obstacle cell, set the statuses of the cells around it to
        boundary status"""
        if not self.is_obstacle_status(obstacle_cell.get_cell_status()):
            raise Exception("Attempt to set boundary around a non-obstacle cell.")

        boundary_cells = self.get_boundary_cells_coords(obstacle_cell)
        # Set status of cells around obstacle as boundary
        for x, y in boundary_cells:
            if not self.is_xy_coords_within_grid(x, y):
                continue

            boundary_cell = self.get_cell_by_xycoords(x, y)
            if not self.is_obstacle_status(boundary_cell.get_cell_status()):
                boundary_cell.set_obstacle_boundary_status()

    def is_x_coord_within_grid(self, x):
        return 0 <= x <= self.max_x
    def is_y_coord_within_grid(self, y):
        return 0 <= y <= self.max_y
    def is_xy_coords_within_grid(self, x, y):
        return self.is_x_coord_within_grid(x) and self.is_y_coord_within_grid(y)

    def unset_obstacle_boundary_cells(self, removed_obstacle_cell):
        """When an obstacle cell is removed, unset the statuses of boundary
        cells around it.

        Note that this function does not take into account if these boundary
        cells also belong to another obstacle. The best thing to do after unset
        all the boundary of removed obstacle cells is to set boundary for all
        the remaining obstacle cells."""
        if self.is_obstacle_status(removed_obstacle_cell):
            raise Exception("Attempted to remove boundary cells from an existing obstacle. Remove the obstacle before doing this!")

        boundary_cells = self.get_boundary_cells_coords(removed_obstacle_cell)
        # Unset status of cells around obstacle
        for x, y in boundary_cells:
            if not self.is_xy_coords_within_grid(x, y):
                continue

            removed_obstacle_cell = self.get_cell_by_xycoords(x, y)
            if removed_obstacle_cell.get_cell_status() == CellStatus.BOUNDARY:
                removed_obstacle_cell.set_empty_status()

    def set_obstacle_as_visited(self, obstacle_cell):
        obstacle_cell.set_obstacle_visited_status()

    def update_grid(self, screen):
        if constants.HEADLESS:
            return
        # Draw the grid
        for row in range(20):
            for column in range(20):
                cell = self.get_cell_by_row_column(row, column)
                color = COLOR_DICT[cell.get_cell_status()]
                pygame.draw.rect(screen,
                                 color,
                                 [OUTER_MARGIN + (MARGIN + self.block_size) * column + MARGIN,
                                  OUTER_MARGIN + (MARGIN + self.block_size) * row + MARGIN,
                                  self.block_size,
                                  self.block_size])

                obstacle_direction = cell.get_obstacle_direction()
                if obstacle_direction is None:
                    continue
                elif obstacle_direction == constants.NORTH:
                    pygame.draw.rect(screen,
                                     constants.RED,
                                     [OUTER_MARGIN + (MARGIN + self.block_size) * column + MARGIN,
                                      OUTER_MARGIN + (MARGIN + self.block_size) * row + MARGIN,
                                      self.block_size,
                                      # 2
                                      8  # change width of obstacle image line
                                      ])
                elif obstacle_direction == constants.EAST:
                    pygame.draw.rect(screen,
                                     constants.RED,
                                     [
                                         # OUTER_MARGIN + (MARGIN + self.block_size) * column + MARGIN + 18,
                                         # change starting point of obstacle image line
                                         OUTER_MARGIN + (MARGIN + self.block_size) * column + MARGIN + 13,
                                         OUTER_MARGIN + (MARGIN + self.block_size) * row + MARGIN,
                                         # 2,
                                         8,  # change width of obstacle image line
                                         self.block_size])
                elif obstacle_direction == constants.SOUTH:
                    pygame.draw.rect(screen,
                                     constants.RED,
                                     [OUTER_MARGIN + (MARGIN + self.block_size) * column + MARGIN,
                                      # OUTER_MARGIN + (MARGIN + self.block_size) * row + MARGIN + 18,
                                      # change starting point of obstacle image line
                                      OUTER_MARGIN + (MARGIN + self.block_size) * row + MARGIN + 13,
                                      self.block_size,
                                      # 2
                                      8  # change width of obstacle image line
                                      ])
                elif obstacle_direction == constants.WEST:
                    pygame.draw.rect(screen,
                                     constants.RED,
                                     [OUTER_MARGIN + (MARGIN + self.block_size) * column + MARGIN,
                                      OUTER_MARGIN + (MARGIN + self.block_size) * row + MARGIN,
                                      # 2,
                                      8,  # change width of obstacle image line
                                      self.block_size])

    def grid_to_pixel(self, pos):
        x_pixel = (pos[0]) * (self.block_size + MARGIN) + 120 + (self.block_size + MARGIN) / 2
        y_pixel = (19 - pos[1]) * (self.block_size + MARGIN) + 120 + (self.block_size + MARGIN) / 2
        return [x_pixel, y_pixel]

    def pixel_to_grid(self, pos):
        x_grid = (pos[0] - 120) // (self.block_size + MARGIN)
        y_grid = 19 - ((pos[1] - 120) // (self.block_size + MARGIN))
        return [x_grid, y_grid]
