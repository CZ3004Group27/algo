import logging

from mdpalgo import constants
import pygame
from mdpalgo.map.cell import Cell, CellStatus

# This sets the margin between each Cell
MARGIN = 2

# This is the margin around the top and left of the grids
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

    def __init__(self, grid_column, grid_row, block_size):
        self.grid_column = grid_column
        self.grid_row = grid_row
        self.block_size = block_size
        self.cells = [[0 for x in range(grid_column)] for y in range(grid_row)]
        self.cells_virtual = [[0 for x in range(grid_column)] for y in range(grid_row)]
        self.optimized_target_locations = None
        self.obstacle_cells = {}
        self.reset_data()

    def reset_data(self):
        """Reset the grid data"""
        # NOTE!! row and columns start from the top right, but coordinates have to start from the bottom left
        # x-coord = column; y-coord = 19-row
        self.obstacle_cells = {}
        for row in range(self.grid_row):
            for column in range(self.grid_column):
                if column < 4 and row > 15:
                    self.cells[row][column] = Cell(column, (19 - row), CellStatus.START)  # 19 is to correct the positive direction
                else:
                    self.cells[row][column] = Cell(column, (19 - row), CellStatus.EMPTY)

    def reset(self, screen):
        """Reset both grid data and reflect this on the UI"""
        self.reset_data()
        self.update_grid(screen)

    def get_block_size(self):
        return self.block_size

    def get_cells(self):
        return self.cells

    def get_cell(self, row, column) -> Cell:
        return self.cells[row][column]

    def get_cell_by_xycoords(self, x, y) -> Cell:
        column = x
        row = 19 - y
        return self.cells[row][column]

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

        # Add/remove cell from dict of obstacles accordingly
        if cell.get_cell_status() == CellStatus.OBS:
            if cell.get_obstacle().get_obstacle_id() not in self.obstacle_cells.keys():
                self.obstacle_cells[cell.get_obstacle().get_obstacle_id()] = cell  # '1-12': cell()
        for r in range(self.grid_row):
            for c in range(self.grid_column):
                a = self.get_cell(r, c)
                self.set_obstacle_boundary_cells(a)  # runs only for obstacle cell

        # Update virtual map for path planning
        for r in range(20):
            for c in range(20):
                cell_status = self.cells[r][c].get_cell_status()
                self.cells_virtual[r][c] = cell_status

    def grid_clicked(self, x_coordinate, y_coordinate):
        # Change the x/y screen coordinates to grid coordinates
        column = (x_coordinate - 120) // (self.block_size + MARGIN)
        row = (y_coordinate - 120) // (self.block_size + MARGIN)

        # Set that location to one
        cell = self.get_cell(row, column)
        cell.cell_clicked()
        # Add/remove cell from dict of obstacles accordingly
        if cell.get_cell_status() == CellStatus.OBS:
            if cell.get_obstacle().get_obstacle_id() not in self.obstacle_cells:
                self.obstacle_cells[cell.get_obstacle().get_obstacle_id()] = cell  # '1-12': cell()
        elif cell.get_cell_status() == CellStatus.EMPTY:
            key_to_remove = str(cell.get_xcoord()) + '-' + str(cell.get_ycoord())  # '1-12'
            # remove the key if it exists, else does nothing
            self.obstacle_cells.pop(key_to_remove, None)
        self.unset_obstacle_boundary_cells(cell)  # runs only for empty cell
        for r in range(self.grid_row):
            for c in range(self.grid_column):
                a = self.get_cell(r, c)
                self.set_obstacle_boundary_cells(a)  # runs only for obstacle cell

        # Update virtual map for path planning
        for r in range(20):
            for c in range(20):
                cell_status = self.cells[r][c].get_cell_status()
                self.cells_virtual[r][c] = cell_status

        logging.info("Clicked (x,y): (" + str(x_coordinate) + "," + str(y_coordinate) + "); column, row: " + str(column)
                     + "," + str(row) + "; Grid coordinates: " + str(cell.get_xcoord()) + " " + str(cell.get_ycoord())
                     + "; Direction: " + str(cell.get_obstacle_direction()))

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

    def set_obstacle_boundary_cells(self, cell: Cell):
        """For an obstacle cell, set the statuses of the cells around it to
        boundary status"""
        if cell.get_cell_status() not in [CellStatus.OBS, CellStatus.VISITED_OBS]:
            return

        boundary_cells = self.get_boundary_cells_coords(cell)
        # Set status of cells around obstacle as boundary
        for [x, y] in boundary_cells:
            if 0 <= x <= 19 and 0 <= y <= 19 and self.get_cell_by_xycoords(
                    x, y).get_cell_status() in [CellStatus.EMPTY, CellStatus.PATH]:

                self.get_cell_by_xycoords(x, y).set_obstacle_boundary_status()

    def unset_obstacle_boundary_cells(self, cell):
        """When an obstacle cell is removed, unset the statuses of boundary
        cells around it.

        Note that this function does not take into account if these boundary
        cells also belong to another obstacle. The best thing to do after unset
        all the boundary of removed obstacle cells is to set boundary for all
        the remaining obstacle cells."""
        if cell.get_cell_status()  != CellStatus.EMPTY:
            return

        boundary_cells = self.get_boundary_cells_coords(cell)
        # Unset status of cells around obstacle
        for [x, y] in boundary_cells:
            if 0 <= x <= 19 and 0 <= y <= 19 and \
                    self.get_cell_by_xycoords(x, y).get_cell_status() == CellStatus.BOUNDARY:
                self.get_cell_by_xycoords(x, y).set_empty_status()

    def set_obstacle_as_visited(self, obstacle_cell):
        obstacle_cell.set_obstacle_visited_status()

    def update_grid(self, screen):
        if constants.HEADLESS:
            return
        # Draw the grid
        for row in range(20):
            for column in range(20):
                cell = self.get_cell(row, column)
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
