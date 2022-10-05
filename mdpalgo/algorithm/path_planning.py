import logging
from mdpalgo.robot.robot import BorderException, CheckingException
from mdpalgo.robot.robot import ObstacleException
from mdpalgo.robot.robot import ObstacleTurnException
from mdpalgo.robot.robot import Robot
from mdpalgo.algorithm.fastest_path_planning import search
from mdpalgo import constants
from mdpalgo.map.configuration import Pose

MARGIN = 2


class PathPlan(object):

    def __init__(self, simulator, grid, robot: Robot, fastest_route):
        self.simulator = simulator
        self.grid = grid
        self.robot = robot
        self.fastest_route = fastest_route
        self.target_pose: Pose = Pose() # pose to take a good photo of image
        self.robot_pose: Pose = self.robot.get_robot_pose()
        self.obstacle_cell = None
        self.collection_of_movements = []
        self.collection_of_robot_pos = []
        self.all_movements_dict = {}
        self.all_robot_pos_dict = {}
        self.all_take_photo_dict = {}
        self.obstacle_list_rpi = []
        self.EXCEPTION_COUNT = 0
        self.REPEATED_LAST_TARGET = 0
        self.IS_ON_PATH = False
        self.skipped_obstacles = []
        self.obstacle_key = None # the current obstacle key that RPi is going for
        self.num_move_completed_rpi = 0 # completed num moves by RPi to self.obstacle_key
        self.total_num_move_required_rpi = 0 # total required num moves by RPi to self.obstacle_key

    def start_robot(self):
        # Remove robot starting position from fastest_route
        self.fastest_route.pop(0)

        count_of_obs = 0

        while len(self.fastest_route) != 0:
            count_of_obs += 1
            if count_of_obs == 2:
                if constants.RPI_CONNECTED:
                    self.send_to_rpi()

            target = self.fastest_route.pop(0)
            print("Current Target: ", target)

            self.target_pose.set_pose(target[:3])
            self.obstacle_cell = target[3]

            self.robot_pose = self.robot.get_robot_pose()

            # if no obstacle exceptions, can use hardcoded shortest path
            if self.check_movement_possible(self.target_pose.x, self.target_pose.y, self.robot_pose.x, self.robot_pose.y,
                                            self.robot_pose.direction, self.target_pose.direction):
                # Target Coordinates: (a, b); Robot Coordinates: (x, y)
                self.plan_trip_by_robot_target_directions(self.target_pose.x, self.target_pose.y, self.robot_pose.x, self.robot_pose.y,
                                                          self.robot_pose.direction, self.target_pose.direction)

            # else, plan a path using astar search on virtual grid
            else:
                # x-coord = column; y-coord = 19-row
                start = [19 - self.robot.grid_y, self.robot.grid_x, self.robot.angle]
                end = [19 - self.target_pose.y,
                       self.target_pose.x,
                       self.target_pose.direction]  # ending position
                cost = 10  # cost per movement
                maze = self.grid.cells_virtual
                search_result = search(maze, cost, start, end)

                if search_result is None:
                    # Skip this obstacle first
                    print("Search result: ", search_result, " ; Skipping obstacle...")
                    self.skip_current_target()
                else:
                    # Execute gray route
                    draw_path, path = search_result[0], search_result[1]

                    for r in range(20):
                        for c in range(20):
                            if draw_path[r][c] >= 5:
                                self.grid.cells[r][c].set_path_status(draw_path[r][c])
                    # Colour rough route gray
                    self.robot.redraw_car()

                    movements = self.translate_path_to_movements(path)
                    self.IS_ON_PATH = True
                    for move in movements:
                        self.do_move(move)
                    self.IS_ON_PATH = False

                    # Last step is to rotate on the spot
                    print("LAST STEP")
                    self.plan_trip_by_robot_target_directions(self.target_pose.x, self.target_pose.y, self.robot.grid_x,
                                                              self.robot.grid_y,
                                                              self.robot.angle, self.target_pose.direction)
        self.restart_robot()

    def skip_current_target(self):
        self.skipped_obstacles.append(self.target_pose)

    def restart_robot(self):
        if len(self.skipped_obstacles) == 0:
            print("No skipped obstacles to run")
            return

        while len(self.skipped_obstacles) != 0:

            target = self.skipped_obstacles.pop(0)
            print("Current Target: ", target)

            self.target_pose.set_pose(target[:3])
            self.obstacle_cell = target[3]

            self.robot_pose = self.robot.get_robot_pose()

            # if no obstacle exceptions, can use hardcoded shortest path
            if self.check_movement_possible(self.target_pose.x, self.target_pose.y, self.robot_pose.x, self.robot_pose.y,
                                            self.robot_pose.direction, self.target_pose.direction):
                # Target Coordinates: (a, b); Robot Coordinates: (x, y)
                self.plan_trip_by_robot_target_directions(self.target_pose.x, self.target_pose.y, self.robot_pose.x, self.robot_pose.y,
                                                          self.robot_pose.direction, self.target_pose.direction)

            # else, plan a path using astar search on virtual grid
            else:
                # x-coord = column; y-coord = 19-row
                start = [19 - self.robot.grid_y, self.robot.grid_x, self.robot.angle]
                end = [19 - self.target_pose.y,
                       self.target_pose.x,
                       self.target_pose.direction]  # ending position
                cost = 10  # cost per movement
                maze = self.grid.cells_virtual
                search_result = search(maze, cost, start, end)

                if search_result is None:
                    # Force run hardcoded path
                    print("Search result: ", search_result, " ; FORCING hardcoded path...")
                    self.plan_trip_by_robot_target_directions(self.target_pose.x, self.target_pose.y, self.robot_pose.x, self.robot_pose.y,
                                                              self.robot_pose.direction, self.target_pose.direction)
                else:
                    # Execute gray route
                    draw_path, path = search_result[0], search_result[1]

                    for r in range(20):
                        for c in range(20):
                            if draw_path[r][c] >= 5:
                                self.grid.cells[r][c].set_path_status(draw_path[r][c])
                    # Colour rough route gray
                    self.robot.redraw_car()

                    movements = self.translate_path_to_movements(path)
                    self.IS_ON_PATH = True
                    for move in movements:
                        self.do_move(move)
                    self.IS_ON_PATH = False

                    # Last step is to rotate on the spot
                    print("LAST STEP")
                    self.plan_trip_by_robot_target_directions(self.target_pose.x, self.target_pose.y, self.robot.grid_x,
                                                              self.robot.grid_y,
                                                              self.robot.angle, self.target_pose.direction)

    def do_move(self, move):
        if move == "F":
            self.move_forward_by(1)
        elif move == "B":
            self.move_backward_by(1)
        elif move == "FR":
            self.turn_forward_right()
        elif move == "FL":
            self.turn_forward_left()
        elif move == "BR":
            self.turn_backward_right()
        elif move == "BL":
            self.turn_backward_left()

    def translate_path_to_movements(self, path):
        no_of_steps = len(path)
        list_of_movements = []
        grid_path = path

        # Process robot starting node
        row, col, curr_dir = grid_path[0][0], grid_path[0][1], grid_path[0][2]
        path.insert(1, [col, 19 - row, grid_path[0][2]])
        path.pop(0)
        # Process the rest of the nodes
        for step in grid_path[1:]:
            # Convert to grid coordinates x-coord = col; y-coord = 19-row
            row, col, curr_dir = step[0], step[1], step[2]
            step[0] = col
            step[1] = 19 - row

        prev_dir = grid_path[0][2]
        prev_x = grid_path[0][0]
        prev_y = grid_path[0][1]
        prev_grid = [prev_x, prev_y]
        prev_prev_grid = prev_grid

        move = [[0, 1],  # go up
                [1, 0],  # go right
                [0, -1],  # go down
                [-1, 0]]  # go left

        print(grid_path)

        count = 0
        for step in grid_path[1:]:
            curr_dir = step[2]
            curr_grid = [step[0], step[1]]
            diff_grid = [prev_grid[0] - prev_prev_grid[0], prev_grid[1] - prev_prev_grid[1]]
            if prev_dir == constants.NORTH:
                if curr_dir == constants.EAST:
                    if diff_grid == move[2]:
                        self.backward_l(list_of_movements)
                    else:
                        self.forward_r(list_of_movements)
                elif curr_dir == constants.WEST:
                    if diff_grid == move[2]:
                        self.backward_r(list_of_movements)
                    else:
                        self.forward_l(list_of_movements)
                elif curr_dir == constants.SOUTH:
                    self.uturn(list_of_movements, curr_dir)
                elif curr_dir == prev_dir:
                    if prev_y > step[1]:
                        list_of_movements.append("B")
                    else:
                        list_of_movements.append("F")
            elif prev_dir == constants.SOUTH:
                if curr_dir == constants.WEST:
                    if diff_grid == move[0]:
                        self.backward_l(list_of_movements)
                    else:
                        self.forward_r(list_of_movements)
                elif curr_dir == constants.EAST:
                    if diff_grid == move[0]:
                        self.backward_r(list_of_movements)
                    else:
                        self.forward_l(list_of_movements)
                elif curr_dir == constants.NORTH:
                    self.uturn(list_of_movements, curr_dir)
                elif curr_dir == prev_dir:
                    if prev_y < step[1]:
                        list_of_movements.append("B")
                    else:
                        list_of_movements.append("F")
            elif prev_dir == constants.EAST:
                if curr_dir == constants.SOUTH:
                    if diff_grid == move[3]:
                        self.backward_l(list_of_movements)
                    else:
                        self.forward_r(list_of_movements)
                elif curr_dir == constants.NORTH:
                    if diff_grid == move[3]:
                        self.backward_r(list_of_movements)
                    else:
                        self.forward_l(list_of_movements)
                elif curr_dir == constants.WEST:
                    self.uturn(list_of_movements, curr_dir)
                elif curr_dir == prev_dir:
                    if prev_x > step[0]:
                        list_of_movements.append("B")
                    else:
                        list_of_movements.append("F")
            elif prev_dir == constants.WEST:
                if curr_dir == constants.NORTH:
                    if diff_grid == move[1]:
                        self.backward_l(list_of_movements)
                    else:
                        self.forward_r(list_of_movements)
                elif curr_dir == constants.SOUTH:
                    if diff_grid == move[1]:
                        self.backward_r(list_of_movements)
                    else:
                        self.forward_l(list_of_movements)
                elif curr_dir == constants.EAST:
                    self.uturn(list_of_movements, curr_dir)
                elif curr_dir == prev_dir:
                    if prev_x < step[0]:
                        list_of_movements.append("B")
                    else:
                        list_of_movements.append("F")

            prev_prev_grid = prev_grid
            prev_dir = curr_dir
            prev_x = step[0]
            prev_y = step[1]
            prev_grid = curr_grid

            count += 1

        print(list_of_movements)

        # Clean up and eliminate all FB pairs
        FB_present = True
        while FB_present:
            if len(list_of_movements) < 1:
                break
            prev_move = list_of_movements[0]
            index = -1
            initial_len = len(list_of_movements)
            for move in list_of_movements:
                index += 1
                if (prev_move == "F" and move == "B") or (prev_move == "B" and move == "F"):
                    list_of_movements.pop(index)
                    list_of_movements.pop(index - 1)
                    break
                prev_move = move

            if len(list_of_movements) == initial_len:
                FB_present = False
        print("FINAL: ", list_of_movements)

        return list_of_movements

    def forward_l(self, list_of_movements):
        list_of_movements.append("B")
        list_of_movements.append("B")
        list_of_movements.append("B")
        list_of_movements.append("FL")
        list_of_movements.append("B")
        list_of_movements.append("B")

    def forward_r(self, list_of_movements):
        list_of_movements.append("B")
        list_of_movements.append("B")
        list_of_movements.append("B")
        list_of_movements.append("FR")
        list_of_movements.append("B")
        list_of_movements.append("B")

    def uturn(self, list_of_movements, curr_dir):
        print("FIRST STEP")
        self.IS_ON_PATH = True
        self.plan_trip_by_robot_target_directions(self.robot.grid_x, self.robot.grid_y, self.robot.grid_x,
                                                  self.robot.grid_y,
                                                  self.robot.angle, curr_dir)
        self.IS_ON_PATH = False
        list_of_movements.append("F")

    def backward_l(self, list_of_movements):
        list_of_movements.append("F")
        list_of_movements.append("F")
        list_of_movements.append("F")
        list_of_movements.append("BL")
        list_of_movements.append("F")
        list_of_movements.append("F")

    def backward_r(self, list_of_movements):
        list_of_movements.append("F")
        list_of_movements.append("F")
        list_of_movements.append("F")
        list_of_movements.append("BR")
        list_of_movements.append("F")
        list_of_movements.append("F")

    def transpose_orientation(self, target_x, target_y, target_direction, robot_x, robot_y):
        if target_direction == constants.NORTH:
            a = target_x
            b = target_y
            x = robot_x
            y = robot_y
            return a, b, x, y
        elif target_direction == constants.EAST:
            a = -target_y
            b = target_x
            x = -robot_y
            y = robot_x
            return a, b, x, y
        elif target_direction == constants.SOUTH:
            a = -target_x
            b = -target_y
            x = -robot_x
            y = -robot_y
            return a, b, x, y
        elif target_direction == constants.WEST:
            a = target_y
            b = -target_x
            x = robot_y
            y = -robot_x
            return a, b, x, y
        else:
            return 0, 0, 0, 0

    def undo_target_transpose(self, target_x, target_y, target_direction):
        if target_direction == constants.NORTH:
            a = target_x
            b = target_y
            return a, b
        elif target_direction == constants.EAST:
            a = target_y
            b = -target_x
            return a, b
        elif target_direction == constants.SOUTH:
            a = -target_x
            b = -target_y
            return a, b
        elif target_direction == constants.WEST:
            a = -target_y
            b = target_x
            return a, b
        else:
            return 0, 0

    # Refer to Google Doc for details of different permutations
    def plan_trip_by_robot_target_directions(self, a, b, x, y, robot_direction, target_direction):
        try:
            print(a, b, x, y)
            print(target_direction, robot_direction)
            initial_a = a
            initial_b = b
            path_to_run = None
            a, b, x, y = self.transpose_orientation(a, b, target_direction, x, y)
            if robot_direction == target_direction:
                if abs(a - x) <= 2 and b < y:
                    print("AR1")
                    path_to_run = "AR1"
                    self.AR1(a, b, x, y)
                elif a < x and b == y:
                    print("AR2")
                    path_to_run = "AR2"
                    self.AR2(a, b, x, y)
                elif a == x and b > y:
                    print("AR3")
                    path_to_run = "AR3"
                    self.AR3(a, b, x, y)
                elif a > x and b == y:
                    print("AR4")
                    path_to_run = "AR4"
                    self.AR4(a, b, x, y)
                elif abs(a - x) > 2 and a < x and b < y:
                    print("AR5")
                    path_to_run = "AR5"
                    self.AR5(a, b, x, y)
                elif a < x and b > y:
                    print("AR6")
                    path_to_run = "AR6"
                    self.AR6(a, b, x, y)
                elif a > x and b > y:
                    print("AR7")
                    path_to_run = "AR7"
                    self.AR7(a, b, x, y)
                elif abs(a - x) > 2 and a > x and b < y:
                    print("AR8")
                    path_to_run = "AR8"
                    self.AR8(a, b, x, y)
                elif a == x and b == y:
                    path_to_run = "AR9"
                else:
                    print("no case")

            elif abs(robot_direction - target_direction) == 180:
                if a == x and b < y:
                    print("BR1")
                    path_to_run = "BR1"
                    self.BR1(a, b, x, y)
                elif a < x and b == y:
                    print("BR2")
                    path_to_run = "BR2"
                    self.BR2(a, b, x, y)
                elif a == x and b > y:
                    print("BR3")
                    path_to_run = "BR3"
                    self.BR3(a, b, x, y)
                elif a > x and b == y:
                    print("BR4")
                    path_to_run = "BR4"
                    self.BR4(a, b, x, y)
                elif a < x and b < y:
                    print("BR5")
                    path_to_run = "BR5"
                    self.BR5(a, b, x, y)
                elif a < x and b > y:
                    print("BR6")
                    path_to_run = "BR6"
                    self.BR6(a, b, x, y)
                elif a > x and b > y:
                    print("BR7")
                    path_to_run = "BR7"
                    self.BR7(a, b, x, y)
                elif a > x and b < y:
                    print("BR8")
                    path_to_run = "BR8"
                    self.BR8(a, b, x, y)
                elif a == x and b == y:
                    print("BR9")
                    path_to_run = "BR9"
                    self.BR9(a, b, x, y)
                else:
                    print("no case")

            elif (robot_direction - target_direction == 90) or \
                    (target_direction == constants.SOUTH and robot_direction == constants.EAST):
                if a == x and b < y:
                    print("CR1")
                    path_to_run = "CR1"
                    self.CR1(a, b, x, y)
                elif a < x and b == y:
                    self.CR2(a, b, x, y)
                    print("CR2")
                    path_to_run = "CR2"
                elif a == x and b > y:
                    print("CR3")
                    path_to_run = "CR3"
                    self.CR3(a, b, x, y)
                elif a > x and b == y:
                    print("CR4")
                    path_to_run = "CR4"
                    self.CR4(a, b, x, y)
                elif a < x and b < y:
                    print("CR5")
                    path_to_run = "CR5"
                    self.CR5(a, b, x, y)
                elif a < x and b > y:
                    print("CR6")
                    path_to_run = "CR6"
                    self.CR6(a, b, x, y)
                elif a > x and b > y:
                    print("CR7")
                    path_to_run = "CR7"
                    self.CR7(a, b, x, y)
                elif a > x and b < y:
                    print("CR8")
                    path_to_run = "CR8"
                    self.CR8(a, b, x, y)
                elif a == x and b == y:
                    print("CR9")
                    path_to_run = "CR9"
                    self.CR9(a, b, x, y)
                else:
                    print("no case")

            elif (robot_direction - target_direction == -90) or \
                    (target_direction == constants.EAST and robot_direction == constants.SOUTH):
                if a == x and b < y:
                    print("DR1")
                    path_to_run = "DR1"
                    self.DR1(a, b, x, y)
                elif a < x and b == y:
                    print("DR2")
                    path_to_run = "DR2"
                    self.DR2(a, b, x, y)
                elif a == x and b > y:
                    print("DR3")
                    path_to_run = "DR3"
                    self.DR3(a, b, x, y)
                elif a > x and b == y:
                    print("DR4")
                    path_to_run = "DR4"
                    self.DR4(a, b, x, y)
                elif a < x and b < y:
                    print("DR5")
                    path_to_run = "DR5"
                    self.DR5(a, b, x, y)
                elif a < x and b > y:
                    print("DR6")
                    path_to_run = "DR6"
                    self.DR6(a, b, x, y)
                elif a > x and b > y:
                    print("DR7")
                    path_to_run = "DR7"
                    self.DR7(a, b, x, y)
                elif a > x and b < y:
                    print("DR8")
                    path_to_run = "DR8"
                    self.DR8(a, b, x, y)
                elif a == x and b == y:
                    print("DR9")
                    path_to_run = "DR9"
                    self.DR9(a, b, x, y)
                else:
                    print("no case")

            else:
                return

            if not constants.IS_CHECKING and not self.IS_ON_PATH:
                if self.check_reached_target(initial_a, initial_b):
                    return
                else:
                    print("!! Target position not reached! Check", path_to_run)
                    self.replan_trip()

        except BorderException:
            print("BorderException--")
            if self.check_exceed_exception_count():
                return

            direction = self.robot.angle
            # TODO: Write how to handle border collision
            if direction == constants.NORTH:
                if x == 1:
                    if y >= 16:
                        self.move_backward_by(y - 18)
                    self.robot.move_forward_steer_right()
                elif x == 18:
                    if y >= 16:
                        self.move_backward_by(y - 16)
                    self.robot.move_forward_steer_left()
                elif y == 18:
                    self.move_backward_by(3)
                    try:
                        i = 0
                        self.robot.move_forward_steer_right()
                        self.move_backward_by(3)
                        i += 1
                        self.robot.move_forward_steer_right()
                    except:
                        if i == 0:
                            self.robot.move_forward_steer_left()
                            self.move_backward_by(3)
                            self.robot.move_forward_steer_left()
                        self.replan_trip()
                elif y == 1:
                    i = 0
                    try:
                        self.robot.move_forward_steer_right()
                        i += 1
                        self.robot.move_forward_steer_left()
                    except:
                        if i == 0:
                            self.robot.move_forward_steer_left()
                            self.robot.move_forward_steer_right()
                    self.replan_trip()
            # if robot direction is south
            elif direction == constants.SOUTH:
                if x == 1:
                    if y <= 5:
                        self.move_backward_by(6 - y)
                    self.robot.move_forward_steer_left()
                elif x == 18:
                    if y <= 5:
                        self.move_backward_by(6 - y)
                    self.robot.move_forward_steer_left()
                elif y == 18:
                    i = 0
                    try:
                        self.robot.move_forward_steer_right()
                        i += 1
                        self.robot.move_forward_steer_left()
                    except:
                        if i == 0:
                            self.robot.move_forward_steer_left()
                            self.robot.move_forward_steer_right()
                        self.replan_trip()
                elif y == 1:
                    self.move_backward_by(3)
                    try:
                        i = 0
                        self.robot.move_forward_steer_right()
                        self.move_backward_by(3)
                        i += 1
                        self.robot.move_forward_steer_right()

                    except:
                        if i == 0:
                            self.robot.move_forward_steer_left()
                            self.move_backward_by(3)
                            self.robot.move_forward_steer_left()
                        self.replan_trip()

            # if robot direction is east

            elif direction == constants.EAST:
                if y == 1:
                    if x >= 16:
                        self.move_backward_by(x - 18)
                    self.robot.move_forward_steer_left()
                if y == 18:
                    if x >= 16:
                        self.move_backward_by(x - 18)
                    self.robot.move_forward_steer_right()
                elif x == 1:
                    i = 0
                    try:
                        self.robot.move_forward_steer_right()
                        i += 1
                        self.robot.move_forward_steer_left()
                    except:
                        if i == 0:
                            self.robot.move_forward_steer_left()
                            self.robot.move_forward_steer_right()
                        self.replan_trip()

                elif x == 18:
                    self.move_backward_by(3)
                    try:
                        i = 0
                        self.robot.move_forward_steer_right()
                        self.move_backward_by(3)
                        i += 1
                        self.robot.move_forward_steer_right()

                    except:
                        if i == 0:
                            self.robot.move_forward_steer_left()
                            self.move_backward_by(3)
                            self.robot.move_forward_steer_left()
                        self.replan_trip()
            # if robot direction is west
            elif direction == constants.WEST:
                if y == 18:
                    if x <= 5:
                        self.move_backward_by(6 - x)
                    self.robot.move_forward_steer_left()
                if y == 1:
                    if x <= 5:
                        self.move_backward_by(6 - x)
                    self.robot.move_forward_steer_right()
                elif x == 18:
                    i = 0
                    try:
                        self.robot.move_forward_steer_right()
                        i += 1
                        self.robot.move_forward_steer_left()
                    except:
                        if i == 0:
                            self.robot.move_forward_steer_left()
                            self.robot.move_forward_steer_right()
                        self.replan_trip()
                elif x == 1:
                    self.move_backward_by(3)
                    try:
                        i = 0
                        self.robot.move_forward_steer_right()
                        self.move_backward_by(3)
                        i += 1
                        self.robot.move_forward_steer_right()

                    except:
                        if i == 0:
                            self.robot.move_forward_steer_left()
                            self.move_backward_by(3)
                            self.robot.move_forward_steer_left()
                        self.replan_trip()
            self.replan_trip()

        except ObstacleException:
            print("ObstacleException--")
            print(self.EXCEPTION_COUNT)
            if self.check_exceed_exception_count():
                return

            self.replan_trip()

        except ObstacleTurnException:
            print("ObstacleTurnException--")
            print(self.EXCEPTION_COUNT)
            if self.check_exceed_exception_count():
                return

            self.replan_trip()

    def check_exceed_exception_count(self):
        if self.EXCEPTION_COUNT > 5:
            logging.info("!! Exception Loop Detected! Giving up on current target obstacle...")
            self.fastest_route.append(self.target_pose)
            logging.info("Astar route: " + str(self.fastest_route))
            return True

        self.EXCEPTION_COUNT += 1
        return False

    def replan_trip(self):
        print("Replanning trip...")
        a, b, x, y = self.target_pose.x, self.target_pose.y, self.robot.get_grid_pos()[0], self.robot.get_grid_pos()[1]
        robot_direction, target_direction = self.robot.get_angle_of_rotation(), self.target_pose.direction
        self.plan_trip_by_robot_target_directions(a, b, x, y, robot_direction, target_direction)

    def preprocess_coords(self, a, b, x, y):
        a, b = self.undo_target_transpose(a, b, self.target_pose.direction)
        a, b, x, y = self.transpose_orientation(a, b, self.target_pose.direction,
                                                self.robot.get_grid_pos()[0], self.robot.get_grid_pos()[1])
        return a, b, x, y

    def AR1(self, a, b, x, y):
        if abs(b - y) <= 2:
            self.move_backward_by(abs(b - y))
            if a - x < 0:
                a, b, x, y = self.preprocess_coords(a, b, x, y)
                self.AR2(a, b, x, y)
            elif a - x > 0:
                a, b, x, y = self.preprocess_coords(a, b, x, y)
                self.AR4(a, b, x, y)
        else:
            if a - x <= 0:
                self.turn_forward_left()
                self.move_forward_by(abs(a - x))
                self.turn_forward_left()
                self.move_forward_by(abs(b - y))
                self.turn_forward_left()
                self.turn_forward_left()

            else:
                self.turn_forward_right()
                self.move_forward_by(abs(a - x))
                self.turn_forward_right()
                self.move_forward_by(abs(b - y))
                self.turn_forward_right()
                self.turn_forward_right()

    def AR2(self, a, b, x, y):
        a, b, x, y = self.preprocess_coords(a, b, x, y)
        self.AR6(a, b, x, y)

    def AR3(self, a, b, x, y):
        # Forward by abs(b-y)
        self.move_forward_by(abs(b - y))

    def AR4(self, a, b, x, y):
        a, b, x, y = self.preprocess_coords(a, b, x, y)
        self.AR7(a, b, x, y)

    def AR5(self, a, b, x, y):
        self.move_backward_by(abs(b - y))
        a, b, x, y = self.preprocess_coords(a, b, x, y)
        self.AR6(a, b, x, y)

    def AR6(self, a, b, x, y):
        if abs(b - y) < 2:
            self.move_backward_by(2 - abs(b - y))
        else:
            self.move_forward_by(abs(b - y) - 2)
        self.turn_forward_left()
        self.move_forward_by(abs(a - x))
        self.turn_backward_left()
        self.move_forward_by(2)

    def AR7(self, a, b, x, y):
        if abs(b - y) < 2:
            self.move_backward_by(2 - abs(b - y))
        else:
            self.move_forward_by(abs(b - y) - 2)
        self.turn_forward_right()
        self.move_forward_by(abs(a - x))
        self.turn_backward_right()
        self.move_forward_by(2)

    def AR8(self, a, b, x, y):
        self.move_backward_by(abs(b - y))
        a, b, x, y = self.preprocess_coords(a, b, x, y)
        self.AR7(a, b, x, y)

    def BR1(self, a, b, x, y):
        if abs(b - y) <= 2:
            self.move_forward_by(abs(b - y))
            a, b, x, y = self.preprocess_coords(a, b, x, y)
            self.BR9(a, b, x, y)
        else:
            if abs(b - y) < 10:
                self.move_backward_by(10 - abs(b - y))
            else:
                self.move_forward_by(abs(b - y) - 10)
            self.turn_forward_right()
            self.move_backward_by(3)
            self.turn_forward_left()
            self.turn_forward_left()
            self.move_forward_by(3)
            self.turn_backward_right()
            self.move_forward_by(2)

    def BR2(self, a, b, x, y):
        if abs(a - x) <= 2:
            self.turn_forward_right()
            self.move_backward_by(6 - abs(a - x))
            self.turn_forward_right()
        else:
            self.move_backward_by(3)
            self.turn_forward_right()
            self.move_forward_by(abs(a - x))
            self.turn_backward_left()
            self.move_forward_by(3)

    def BR3(self, a, b, x, y):
        self.move_backward_by(abs(b - y))
        a, b, x, y = self.preprocess_coords(a, b, x, y)
        self.BR9(a, b, x, y)

    def BR4(self, a, b, x, y):
        if abs(a - x) <= 2:
            self.turn_forward_left()
            self.move_backward_by(6 - abs(a - x))
            self.turn_forward_left()
        else:
            self.move_backward_by(3)
            self.turn_forward_left()
            self.move_forward_by(abs(a - x))
            self.turn_backward_right()
            self.move_forward_by(3)

    def BR5(self, a, b, x, y):
        if abs(a - x) <= 2:
            if abs(b - y) <= 2:
                self.move_forward_by(abs(b - y))
                self.turn_forward_right()
                self.move_backward_by(6 - abs(a - x))
                self.turn_forward_right()
            else:
                self.turn_backward_right()
                a, b, x, y = self.preprocess_coords(a, b, x, y)
                self.DR8(a, b, x, y)

        else:
            # if abs(b - y) - 3 >= 0:
            #     self.move_forward_by(abs(b - y) - 3)
            # else:
            #     self.move_backward_by(3 - abs(b - y))
            # self.turn_forward_right()
            # self.move_forward_by(abs(a - x))
            # self.turn_backward_left()
            # self.move_forward_by(3)
            if abs(b - y) - 3 >= 0:
                self.move_forward_by(abs(b - y))
            else:
                self.move_backward_by(abs(b - y))
            self.turn_forward_right()
            if (abs(a - x) - 6) >= 0:
                self.move_forward_by(abs(a - x) - 6)
            else:
                self.move_backward_by(6 - abs(a - x))
            self.turn_forward_right()

    def BR6(self, a, b, x, y):
        self.move_backward_by(abs(b - y))
        if abs(a - x) > 0:
            self.turn_forward_right()
            self.move_forward_by(abs(a - x))
            self.turn_backward_left()
            self.move_forward_by(6)
        else:
            a, b, x, y = self.preprocess_coords(a, b, x, y)
            self.BR2(a, b, x, y)

    def BR7(self, a, b, x, y):
        self.move_backward_by(abs(b - y))
        if abs(a) - abs(x) < 0:
            self.turn_forward_left()
            self.move_forward_by(abs(a - x))
            self.turn_backward_right()
            self.move_forward_by(6)
        else:
            a, b, x, y = self.preprocess_coords(a, b, x, y)
            self.BR4(a, b, x, y)

    def BR8(self, a, b, x, y):
        if abs(a - x) <= 2:
            if abs(b - y) <= 2:
                self.move_forward_by(abs(b - y))
                self.turn_forward_left()
                self.move_backward_by(6 - abs(a - x))
                self.turn_forward_left()
            else:
                self.turn_backward_left()
                a, b, x, y = self.preprocess_coords(a, b, x, y)
                self.CR5(a, b, x, y)

        else:
            if abs(b - y) >= 3:
                self.move_forward_by(abs(b - y) - 3)
            else:
                self.move_backward_by(3 - abs(b - y))
            self.turn_forward_left()
            self.move_forward_by(abs(a - x))
            self.turn_backward_right()
            self.move_forward_by(3)

    def BR9(self, a, b, x, y):
        if (abs(a) > 15 and (self.robot.get_angle_of_rotation() == 0 or self.robot.get_angle_of_rotation() == 90)) or (
                abs(a) < 4 and (
                self.robot.get_angle_of_rotation() == 180 or self.robot.get_angle_of_rotation() == -90)):
            self.move_backward_by(2)
            self.turn_forward_left()
            self.turn_backward_right()
            self.move_forward_by(4)
        elif (abs(a) < 4 and (self.robot.get_angle_of_rotation() == 0 or self.robot.get_angle_of_rotation() == 90)) or (
                abs(a) > 15 and (
                self.robot.get_angle_of_rotation() == 180 or self.robot.get_angle_of_rotation() == -90)):
            self.move_backward_by(2)
            self.turn_forward_right()
            self.turn_backward_left()
            self.move_forward_by(4)
        else:
            self.turn_forward_right()
            self.move_backward_by(6)
            self.turn_forward_right()

    def CR1(self, a, b, x, y):
        if abs(b - y) <= 2:
            self.move_forward_by(3)
            self.turn_backward_left()
            self.move_forward_by(3 - abs(b - y))
            return
        # Backward Left
        self.turn_backward_left()
        a, b, x, y = self.preprocess_coords(a, b, x, y)
        self.AR5(a, b, x, y)

    def CR2(self, a, b, x, y):
        self.move_forward_by(abs(a - x) + 3)
        a, b, x, y = self.preprocess_coords(a, b, x, y)
        self.CR4(a, b, x, y)
        pass

    def CR3(self, a, b, x, y):
        a, b, x, y = self.preprocess_coords(a, b, x, y)
        self.CR6(a, b, x, y)
        pass

    def CR4(self, a, b, x, y):
        if abs(a - x) <= 2:
            self.move_forward_by(3 - abs(a - x))
        else:
            self.move_backward_by(abs(x - a) - 3)
        # Backward Left
        self.turn_backward_left()
        self.move_forward_by(abs(b - y) + 3)

    def CR5(self, a, b, x, y):
        # Backward Left
        if abs(y - b) < 2:
            self.move_forward_by(abs(a - x) + 3)
            self.turn_backward_left()
            self.move_forward_by(abs(b - y) + 1)
            return
        if abs(a - x) > 6:
            self.move_forward_by(abs(a - x) - 6)
        else:
            self.move_backward_by(6 - abs(a - x))
        self.turn_forward_left()
        if abs(y - b) < 7:
            self.move_backward_by(7 - abs(b - y))
        else:
            self.move_forward_by(abs(b - y) - 7)
        self.turn_forward_right()
        self.move_forward_by(3)
        self.turn_backward_left()
        self.move_forward_by(2)

    def CR6(self, a, b, x, y):
        if abs(a - x) == 3 and abs(b - y) == 3:
            self.turn_forward_right()
            return

        self.move_forward_by(abs(a - x) + 3)
        a, b, x, y = self.preprocess_coords(a, b, x, y)
        self.CR7(a, b, x, y)

    def CR7(self, a, b, x, y):
        if abs(x - a) < 3:
            self.move_forward_by(3 - abs(x - a))
        else:
            self.move_backward_by(abs(x - a) - 3)
        self.turn_backward_left()
        self.move_forward_by(abs(b - y) + 3)

    def CR8(self, a, b, x, y):
        if abs(b - y) == 1:
            if abs(a - x) < 3:
                self.move_forward_by(3 - abs(a - x))
            else:
                self.move_backward_by(abs(a - x) - 3)
            self.robot.move_backward_steer_left()
            self.move_forward_by(2)
        elif abs(b - y) < 7:
            if abs(a - x) < 3:
                self.move_forward_by(3 - abs(a - x))
            else:
                self.move_backward_by(abs(a - x) - 3)
            self.turn_forward_left()
            if abs(b - y) < 3:
                self.move_backward_by(3 - abs(b - y))
            else:
                self.move_forward_by(abs(b - y) - 3)
            self.turn_forward_left()
            self.turn_forward_left()
        else:
            self.move_backward_by(abs(a - x))
            self.robot.move_forward_steer_left()
            self.move_forward_by(abs(b - y) - 3)
            self.robot.move_forward_steer_left()
            self.move_backward_by(3)
            self.robot.move_forward_steer_left()

    def CR9(self, a, b, x, y):

        if (abs(a) > 15 and (self.robot.get_angle_of_rotation() == 0 or self.robot.get_angle_of_rotation() == -90)) or (
                abs(a) < 4 and (
                self.robot.get_angle_of_rotation() == 180 or self.robot.get_angle_of_rotation() == 90)):
            print("```Border case CR9")
            self.turn_backward_right()
            self.move_forward_by(3)
            self.turn_forward_right()
            self.move_backward_by(3)
            self.turn_forward_right()
        else:
            self.move_forward_by(3)
            self.turn_backward_left()
            self.move_forward_by(3)

    def DR1(self, a, b, x, y):
        if abs(b - y) <= 2:
            self.move_forward_by(3)
            self.turn_backward_right()
            self.move_forward_by(2)
        else:
            self.turn_backward_right()
            a, b, x, y = self.preprocess_coords(a, b, x, y)
            self.AR8(a, b, x, y)

    def DR2(self, a, b, x, y):
        if abs(a - x) <= 2:
            self.move_forward_by(3 - abs(a - x))
        else:
            self.move_backward_by(abs(a - x) - 3)
        # Backward Right
        self.turn_backward_right()
        self.move_forward_by(abs(b - y) + 3)

    def DR3(self, a, b, x, y):
        a, b, x, y = self.preprocess_coords(a, b, x, y)
        self.DR7(a, b, x, y)

    def DR4(self, a, b, x, y):
        self.move_forward_by(abs(a - x) + 3)
        self.turn_backward_right()
        self.move_forward_by(3)

    def DR5(self, a, b, x, y):
        if abs(b - y) == 1:
            if abs(a - x) < 3:
                self.move_forward_by(3 - abs(a - x))
            else:
                self.move_backward_by(abs(a - x) - 3)
            self.turn_backward_right()
            self.move_forward_by(2)
        elif abs(b - y) < 7:
            if abs(a - x) < 3:
                self.move_forward_by(3 - abs(a - x))
            else:
                self.move_backward_by(abs(a - x) - 3)
            self.turn_forward_right()
            if abs(b - y) < 3:
                self.move_backward_by(3 - abs(b - y))
            else:
                self.move_forward_by(abs(b - y) - 3)
            self.turn_forward_right()
            self.turn_forward_right()
        else:
            self.move_backward_by(abs(a - x))
            self.turn_forward_right()
            self.move_forward_by(abs(b - y) - 3)
            self.turn_forward_right()
            self.move_backward_by(3)
            self.turn_forward_right()

    def DR6(self, a, b, x, y):
        if abs(a - x) <= 2:
            self.move_forward_by(3 - abs(a - x))
        else:
            self.move_backward_by(abs(a - x) - 3)
        self.turn_backward_right()
        self.move_forward_by(abs(b - y) + 3)

    def DR7(self, a, b, x, y):
        if abs(a - x) == 3 and abs(b - y) == 3:
            self.turn_forward_left()
            return

        self.move_forward_by(abs(a - x) + 3)
        a, b, x, y = self.preprocess_coords(a, b, x, y)
        self.DR6(a, b, x, y)

    def DR8(self, a, b, x, y):
        if abs(b - y) == 1:
            self.move_forward_by(abs(a - x) + 3)
            self.turn_backward_right()
            self.move_forward_by(2)
        else:
            if (abs(a - x) < 6):
                self.move_backward_by(6 - abs(a - x))
            else:
                self.move_forward_by(abs(a - x) - 6)
            self.turn_forward_right()
            if abs(b - y) < 3:
                self.move_backward_by(3 - abs(b - y))
            else:
                self.move_forward_by(abs(b - y) - 3)
            self.turn_forward_left()
            self.move_backward_by(3)
            self.turn_forward_left()

    def DR9(self, a, b, x, y):

        if (abs(a) > 15 and (self.robot.get_angle_of_rotation() == 0 or self.robot.get_angle_of_rotation() == -90)) or (
                abs(a) < 4 and (
                self.robot.get_angle_of_rotation() == 180 or self.robot.get_angle_of_rotation() == 90)):
            print("```Border case DR9")
            self.turn_backward_left()
            self.move_forward_by(3)
            self.turn_forward_left()
            self.move_backward_by(3)
            self.turn_forward_left()
        else:
            self.move_forward_by(3)
            self.turn_backward_right()
            self.move_forward_by(3)

    def turn_forward_right(self):
        self.robot.move_forward_steer_right()
        if not constants.IS_CHECKING:
            self.collection_of_movements.append("FR")
            self.collection_of_robot_pos.append(self.get_robot_pos())

    def turn_forward_left(self):
        self.robot.move_forward_steer_left()
        if not constants.IS_CHECKING:
            self.collection_of_movements.append("FL")
            self.collection_of_robot_pos.append(self.get_robot_pos())

    def turn_backward_right(self):
        self.robot.move_backward_steer_right()
        if not constants.IS_CHECKING:
            self.collection_of_movements.append("BR")
            self.collection_of_robot_pos.append(self.get_robot_pos())

    def turn_backward_left(self):
        self.robot.move_backward_steer_left()
        if not constants.IS_CHECKING:
            self.collection_of_movements.append("BL")
            self.collection_of_robot_pos.append(self.get_robot_pos())

    def move_forward_by(self, no_of_steps):
        for i in range(int(no_of_steps)):
            self.robot.move_forward()
            if not constants.IS_CHECKING:
                self.collection_of_movements.append("F")
                self.collection_of_robot_pos.append(self.get_robot_pos())

    def move_backward_by(self, no_of_steps):
        for i in range(int(no_of_steps)):
            self.robot.move_backward()
            if not constants.IS_CHECKING:
                self.collection_of_movements.append("B")
                self.collection_of_robot_pos.append(self.get_robot_pos())

    def reset_collection_of_movements(self):
        self.collection_of_movements = []

    def reset_robot_pos_list(self):
        self.collection_of_robot_pos = []

    def get_collection_of_movements_string(self):
        list_of_movements = self.collection_of_movements

        # Clean up and eliminate all FB pairs
        FB_present = True
        while FB_present:
            if len(list_of_movements) < 1:
                break
            prev_move = list_of_movements[0]
            index = -1
            initial_len = len(list_of_movements)
            for move in list_of_movements:
                index += 1
                if (prev_move == "F" and move == "B") or (prev_move == "B" and move == "F"):
                    list_of_movements.pop(index)
                    list_of_movements.pop(index - 1)
                    break
                prev_move = move

            if len(list_of_movements) == initial_len:
                FB_present = False

        # CLean up and eliminate all TURN pairs
        TURNS_present = True
        while TURNS_present:
            if len(list_of_movements) < 1:
                break
            prev_move = list_of_movements[0]
            index = -1
            initial_len = len(list_of_movements)
            for move in list_of_movements:
                index += 1
                if (prev_move == "FR" and move == "BR") or (prev_move == "BR" and move == "FR"):
                    list_of_movements.pop(index)
                    list_of_movements.pop(index - 1)
                    break
                elif (prev_move == "FL" and move == "BL") or (prev_move == "BL" and move == "FL"):
                    list_of_movements.pop(index)
                    list_of_movements.pop(index - 1)
                    break
                prev_move = move

            if len(list_of_movements) == initial_len:
                TURNS_present = False

        # Clean up and eliminate all FB pairs again
        FB_present = True
        while FB_present:
            if len(list_of_movements) < 1:
                break
            prev_move = list_of_movements[0]
            index = -1
            initial_len = len(list_of_movements)
            for move in list_of_movements:
                index += 1
                if (prev_move == "F" and move == "B") or (prev_move == "B" and move == "F"):
                    list_of_movements.pop(index)
                    list_of_movements.pop(index - 1)
                    break
                prev_move = move

            if len(list_of_movements) == initial_len:
                FB_present = False

        self.collection_of_movements = list_of_movements
        return ','.join([str(movement) for movement in self.collection_of_movements])

    def get_movements_string(self):
        movement_list = ["MOVEMENTS", self.obstacle_cell.get_obstacle().get_obstacle_id(),
                         self.get_collection_of_movements_string()]
        return '/'.join([str(elem) for elem in movement_list])

    def get_current_obstacle_id(self):
        obstacle_id_list = ["OBSTACLE", self.obstacle_cell.get_obstacle().get_obstacle_id()]
        return '/'.join([str(elem) for elem in obstacle_id_list])

    def get_robot_pos(self):
        return (self.robot.grid_x, self.robot.grid_y, self.robot.angle)

    def get_collection_of_robot_pos_string(self):
        return '/'.join([str(pos) for pos in self.collection_of_robot_pos])

    def get_robot_pos_string(self):
        robot_list = ["ROBOT", self.obstacle_cell.get_obstacle().get_obstacle_id(),
                      self.get_collection_of_robot_pos_string()]
        return '/'.join([str(elem) for elem in robot_list])

    def get_take_photo_string(self):
        photo_list = ["PHOTO", self.obstacle_cell.get_obstacle().get_obstacle_id()]
        return '/'.join([str(elem) for elem in photo_list])

    def get_image_result_string(self, target_id):
        image_result_list = ["TARGET", target_id, self.obstacle_key]
        return '/'.join([str(elem) for elem in image_result_list])

    def check_reached_target(self, target_a, target_b):
        x, y = self.robot.get_grid_pos()[0], self.robot.get_grid_pos()[1]
        if target_a == x and target_b == y:
            print(self.get_movements_string())
            print(self.get_current_obstacle_id())
            print(self.get_robot_pos_string())
            print(self.get_take_photo_string())

            # Add into dictionary
            self.all_movements_dict[self.get_current_obstacle_id()] = self.get_movements_string()
            self.all_robot_pos_dict[self.get_current_obstacle_id()] = self.get_robot_pos_string()
            self.all_take_photo_dict[self.get_current_obstacle_id()] = self.get_take_photo_string()
            self.obstacle_list_rpi.append(self.get_current_obstacle_id())

            col,row = 0,0
            i=0
            previous_row, previous_col = 18, 1

            for position in self.collection_of_robot_pos:
                row, col = 19-int(position[1]), int(position[0]) # row = 19-y-coord, column = x-coord
                # print(position, row, col)
                robot_dir = position[2]

                # if row or col out of boundary, ignore and go to next position
                if row < 0 or row > 19 or col < 0 or col > 19:
                    continue

                # Set cell status as 5 for cell in position
                self.grid.cells[row][col].set_path_status(5)

                # Set cell status as 5 for cells which are part of turns
                if previous_row != row and previous_col != col and i < len(self.collection_of_movements):
                    # previous row > current row and movement == "FR"
                    if previous_row > row and (self.collection_of_movements[i] == "FR"):
                        if robot_dir != 0:
                            for row_idx in range(row, previous_row):
                                self.grid.cells[row_idx][previous_col].set_path_status(5)

                            # previous col < current col
                            if previous_col < col:
                                for col_idx in range(previous_col, col):
                                    self.grid.cells[row][col_idx].set_path_status(5)

                        else:
                            for row_idx in range(row, previous_row):
                                self.grid.cells[row_idx][col].set_path_status(5)

                            # previous col < current col
                            if previous_col < col:
                                for col_idx in range(previous_col, col):
                                    self.grid.cells[row][col_idx].set_path_status(5)

                    # previous row > current row and movement == "FL"
                    elif previous_row > row and (self.collection_of_movements[i] == "FL"):
                        if robot_dir == 0:
                            for row_idx in range(row, previous_row):
                                self.grid.cells[row_idx][col].set_path_status(5)

                            # previous col < current col
                            if previous_col < col:
                                for col_idx in range(previous_col, col):
                                    self.grid.cells[previous_row][col_idx].set_path_status(5)

                        if robot_dir == 90:
                            for row_idx in range(row, previous_row):
                                self.grid.cells[row_idx][previous_col].set_path_status(5)

                            # previous col > current col
                            if previous_col > col:
                                for col_idx in range(col, previous_col):
                                    self.grid.cells[row][col_idx].set_path_status(5)

                    # previous row < current row and movement == "FR"
                    elif previous_row < row and (self.collection_of_movements[i] == "FR"):
                        if robot_dir == 180:
                            for row_idx in range(previous_row, row):
                                self.grid.cells[row_idx][col].set_path_status(5)

                            # previous col < current col
                            if previous_col < col:
                                for col_idx in range(previous_col, col):
                                    self.grid.cells[previous_row][col_idx].set_path_status(5)

                        elif robot_dir == 90:
                            for row_idx in range(previous_row, row):
                                self.grid.cells[row_idx][previous_col].set_path_status(5)

                            # previous col > current col
                            if previous_col > col:
                                for col_idx in range(col, previous_col):
                                    self.grid.cells[row][col_idx].set_path_status(5)

                    # previous row < current row and movement == "FL"
                    elif previous_row < row and (self.collection_of_movements[i] == "FL"):
                        if robot_dir == -90:
                            for row_idx in range(previous_row, row):
                                self.grid.cells[row_idx][previous_col].set_path_status(5)

                            # previous col < current col
                            if previous_col < col:
                                for col_idx in range(previous_col, col):
                                    self.grid.cells[row][col_idx].set_path_status(5)

                    # previous row < current row and movement == "BL"
                    elif previous_row < row and (self.collection_of_movements[i] == "BL"):
                        if robot_dir == 0:
                            for row_idx in range(previous_row, row):
                                self.grid.cells[row_idx][col].set_path_status(5)

                            # previous col < current col
                            if previous_col < col:
                                for col_idx in range(previous_col, col):
                                    self.grid.cells[previous_row][col_idx].set_path_status(5)

                    # previous row > current row and movement == "BR"
                    elif previous_row > row and (self.collection_of_movements[i] == "BR"):
                        if robot_dir == 180:
                            for row_idx in range(row, previous_row):
                                self.grid.cells[row_idx][col].set_path_status(5)

                            # previous col < current col
                            if previous_col < col:
                                for col_idx in range(previous_col, col):
                                    self.grid.cells[previous_row][col_idx].set_path_status(5)

                # Colour rough route gray
                self.robot.redraw_car()

                previous_row, previous_col = row, col
                i += 1

            # Move car 2 steps backwards for next move
            self.reset_collection_of_movements()
            self.reset_robot_pos_list()
            #self.move_backward_by(2)

            return True
        print(self.get_movements_string())
        print(self.get_current_obstacle_id())
        print(self.get_robot_pos_string())
        print(self.get_take_photo_string())
        return False

    def send_to_rpi(self):
        if self.obstacle_list_rpi:
            self.obstacle_key = self.obstacle_list_rpi.pop(0)
            self.reset_num_move_completed_rpi()
            self.total_num_move_required_rpi = len(self.all_movements_dict[self.obstacle_key].split("/")[-1].split(","))
            print("Remaining obstacles: ", self.obstacle_list_rpi)
            self.simulator.comms.send(self.all_robot_pos_dict[self.obstacle_key])
            self.simulator.comms.send(self.all_movements_dict[self.obstacle_key])
        else:
            # Call predict function on finish
            self.simulator.predict_on_finish()
            self.simulator.comms.send("FINISH/EXPLORE/")

    def reset_num_move_completed_rpi(self):
        self.num_move_completed_rpi = 0

    def update_num_move_completed(self, num_moves_completed: int):
        self.num_move_completed_rpi += num_moves_completed

    def is_move_to_current_obstacle_done(self) -> bool:
        return self.num_move_completed_rpi == self.total_num_move_required_rpi

    def request_photo_from_rpi(self):
        self.simulator.comms.send(self.all_take_photo_dict[self.obstacle_key])

    def send_to_rpi_recalculated(self, arglist):
        robot_x = int(arglist[0])
        robot_y = int(arglist[1])
        robot_dir = int(arglist[2])

        # Reset collection first
        self.reset_collection_of_movements()
        self.reset_robot_pos_list()
        # self.move_backward_by(2)

        if self.obstacle_list_rpi:
            self.obstacle_key = self.obstacle_list_rpi.pop(0)
            print("Remaining obstacles: ", self.obstacle_list_rpi)
            # Set robot position
            self.robot.grid_x = robot_x
            self.robot.grid_y = robot_y
            self.robot.angle = robot_dir
            self.robot.redraw_car()
            # Set target position
            i = 0 - len(self.obstacle_list_rpi) - 1
            target = self.robot.optimized_target_locations[i:][0]

            # Replan path for this particular obstacle with new robot position
            self.target_pose.set_pose(target[:3])
            self.obstacle_cell = target[3]

            self.robot_pose = self.robot.get_robot_pose()

            # if no obstacle exceptions, can use hardcoded shortest path
            if self.check_movement_possible(self.target_pose.x, self.target_pose.y, self.robot_pose.x, self.robot_pose.y,
                                            self.robot_pose.direction, self.target_pose.direction):
                # Target Coordinates: (a, b); Robot Coordinates: (x, y)
                self.plan_trip_by_robot_target_directions(self.target_pose.x, self.target_pose.y, self.robot_pose.x, self.robot_pose.y,
                                                          self.robot_pose.direction, self.target_pose.direction)

            # else, plan a path using astar search on virtual grid
            else:
                # x-coord = column; y-coord = 19-row
                start = [19 - self.robot.grid_y, self.robot.grid_x, self.robot.angle]
                end = [19 - self.target_pose.y,
                       self.target_pose.x,
                       self.target_pose.direction]  # ending position
                cost = 10  # cost per movement
                maze = self.grid.cells_virtual
                search_result = search(maze, cost, start, end)

                if search_result is None:
                    # Force run hardcoded path
                    print("Search result: ", search_result, " ; FORCING hardcoded path...")
                    self.plan_trip_by_robot_target_directions(self.target_pose.x, self.target_pose.y, self.robot_pose.x, self.robot_pose.y,
                                                              self.robot_pose.direction, self.target_pose.direction)
                else:
                    # Execute gray route
                    draw_path, path = search_result[0], search_result[1]


                    self.IS_ON_PATH = True
                    for r in range(20):
                        for c in range(20):
                            if draw_path[r][c] >= 5:
                                self.grid.cells[r][c].set_path_status(draw_path[r][c])
                    # Colour rough route gray
                    self.robot.redraw_car()

                    self.IS_ON_PATH = True
                    movements = self.translate_path_to_movements(path)
                    for move in movements:
                        self.do_move(move)

                    self.IS_ON_PATH = True
                    # Last step is to rotate on the spot
                    print("LAST STEP")
                    self.plan_trip_by_robot_target_directions(self.target_pose.x, self.target_pose.y, self.robot.grid_x,
                                                              self.robot.grid_y,
                                                              self.robot.angle, self.target_pose.direction)

            x, y = self.robot.get_grid_pos()[0], self.robot.get_grid_pos()[1]
            if self.target_pose.x == x and self.target_pose.y == y:
                print(self.get_movements_string())
                print(self.get_current_obstacle_id())
                print(self.get_robot_pos_string())
                print(self.get_take_photo_string())
                print("test")

            # Change all movements dict and all robot pos dict for obstacle key replanned
            self.all_movements_dict[self.obstacle_key] = self.get_movements_string()
            self.all_robot_pos_dict[self.obstacle_key] = self.get_robot_pos_string()
            self.all_take_photo_dict[self.obstacle_key] = self.get_take_photo_string()
            # Reset
            self.IS_ON_PATH = False
            self.reset_collection_of_movements()
            self.reset_robot_pos_list()

            self.reset_num_move_completed_rpi()
            self.total_num_move_required_rpi = len(self.all_movements_dict[self.obstacle_key].split("/")[-1].split(","))
            self.simulator.comms.send(self.all_robot_pos_dict[self.obstacle_key])
            self.simulator.comms.send(self.all_movements_dict[self.obstacle_key])
            self.simulator.comms.send(self.all_take_photo_dict[self.obstacle_key])
        else:
            self.simulator.comms.send("No more movements.")

    def check_movement_possible(self, a, b, x, y, robot_direction, target_direction):
        constants.IS_CHECKING = True
        area = None
        initial_x = x
        initial_y = y
        initial_dir = robot_direction
        a, b, x, y = self.transpose_orientation(a, b, target_direction, x, y)
        try:
            if robot_direction == target_direction:
                if abs(a - x) <= 2 and b < y:
                    area = "AR1"
                    self.AR1(a, b, x, y)
                elif a < x and b == y:
                    area = "AR2"
                    self.AR2(a, b, x, y)
                elif a == x and b > y:
                    area = "AR3"
                    self.AR3(a, b, x, y)
                elif a > x and b == y:
                    area = "AR4"
                    self.AR4(a, b, x, y)
                elif abs(a - x) > 2 and a < x and b < y:
                    area = "AR5"
                    self.AR5(a, b, x, y)
                elif a < x and b > y:
                    area = "AR6"
                    self.AR6(a, b, x, y)
                elif a > x and b > y:
                    area = "AR7"
                    self.AR7(a, b, x, y)
                elif abs(a - x) > 2 and a > x and b < y:
                    area = "AR8"
                    self.AR8(a, b, x, y)
                else:
                    area = "AR0"

            elif abs(robot_direction - target_direction) == 180:
                if a == x and b < y:
                    area = "BR1"
                    self.BR1(a, b, x, y)
                elif a < x and b == y:
                    area = "BR2"
                    self.BR2(a, b, x, y)
                elif a == x and b > y:
                    area = "BR3"
                    self.BR3(a, b, x, y)
                elif a > x and b == y:
                    area = "BR4"
                    self.BR4(a, b, x, y)
                elif a < x and b < y:
                    area = "BR5"
                    self.BR5(a, b, x, y)
                elif a < x and b > y:
                    area = "BR6"
                    self.BR6(a, b, x, y)
                elif a > x and b > y:
                    area = "BR7"
                    self.BR7(a, b, x, y)
                elif a > x and b < y:
                    area = "BR8"
                    self.BR8(a, b, x, y)
                elif a == x and b == y:
                    area = "BR9"
                    self.BR9(a, b, x, y)
                else:
                    area = "BR0"

            elif (robot_direction - target_direction == 90) or \
                    (target_direction == constants.SOUTH and robot_direction == constants.EAST):
                if a == x and b < y:
                    area = "CR1"
                    self.CR1(a, b, x, y)
                elif a < x and b == y:
                    area = "CR2"
                    self.CR2(a, b, x, y)
                elif a == x and b > y:
                    area = "CR3"
                    self.CR3(a, b, x, y)
                elif a > x and b == y:
                    area = "CR4"
                    self.CR4(a, b, x, y)
                elif a < x and b < y:
                    area = "CR5"
                    self.CR5(a, b, x, y)
                elif a < x and b > y:
                    area = "CR6"
                    self.CR6(a, b, x, y)
                elif a > x and b > y:
                    area = "CR7"
                    self.CR7(a, b, x, y)
                elif a > x and b < y:
                    area = "CR8"
                    self.CR8(a, b, x, y)
                elif a == x and b == y:
                    area = "CR9"
                    self.CR9(a, b, x, y)
                else:
                    area = "CR0"
            elif (robot_direction - target_direction == -90) or \
                    (target_direction == constants.EAST and robot_direction == constants.SOUTH):
                if a == x and b < y:
                    area = "DR1"
                    self.DR1(a, b, x, y)
                elif a < x and b == y:
                    area = "DR2"
                    self.DR2(a, b, x, y)
                elif a == x and b > y:
                    area = "DR3"
                    self.DR3(a, b, x, y)
                elif a > x and b == y:
                    area = "DR4"
                    self.DR4(a, b, x, y)
                elif a < x and b < y:
                    area = "DR5"
                    self.DR5(a, b, x, y)
                elif a < x and b > y:
                    area = "DR6"
                    self.DR6(a, b, x, y)
                elif a > x and b > y:
                    area = "DR7"
                    self.DR7(a, b, x, y)
                elif a > x and b < y:
                    area = "DR8"
                    self.DR8(a, b, x, y)
                elif a == x and b == y:
                    area = "DR9"
                    self.DR9(a, b, x, y)
                else:
                    area = "DR0"

            # Reset robot position
            self.robot.correct_coords_and_angle(initial_dir, self.grid.grid_to_pixel((initial_x, initial_y)))

            constants.IS_CHECKING = False
            print("Check-pass:", area)
            return True
            # return False

        except CheckingException:
            # Reset robot position
            self.robot.correct_coords_and_angle(initial_dir, self.grid.grid_to_pixel((initial_x, initial_y)))

            constants.IS_CHECKING = False
            print("Check-fail:", area)
            return False
