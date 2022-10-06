from queue import PriorityQueue
import numpy as np

from mdpalgo import constants
from mdpalgo.map.cell import Cell, CellStatus
from mdpalgo.map.configuration import Pose
from enum import Enum

class RobotMovement(Enum):
    FORWARD = "F"
    BACKWARD = "B"
    FORWARD_RIGHT = "FR"
    FORWARD_LEFT = "FL"
    BACKWARD_RIGHT = "BR"
    BACKWARD_LEFT = "BL"

class Node:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


# This function return the path of the search
def return_path(current_node, maze):
    path = []
    no_rows, no_columns = np.shape(maze)
    # here we create the initialized result maze with -1 in every position
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    start_value = 5
    # we update the path of start to end found by A-star search with every step incremented by 1
    for i in range(len(path)):
        result[int(path[i][0])][int(path[i][1])] = start_value
        start_value += 1
    return (result, path)


# maze is in grid.cells_virtual
# start consists of [row, column, dir]  usually would be start = [18, 1, 0]
# target consists of [row, column, dir]
# dir is according to constants.py file
def search(maze, cost, start, end):
    """
        Returns a list of tuples as a path from the given start to the given end in the given maze
        :param maze:
        :param cost
        :param start:
        :param end:
        :return:
    """

    # Create start and end node with initialized values for g, h and f
    start_node = Node(None, tuple(start))
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, tuple(end))
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both yet_to_visit and visited list
    # in this list we will put all node that are yet_to_visit for exploration.
    # From here we will find the lowest cost node to expand next
    yet_to_visit_list = []
    # in this list we will put all node those already explored so that we don't explore it again
    visited_list = []

    # Add the start node
    yet_to_visit_list.append(start_node)

    # Adding a stop condition. This is to avoid any infinite loop and stop
    # execution after some reasonable number of steps
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # what squares do we search . search movement is left-right-top-bottom
    # (4 movements) from every position

    # [row, column]
    move = [[-1, 0],  # go up
            [0, 1],  # go right
            [1, 0],  # go down
            [0, -1]]  # go left

    """
        1) We first get the current node by comparing all f cost and selecting the lowest cost node for further expansion
        2) Check max iteration reached or not. Set a message and stop execution
        3) Remove the selected node from yet_to_visit list and add this node to visited list
        4) Perform Goal test and return the path else perform below steps
        5) For selected node find out all children (use move to find children)
            a) get the current position for the selected node (this becomes parent node for the children)
            b) check if a valid position exist (boundary will make few nodes invalid)
            c) if any node is a wall then ignore that
            d) add to valid children node list for the selected parent

            For all the children node
                a) if child in visited list then ignore it and try next node
                b) calculate child node g, h and f values
                c) if child in yet_to_visit list then ignore it
                d) else move the child to yet_to_visit list
    """
    # find maze has got how many rows and columns
    no_rows, no_columns = np.shape(maze)

    # Loop until you find the end

    while len(yet_to_visit_list) > 0:

        # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
        outer_iterations += 1

        # Get the current node
        current_node = yet_to_visit_list[0]
        current_index = 0
        for index, item in enumerate(yet_to_visit_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # if we hit this point return the path such as it may be no solution or
        # computation cost is too high
        if outer_iterations > max_iterations:
            print("Giving up on pathfinding too many iterations")
            return return_path(current_node, maze)

        # Pop current node out off yet_to_visit list, add to visited list
        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)

        # test if goal is reached or not, if yes then return the path
        if current_node.position[0] == end_node.position[0] and current_node.position[1] == end_node.position[1] and current_node.position[2] == end_node.position[2]:
            return return_path(current_node, maze)

        # Generate children from all adjacent squares
        children = []
        i = 0

        for new_position in move:
            direction = [constants.NORTH, constants.EAST, constants.SOUTH, constants.WEST]

            # Get node position
            node_position = [
                current_node.position[0] + new_position[0], current_node.position[1] + new_position[1], direction[i]]
            i += 1
            # Make sure within range (check if within maze boundary)
            if (node_position[0] > (no_rows - 2) or
                    node_position[0] < 1 or
                    node_position[1] > (no_columns - 2) or
                    node_position[1] < 1):
                continue

            # Make sure walkable terrain
            if maze[int(node_position[0])][int(node_position[1])] in [CellStatus.BOUNDARY, CellStatus.OBS]:
                continue

            # Create new node
            new_node = Node(current_node, node_position)
            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            tempCost = cost

            # Child is on the visited list (search entire visited list)
            if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                continue

            # # Increase the cost of turning by 2x
            # # Robot facing North or South and making Left/Right turns
            # if child.parent.position[2] in [constants.NORTH, constants.SOUTH]:
            #     if [child.position[0] - child.parent.position[0],
            #             child.position[1] - child.parent.position[1]] == move[1]:
            #         tempCost *= 2
            #         child.position[2] = constants.EAST
            #     elif [child.position[0] - child.parent.position[0],
            #           child.position[1] - child.parent.position[1]] == move[3]:
            #         tempCost *= 2
            #         child.position[2] = constants.WEST
            # # Robot facing East or West and making Left/Right turns
            # elif child.parent.position[2] in [constants.EAST, constants.WEST]:
            #     if [child.position[0] - child.parent.position[0],
            #             child.position[1] - child.parent.position[1]] == move[0]:
            #         tempCost *= 2
            #         child.position[2] = constants.NORTH
            #     elif [child.position[0] - child.parent.position[0],
            #           child.position[1] - child.parent.position[1]] == move[2]:
            #         tempCost *= 2
            #         child.position[2] = constants.SOUTH

            # Increase the cost of turning by 2x
            # Robot facing North or South and making Left/Right turns
            if child.parent.position[2] in [constants.NORTH]:
                if [child.position[0] - child.parent.position[0],
                    child.position[1] - child.parent.position[1]] == move[1]:
                    tempCost *= 3
                    if child.parent.parent and [child.parent.position[0] - child.parent.parent.position[0],
                    child.parent.position[1] - child.parent.parent.position[1]] == move[2]:  # if prev movement was backwards, this is a backward turn
                        child.position[2] = constants.WEST
                    else:
                        child.position[2] = constants.EAST
                elif [child.position[0] - child.parent.position[0],
                      child.position[1] - child.parent.position[1]] == move[3]:
                    tempCost *= 3
                    if child.parent.parent and [child.parent.position[0] - child.parent.parent.position[0],
                    child.parent.position[1] - child.parent.parent.position[1]] == move[2]:
                        child.position[2] = constants.EAST
                    else:
                        child.position[2] = constants.WEST
                # Backward movement
                elif [child.position[0] - child.parent.position[0],
                      child.position[1] - child.parent.position[1]] == move[2]:
                    child.position[2] = constants.NORTH
            elif child.parent.position[2] in [constants.SOUTH]:
                if [child.position[0] - child.parent.position[0],
                    child.position[1] - child.parent.position[1]] == move[1]:
                    tempCost *= 3
                    if child.parent.parent and [child.parent.position[0] - child.parent.parent.position[0],
                    child.parent.position[1] - child.parent.parent.position[1]] == move[0]:
                        child.position[2] = constants.WEST
                    else:
                        child.position[2] = constants.EAST
                elif [child.position[0] - child.parent.position[0],
                      child.position[1] - child.parent.position[1]] == move[3]:
                    tempCost *= 3
                    if child.parent.parent and [child.parent.position[0] - child.parent.parent.position[0],
                    child.parent.position[1] - child.parent.parent.position[1]] == move[0]:
                        child.position[2] = constants.EAST
                    else:
                        child.position[2] = constants.WEST
                # Backward movement
                elif [child.position[0] - child.parent.position[0],
                      child.position[1] - child.parent.position[1]] == move[0]:
                    child.position[2] = constants.SOUTH
            # Robot facing East or West and making Left/Right turns
            elif child.parent.position[2] in [constants.EAST]:
                if [child.position[0] - child.parent.position[0],
                    child.position[1] - child.parent.position[1]] == move[0]:
                    tempCost *= 3
                    if child.parent.parent and [child.parent.position[0] - child.parent.parent.position[0],
                    child.parent.position[1] - child.parent.parent.position[1]] == move[3]:
                        child.position[2] = constants.SOUTH
                    else:
                        child.position[2] = constants.NORTH
                elif [child.position[0] - child.parent.position[0],
                      child.position[1] - child.parent.position[1]] == move[2]:
                    tempCost *= 3
                    if child.parent.parent and [child.parent.position[0] - child.parent.parent.position[0],
                    child.parent.position[1] - child.parent.parent.position[1]] == move[3]:
                        child.position[2] = constants.NORTH
                    else:
                        child.position[2] = constants.SOUTH
                # Backward movement
                elif [child.position[0] - child.parent.position[0],
                      child.position[1] - child.parent.position[1]] == move[3]:
                    child.position[2] = constants.EAST
            elif child.parent.position[2] in [constants.WEST]:
                if [child.position[0] - child.parent.position[0],
                    child.position[1] - child.parent.position[1]] == move[0]:
                    tempCost *= 3
                    if child.parent.parent and [child.parent.position[0] - child.parent.parent.position[0],
                    child.parent.position[1] - child.parent.parent.position[1]] == move[1]:
                        child.position[2] = constants.SOUTH
                    else:
                        child.position[2] = constants.NORTH
                elif [child.position[0] - child.parent.position[0],
                      child.position[1] - child.parent.position[1]] == move[2]:
                    tempCost *= 3
                    if child.parent.parent and [child.parent.position[0] - child.parent.parent.position[0],
                    child.parent.position[1] - child.parent.parent.position[1]] == move[1]:
                        child.position[2] = constants.NORTH
                    else:
                        child.position[2] = constants.SOUTH
                # Backward movement
                elif [child.position[0] - child.parent.position[0],
                      child.position[1] - child.parent.position[1]] == move[1]:
                    child.position[2] = constants.WEST

            # Create the f, g, and h values
            child.g = current_node.g + tempCost

            # Heuristic costs calculated here, this is using MANHATTAN distance
            child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])

            # Heuristic costs calculated here, this is using EUCLIDEAN distance
            # child.h = (((child.position[0] - end_node.position[0]) ** 2) +
            #            ((child.position[1] - end_node.position[1]) ** 2))

            child.f = child.g + child.h

            # Child is already in the yet_to_visit list and g cost is already lower
            if len([i for i in yet_to_visit_list if child == i and child.g > i.g]) > 0:
                continue

            # Add the child to the yet_to_visit list
            yet_to_visit_list.append(child)

class ImprovedNode:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """

    def __init__(self, parent=None, pose:list=None):
        self.parent = parent
        self.pose = Pose(pose)
        self.move_from_parent: RobotMovement = None
        self.displacement_from_parent: np.ndarray

        self.g = 0
        self.h = 0
        self.f = 0

class AutoPlanner():
    def __init__(self):
        # map movement to a relative vector wrt the current direction
        self.map_move_to_relative_displacement =  {
            RobotMovement.FORWARD: [0, 1],
            RobotMovement.BACKWARD: [0, -1],
            RobotMovement.FORWARD_RIGHT: [3, 3],
            RobotMovement.FORWARD_LEFT: [-3, 3],
            RobotMovement.BACKWARD_RIGHT: [3, -3],
            RobotMovement.BACKWARD_LEFT: [-3, -3],
        }
        self.map_move_to_relative_direction = {
            RobotMovement.FORWARD: constants.NORTH,
            RobotMovement.BACKWARD: constants.NORTH,
            RobotMovement.FORWARD_RIGHT: constants.EAST,
            RobotMovement.FORWARD_LEFT: constants.WEST,
            RobotMovement.BACKWARD_RIGHT: constants.WEST,
            RobotMovement.BACKWARD_LEFT: constants.EAST,
        }
        self.turning_moves = {
            RobotMovement.FORWARD_RIGHT,
            RobotMovement.FORWARD_LEFT,
            RobotMovement.BACKWARD_RIGHT,
            RobotMovement.BACKWARD_LEFT,
        }

        self.collision_statuses = [CellStatus.BOUNDARY, CellStatus.OBS, CellStatus.VISITED_OBS]

        # change the current direction to rotation matrix
        self.direction_to_rotation_matrixes = {
            constants.NORTH: np.array([[1, 0],
                                       [0, 1]]),
            constants.SOUTH: np.array([[-1, 0],
                                       [0, -1]]),
            constants.EAST: np.array([[0, 1],
                                      [-1, 0]]),
            constants.WEST: np.array([[0, -1],
                                      [1, 0]]),
        }

        self.direction_to_unit_forward_vector = {
            constants.NORTH: np.array([0, 1]),
            constants.SOUTH: np.array([0, -1]),
            constants.EAST: np.array([1, 0]),
            constants.WEST: np.array([-1, 0]),
        }

        # turning cost = straight cost * turning factor
        self.turning_factor = 5 # assume perfect circle, about 4.71

        self.node_index_in_yet_to_visit = 2

    def initialize_node(self, node_position: list) -> ImprovedNode:
        """f, h, g are all initialized to 0"""
        return ImprovedNode(None, tuple(node_position))

    def initialize_counters(self):
        """Initialize the counters used to halt the algorithm"""
        self.outer_iterations = 0

    def increment_counters(self):
        self.outer_iterations += 1

    def set_max_iterations(self):
        self.max_iterations = (len(self.maze) // 2) ** 10

    def does_iterations_exceed_max(self):
        return self.outer_iterations > self.max_iterations

    def initialize_yet_to_visit(self):
        # in this list we will put all node that are yet_to_visit for exploration.
        # From here we will find the lowest cost node to expand next
        self.yet_to_visit = PriorityQueue()

    def initialize_visited_nodes(self):
        # we will put all node those already explored so that we
        # don't explore it again
        # dictionary (x, y, dir) map
        self.visited_list = set()

    def add_node_to_yet_to_visit(self, node: ImprovedNode):
        self.yet_to_visit.put((node.f, self.get_id(), node))

    def reset_id(self):
        # id for the node in the yet_to_visit_queue
        self.id = 0

    def get_id(self):
        self.id += 1
        return self.id

    def get_node_from_yet_to_visit(self) -> ImprovedNode:
        """Remove and return the node with lowest f from queue"""
        return self.yet_to_visit.get()[self.node_index_in_yet_to_visit]

    def is_yet_to_visit_empty(self):
        return self.yet_to_visit.empty()

    def set_maze(self, maze):
        self.maze = maze
        self.get_maze_sizes()

    def get_maze_sizes(self):
        self.size_x, self.size_y = np.shape(self.maze)

    def is_in_collision(self, node: ImprovedNode) -> bool:
        return self.maze[node.pose.x][node.pose.y] in self.collision_statuses

    def is_node_unreachable_from_parent(self, node: ImprovedNode) -> bool:
        if self.is_in_collision(node):
            return True

        parent_node = node.parent
        move = node.move_from_parent
        displacement_from_current = node.displacement_from_parent

        # for turning move, also check the corner of the move
        if move in self.turning_moves:
            unit_forward_vector = self.direction_to_unit_forward_vector[node.parent.pose.direction]
            corner_displacement = np.matmul(unit_forward_vector, displacement_from_current) * unit_forward_vector
            corner_node = ImprovedNode(None, [corner_displacement[0] + parent_node.pose.x,
                                              corner_displacement[1] + parent_node.pose.y,
                                              constants.NORTH]) # the direction is not important since we only checking collision
            return self.is_in_collision(corner_node)


    def is_goal_reached(self) -> bool:
        for potential_goal_node in self.potential_goal_nodes:
            if self.current_node.pose == potential_goal_node.pose:
                return True
        return False

    def add_node_to_visited(self, node: ImprovedNode):
        self.visited_list.add(node.pose.to_tuple())

    def is_robot_within_boundary_at_node(self, node: ImprovedNode):
        """Check if the robot will be within boundary if staying in this
        node"""
        right_boundary = self.size_x-2 # right most node that robot is not out of maze
        top_boundary = self.size_y-2 # top most node that robot is not out of maze
        return (1 <= node.pose.x <= right_boundary) and (1 <= node.pose.y <= top_boundary)

    def get_children_of_current_node(self):
        # Generate children from all adjacent squares
        self.children_current_node = []
        i = 0

        for move in self.map_move_to_relative_displacement:
            new_node = self.get_child_node_after_move(move)

            i += 1
            # Make sure within range (check if within maze boundary)
            if not self.is_robot_within_boundary_at_node(new_node):
                continue

            # Make sure node is reachable
            if self.is_node_unreachable_from_parent(new_node):
                continue

            # Append
            self.children_current_node.append(new_node)

    def get_child_node_after_move(self, move: RobotMovement):
        relative_displacement = self.map_move_to_relative_displacement[move]
        displacement_from_current_node = self.get_absolute_vector(relative_displacement, self.current_node.pose.direction)

        relative_direction_from_current_node = self.map_move_to_relative_direction[move]
        # Get node position
        node_position = [
            self.current_node.pose.x + displacement_from_current_node[0],
            self.current_node.pose.y + displacement_from_current_node[1],
            self.get_absolute_direction(relative_direction_from_current_node, self.current_node.pose.direction)]
        child_node = ImprovedNode(self.current_node, node_position)
        child_node.move_from_parent = move
        child_node.displacement_from_parent = displacement_from_current_node
        return child_node

    def get_absolute_direction(self, relative_direction: int, current_direction: int) -> int:
        resultant_direction = relative_direction + current_direction
        if resultant_direction > 180:
            return resultant_direction - 360
        elif resultant_direction <= -180:
            return resultant_direction + 360
        return resultant_direction

    def get_absolute_vector(self, relative_vector: list, current_direction) -> list:
        rotation_matrix = self.direction_to_rotation_matrixes[current_direction]
        return np.matmul(rotation_matrix, relative_vector).astype(int)


    def is_node_already_visited(self, node: ImprovedNode):
        return node.pose.to_tuple() in self.visited_list

    def heuristics(self, node: ImprovedNode):
        """Return estimated cost to go to the end node"""
        xA, yA = node.pose.x, node.pose.y
        xB, yB = self.end_node.pose.x, self.end_node.pose.y
        return abs(xA - xB) + abs(yA - yB)

    def is_node_higher_cost_than_yet_to_visit(self, node: ImprovedNode):
        """Check if node is already in yet to visit and has higher cost than
        stored in yet to visit"""
        for item in self.yet_to_visit.queue:
            node_in_yet_to_visit = item[self.node_index_in_yet_to_visit]
            if node.pose == node_in_yet_to_visit.pose and node.g > node_in_yet_to_visit.g:
                return True
        return False

    def set_straight_cost(self, cost):
        self.straight_cost = cost

    def get_cost_current_node_to_child(self, child_node: ImprovedNode) -> int:
        move_to_child = child_node.move_from_parent
        weighted_cost = self.straight_cost

        if self.is_turning_move(move_to_child):
            weighted_cost *= self.turning_factor

        return weighted_cost

    def is_turning_move(self, move: RobotMovement):
        return move in self.turning_moves

    def set_neighbours_as_potential_goal_nodes(self):
        """Set the neighbouring nodes of the end nodes as potential targets"""
        target_x = self.end_node.pose.x
        target_y = self.end_node.pose.y
        target_direction = self.end_node.pose.direction

        # Get the coordinates of other neighbour potential target cells
        if target_direction == constants.NORTH:
            neighbour_coords = [(target_x - 1, target_y),
                                (target_x + 1, target_y)]
        elif target_direction == constants.SOUTH:
            neighbour_coords = [(target_x + 1, target_y),
                                (target_x - 1, target_y)]
        elif target_direction == constants.EAST:
            neighbour_coords = [(target_x, target_y + 1),
                                (target_x, target_y - 1)]
        elif target_direction == constants.WEST:
            neighbour_coords = [(target_x, target_y - 1),
                                (target_x, target_y + 1)]

        for neighbour_x, neighbour_y in neighbour_coords:
            neighbour_node = ImprovedNode(None, [neighbour_x, neighbour_y, target_direction])
            if self.is_robot_within_boundary_at_node(neighbour_node) and not self.is_in_collision(neighbour_node):
                self.add_potential_goal_node(neighbour_node)

    def reset_potential_goal_nodes(self):
        self.potential_goal_nodes = list()

    def add_potential_goal_node(self, node: ImprovedNode):
        self.potential_goal_nodes.append(node)

    def search_path(self, maze, cost, start, end):
        self.set_maze(maze)
        self.set_straight_cost(cost)
        # Create start and end node with initialized values for g, h and f
        self.start_node = self.initialize_node(start)
        self.end_node = self.initialize_node(end)
        self.reset_potential_goal_nodes()
        self.add_potential_goal_node(self.end_node)
        self.set_neighbours_as_potential_goal_nodes()

        self.initialize_yet_to_visit()
        self.initialize_visited_nodes()
        self.reset_id()

        self.add_node_to_yet_to_visit(self.start_node)

        # Adding a stop condition. This is to avoid any infinite loop and stop
        # execution after some reasonable number of steps
        self.initialize_counters()
        self.set_max_iterations()

        # Loop until you find the end

        while not self.is_yet_to_visit_empty():

            # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
            self.increment_counters()

            self.current_node = self.get_node_from_yet_to_visit()

            # if we hit this point return the path such as it may be no solution or
            # computation cost is too high
            if self.does_iterations_exceed_max():
                print("Giving up on pathfinding too many iterations")
                break

            self.add_node_to_visited(self.current_node)

            # test if goal is reached or not, if yes then return the path
            if self.is_goal_reached():
                return self.reconstruct_path(self.current_node)

            self.get_children_of_current_node()

            # Loop through children
            for child in self.children_current_node:
                # Child is on the visited list (search entire visited list)
                if self.is_node_already_visited(child):
                    continue

                # Create the f, g, and h values
                child.g = self.current_node.g + self.get_cost_current_node_to_child(child)

                # Heuristic costs calculated here, this is using MANHATTAN distance
                child.h = self.heuristics(child)

                child.f = child.g + child.h

                # Child is already in the yet_to_visit list and child has
                # higher g cost than that in yet_to_visit
                if self.is_node_higher_cost_than_yet_to_visit(child):
                    continue

                # Add the child to the yet_to_visit list
                self.add_node_to_yet_to_visit(child)
        return None

    def reconstruct_path(self, current_node: ImprovedNode) -> list:
        """Return a list of string of movements"""
        node = current_node
        path = []

        while node.pose != self.start_node.pose:
            # grow the path backwards and backtrack
            path.append(node.move_from_parent.value)
            node = node.parent

        path.reverse()                 # reverse the path from start to goal
        return path


if __name__ == "__main__":
    _ = CellStatus.EMPTY
    c = CellStatus.EMPTY # current node
    s = CellStatus.EMPTY # start node
    t = CellStatus.EMPTY # target node
    o = CellStatus.OBS
    b = CellStatus.BOUNDARY
    maze = np.array([[_, _, _, _, _, _, _, _, _, _],
                     [_, s, _, _, _, _, _, _, _, _],
                     [_, _, _, _, _, _, b, b, b, _],
                     [_, _, _, _, t, _, b, o, b, _],
                     [_, _, _, _, _, _, b, b, b, _],
                     [_, _, _, _, _, _, _, _, _, _],
                     [_, c, _, _, _, _, _, _, _, _],
                     [_, _, _, _, _, _, _, _, _, _],
                     [_, _, _, _, _, _, _, _, _, _],
                     [_, _, _, _, _, _, _, _, _, _],])
    cost = 10
    start = [1, 1, constants.NORTH]
    end = [3, 4, constants.NORTH]
    auto_planner = AutoPlanner()
    auto_planner.set_maze(maze)
    auto_planner.current_node = ImprovedNode(None, [6, 1, constants.EAST])
    auto_planner.get_children_of_current_node()
    for node in auto_planner.children_current_node:
        print(f"Child: (x, y, dir) = ({node.pose.x}, {node.pose.y}, {node.pose.direction})")
    search_result = auto_planner.search_path(maze, cost, start, end)
    print(search_result)
    assert search_result == ['F', 'F', 'F', 'BR', 'B', 'B', 'FR']

    # test the transformation methods
    relative_vector = auto_planner.map_move_to_relative_displacement[RobotMovement.FORWARD]

    abs_vector = auto_planner.get_absolute_vector(relative_vector, constants.NORTH)
    assert (abs_vector == np.array([0, 1])).all()
    abs_vector = auto_planner.get_absolute_vector(relative_vector, constants.SOUTH)
    assert (abs_vector == np.array([0, -1])).all()
    abs_vector = auto_planner.get_absolute_vector(relative_vector, constants.WEST)
    assert (abs_vector == np.array([-1, 0])).all()
    abs_vector = auto_planner.get_absolute_vector(relative_vector, constants.EAST)
    assert (abs_vector == np.array([1, 0])).all()

    relative_vector = auto_planner.map_move_to_relative_displacement[RobotMovement.BACKWARD_LEFT]

    abs_vector = auto_planner.get_absolute_vector(relative_vector, constants.NORTH)
    assert (abs_vector == np.array([-3, -3])).all()
    abs_vector = auto_planner.get_absolute_vector(relative_vector, constants.SOUTH)
    assert (abs_vector == np.array([3, 3])).all()
    abs_vector = auto_planner.get_absolute_vector(relative_vector, constants.WEST)
    assert (abs_vector == np.array([3, -3])).all()
    abs_vector = auto_planner.get_absolute_vector(relative_vector, constants.EAST)
    assert (abs_vector == np.array([-3, 3])).all()
