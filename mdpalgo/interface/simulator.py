import logging
import os
import queue
import threading

from mdpalgo import constants
import pygame
from mdpalgo.algorithm.astar import AStar
from mdpalgo.algorithm.astar_hamiltonian import AStarHamiltonian
from mdpalgo.algorithm.hamiltonian_path_planners import ExhaustiveHamiltonianPathPlanner
from mdpalgo.algorithm.path_planning import PathPlan
from mdpalgo.communication.comms import AlgoClient
from mdpalgo.communication.message_parser import MessageParser, MessageType, TaskType
from mdpalgo.interface.panel import Panel
from mdpalgo.map.grid import Grid
from mdpalgo.robot.robot import Robot
# import mdpalgo.image # TODO: configure the path to save image
from imagerec.helpers import get_path_to

# for image recognition
from imagerec.infer import infer, get_image_from

# Set the HEIGHT and WIDTH of the screen
WINDOW_SIZE = [960, 660]


class Simulator:

    def __init__(self):
        self.comms = None

        # Initialize pygame
        self.root = pygame
        self.root.init()
        self.root.display.set_caption("MDP Algorithm Simulator")
        self.screen = None
        if not constants.HEADLESS:
            self.screen = pygame.display.set_mode(WINDOW_SIZE)
            self.screen.fill(constants.GRAY)

        # Callback methods queue - for passing of callback functions from worker thread to main UI thread
        self.callback_queue = queue.Queue()

        # Astar class
        self.astar = None
        # Path planner class
        self.path_planner = None
        # Astar hamiltonian class
        self.astar_hamiltonian = None
        # Hamiltonian path planner class
        self.hamiltonian_path_planner = None

        # Initialise 20 by 20 Grid
        self.grid = Grid(20, 20, 20)
        # self.grid.update_grid(self.screen)
        # Outline Grid
        self.grid_surface = self.root.Surface((442, 442))
        self.grid_surface.fill(constants.BLACK)
        if not constants.HEADLESS:
            self.screen.blit(self.grid_surface, (120, 120))
        # Draw the grid
        self.grid.update_grid(self.screen)

        # Initialise side panel with buttons
        self.panel = Panel(self.screen)

        # Used to manage how fast the screen updates
        # self.clock = pygame.time.Clock()
        self.startTime = pygame.time.get_ticks() / 1000
        self.ticks = 0

        # Car printing process
        current_dir = os.path.dirname(os.path.abspath(__file__))
        image_path = os.path.join(current_dir, "car.png")
        car_image = pygame.image.load(image_path)
        self.car = Robot(self, self.screen, self.grid, self.grid_surface, constants.ROBOT_W, constants.ROBOT_H,
                         constants.ROBOT_STARTING_X, constants.ROBOT_STARTING_Y, constants.ROBOT_STARTING_ANGLE,
                         car_image)
        # Draw the car
        self.car.draw_car()

        # parser to parse messages from RPi
        self.parser = MessageParser()

        # TODO: configure the image path
        self.image_folder = get_path_to(mdpalgo.image)

    def run(self):
        # Loop until the user clicks the close button.
        done = False

        # -------- Main Program Loop -----------
        if constants.HEADLESS:  # to simplify implementation, we use 2 threads even if headless
            print("Waiting to connect")
            self.start_algo_client()
            while True:
                try:
                    self.handle_worker_callbacks()
                except queue.Empty:  # raised when queue is empty
                    continue

        else:
            while not done:
                # Check for callbacks from worker thread
                while True:
                    try:
                        self.handle_worker_callbacks()
                    except queue.Empty:  # raised when queue is empty
                        break

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        done = True
                    elif event.type == pygame.MOUSEBUTTONDOWN:
                        # User clicks the mouse. Get the position
                        pos = pygame.mouse.get_pos()
                        if (constants.min_pixel_pos_x < pos[0] < constants.max_pixel_pos_x) and (
                            constants.min_pixel_pos_y < pos[1] < constants.max_pixel_pos_y
                        ):  # if area clicked is within grid
                            self.grid.grid_clicked(pos[0], pos[1])
                            self.screen.blit(self.grid_surface, (120, 120))  # Redraw the grid outlines
                            self.grid.update_grid(self.screen)  # Update grid if obstacles added
                            self.car.draw_car()  # Redraw the car

                        else:  # otherwise, area clicked is outside of grid
                            self.check_button_clicked(pos)

                # Limit to 20 frames per second
                now = pygame.time.get_ticks() / 1000
                if now - self.startTime > 1 / constants.FPS:
                    self.startTime = now
                    self.root.display.flip()

        # Be IDLE friendly. If you forget this line, the program will 'hang' on exit.
        self.root.quit()

    def start_algo_client(self):
        """Connect to RPi wifi server and start a thread to receive messages """
        self.comms = AlgoClient()
        self.comms.connect()
        self.recv_thread = threading.Thread(target=self.receiving_process)
        constants.RPI_CONNECTED = True
        self.recv_thread.start()

    def handle_worker_callbacks(self):
        """Check for callbacks from worker thread and handle them

        Raises:
            queue.Empty if the callback queue is empty
        """
        callback = self.callback_queue.get(False)  # doesn't block
        if isinstance(callback, list):
            logging.info("Current callback: \n%s", callback)
            callback[0](callback[1])
        else:
            callback()

    def receiving_process(self):
        """
        Method to be run in a separate thread to listen for commands from the socket
        Methods that update the UI must be passed into self.callback_queue for running in the main UI thread
        Running UI updating methods in a worker thread will cause a flashing effect as both threads attempt to update the UI
        """

        while constants.RPI_CONNECTED:
            try:
                txt = self.comms.recv()
                if (txt == None):
                    continue

                message_dict = self.parser.parse(txt)
                message_data = message_dict["data"]
                if message_dict["type"] == MessageType.START_TASK:  # From Android
                    self.on_receive_start_task_message(message_data)

                elif message_dict["type"] == MessageType.UPDATE_ROBOT_POSE:
                    print("Received updated robot pose")
                    # E.g. ROBOT/NEXT/3,3,90 or ROBOT/NEXT/NIL
                    status = message_data["status"]
                    robot_pos = message_data["robot"]
                    if status == "DONE":
                        self.callback_queue.put(self.path_planner.send_to_rpi)
                    else:
                        robot_x = robot_pos["x"]
                        robot_y = robot_pos["y"]
                        robot_dir = robot_pos["dir"]
                        self.callback_queue.put(
                            [self.path_planner.send_to_rpi_recalculated, [robot_x, robot_y, robot_dir]])

                elif message_dict["type"] == MessageType.IMAGE_TAKEN:
                    self.on_receive_image_taken_message(self, message_data)

            except (IndexError, ValueError) as e:
                self.comms.send("Invalid command: " + txt)
                print("Invalid command: " + txt)

    def on_receive_image_taken_message(self, data_dict: dict):
        image = data_dict["image"]
        target_id = infer(image)
        # TODO: choose the image save path

        # image.save(some image path here)


    def on_receive_start_task_message(self, data_dict: dict):
        task = data_dict["task"]

        if task == TaskType.TASK_EXPLORE:  # Week 8 Task
            # Reset first
            self.callback_queue.put(self.reset_button_clicked)

            # Set robot starting pos
            robot_params = data_dict['robot']
            logging.info("Setting robot position: %s", robot_params)
            robot_x, robot_y, robot_dir = int(robot_params["x"]), int(robot_params["y"]), int(robot_params["dir"])

            self.callback_queue.put([self.car.update_robot, [robot_dir, self.grid.grid_to_pixel((robot_x, robot_y))]])
            self.callback_queue.put(self.car.redraw_car)

            # Create obstacles given parameters
            logging.info("Creating obstacles...")
            for obstacle in data_dict["obs"]:
                logging.info("Obstacle: %s", obstacle)
                id, grid_x, grid_y, dir = obstacle["id"], int(obstacle["x"]), int(obstacle["y"]), int(obstacle["dir"])
                self.callback_queue.put([self.grid.create_obstacle, [grid_x, grid_y, dir]])

            # Update grid, start explore
            self.callback_queue.put(self.car.redraw_car)

            logging.info("[AND] Doing path calculation...")
            self.callback_queue.put(self.start_button_clicked)

        elif task == TaskType.TASK_PATH:  # Week 9 Task
            pass

    def reprint_screen_and_buttons(self):
        self.screen.fill(constants.GRAY)
        self.panel.redraw_buttons()

    def check_button_clicked(self, pos):
        # Check if start button was pressed first:
        start_button = self.panel.buttons[-1]
        x, y, l, h = start_button.get_xy_and_lh()
        if (x < pos[0] < (l + x)) and (y < pos[1] < (h + y)):
            self.start_button_clicked()
            return

        for button in self.panel.buttons[0:-1]:
            x, y, l, h = button.get_xy_and_lh()
            if (x < pos[0] < (l + x)) and (y < pos[1] < (h + y)):
                button_func = self.panel.get_button_clicked(button)
                if button_func == "RESET":
                    print("Reset button pressed.")
                    self.reset_button_clicked()
                if button_func == "CONNECT":
                    print("Connect button pressed.")
                    self.comms = AlgoClient()
                    self.comms.connect()
                    self.recv_thread = threading.Thread(target=self.receiving_process)
                    constants.RPI_CONNECTED = True
                    self.recv_thread.start()
                elif button_func == "DISCONNECT":
                    print("Disconnect button pressed.")
                    self.comms.disconnect()
                    constants.RPI_CONNECTED = False
                    self.comms = None

                # for testing purposes
                elif button_func == "FORWARD":
                    self.car.move_forward()
                elif button_func == "BACKWARD":
                    self.car.move_backward()
                elif button_func == "FORWARD_RIGHT":
                    self.car.move_forward_steer_right()
                elif button_func == "FORWARD_LEFT":
                    self.car.move_forward_steer_left()
                elif button_func == "BACKWARD_RIGHT":
                    self.car.move_backward_steer_right()
                elif button_func == "BACKWARD_LEFT":
                    self.car.move_backward_steer_left()
                else:
                    return
            else:
                pass

    def start_button_clicked(self):
        print("START button clicked!")

        # Get fastest route (currently not using this)
        self.astar = AStar(self.grid, self.car.grid_x, self.car.grid_y)
        # fastest_route = self.astar.get_astar_route()
        # logging.info("Astar route: " + str(fastest_route))

        # Get fastest route using AStar Hamiltonian
        if len(self.grid.get_target_locations()) != 0:
            self.astar_hamiltonian = AStarHamiltonian(self.grid, self.car.grid_x, self.car.grid_y)
            graph = self.astar_hamiltonian.create_graph()
            self.hamiltonian_path_planner = ExhaustiveHamiltonianPathPlanner(graph, "start")
            shortest_path, path_length = self.hamiltonian_path_planner.find_path()
            fastest_route = self.astar_hamiltonian.convert_shortest_path_to_ordered_targets(shortest_path)
            logging.info("Astar route: " + str(fastest_route))

            optimized_fastest_route = self.grid.get_optimized_target_locations(fastest_route)
            self.car.optimized_target_locations = optimized_fastest_route[1:]
            logging.info("Optimized Astar route: " + str(optimized_fastest_route))

            # Path finding
            self.path_planner = PathPlan(self, self.grid, self.car, optimized_fastest_route)
            self.path_planner.start_robot()

        # if constants.RPI_CONNECTED:
        #     self.path_planner.send_to_rpi()

    def reset_button_clicked(self):
        self.grid.reset(self.screen)
        self.car.reset()

if __name__ == "__main__":
    # Set info logging mode
    logging.basicConfig(level=logging.INFO)

    x = Simulator()

    # Test the method to parse Android messages
    message = "START/EXPLORE/(R,1,1,0)/(00,04,15,-90)/(01,16,17,90)/(02,12,11,180)/(03,07,03,0)/(04,17,04,90)"
    data_dict = x.parser.parse(message)["data"]
    # Test the threading without Android connected
    thread = threading.Thread(target=lambda: x.on_receive_start_task_message(data_dict))

    # TODO: test the receiving image function
    # data_dict = {"image": Image object}
    thread.start()
    x.run()
