from mdpalgo.interface.buttons import Button
from mdpalgo import constants
import pygame


class Panel(object):

    def __init__(self):
        self.surface = pygame.Surface((200, 420))
        self.surface.fill(constants.GRAY)

        # Buttons
        self.buttons = []

        connect_button = Button(self.surface, constants.LIGHT_BLUE, 0, 0, 110, 25, "Connect to RPI", constants.BLACK, "CONNECT")
        self.buttons.append(connect_button)
        disconnect_button = Button(self.surface, constants.LIGHT_BLUE, 0, 30, 150, 25, "Disconnect from RPI", constants.BLACK, "DISCONNECT")
        self.buttons.append(disconnect_button)

        # For testing
        forward_button = Button(self.surface, constants.LIGHT_GREEN, 0, 90, 100, 25, "Forward", constants.BLACK, "FORWARD")
        self.buttons.append(forward_button)
        backward_button = Button(self.surface, constants.LIGHT_GREEN, 0, 120, 100, 25, "Backward", constants.BLACK, "BACKWARD")
        self.buttons.append(backward_button)
        for_right_button = Button(self.surface, constants.LIGHT_GREEN, 0, 150, 100, 25, "Forward R", constants.BLACK, "FORWARD_RIGHT")
        self.buttons.append(for_right_button)
        for_left_button = Button(self.surface, constants.LIGHT_GREEN, 0, 180, 100, 25, "Forward L", constants.BLACK, "FORWARD_LEFT")
        self.buttons.append(for_left_button)
        back_right_button = Button(self.surface, constants.LIGHT_GREEN, 0, 210, 100, 25, "Backward R", constants.BLACK, "BACKWARD_RIGHT")
        self.buttons.append(back_right_button)
        back_left_button = Button(self.surface, constants.LIGHT_GREEN, 0, 240, 100, 25, "Backward L", constants.BLACK, "BACKWARD_LEFT")
        self.buttons.append(back_left_button)

        reset_button = Button(self.surface, constants.RED, 0, 350, 100, 25, "Reset", constants.BLACK, "RESET")
        self.buttons.append(reset_button)
        start_button = Button(self.surface, constants.GREEN, 0, 380, 100, 25, "START", constants.BLACK, "START")
        self.buttons.append(start_button)

    def redraw_buttons(self):
        for button in self.buttons:
            button.draw_button(button.surface, button.color, button.length, button.height, button.x, button.y, button.width)
            button.write_text(button.surface, button.text, button.text_color, button.length, button.height, button.x, button.y)

    def button_clicked(self, button):
        return button.pressed()

    def get_button_clicked(self, button):
        return button.get_function()

    def get_surface(self):
        return self.surface
