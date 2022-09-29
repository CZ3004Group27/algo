import logging
import os
from mdpalgo import constants
from mdpalgo.communication.comms import AlgoClient
from mdpalgo.communication.message_parser import MessageParser, MessageType, TaskType
from imagerec.infer import infer
from imagerec.helpers import get_path_to

import fastestalgo.images

class Week9Task:
    def __init__(self):
        self.comms = None
        self.parser = MessageParser()
        self.image_folder = get_path_to(fastestalgo.images)
        self.obstacle_id = 1 # obstacle_id = 1 or 2
        self.no_image_result_count = 0

    def start_algo_client(self):
        """Connect to RPi wifi server"""
        self.comms = AlgoClient()
        self.comms.connect()
        constants.RPI_CONNECTED = True

    def run(self):
        """Listen to RPI for image data and send image rec result"""
        self.start_algo_client()
        while constants.RPI_CONNECTED:
            try:
                txt = self.comms.recv()

                if txt is None:
                    continue

                message_dict = self.parser.parse(txt)
                message_data = message_dict["data"]
                if message_dict["type"] == MessageType.IMAGE_TAKEN:
                    self.on_receive_image_taken_message(message_data)

            except (IndexError, ValueError) as e:
                self.comms.send("Invalid command: " + txt)
                print("Invalid command: " + txt)

    def on_receive_image_taken_message(self, data_dict: dict):
        image = data_dict["image"]
        infer_result = infer(image)
        try:
            target_id = self.check_infer_result(infer_result)

            # reset exception count if there is an image result returned after retaking photo once
            if self.no_image_result_count == 1:
                self.no_image_result_count = 0

        except Exception as e:
            logging.exception(e)
            self.no_image_result_count += 1

            # if no image result for 2 times, return early to prevent request photo loop
            if self.no_image_result_count == 2:
                self.no_image_result_count = 0
                return

            self.request_photo_from_rpi() # take photo again if exception raised
            return

        # get list of images
        list_of_images = list(self.image_folder.glob("*.jpg"))
        print("List of images:", list_of_images)

        # set image name
        if len(list_of_images) == 0:
            image_name = "img_1"
        else:
            latest_image = max(list_of_images, key=os.path.getctime)
            print("Latest:", latest_image)
            previous_image_name = latest_image.stem
            print("Previous image name:", previous_image_name)
            image_number = int(previous_image_name.split("_")[-1]) + 1
            image_name = "img_" + str(image_number)

        print("Image name:", image_name)
        image.save(self.image_folder.joinpath(f"{image_name}.jpg"))
        image_result_string = self.get_image_result_string(target_id)

        # send image result string to rpi
        self.comms.send(image_result_string)

        # change obstacle_id to 2 after sending first image result
        if self.obstacle_id == 1:
            self.obstacle_id = 2

    def check_infer_result(self, infer_result: list):
        # remove all elements in infer_result that are "Bullseye"
        result = [elem for elem in infer_result if elem != "Bullseye"]

        # if all elements in list are "Bullseye", raise exception
        if len(result) == 0:
            raise Exception("No image result")
        # get first element that is not "Bullseye"
        else:
            return result[0]

    def get_image_result_string(self, target_id):
        image_result_list = ["TARGET", target_id, self.obstacle_id]
        return '/'.join([str(elem) for elem in image_result_list])

    def request_photo_from_rpi(self):
        self.comms.send(self.get_take_photo_string())

    def get_take_photo_string(self):
        photo_list = ["PHOTO", self.obstacle_id]
        return '/'.join([str(elem) for elem in photo_list])

if __name__ == "__main__":
    X = Week9Task
    X.run()
