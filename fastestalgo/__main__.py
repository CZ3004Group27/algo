from fastestalgo.week9task import Week9Task
from mdpalgo import constants

import argparse
# parse the arguments
parser = argparse.ArgumentParser()
parser.add_argument("--testwifi", help="use test wifi server on PC instead of real rpi",
                action="store_true")
args = parser.parse_args()

if args.testwifi:
    constants.WIFI_IP = constants.TEST_IP
    print("Use local IP address for integration testing without RPi")

x = Week9Task()
x.run()
