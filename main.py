import logging
import argparse
import constants

from interface.simulator import Simulator

# Logging
logging.basicConfig(level=logging.INFO)

# parse the arguments
parser = argparse.ArgumentParser()
parser.add_argument("--hl", help="run in headless mode", action="store_true")
parser.add_argument("--cen", help="center pathing on obstacle",
                    action="store_true")

def main():
    constants.HEADLESS = False
    constants.CENTER_ON_OBS = False
    args = parser.parse_args()
    if args.hl:
        constants.HEADLESS = True
        print("Running in headless mode")
    if args.cen:
        constants.CENTER_ON_OBS = True
        print("Pathing will center on obstacle")
    x = Simulator()
    x.run()



if __name__ == '__main__':
    main()
