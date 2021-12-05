#!/usr/bin/python3

import time
import arm.interface


def main():
    interface = arm.interface.ArmController(serial_comms=True, positions_file='arm/positions.pickle')
    while True:
        # wait for commands
        cmd = input('(move, raise, close, open, manual, reset) >> ')
        if not cmd:
            continue
        args = cmd.split()
        if args[0] == 'move':
            try:
                coordinates = [int(x)+0.5 for x in args[1:]]
            except ValueError:
                continue
            if len(coordinates) != 3:
                continue
            interface.move_to(tuple(coordinates), unit="legos")
            interface.send_to_arduino()
        elif args[0] == 'lock':
            try:
                coordinates = [int(x)+0.5 for x in args[1:]]
            except ValueError:
                continue
            if len(coordinates) != 3:
                continue
            interface.open_claw()
            coordinates[2] += 2
            interface.move_to(tuple(coordinates), unit="legos")
            interface.send_to_arduino()
            time.sleep(2.5)
            interface.close_claw()
            interface.send_to_arduino()
            time.sleep(2.5)
            coordinates[2] -= 4
            interface.move_to(tuple(coordinates), unit="legos")
            interface.send_to_arduino()
            time.sleep(2.5)
            coordinates[2] += 6
            interface.move_to(tuple(coordinates), unit="legos")
            interface.send_to_arduino()
        elif args[0] == "manual":
            try:
                coordinates = [int(x)+0.5 for x in args[1:]]
            except ValueError:
                continue
            for i,s in enumerate(coordinates):
                interface.arm_state[i+1] = s
            interface.send_to_arduino()
        elif args[0] == 'close':
            interface.close_claw()
            interface.send_to_arduino()
        elif args[0] == 'open':
            interface.open_claw()
            interface.send_to_arduino()
        elif args[0] == "reset":
            interface.reset_position()
            interface.send_to_arduino()
        elif args[0] == "raise":
            interface.raise_arm()
            interface.send_to_arduino()
        

if __name__ == "__main__":
    main()
