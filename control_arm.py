#!/usr/bin/python3

import arm.interface

def move(interface, coordinates):
    if len(coordinates) != 3:
        return False
    interface.move_to(coordinates, unit="legos")
    interface.send_to_arduino()
    return True

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
                coordinates = [int(x) for x in args[1:]]
            except ValueError:
                continue
            move(interface, tuple(coordinates))
        elif args[0] == "manual":
            try:
                coordinates = [int(x) for x in args[1:]]
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
