#!/usr/bin/python3

import arm.interface

def move(interface, coordinates):
    if len(coordinates) != 3:
        return False
    interface.move_to(coordinates, unit="legos")
    return True

def main():
    interface = arm.interface.ArmController(serial_comms=True, positions_file='arm/positions.pickle')
    while True:
        # wait for commands
        cmd = input('(move, close, open) >> ')
        args = cmd.split()
        if args[0] == 'move':
            try:
                coordinates = [int(x) for x in args[1:]]
            except ValueError:
                continue
            move(interface, coordinates)
        elif args[0] == 'close':
            interface.close_claw()
        elif args[0] == 'open':
            interface.open_claw()

if __name__ == "__main__":
    main()
