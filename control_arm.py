#!/usr/bin/python3

import arm.interface
import arm.simulation.rectangular_to_angles as to_angles

def main():
    interface = arm.interface.ArmController(serial_comms=True)

    while True:
        valid = False
        while not valid:
            vals = input('lego coordinate: ')
            try:
                coordinates = [int(x) for x in vals.split()]
            except ValueError:
                continue
            except Exception as e:
                print(e)
            else:
                valid = (len(coordinates) == 3)
        interface.move_to(coordinates, unit="legos")

if __name__ == "__main__":
    main()
