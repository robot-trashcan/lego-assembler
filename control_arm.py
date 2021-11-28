#!/usr/bin/python3

import arm.interface
import arm.simulation.rectangular_to_angles as to_angles

def main():
    interface = arm.interface.ArmController(serial_comms=True)

    while True:
        valid = False
        while not valid:
            vals = input('servo values: ')
            try:
                positions = [int(x) for x in vals.split()]
            except ValueError:
                continue
            except Exception as e:
                print(e)
            else:
                valid = (len(positions) == 4)

        interface.arm_state[6] = positions[0] # base rotater
        interface.arm_state[5] = positions[1] # bottom joint
        interface.arm_state[4] = positions[2] # middle joint
        interface.arm_state[3] = positions[3] # upper joint
        interface.send_to_arduino()

if __name__ == "__main__":
    main()
