#!/usr/bin/python3

import arm.interface

def main():
    interface = arm.interface.ArmController()
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
    return

    coordinates = (-5, 5, 0)
    print(f"target coordinate (inches): {coordinates}")
    print("attempting to calculate angles... ", end="")
    servos = interface.calculate_angles(coordinates, unit="inches")
    if servos is not None:
        print("success")
        for s in servos:
            print(s, servos[s])

if __name__ == "__main__":
    main()
