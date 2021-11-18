#!/usr/bin/python3

import arm.interface

def main():
    interface = arm.interface.ArmController()
    coordinates = (11, 1, 0)
    print(f"target coordinate (inches): {coordinates}")
    print("attempting to calculate angles... ", end="")
    servos = interface.calculate_angles(coordinates, unit="inches")
    if servos is not None:
        print("success")
        for s in servos:
            print(s, servos[s])

if __name__ == "__main__":
    main()