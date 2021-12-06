#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

from arm.interface import ArmController,ArmServo
from arm.simulation.rectangular_to_angles import Converter
import arm.simulation.arm_model as arm_model

PLOT = False

def main():
    
    controller = ArmController(serial_comms=False, positions_file='arm/positions.pickle')
    coordinates = (-3,5,0) # legos

    controller.move_to(coordinates, unit='legos')
    controller.close_claw()
    # controller.close_claw()
    sorted_vals = sorted([(k,v) for k,v in controller.arm_state.items()], key=lambda i: int(i[0]))
    print(" ".join(str(i[1]) for i in sorted_vals))
    
    if PLOT:
        thetas, points, _t, _p = controller.calculate_angles(coordinates, unit='legos')
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_aspect('equal')
        arm_model.plot_arm(arm_model.construct_angles(*thetas[1:]))
        plt.plot(points[:, 0], points[:, 1], 'o-')
        plt.show()

if __name__ == "__main__":
    main()
