#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

import arm.interface
from arm.simulation.rectangular_to_angles import Converter
import arm.simulation.arm_model as arm_model

def main():
    controller = arm.interface.ArmController(serial_comms=False)

    point = np.array([0, 10, 1])
    # thetas = conv(point)
    thetas, actual_points, theta, phi = controller.converter(point)
    print(controller.servo_angles(thetas))

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    arm_model.plot_arm(arm_model.construct_angles(*thetas[1:]))
    plt.plot(actual_points[:, 0], actual_points[:, 1], 'o-')
    plt.show()

if __name__ == "__main__":
    main()
