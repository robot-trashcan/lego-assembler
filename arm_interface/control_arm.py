#!/usr/bin/python3

print('importing libraries...')

import sys
import numpy as np

import simulation.arm_model as arm
from simulation.rectangular_to_angles import Converter

converter = Converter(arm.joint_distances, num_theta=1000, precision=4)

# coords = input('enter coordinates (cm): ')
coords = '20 0 0'

try:
    point = np.array([float(i)/2.54 for i in coords.split(' ')])
except ValueError:
    print('Please ensure you have proper formatting!')
    sys.exit(1)

print(f'coordinates: {point}')
print('calculating angles...')
try:
    thetas, actual_points, theta, phi = converter(point)
    # thetas = [ 0.7853981, 0.15287813,  0.13508513, -1.40716941]
except ValueError:
    print('failed to converge on solution')
    sys.exit(2)

print(f'calculated angles (radians): {thetas}')
bounded_thetas = list(thetas)


angles = [d*180/np.pi for d in thetas]
print(f'calculated angles (degrees): {angles}')

servos = [int(a*2000/180+500) for a in angles]
print(f'servo input values: {servos}')

