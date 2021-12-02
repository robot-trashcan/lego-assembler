import pickle
from multiprocessing import Pool
import time

import arm.interface

positions = {}
coordinates = []

x_range = range(-5, 5)
y_range = range(0, 15)
z_range = range(0, 3)
for x in x_range:
    for y in y_range:
        for z in z_range:
            coordinates.append((x,y,z))
cnum = len(x_range)*len(y_range)*len(z_range)
controller = arm.interface.ArmController(serial_comms=False, positions_file='arm/positions.pickle')
processes = 8

def calc_pos(pos):
    print(f'position: {pos}       \r', end='')
    return controller.calculate_servos(pos, unit='legos')

if __name__ == '__main__':
    print(f'Calculating positions through {x_range}, {y_range}, {z_range}: {cnum} positions')

    print(f'Pooling with {processes} processes')

    start = time.time()

    with Pool(processes) as pool:
        calculated = pool.map(calc_pos, coordinates)

    for index,coord in enumerate(coordinates):
        positions[coord] = calculated[index]

    with open('positions.pickle', 'wb') as pfile:
        pickle.dump(positions, pfile)
    
    elapsed = time.time() - start
    print('\nDone!')
    print(f'Total time elapsed: {elapsed:.2f} seconds')
    print(f'Approximate time per calculation: {elapsed/cnum*1000:.2f} milliseconds')