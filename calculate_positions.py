import pickle
from multiprocessing import Pool
import time

from matplotlib.pyplot import savefig

import arm.interface

save_file = 'arm/positions.pickle'

positions = {}
coordinates = []

DONE = 0

x_range = range(-10, 10)
y_range = range(0, 20)
z_range = range(-1, 5)
for x in x_range:
    for y in y_range:
        for z in z_range:
            coordinates.append((x,y,z))
cnum = len(x_range)*len(y_range)*len(z_range)
controller = arm.interface.ArmController(serial_comms=False)
processes = 8

def calc_pos(pos):
    p = controller.calculate_servos(pos, unit='legos')
    global DONE
    DONE += 1
    print(f'progress: {DONE/cnum*100:.2f}%\r', end='')
    return p

if __name__ == '__main__':
    print(f'Calculating positions through x={x_range}, y={y_range}, z={z_range}: {cnum} positions')

    print(f'Pooling with {processes} processes')

    start = time.time()

    with Pool(processes) as pool:
        calculated = pool.map(calc_pos, coordinates)

    for index,coord in enumerate(coordinates):
        positions[coord] = calculated[index]

    with open(save_file, 'wb') as pfile:
        pickle.dump(positions, pfile)
    
    elapsed = time.time() - start
    print('\nDone!')
    print(f'Total time elapsed: {elapsed:.2f} seconds')
    print(f'Approximate time per calculation: {elapsed/cnum*1000:.2f} milliseconds')