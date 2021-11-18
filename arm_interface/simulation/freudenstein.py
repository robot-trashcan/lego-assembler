# Import libraries
import numpy as np
from scipy.optimize import newton
from arm_model import joint_distances

def compute_a(h0, x_dest):
    return (h0**2 + x_dest**2)**0.5


def create_freudenstein_equation(a, b, c, d):
    # Normalize by a
    b = b / a
    c = c / a
    d = d / a

    # Compute R1, R2, R3
    R1 = 1 / d
    R2 = 1 / b
    R3 = (1 + b**2 + c**2 + d**2) / (2*b*d)

    def freudenstein(psi, phi):
        return R1*np.cos(phi) - R2*np.cos(psi) + R3 - np.cos(phi-psi)

    return freudenstein

if __name__ == '__main__':
    # Set the desired distance
    x_dest = 10

    # Get the function inputs
    h0, b, c, d = joint_distances
    a = compute_a(h0, x_dest)

    f = create_freudenstein_equation(a, b, c, d)
    phi_init = 3*np.pi/4
