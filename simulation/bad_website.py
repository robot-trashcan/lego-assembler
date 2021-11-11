import numpy as np

# Functions
def fxy(x, y):
    return np.cos(x) - 2*np.cos(y) + 5/4 - np.cos(x - y)

def gxy(x, y):
    return 2*np.sin(y) - np.sin(x - y)

# Ranges for x (phi) and initial y (psi)
x = np.linspace(0, 2*np.pi, 1000)
y0 = (41.4*np.pi/180)
y = np.zeros_like(x)

# Rest of setup
n = 5
epsilon = 10**-(n+1)

for i in range(len(x)):
    for _ in range(100):
        # Calculate the values of f and its derivative
        f0 = fxy(x[i], y0)
        f0_der = gxy(x[i], y0)

        # Compute the new y and the error
        yn = y0 - f0 / f0_der
        err = np.abs(yn - y0)

        if err < epsilon:
            break

        y0 = yn

    y[i] = yn

    print(f'theta={x[i]*180/np.pi}, phi={y[i]*180/np.pi}')
