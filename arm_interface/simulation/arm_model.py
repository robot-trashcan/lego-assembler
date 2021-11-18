# Import numpy as np
import numpy as np

# Constants (inches)
joint_distances = [3 + 13/16, 4 + 1/16, 3 + 13/32, 6]

# Helper functions
def polar_to_vec(dist, theta):
    return dist * np.array([np.cos(theta), np.sin(theta)])

def construct_angles(*thetas):
    sum_angles = [np.pi/2] + [sum(thetas[:i+1]) for i in range(len(thetas))]
    polars = np.array([polar_to_vec(dist, theta) for dist, theta in zip(joint_distances, sum_angles)])

    return polars

def plot_arm(polars):
    polars = np.vstack((np.zeros((1, 2)), polars))
    actual_points = np.array([np.sum(polars[:i+1], axis=0) for i in range(polars.shape[0])])

    plt.plot(actual_points[:, 0], actual_points[:, 1], 'o-')

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    for i in [4, 6, 8]:
        for j in [4, 6, 8]:
            for k in [4, 6, 8]:
                plot_arm(construct_angles(np.pi/i, -np.pi/j, -np.pi/k))
    plt.show()
