"""
This file contains all of the functionality required to convert a rectangular
(x, y, z) position to a set of servo angles.
"""

# Libraries
from __future__ import annotations
import arm_model
import numpy as np
import matplotlib.pyplot as plt

# Helper functions
def hypotinuse(x: float, y: float) -> float:
    """
    This function does basic Pythagorean theorem.
    """
    return (x**2 + y**2)**0.5


def rectangular_to_cylindrical(in_pt: np.ndarray) -> np.ndarray:
    """
    This function converts a rectangular (x, y, z) coordinate into
    a cylindrical (r, θ, z).
    """

    # Unpack input
    x, y, z = in_pt

    # Compute r
    r = hypotinuse(x, y)

    # Compute theta
    if x == 0 and y == 0:
        theta = 0
    elif y == 0:
        theta = np.sign(x) * np.pi / 2
    else:
        theta = np.arctan(x / y)

    return np.array([r, theta, z])


def distance(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    This function computes the Euclidean distance between two points.
    """
    return np.linalg.norm(x - y)


def simplify_angles(angles: np.ndarray) -> np.ndarray:
    """
    This function takes an arbitary angle and performs necessary
    simplifications to get it into a form that a servo angle could
    actually rotate to, while also respecting the following:

    cos(φ) = cos(φ')
    cos(θ - φ) = cos(θ - φ)
    """

    # Get in the range [0, 2pi)
    angles_reduced = np.fmod(np.fmod(angles, 2*np.pi) + 2*np.pi, 2*np.pi)

    return angles_reduced


def half_normalize(angles: np.ndarray) -> np.ndarray:
    """
    This function converts angles in the range [0, 2pi]
    to angles in the range [-pi, pi]
    """

    unit_angles = simplify_angles(angles)
    unit_angles[unit_angles > np.pi] -= 2*np.pi

    return unit_angles


def get_polar_offset_angle(s1: np.ndarray, s2: np.ndarray) -> float:
    """
    This function gets the angle between two vectors in a somewhat
    specialized way. Let s1, s2 \in R^2 be two 2D vectors such that

    s2 = s1 + d*<cos(θ), sin(θ)>

    for some θ. This function returns that angle.
    """

    # Get a direction vector
    dist = s2 - s1
    dist /= np.linalg.norm(dist)
    x, y = dist

    # Find theta
    if x == 0 and y == 0:
        theta = 0
    elif x == 0:
        theta = np.sign(y) * np.pi / 2
    else:
        theta = np.arctan(y / x)

    return theta


# Main class
class Converter:
    """
    This class contains all of the logic for performing the conversion.

    Inverse kinematics done by simplifying robotic arm to a 3-bar
    system and using Freudenstein's equation. Solution to Freudenstein's
    equation found using:

    https://matlabcodebysoumikc.blogspot.com/2019/10/matlab-code-for-solving-freudensteins.html

    Inputs: joint_distances - a list of the lengths of each joint, where the first
                              entry is the height of the base servo
            precision - the number of decimal points to compute an answer to
            num_theta - number of theta to simulate
            num_iters - number of iterations to refine phi
    """
    def __init__(self,
                 joint_distances: [float],
                 precision: int = 4,
                 min_num_theta: int = 1000,
                 max_num_theta: int = 10000,
                 theta_step_size: int = 10,
                 num_iters: int = 100):
        # Log inputs
        self.platform_height, *self.joint_distances = joint_distances
        self.min_num_theta = min_num_theta
        self.max_num_theta = max_num_theta
        self.theta_step_size = theta_step_size
        self.num_iters = num_iters

        # Derive some useful information from the distances
        self.arm_length = sum(self.joint_distances)
        self.arm_base = np.array([0, 0, self.platform_height])

        # Derive epsilon from the precision
        self.epsilon = 10**-(precision+1)

    def validate_point(self, in_pt: np.ndarray) -> bool:
        # Get the distance
        dist = distance(in_pt, self.arm_base)

        return dist <= self.arm_length

    def compute_4bar_params(self, r: float, z: float):
        # Compute base length
        vert_offset = np.abs(z - self.platform_height)
        d = hypotinuse(r, vert_offset)

        # Unpack a, b, c
        c, b, a = self.joint_distances

        # Compute R1, R2, R3
        R1 = d / c
        R2 = d / a
        R3 = (d**2 + a**2 - b**2 + c**2) / (2*c*a)

        return R1, R2, R3

    def freudenstein(self, num_theta: int, R1: float, R2: float, R3: float) -> (np.ndarray, np.ndarray, np.ndarray):
        # Initialize theta and phi
        thetas = np.linspace(0, 2*np.pi, num_theta)
        phis = np.zeros_like(thetas)
        errs = np.zeros_like(thetas)
        phi_0 = 41.4 * np.pi / 180

        # Create functions to minimize
        f = lambda x, y: R1*np.cos(x) - R2*np.cos(y) + R3 - np.cos(x - y)
        g = lambda x, y: R2*np.sin(y) - np.sin(x - y)

        for i in range(num_theta):
            for j in range(self.num_iters):
                # Compute f and its derivative
                f0 = f(thetas[i], phi_0)
                f0_der = g(thetas[i], phi_0)

                # Perform "gradient descent"
                phi_n = phi_0 - f0/f0_der

                # Compute error
                err = np.abs(phi_n - phi_0)

                # Finish if displacement is low enough
                if err < self.epsilon:
                    break

                # Set a new value for phi_0
                phi_0 = phi_n

            phis[i] = phi_n
            errs[i] = err

        return thetas, phis, errs

    def get_psi(self, r: float, z: float) -> float:
        off = np.abs(z - self.platform_height)
        psi = np.arctan(off / r)

        return psi

    def ideal_theta(self, r: float, z: float) -> float:
        # Option 1: at same height as base
        if z == self.platform_height:
            return np.pi/2

        # Get offset
        psi = self.get_psi(r, z)

        # Option 2: above base
        if z > self.platform_height:
            return np.pi/2 - psi

        # Option 3: below base
        return np.pi/2 + psi

    def filter_by_phi(self, r: float, z: float, thetas: np.ndarray, phis: np.ndarray) -> (np.ndarray, np.ndarray):
        # Compute psi
        psi = self.get_psi(r, z)
        direction = np.sign(z - self.platform_height)

        # Option 1: z == platform_height
        if direction == 0:
            spots = (phis >= 0) & (phis <= np.pi)

        # Option 2: z > platform_height
        elif direction == 1:
            spots = (phis >= 2*np.pi - psi) | (phis <= np.pi - psi)

        # Option 3: z < platform_height
        else:
            spots = (phis >= psi) & (phis <= np.pi + psi)

        return thetas[spots], phis[spots]

    def compute_joint_angles(self, r: float, z: float, theta: float, phi: float) -> np.ndarray:
        # Initialize array
        out_thetas = np.zeros(4)

        # Get the base angle
        psi = self.get_psi(r, z)
        psi_signed = psi * np.sign(z - self.platform_height)
        out_thetas[1] = phi + psi_signed

        # Get the elbow angles
        s0 = np.zeros(2)
        s1 = np.array([0, self.platform_height])
        s4 = np.array([r, z])
        s2 = s1 + arm_model.polar_to_vec(self.joint_distances[0], out_thetas[1])
        theta_corrected = theta + psi_signed
        s3 = s4 + arm_model.polar_to_vec(self.joint_distances[2], theta_corrected)

        out_thetas[2] = get_polar_offset_angle(s2, s3) - out_thetas[1]
        out_thetas[3] = get_polar_offset_angle(s4, s3) - out_thetas[1] - out_thetas[2]

        # Validate some of the angles
        s4_val = s3 + arm_model.polar_to_vec(self.joint_distances[2], out_thetas[1] + out_thetas[2] + out_thetas[3])
        if np.linalg.norm(s4_val - s4) > self.epsilon:
            out_thetas[3] += np.pi

        actual_points = np.array([s0, s1, s2, s3, s4])

        return out_thetas, actual_points

    def validate_solution(self, angles: np.ndarray, actual_points: np.ndarray) -> bool:
        # Derive the points from the angles
        adjusted_angles = arm_model.construct_angles(*angles[1:])
        adjusted_angles = np.vstack((np.zeros((1, 2)), adjusted_angles))
        derived_points = np.array([np.sum(adjusted_angles[:i+1], axis=0) for i in range(adjusted_angles.shape[0])])

        return np.linalg.norm(derived_points - actual_points) < self.epsilon

    def run_simulation(self, in_pt: np.ndarray, num_theta: int) -> np.ndarray:
        # Step 0: Validate input
        if not self.validate_point(in_pt):
            raise ValueError(f'{in_pt} is too far from the origin')

        # Step 1: Convert to Cylindrical
        r, base_theta, z = rectangular_to_cylindrical(in_pt)

        # Step 2: Get 4-bar parameters
        R1, R2, R3 = self.compute_4bar_params(r, z)

        # Step 3: Get possible θ, Φ values
        thetas, phis, errs = self.freudenstein(num_theta, R1, R2, R3)

        # Step 3a: filter down to just the optimal values
        # thetas = thetas[errs == np.min(errs)]
        # phis = phis[errs == np.min(errs)]
        thetas = thetas[errs <= self.epsilon]
        phis = phis[errs <= self.epsilon]

        # Step 4: reduce phis to correct values
        phis = simplify_angles(phis)
        thetas, phis = self.filter_by_phi(r, z, thetas, phis)

        if len(thetas) == 0:  # TODO: change threshold (if necessary)
            raise ValueError(f'{in_pt} has no solutions within the margin of error')

        # Step 5: Determine the best θ, Φ pair
        theta_best = self.ideal_theta(r, z)
        theta_dist = np.abs(thetas - theta_best)
        thetas_sorted = thetas[np.argsort(theta_dist)]
        phis_sorted = phis[np.argsort(theta_dist)]
        # best_idx = int(np.where(theta_dist == np.min(theta_dist))[0])
        # theta, phi = thetas[best_idx], phis[best_idx]

        for theta, phi in zip(thetas_sorted, phis_sorted):
            angles, actual_points = self.compute_joint_angles(r, z, theta, phi)
            angles = half_normalize(angles)
            condition = (angles[2:] < np.pi/2) & (angles[2:] > -np.pi/2)

            if np.all(condition) and self.validate_solution(angles, actual_points):
            # if True:
                angles[0] = base_theta

                # self.validate_solution(angles, actual_points)
                # assert self.validate_solution(angles, actual_points), f"Chosen solution has mismatch between stated and actual point for {in_pt}"
                return angles, actual_points, theta, phi

        raise ValueError(f'No solution has all abs(theta) < 90 degrees for {in_pt}')
        return

    def __call__(self, in_pt: np.ndarray) -> np.ndarray:
        theta = self.min_num_theta

        while theta <= self.max_num_theta:
            try:
                return self.run_simulation(in_pt, theta)
            except ValueError as e:
                if 'origin' in str(e):
                    raise ValueError(str(e))

            theta += self.theta_step_size

        raise ValueError(f'No solution found after trying all thetas for {in_pt}')



        # # Step 6: Map θ and φ to joint angles
        # out_thetas = np.zeros(4)

        # # Step 6a: x/y angle
        # out_thetas[0] = base_theta

        # # Step 6b: base angle
        # psi = self.get_psi(r, z)
        # psi_signed = psi * np.sign(z - self.platform_height)
        # out_thetas[1] = phi + psi_signed

        # # Step 6c: Elbows
        # s0 = np.zeros(2)
        # s1 = np.array([0, self.platform_height])
        # s4 = np.array([r, z])
        # s2 = s1 + arm_model.polar_to_vec(self.joint_distances[0], out_thetas[1])
        # theta_corrected = theta + psi_signed
        # s3 = s4 + arm_model.polar_to_vec(self.joint_distances[2], theta_corrected)

        # out_thetas[2] = get_polar_offset_angle(s2, s3) - out_thetas[1]
        # out_thetas[3] = get_polar_offset_angle(s4, s3) - out_thetas[1] - out_thetas[2]

        # # Validate some of the angles
        # s4_val = s3 + arm_model.polar_to_vec(self.joint_distances[2], out_thetas[1] + out_thetas[2] + out_thetas[3])
        # if np.sum(s4_val - s4) > self.epsilon:
        #     out_thetas[3] += np.pi
        #     # print(s4, s4_val)
        #     # print('Last angle is wrong')

        # actual_points = np.array([s0, s1, s2, s3, s4])

#         return out_thetas, actual_points, theta, phi


if __name__ == '__main__':
    conv = Converter(arm_model.joint_distances, precision=4)
    mode = 'static'

    # Active configuration
    if mode == 'active':
        while True:
            coords = input('Enter x, y, z separated by space: ')

            # Read input
            try:
                point = np.array([float(i) for i in coords.split(' ')])
            except ValueError:
                print('Please ensure you have proper formatting!')
                continue

            # Generate output
            try:
                thetas, actual_points, theta, phi = conv(point)
                # thetas = conv(point)
            except ValueError as e:
                print(e)
                # print('No valid point found for this configuration.')
                continue

            # Plot result
            arm_model.plot_arm(arm_model.construct_angles(*thetas[1:]))
            # plt.plot(actual_points[:, 0], actual_points[:, 1], 'o-')
            plt.show()

    elif mode == 'testing':
        N = 10
        X = np.linspace(1, 13, N*10)
        Z = np.linspace(1, 13, N)
        out = np.zeros((N, N))

        for i in range(N):
            for j in range(N):
                in_pt = np.array([X[i], 0, Z[i]])

                try:
                    conv(in_pt)
                except ValueError as e:
                    if 'valid' in str(e):
                        out[i, j] = 1
                    else:
                        out[i, j] = -1

        plt.imshow(out)
        plt.colorbar()
        plt.show()

    elif mode == 'static':
        point = np.array([10, 0, 0])
        # thetas = conv(point)
        thetas, actual_points, theta, phi = conv(point)

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_aspect('equal')
        arm_model.plot_arm(arm_model.construct_angles(*thetas[1:]))
        plt.plot(actual_points[:, 0], actual_points[:, 1], 'o-')
        fig.show()
