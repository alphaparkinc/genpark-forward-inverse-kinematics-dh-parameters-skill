"""Forward and Inverse Kinematics Engine for Robot Manipulators.
100% Python Standard Library.
"""

import math

def matrix_mult(A, B):
    rows_A, cols_A = len(A), len(A[0])
    rows_B, cols_B = len(B), len(B[0])
    assert cols_A == rows_B, "Dimension mismatch in matrix multiplication"
    return [[sum(A[i][k] * B[k][j] for k in range(cols_A)) for j in range(cols_B)] for i in range(rows_A)]

class KinematicsEngine:
    """Rigid body kinematics using standard Denavit-Hartenberg (DH) convention."""
    @staticmethod
    def dh_transform(theta_rad, d, a, alpha_rad):
        ct = math.cos(theta_rad)
        st = math.sin(theta_rad)
        ca = math.cos(alpha_rad)
        sa = math.sin(alpha_rad)
        return [
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,      sa,       ca,      d],
            [0.0,     0.0,      0.0,    1.0]
        ]

    @staticmethod
    def forward_kinematics_dh(dh_params):
        """dh_params: list of tuples (theta_rad, d, a, alpha_rad)
        Returns: list of (x, y, z) joint positions and overall 4x4 transform matrix T.
        """
        T = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
        positions = [(0.0, 0.0, 0.0)]
        for theta, d, a, alpha in dh_params:
            A_i = KinematicsEngine.dh_transform(theta, d, a, alpha)
            T = matrix_mult(T, A_i)
            positions.append((round(T[0][3], 6), round(T[1][3], 6), round(T[2][3], 6)))
        return positions, T

    @staticmethod
    def inverse_kinematics_2link_planar(l1, l2, target_x, target_y):
        """Analytical inverse kinematics for 2-link planar arm.
        Returns: tuple of joint angles (theta1, theta2) in radians, or None if unreachable.
        """
        r_sq = target_x**2 + target_y**2
        cos_theta2 = (r_sq - l1**2 - l2**2) / (2.0 * l1 * l2)
        if cos_theta2 < -1.0 or cos_theta2 > 1.0:
            return None
        sin_theta2 = math.sqrt(max(0.0, 1.0 - cos_theta2**2))
        theta2 = math.atan2(sin_theta2, cos_theta2)
        k1 = l1 + l2 * cos_theta2
        k2 = l2 * sin_theta2
        theta1 = math.atan2(target_y, target_x) - math.atan2(k2, k1)
        return (theta1, theta2)
