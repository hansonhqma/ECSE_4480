import numpy as np
from math import sin, cos, pi


def rotx(theta:float) -> np.ndarray:
    """
    rotation function about x axis
    """

    return np.array([[1, 0, 0], [0, cos(theta), -sin(theta)], [0, sin(theta), cos(theta)]])

def roty(theta:float) -> np.ndarray:
    """
    rotation function about y axis
    """

    return np.array([[cos(theta), 0, sin(theta)], [0, 1, 0], [-sin(theta), 0, cos(theta)]])

def rotz(theta:float) -> np.ndarray:
    """
    rotation function about z axis
    """

    return np.array([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0], [0, 0, 1]])

def dofbot_fk_dh(joint_distances:np.ndarray, joint_angles:np.ndarray) -> tuple:
    """
    foward kinematics for dofbot using dh parameters

    parameters:

    joint_distances : np.ndarray
        joint distances in meters

    joint_angles : np.ndarray
        joint angles q1, q2, q3, q4, q5 in radians

    returns:

    tuple
        rotation matrix R0T, translation vector P0T in meters
    """

    # define unit frame vectors

    ex = np.array([[1], [0], [0]])
    ey = np.array([[0], [1], [0]])
    ez = np.array([[0], [0], [1]])

    # unpack joint distances

    (l0, l1, l2, l3, l4, l5) = joint_distances

    # unpack joint angles
    (q1, q2, q3, q4, q5) = joint_angles

    # define rotation matrices

    r01 = rotz(q1) @ rotx(pi/2)
    r12 = rotz(q2)
    r23 = rotz(q3+pi/2)
    r34 = rotz(q4) @ rotx(-pi/2)
    r45 = rotz(q5)
    r5T = roty(pi/2)

    # define translation vectors

    p01 = (l0+l1)*ez
    p12 = rotz(q2) @ (l2*ex)
    p23 = -rotz(q3+pi/2) @ (l3*ex)
    p34 = np.array([[0], [0], [0]])
    p45 = (l4+l5)*ez

    r0T = r01 @ r12 @ r23 @ r34 @ r45 @ r5T
    p0T = p01 + r01 @ (p12 + r12 @ (p23 + r23 @ (p34 + r34 @ p45)))

    return r0T, p0T



if __name__ == '__main__':

    dofbot_joint_distances = np.array([61, 43.5, 82.85, 82.85, 73.85, 54.57])/1000
    
    zero_config = np.array([0, 0, 0, 0, 0])
    case1 = np.array([pi/2, pi/2, pi/2, pi/2, pi/2])
    case2 = np.array([0, pi/4, 3*pi/4, pi/4, 3*pi/4])

