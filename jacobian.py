from robot_arms import *
from math import cos, sin, pi, sqrt
import numpy as np
import forward_kinematics

z0 = np.array((0,0,1))


def jacobian(q1, q2, q3, q4, end_effector:str = 'stylus') -> np.ndarray:
    """
    
    """

    def vector_to_new_origin(angle, link_lenght):
        x = -sin(angle) * link_lenght * cos(q1)
        y = -sin(angle) * link_lenght * sin(q1)
        z = cos(angle) * link_lenght

        return np.array((x, y, z))

    o0 = np.array((0, 0, 0))
    o1 = o0 + vector_to_new_origin(0, LINK1) # This creates [0, 0, LINK1]
    o2 = o1 + vector_to_new_origin(q2, LINK2)
    o3 = o2 + vector_to_new_origin(q2 + q3, LINK3)
    if end_effector == 'stylus':
        o4 = o3 + vector_to_new_origin(q2 + q3 + q4, LINK4)
    elif end_effector == 'camera':
        o4 = o3 + vector_to_new_origin(q2 + q3 + q4 + CAMERA_STYLUS_ANGLE, LINK5)
    else:
        raise NotImplementedError(F'End effector has to be either "stylus" or "camera". You tried "{end_effector}".')

    zi = np.array((sin(q1), -cos(q1),0))

    rotation_axes = [z0, zi, zi, zi]
    origins = [o0, o1, o2, o3]
    jacobian = []
    for z, o in zip(rotation_axes, origins):
        jv = np.cross(z, o4-o)
        jw = z
        ji = np.concatenate((jv, jw))
        jacobian.append(ji)

    return np.array(jacobian).T


def get_J_4x4(q1,q2,q3,q4):
    T04 = forward_kinematics.T04(q1, q2, q3, q4)
    Ryx = T04[1,0]
    Rxx = T04[0,0]
    J_4x4 = np.array(((1,0,0,0,0,0),
                      (0,1,0,0,0,0),
                      (0,0,1,0,0,0),
                      (0,0,0,Ryx, -Rxx, 0)))
    return J_4x4

def get_full_J(q1,q2,q3,q4):
    J = jacobian(q1,q2,q3,q4)
    J_4x4 = get_J_4x4(q1,q2,q3,q4)
    return J_4x4 @ J

def transform(angles, vector):
    inv_jacobian = np.linalg.inv(get_full_J(*angles))
    return inv_jacobian @  vector

def print_matrix(matrix):
    for row in matrix:
        for column in row:
            print(F'{column:.2f}', end=" ")
            print("&", end= ' ')
        print(" \\\\ ")

if __name__ == "__main__":
    import circle
    from reverse_kinematics import get_angles
    phis = [0,  pi/2, pi, 3*pi/2]
    lin_velocities = lin_velocities = np.array(((0,0,0, 0), (0,-0.027, 0, 0), (0,0,-0.027, 0), (0,0.027,0, 0)))
    
    for phi, lin_velocity in zip(phis, lin_velocities):
        print("_____________________")
        print("PHI: ", phi)
        o04 = circle.get_o04(phi, circle.P0C, circle.R)
        x04 = circle.get_x04(phi, circle.BASE_X04, circle.R)
        angles = get_angles(o04, x04)
        
        angles = circle.get_angles_for_phi(phi)
        print('Stylus:')
        j = jacobian(*angles, end_effector='stylus')
        print('Jacobian Stylus: \n', j)

        j = jacobian(*angles, end_effector='camera')
        print('Jacobian Camera: \n', j)
        # print('Inv jacobian: \n', np.linalg.inv(get_full_J(*angles)))
        # print('Angles: ', angles)
        # print('Joint velocities: ', transform(angles, lin_velocity))






