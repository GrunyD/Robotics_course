from robot_arms import *
import numpy as np
from math import sin, cos, pi


def T01(q1) -> np.ndarray: 
   return np.array(((cos(q1), 0, sin(q1), 0),
                   (sin(q1), 0, -cos(q1), 0),
                   (0,      1,  0,      LINK1), 
                   (0,      0,  0,      1)))


def T12(q2) -> np.ndarray:
    return np.array(((-sin(q2),    -cos(q2),  0,      -LINK2 * sin(q2)),
                   (cos(q2),    -sin(q2),   0,      LINK2 * cos(q2)),
                   (0,          0,          1,      0),
                   (0,          0,          0,      1)))


def T23(q3) -> np.ndarray:
    return np.array(((cos(q3),    -sin(q3),   0,  LINK3*cos(q3)),
                   (sin(q3),    cos(q3),    0,  LINK3*sin(q3)),
                   (0,          0,          1,  0),
                   (0,          0,          0,  1)))
    


def T34(q4) -> np.ndarray:
    return np.array(((cos(q4),    -sin(q4),   0,  LINK4* cos(q4)),
                   (sin(q4),    cos(q4),    0,  LINK4* sin(q4)),
                   (0,          0,          1,  0),
                   (0,          0,          0,  1)))
    
def T45(*args):
    return np.array(((1, 0, 0, CAMERA_X_TO_4),
                    (0, 1, 0, CAMERA_Y),
                    (0, 0, 1, 0),
                    (0, 0, 0, 1)))

def T03(q1, q2, q3):
    return T01(q1) @ T12(q2) @ T23(q3)

def T04(q1, q2, q3, q4):
    return T01(q1) @ T12(q2) @ T23(q3) @ T34(q4)

def T05(q1, q2, q3, q4):
    return T04(q1, q2, q3, q4) @ T45()

if __name__ == "__main__":
    # print(T03(0, -pi/2, 0))
    import circle
    angles = circle.get_angles_for_phi(0)
    print(angles)
    print(T04(*angles))