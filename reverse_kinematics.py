from math import acos,atan, atan2, sqrt, pi
import numpy as np
from robot_arms import *
from forward_kinematics import T03

def get_o03(o04, x04):
    return o04 - x04 * LINK4

def get_q1(o03):
    x, y, _ = o03
    return atan2(y, x)

def get_beta(x, y, z):
    a = x**2 + y**2 + (z-LINK1)**2
    return acos((sqrt(a))/(2*LINK2))

def get_q2(o03, elbow_up = True):
    x, y, z = o03
    sign = 1 if elbow_up else -1
    alpha = atan((z - LINK1)/sqrt(x**2 + y**2))
    beta = get_beta(x, y, z)
    return -(pi/2 - (alpha + sign*beta))

def get_q3(o03, elbow_up = True):
    """
    If it is elbow up, this joint spins clockwise, if elbow down, it spins counter clockwise"""
    x, y, z = o03
    sign = -1 if elbow_up else 1
    beta = get_beta(x,y,z)
    return sign*beta*2


def get_q4(x02, x04):
    #TODO which direction should we point?
    angle = acos(np.dot(x02, x04))
    sign = -1 if x04[2] < x02[2] else 1 # compares z components of those two vectors. If x02 z component is higher, the wrist has to go down (negative rotation)
    return sign*angle


def get_angles(o04, x04, elbow_up:bool = True):
    o03 = get_o03(o04, x04)
    q1 = get_q1(o03)
    q2 = get_q2(o03, elbow_up=elbow_up)
    q3 = get_q3(o03, elbow_up=elbow_up)
    t03 = T03(q1, q2, q3)
    x02 = t03[:3, 0]
    q4 = get_q4(x02, x04)
    return q1, q2, q3, q4

if __name__ == "__main__":
    angles = get_angles(np.array((150, 0, 120)), np.array((1, 0, 0)))
    import forward_kinematics
    print(forward_kinematics.T04(*angles))
    print(angles)
