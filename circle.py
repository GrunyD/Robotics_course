from math import sin, cos, pi
import numpy as np
from reverse_kinematics import get_angles
BASE_X04 = np.array((0.150, 0, 0))
P0C= np.array((0.150, 0, 0.120))
R= 0.032
N = 37
ELBOW_UP = True

def get_x04(phi, base_x04 = BASE_X04, r = R):
    """
    It is always supposed to be horizontal -> z axis is always 0
    We get the direction in which it is pointing and divide by its magitude to make it unit vector
    """
    x04 = base_x04 + r * np.array((0, cos(phi), 0))
    x04 = x04/np.sqrt(np.dot(x04, x04))
    return x04
    
get_phi_by_index = lambda index: index*2*pi/36

def get_o04(phi, p0c = P0C, r = R):
    return p0c + r*np.array((0, cos(phi), sin(phi)))

def get_angles_for_phi(phi, p0c = P0C, base_x04 = BASE_X04, r = R):
    o04 = get_o04(phi, p0c, r)
    x04 = get_x04(phi, base_x04, r)
    return get_angles(o04, x04)

def x_dot(phi):
    #TODO make this more general
    # return np.array((0, 0, (-R/150 *sin(phi))/(1 + (R/150*cos(phi))**2)))
    return np.array((0, 0, (-R/0.150 *sin(phi))/(1 + (R/0.150*cos(phi))**2)))

if __name__ == "__main__":
    

    for step in range(N):
        phi = 2*pi*step/(N - 1)
        x04 = get_x04(phi, BASE_X04, R)
        o04 = get_o04(phi, P0C, R)
        angles = get_angles(o04, x04, elbow_up=ELBOW_UP)
        print(F'$q_{{{step}}} = [{angles[0]:.2f}, {angles[1]:.2f}, {angles[2]:.2f}, {angles[3]:.2f}]$\\\\')

