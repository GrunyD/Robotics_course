import numpy as np
from math import sin, cos, pi, atan


def omega_given_circle(phi, r = 32, x = 150, v = 27):
    """
    This is not applicable on general case, this is only for specific case given in the exercise
    The tip of the stylus should follow a acirlce of R=32 mm and center point at (150, 0, 120). The path of the circle is given as
    function of phi, 
    path(phi) = center point + R*(0, cos(phi), sin(phi))
    stylus should be horizontal the whole time
    """
    theta = atan((r*cos(phi))/x)
    der_theta = (-r*sin(phi)/150)*(v/r)/(1 + (r*cos(phi)/150)**2)
    R = np.array(((cos(theta),  0,  sin(theta)),
                  (sin(theta),  0,  -cos(theta)),
                  (0,           1,  0)))
    der_R = np.array(((-sin(theta)*der_theta,   0,  cos(theta)*der_theta),
                      (cos(theta)*der_theta,    0,  sin(theta)*der_theta),
                      (0,                       0,  0)))
    
    S = der_R @ R.T
    w = np.array((S[2,1], S[0,2], S[1,0]))
    return w