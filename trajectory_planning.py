
import numpy as np
import circle
from jacobian import transform
import angular_velocity_for_movement

N = 36
def position_t_array(t):
    return np.array((1, t, t**2, t**3, t**4, t**5))

def get_t_matrix(initial_t:float, end_t:float, order:int = 6):
    """
    order: how many coefficients the interpolation is gonna have
    """
    velocity = lambda t: np.array(((0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4)))
    accelaration = lambda t: np.array((0, 0, 2, 6*t, 12*t**2, 20*t**3))
    return np.stack((position_t_array(initial_t), position_t_array(end_t), velocity(initial_t), velocity(end_t), accelaration(initial_t), accelaration(end_t)))

def get_joint_polynomial(angles:np.ndarray, initial_t:float = 0, end_t:float = 2):
    """
    angles: array of inital and end angles and their derivatives
    initial_t: initial time of this movement (usually 0)
    end_t: end time of this movement, how long is the whole movement going to take
    """
    t_matrix = get_t_matrix(initial_t, end_t, order = len(angles))
    t_matrix_inverse = np.linalg.inv(t_matrix)
    return t_matrix_inverse @ angles

def get_interpolation_matrix(joint_information:np.ndarray):
    """
    joint_information: array of shape (3*number of joints, 2)
        first column is intial condition, second column is end condtion
        the order in column is all angles, then all angle velocities and then all angle accelarations
    """
    n_joints = joint_information.shape[0]//3

    matrix = np.empty((n_joints,6))
    for j in range(n_joints):
        q = np.concatenate((joint_information[j,:], joint_information[n_joints + j], joint_information[2*n_joints + j]))
        a = get_joint_polynomial(np.squeeze(q)) # making sure its 1D
        matrix[j,:] = a

    return matrix
    

DEFAULT_ANGLE_ACCELERATIONS = [np.zeros((4,2)) for _ in range(5)]

def circle_path(n_knots:int, lin_velocities, angular_velocities_func, angle_accelerations = np.zeros((4,2))):
    
    # accelaritons = np.zeros((4, 2))


    # velocities = np.array(((0,0,0), (0,-27, 0), (0,0,-27), (0,27,0), (0,0,0)))
    # angle_velocities = [np.array((1, 2, 3, 4)), np.array((5, 6, 7, 8))] # TODO get them from jacobian transformation of normal velocities
    # angle_velocities = transform()
    step = N//(n_knots - 1)
    indices = [i for i in range(0, N+1, step)]
    if indices[-1] != 36:
        indices.append(36)
    phis = [circle.get_phi_by_index(index) for index in indices]

        
    interpolation_matricies = []
    for movement_index in range(len(phis)-1):
        print(phis[movement_index])
        angle_positions1 = circle.get_angles_for_phi(phis[movement_index])
        import forward_kinematics
        print(forward_kinematics.T04(*angle_positions1)) 
        angle_positions2 = circle.get_angles_for_phi(phis[movement_index+1])

        eta1 = np.concatenate((lin_velocities[movement_index], angular_velocities_func(phis[movement_index])))
        q_dot_1 = transform(angle_positions1, eta1)
        eta2 = np.concatenate((lin_velocities[movement_index + 1], angular_velocities_func(phis[movement_index + 1])))
        q_dot_2 = transform(angle_positions2, eta2)
        

        joint_information1 = np.concatenate((angle_positions1, q_dot_1, np.zeros(4)))
        joint_information2 = np.concatenate((angle_positions2, q_dot_2, np.zeros(4)))
        joint_information = np.stack((joint_information1, joint_information2)).T
        # print(joint_information)
        # return get_interpolation_matrix(joint_information)
        # break
        interpolation_matricies.append(get_interpolation_matrix(joint_information))
    return interpolation_matricies

    

if __name__ == "__main__":
    # matrix =get_t_matrix(0, 2)
    # print(matrix)
    # print(np.linalg.det(matrix))
    lin_velocities = np.array(((0,0,0), (0,-27, 0), (0,0,-27), (0,27,0), (0,0,0)))
    circle_path(n_knots = 5, lin_velocities=lin_velocities, angular_velocities_func= angular_velocity_for_movement.omega_given_circle)


