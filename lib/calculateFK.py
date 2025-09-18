import numpy as np
from math import pi

class FK():

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

        pass
    
    def transform_matrix(self, a, alpha, d, theta):
        # Creating the DH tranformation from params
        T_matrix = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        return T_matrix

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here

        # Setting the DH parameters of the Franka Emika Panda
        dh_params = [
            #[0, 0, 0.333, q[0]],
            #[0, -pi/2, 0, q[1]], 
            #[0.195, pi/2, 0.316, q[2]],
            #[0, pi/2, 0, q[3]],
            #[0.2035, -pi/2, 1.7, q[4]],
            #[0, pi/2, 0, q[5]],
            #[0, -pi/2, 0, q[6]]
            [0, pi/2, 0.333, q[0]],
            [0, -pi/2, 0, q[1]],
            [0.0825, pi/2, 0.316, q[2]],
            [-0.825, -pi/2, 0, q[3]],
            [0, pi/2, 0.384, q[4]],
            [0, -pi/2, 0, q[5]],
            [0, 0, 0.21, q[6]]
            ]

        joint_positions = np.zeros((8,3))
        T0e = np.identity(4)
        T = np.identity(4)

        # Base position
        joint_positions[0] = [0, 0, 0]

        for i, (a, alpha, d, theta) in enumerate(dh_params):
            T_i = self.transform_matrix(a, alpha, d, theta)
            T = T @ T_i
            joint_positions[i+1] = T[0:3, 3]
        
        T0e = T

        # Your code ends here

        return joint_positions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1

    
    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
