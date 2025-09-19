import numpy as np
from math import pi

class FK():

    def __init__(self):
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

        # Setting the DH parameters of the Franka Emika Panda
        dh_params = [
            [0, -pi/2, 0.333, q[0]],
            [0, pi/2, 0, q[1]],
            [0.0825, pi/2, 0.316, q[2]],
            [0.0825, pi/2, 0, pi + q[3]],
            [0, pi/2, 0.384, pi + q[4]],
            [0.088, pi/2, 0, q[5]],
            [0, 0, 0.21, -pi/4 + q[6]]
            ]

        joint_positions = np.zeros((8,3))
        T0e = np.identity(4)
        T = np.identity(4)
        joint_positions[i+1] = T[0:3, 3]

        # Base position
        joint_positions[0] = [0, 0, 0]

        for i, (a, alpha, d, theta) in enumerate(dh_params):
            T_i = self.transform_matrix(a, alpha, d, theta)
            T = T @ T_i
    
        T0e = T

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

    # Test 2: All zeros 
    q2 = np.zeros(7)
    joint_positions, T0e = fk.forward(q2)
    print("\nTest 2: All joint angles zero")
    print("Joint Positions:\n", joint_positions)
    print("End Effector Pose:\n", T0e)

    # Test 3: Single joint rotation 
    q3 = np.zeros(7)
    q3[4] = pi/2  
    joint_positions, T0e = fk.forward(q3)
    print("\nTest 3: Only joint 3 = 90°")
    print("Joint Positions:\n", joint_positions)
    print("End Effector Pose:\n", T0e)

    # Test 5: Fully extended arm (straight configuration) 
    q5 = np.array([0,0,0,0,0,0,0])
    joint_positions, T0e = fk.forward(q5)
    print("\nTest 5: Straight arm")
    print("End Effector Pose:\n", T0e)

    # Visualization 
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        def plot_robot(positions, title="Robot configuration"):
            fig = plt.figure()
            ax = fig.add_subplot(111, projection="3d")
            xs, ys, zs = positions[:,0], positions[:,1], positions[:,2]
            ax.plot(xs, ys, zs, "-o")
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            ax.set_title(title)
            plt.show()

        plot_robot(joint_positions, title="Final test configuration")
    except ImportError:
        print("Matplotlib not installed — skipping visualization")

