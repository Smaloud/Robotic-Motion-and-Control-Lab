import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

# Link class: Defines a single link of the robotic arm with DH parameters
class Link:
    def __init__(self, dh_params):
        # Initializes the DH parameters for the link
        self.dh_params_ = dh_params

    def transformation_matrix(self, theta):
        # Computes the transformation matrix for a given joint angle (theta)
        alpha = self.dh_params_[0]
        a = self.dh_params_[1]
        d = self.dh_params_[2]
        theta = theta + self.dh_params_[3]  # Add offset from DH parameters
        st = math.sin(theta)
        ct = math.cos(theta)
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        
        # DH transformation matrix
        trans = np.array([[ct, -st, 0, a],
                          [st*ca, ct * ca, - sa, -sa * d],
                          [st*sa, ct * sa,   ca,  ca * d],
                          [0, 0, 0, 1]])
        return trans

    def set_inertial_parameters(self, mass, center: list, inertia, T_dh_link):
        # Sets the inertial parameters for the link, including mass, center of mass, and inertia tensor
        self.mass = mass
        ixx = inertia[0]
        ixy = inertia[1]
        ixz = inertia[2]
        iyy = inertia[3]
        iyz = inertia[4]
        izz = inertia[5]
        I = np.array(
            [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        
        R = T_dh_link[0:3, 0:3]  # Rotation matrix
        new_I = R.dot(I).dot(R.T)  # Inertia tensor transformation
        
        center.append(1.0)  # Add a homogeneous coordinate to the center of mass
        new_center = T_dh_link.dot(np.array(center).T)  # Transform the center of mass
        
        self.center = new_center[:3]  # Extract the center of mass coordinates
        self.inertia_tensor = new_I  # Set the transformed inertia tensor
        
        # Print center of mass and inertia tensor for debugging
        print(f"center of mass: {self.center}")
        print(f"inertia tensor: {self.inertia_tensor}")

    @staticmethod
    def basic_jacobian(trans, ee_pos):
        # Computes the basic Jacobian for a link given the transformation matrix and end effector position
        pos = np.array([trans[0, 3], trans[1, 3], trans[2, 3]])  # Extract the position of the link
        z_axis = np.array([trans[0, 2], trans[1, 2], trans[2, 2]])  # Z-axis of the link frame
        
        # Compute the Jacobian for this link
        basic_jacobian = np.hstack((np.cross(z_axis, ee_pos - pos), z_axis))
        return basic_jacobian


# NLinkArm class: Defines the multi-link robotic arm (N-link arm)
class NLinkArm:

    def __init__(self, dh_params_list) -> None:
        # Initializes the robotic arm with a list of DH parameters for each link
        self.link_list = []
        for i in range(len(dh_params_list)):
            self.link_list.append(Link(dh_params_list[i]))

    def transformation_matrix(self, thetas):
        # Computes the overall transformation matrix for the arm given joint angles
        trans = np.identity(4)
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix(thetas[i]))
        return trans

    def forward_kinematics(self, thetas):
        # Computes the forward kinematics to get the end effector position and orientation
        trans = self.transformation_matrix(thetas)
        x = trans[0, 3]
        y = trans[1, 3]
        z = trans[2, 3]
        
        # Get the Euler angles for the orientation
        alpha, beta, gamma = self.euler_angle(thetas)
        return [x, y, z, alpha, beta, gamma]

    def euler_angle(self, thetas):
        # Computes the Euler angles (alpha, beta, gamma) from the transformation matrix
        trans = self.transformation_matrix(thetas)
        
        # Calculate the three Euler angles
        alpha = math.atan2(trans[1][2], trans[0][2])
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) + math.pi
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) - math.pi
        beta = math.atan2(
            trans[0][2] * math.cos(alpha) + trans[1][2] * math.sin(alpha),
            trans[2][2])
        gamma = math.atan2(
            -trans[0][0] * math.sin(alpha) + trans[1][0] * math.cos(alpha),
            -trans[0][1] * math.sin(alpha) + trans[1][1] * math.cos(alpha))
        
        return alpha, beta, gamma

    def inverse_kinematics(self, ref_ee_pose):
        # Solves the inverse kinematics to find the joint angles for a given end effector pose
        thetas = [0, 0, 0, 0, 0, 0]  # Initialize joint angles to zero
        for cnt in range(5000):  # Iterate to find the solution
            ee_pose = self.forward_kinematics(thetas)
            diff_pose = np.array(ref_ee_pose) - ee_pose  # Difference between current and desired pose

            basic_jacobian_mat = self.basic_jacobian(thetas)
            alpha, beta, gamma = self.euler_angle(thetas)

            # Compute the K_zyz matrix for orientation correction
            K_zyz = np.array(
                [[0, -math.sin(alpha), math.cos(alpha) * math.sin(beta)],
                 [0, math.cos(alpha), math.sin(alpha) * math.sin(beta)],
                 [1, 0, math.cos(beta)]])
            
            # Construct the control gain matrix K_alpha
            K_alpha = np.identity(6)
            K_alpha[3:, 3:] = K_zyz

            # Compute joint angle corrections
            theta_dot = np.dot(np.dot(np.linalg.pinv(basic_jacobian_mat), K_alpha), np.array(diff_pose))
            thetas = thetas + theta_dot / 100.  # Update joint angles
            
            # Break if the change is small enough
            if np.linalg.norm(theta_dot) < 0.001:
                break
        return thetas  # Return the calculated joint angles

    def basic_jacobian(self, thetas):
        # Computes the basic Jacobian for the robotic arm based on the joint angles
        ee_pos = self.forward_kinematics(thetas)[0:3]  # Extract the position of the end effector
        basic_jacobian_mat = []
        trans = np.identity(4)
        
        # Compute the Jacobian for each link
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix(thetas[i]))
            basic_jacobian_mat.append(self.link_list[i].basic_jacobian(trans, ee_pos))
        return np.array(basic_jacobian_mat).T

    def get_torque(self, thetas, thetas_d, theta_dd, f_ext, n_ext):
        # Calculate joint torques based on joint angles, velocities, accelerations, 
        # and consider external forces and moments
        
        f_ext = np.array(f_ext).T  # Convert external forces to column vectors
        n_ext = np.array(n_ext).T  # Convert external moments to column vectors

        link_num = len(self.link_list)  # Number of links in the robot arm

        # Initialize matrices to store rotation matrices, position vectors, and center of mass positions for each link
        R_i_iplus1_list = np.zeros((3, 3, link_num))
        P_i_iplus1_list = np.zeros((3, link_num))
        P_i_c_list = np.zeros((3, link_num + 1))

        # Compute transformation matrices and related data for each link
        for i in range(link_num):
            T_i_iplus1 = self.link_list[i].transformation_matrix(thetas[i])
            R_i_iplus1_list[:, :, i] = T_i_iplus1[:3, :3]
            P_i_iplus1_list[:, i] = T_i_iplus1[:3, 3]
            P_i_c_list[:, i + 1] = self.link_list[i].center

        # Initialize angular velocity, angular acceleration, linear acceleration, etc.
        omega = np.zeros((3, link_num + 1))
        omega_d = np.zeros((3, link_num + 1))
        v_dot_i = np.zeros((3, link_num + 1))
        v_dot_c = np.zeros((3, link_num + 1))
        v_dot_i[:, 0] = [0, 0, 9.8]  # Set gravity acceleration in the base coordinate system
        F = np.zeros((3, link_num + 1))
        N = np.zeros((3, link_num + 1))

        # Forward recursion to compute velocities, accelerations, forces, and moments for each link
        for i in range(link_num):
            R = R_i_iplus1_list[:, :, i].T
            m = self.link_list[i].mass
            P_i_iplus1 = P_i_iplus1_list[:, i]
            P_iplus1_c = P_i_c_list[:, i + 1]
            I_iplus1 = self.link_list[i].inertia_tensor
            theta_dot_z = thetas_d[i] * np.array([0, 0, 1]).T
            omega[:, i + 1] = R.dot(omega[:, i]) + theta_dot_z
            omega_d[:, i + 1] = R.dot(omega_d[:, i]) + np.cross(R.dot(omega[:, i]), theta_dot_z) + theta_dd[i] * np.array([0, 0, 1]).T
            v_dot_i[:, i + 1] = R.dot(np.cross(omega_d[:, i], P_i_iplus1) + np.cross(omega_d[:, i], np.cross(omega_d[:, i], P_i_iplus1)) + v_dot_i[:, i])
            v_dot_c[:, i + 1] = np.cross(omega_d[:, i + 1], P_iplus1_c) + np.cross(omega[:, i + 1], np.cross(omega[:, i + 1], P_iplus1_c)) + v_dot_i[:, i + 1]
            F[:, i + 1] = m * v_dot_c[:, i + 1]
            N[:, i + 1] = I_iplus1.dot(omega_d[:, i + 1]) + np.cross(omega[:, i + 1], I_iplus1.dot(omega[:, i + 1]))

        # Initialize storage arrays for forces, moments, and joint torques
        f = np.zeros((3, link_num + 1))
        n = np.zeros((3, link_num + 1))
        tau = np.zeros(link_num + 1)

        # Backward recursion to compute joint torques
        for i in range(link_num, 0, -1):
            R = T_i_iplus1[:3, :3]
            if i == link_num:
                f[:, i] = f_ext + F[:, i]
                n[:, i] = N[:, i] + n_ext + np.cross(P_i_c_list[:, i], F[:, i])
                tau[i] = n[:, i].T.dot(np.array([0, 0, 1]).T)
            else:
                R = R_i_iplus1_list[:, :, i]
                f[:, i] = R.dot(f[:, i + 1]) + F[:, i]
                n[:, i] = N[:, i] + R.dot(n[:, i + 1]) + np.cross(P_i_c_list[:, i], F[:, i]) + np.cross(P_i_iplus1_list[:, i], R.dot(f[:, i + 1]))
                tau[i] = n[:, i].T.dot(np.array([0, 0, 1]).T)

        return tau[1:]  # Return the calculated joint torques (excluding the torque of the base coordinate system)


if __name__ == "__main__":
    rospy.init_node("dynamics_test")  # Initialize ROS node
    real_torque_pub_list = []
    sim_torque_pub_list = []

    # Set up publishers for real and simulated joint torques
    for i in range(6):
        real_pub = rospy.Publisher(f"/real_torques/joint_{i}", Float64, queue_size=1)
        real_torque_pub_list.append(real_pub)
        sim_pub = rospy.Publisher(f"/sim_torques/joint_{i}", Float64, queue_size=1)
        sim_torque_pub_list.append(sim_pub)

    # Define DH parameters for each link of the arm (6 links)
    dh_params_list = np.array([[0, 0, 243.25/1000, 0],
                               [math.pi/2, 0, 30/1000, 0+math.pi/2],
                               [math.pi, 280/1000, 20/1000, 0+math.pi/2],
                               [math.pi/2, 0, 245/1000, 0+math.pi/2],
                               [math.pi/2, 0, 57/1000, 0],
                               [-math.pi/2, 0, 235/1000, 0-math.pi/2]])

    # Initialize the robotic arm with the DH parameters
    gen3_lite = NLinkArm(dh_params_list)

    # Set inertial parameters for each link (mass, center of mass, inertia tensor)
    gen3_lite.link_list[0].set_inertial_parameters(0.95974404, [
                                                   2.477E-05, 0.02213531, 0.09937686], [0.00165947, 2e-08, 3.6E-07, 0.00140355, 0.00034927, 0.00089493],
                                                   np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -115/1000], [0, 0, 0, 1]]))
    gen3_lite.link_list[1].set_inertial_parameters(1.17756164, [0.02998299, 0.21154808, 0.0453031], [
                                                   0.01149277, 1E-06, 1.6E-07, 0.00102851, 0.00140765, 0.01133492],
                                                   np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]))
    gen3_lite.link_list[2].set_inertial_parameters(0.59767669, [0.0301559, 0.09502206, 0.0073555], [
                                                   0.00163256, 7.11E-06, 1.54E-06, 0.00029798, 9.587E-05, 0.00169091],
                                                   np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -20/1000], [0, 0, 0, 1]]))
    gen3_lite.link_list[3].set_inertial_parameters(0.52693412, [0.00575149, 0.01000443, 0.08719207], [
                                                   0.00069098, 2.4E-07, 0.00016483, 0.00078519, 7.4E-07, 0.00034115],
                                                   np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, -105/1000], [0, 0, 0, 1]]))
    gen3_lite.link_list[4].set_inertial_parameters(0.58097325, [0.08056517, 0.00980409, 0.01872799], [
                                                   0.00021268, 5.21E-06, 2.91E-06, 0.00106371, 1.1E-07, 0.00108465],
                                                   np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, -28.5/1000], [0, 0, 0, 1]]))
    gen3_lite.link_list[5].set_inertial_parameters(0.2018, [0.00993, 0.00995, 0.06136], [
                                                   0.0003428, 0.00000019, 0.0000001, 0.00028915, 0.00000027, 0.00013076],
                                                   np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -130/1000], [0, 0, 0, 1]]))

    last_velocities = [0,0,0,0,0,0]  # Initialize last velocities for differential computation
    last_time = rospy.get_time()  # Initialize the time for time step calculation

    real_torque_pub_list = []
    sim_torque_pub_list = []

    for i in range(6):
        real_pub = rospy.Publisher(f"/real_torques/joint_{i}", Float64, queue_size=1)
        real_torque_pub_list.append(real_pub)
        
        sim_pub = rospy.Publisher(f"/sim_torques/joint_{i}", Float64, queue_size=1)
        sim_torque_pub_list.append(sim_pub)

    
    while not rospy.is_shutdown():  # Main loop running until shutdown

        feedback = rospy.wait_for_message("/my_gen3_lite/joint_states", JointState)  # Get joint states
        thetas = feedback.position[0:6]  # Joint angles
        velocities = feedback.velocity[0:6]  # Joint velocities
        torques = np.array(feedback.effort[0:6])  # Joint torques
        thetas_d = velocities  # Joint velocities as derivatives of thetas
        dt = rospy.get_time() - last_time  # Calculate time difference
        last_time = rospy.get_time()  # Update last time
        thetas_dd = np.subtract(velocities,last_velocities)/dt  # Compute joint accelerations
        last_velocities = velocities

        
