import rospy
import kdl_parser_py.urdf as urdf_parser
import PyKDL as kdl
from trajectory_msgs.msg import JointTrajectory
from cartesian_control_msgs.msg import CartesianTrajectory, CartesianTrajectoryPoint

class JointToCartesianConverter:
    def __init__(self, urdf, joint_names, base_link, tip_link):
        self.joint_names = joint_names
        self.base_link = base_link
        self.tip_link = tip_link

        # Create a KDL tree and chain
        success, robot_tree = urdf_parser.treeFromString(urdf)
        if not success:
            rospy.logerr("Failed to parse the URDF")
            raise RuntimeError("Failed to parse the URDF")
        
        self.robot_chain = robot_tree.getChain(self.base_link, self.tip_link)
        # Add the segments to the chain (use your own code to define the robot's kinematic chain)
        self.fk_vel_solver = kdl.ChainFkSolverVel_recursive(self.robot_chain)
        self.fk_pos_solver = kdl.ChainFkSolverPos_recursive(self.robot_chain)

    def jointToCartesian(self, joint_trajectory):
        cartesian_trajectory = CartesianTrajectory()

        # Initialize the CartesianTrajectory message
        cartesian_trajectory.header.frame_id = self.base_link  # Set your desired frame ID
        cartesian_trajectory.header.stamp = rospy.Time.now()
        cartesian_trajectory.controlled_frame = self.tip_link  # Set your desired controlled frame
        cartesian_trajectory.points = []

        joint_map = self.createJointMap(joint_trajectory.joint_names)

        for i in range(len(joint_trajectory.points)):
            if joint_trajectory.points[i].time_from_start == rospy.Duration(0):
                continue

            # Convert joint positions to KDL JntArray
            joint_positions = kdl.JntArray(len(self.joint_names))
            joint_velocities = kdl.JntArray(len(self.joint_names))
            for j in range(len(self.joint_names)):
                k = joint_map[j]
                joint_positions[j] = joint_trajectory.points[i].positions[k]
                joint_velocities[j] = joint_trajectory.points[i].velocities[k]

            frame = kdl.Frame()
            self.fk_pos_solver.JntToCart(joint_positions, frame)

            # Calculate velocity
            jnt_array_vel = kdl.JntArrayVel(joint_positions, joint_velocities)
            frame_vel = kdl.FrameVel()
            self.fk_vel_solver.JntToCart(jnt_array_vel, frame_vel)
            twist = frame_vel.deriv()

            # Create a CartesianTrajectoryPoint
            cart_point = CartesianTrajectoryPoint()

            cart_point.time_from_start = joint_trajectory.points[i].time_from_start

            cart_point.pose.position.x = frame.p.x()
            cart_point.pose.position.y = frame.p.y()
            cart_point.pose.position.z = frame.p.z()
            quat = frame.M.GetQuaternion()
            cart_point.pose.orientation.x = quat[0]
            cart_point.pose.orientation.y = quat[1]
            cart_point.pose.orientation.z = quat[2]
            cart_point.pose.orientation.w = quat[3]

            # Set Cartesian velocity
            # cart_point.twist.linear.x = twist.vel.x()
            # cart_point.twist.linear.y = twist.vel.y()
            # cart_point.twist.linear.z = twist.vel.z()
            # cart_point.twist.angular.x = twist.rot.x()
            # cart_point.twist.angular.y = twist.rot.y()
            # cart_point.twist.angular.z = twist.rot.z()
            cart_point.twist.linear.x = float('NaN')
            cart_point.acceleration.linear.x = float('NaN')

            # Add the CartesianTrajectoryPoint to the message
            cartesian_trajectory.points.append(cart_point)

        return cartesian_trajectory

    def createJointMap(self, req_joint_names):
        if len(req_joint_names) != len(self.joint_names):
            rospy.logerr("Mismatch in the number of joints. Expected {} joints, but got {} joints."
                         .format(len(self.joint_names), len(req_joint_names)))
            raise RuntimeError("Mismatch in the number of joints")

        joint_map = []
        joint_name_to_index = {}

        for i in range(len(req_joint_names)):
            joint_name_to_index[req_joint_names[i]] = i

        for i in range(len(self.joint_names)):
            joint_name = self.joint_names[i]
            if joint_name in joint_name_to_index:
                joint_map.append(joint_name_to_index[joint_name])

        return joint_map
