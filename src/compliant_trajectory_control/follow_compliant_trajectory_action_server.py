import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import WrenchStamped
from compliant_trajectory_control.cfg import Tolerance6DOFConfig, GoalTimeToleranceConfig
from compliant_trajectory_control.msg import FollowCompliantTrajectoryAction, FollowCompliantTrajectoryFeedback, FollowCompliantTrajectoryResult
from cartesian_control_msgs.msg import FollowCartesianTrajectoryGoal, CartesianTolerance, FollowCartesianTrajectoryAction
from compliant_trajectory_control.joint_to_cartesian_converter import JointToCartesianConverter
from actionlib_msgs.msg import GoalStatus

class GoalTimeToleranceServer:
    def __init__(self, ns):
        self.goal_time_tolerance = 0

        self.srv = Server(GoalTimeToleranceConfig, self.reconfigure_callback, ns)

    def reconfigure_callback(self, config, level):
        self.goal_time_tolerance = config.goal_time_tolerance
        return config

    def getGoalTimeTolerance(self):
        return self.goal_time_tolerance

class Tolerance6DOFServer:
    def __init__(self, ns):
        self.tolerance = [0] * 6

        self.srv = Server(Tolerance6DOFConfig, self.reconfigure_callback, ns)

    def reconfigure_callback(self, config, level):
        self.tolerance = [config.trans_x, config.trans_y, config.trans_z, config.rot_x, config.rot_y, config.rot_z]
        return config

    def getTolerance(self):
        return self.tolerance
    
class CartesianToleranceServer:
    def __init__(self, ns):
        self.pose_tolerance = Tolerance6DOFServer(ns + "/pose_error")
        self.twist_tolerance = Tolerance6DOFServer(ns + "/twist_error")
        self.accel_tolerance = Tolerance6DOFServer(ns + "/accel_error")

    def getTolerance(self):
        return self.pose_tolerance.getTolerance(), self.twist_tolerance.getTolerance(), self.accel_tolerance.getTolerance()
    
class FollowCompliantTrajectoryActionServer:
    def __init__(self):  

        name = rospy.get_name()    

        # Load parameters from the parameter server
        robot_description_param = rospy.search_param("robot_description")
        robot_description = rospy.get_param(robot_description_param)

        controller_ns = rospy.get_param(name + "/controller")

        controller_param = rospy.search_param(controller_ns)
        self.base_link = rospy.get_param(controller_param + "/base")
        self.tip_link = rospy.get_param(controller_param + "/tip")
        self.joint_names = rospy.get_param(controller_param + "/joints")

        self.converter = JointToCartesianConverter(robot_description, self.joint_names, self.base_link, self.tip_link)

        # Create a TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Initialize the wrench publisher
        self.wrench_publisher = rospy.Publisher(name + "/target_wrench", WrenchStamped, queue_size=1)

        self.path_tolerance = CartesianToleranceServer("path_tolerance")
        self.goal_tolerance = CartesianToleranceServer("goal_tolerance")
        self.goal_time_tolerance = GoalTimeToleranceServer("")

        # Initialize the Cartesian Trajectory Action Client
        self.cartesian_trajectory_client = actionlib.SimpleActionClient(
            controller_ns + '/follow_cartesian_trajectory',
            FollowCartesianTrajectoryAction)
        self.cartesian_trajectory_client.wait_for_server()

        # Initialize the action server
        self.compliant_trajectory_action_server = actionlib.SimpleActionServer(
            name + '/follow_compliant_trajectory',
            FollowCompliantTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False)
        self.compliant_trajectory_action_server.start()

    def execute_cb(self, compliant_trajectory_goal):
        joint_trajectory_input = compliant_trajectory_goal.joint_trajectory
        path_tolerance_input = compliant_trajectory_goal.path_tolerance
        goal_tolerance_input = compliant_trajectory_goal.goal_tolerance
        goal_time_tolerance_input = compliant_trajectory_goal.goal_time_tolerance

        cartesian_trajectory = self.converter.jointToCartesian(joint_trajectory_input)

        path_pose_tolerance, path_twist_tolerance, path_accel_tolerance = self.path_tolerance.getTolerance()
        goal_pose_tolerance, goal_twist_tolerance, goal_accel_tolerance = self.goal_tolerance.getTolerance()
        goal_time_tolerance = self.goal_time_tolerance.getGoalTimeTolerance()
        wrench_stamped = compliant_trajectory_goal.wrench

        cartesian_trajectory_goal = FollowCartesianTrajectoryGoal()
        cartesian_trajectory_goal.trajectory = cartesian_trajectory
        cartesian_trajectory_goal.path_tolerance = self.create_cartesian_tolerance(path_pose_tolerance, path_twist_tolerance, path_accel_tolerance, path_tolerance_input)
        cartesian_trajectory_goal.goal_tolerance = self.create_cartesian_tolerance(goal_pose_tolerance, goal_twist_tolerance, goal_accel_tolerance, goal_tolerance_input)
        cartesian_trajectory_goal.goal_time_tolerance = goal_time_tolerance_input if goal_time_tolerance_input != 0 else rospy.Duration(goal_time_tolerance)
        
        # Send the Cartesian Trajectory Goal to the action server
        self.cartesian_trajectory_client.send_goal(cartesian_trajectory_goal)

        # Loop until the action client is done
        while not self.cartesian_trajectory_client.wait_for_result(rospy.Duration(0.01)):
            # Check if the action server is preempted
            if self.compliant_trajectory_action_server.is_preempt_requested():
                rospy.loginfo("Preempting the action client")
                # Preempt the action client by cancelling the goal
                self.cartesian_trajectory_client.cancel_goal()
                # Set the action server as preempted
                self.compliant_trajectory_action_server.set_preempted()
                return
            # Continually publish a wrench in the frame of the tip link
            self.publish_wrench_in_tip_frame(wrench_stamped)

        # The loop is done, check the result from the Cartesian Trajectory action
        cartesian_trajectory_result = self.cartesian_trajectory_client.get_result()
        compliant_trajectory_result = FollowCompliantTrajectoryResult()
        compliant_trajectory_result.error_code = cartesian_trajectory_result.error_code
        compliant_trajectory_result.error_string = cartesian_trajectory_result.error_string
        
        if compliant_trajectory_result.error_code == FollowCompliantTrajectoryResult.SUCCESSFUL:
            self.compliant_trajectory_action_server.set_succeeded(cartesian_trajectory_result)
        else:
            self.compliant_trajectory_action_server.set_aborted(cartesian_trajectory_result)

    def publish_wrench_in_tip_frame(self, wrench_stamped):
        try:
            if wrench_stamped.header.frame_id == "":
                wrench_stamped.header.frame_id = self.tip_link

            # Transform the WrenchStamped message to the tip frame
            transformed_wrench_stamped = self.tf_buffer.transform(wrench_stamped, self.tip_link, timeout=rospy.Duration(1.0))

            # Publish the transformed WrenchStamped message
            self.wrench_publisher.publish(transformed_wrench_stamped)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF2 Exception: {}".format(e))

    def create_cartesian_tolerance(self, pose_tolerance, twist_tolerance, accel_tolerance, input_cartesian_tolerance):
        cartesian_tolerance = CartesianTolerance()
        cartesian_tolerance.position_error.x = pose_tolerance[0]
        cartesian_tolerance.position_error.y = pose_tolerance[1]
        cartesian_tolerance.position_error.z = pose_tolerance[2]
        cartesian_tolerance.orientation_error.x = pose_tolerance[3]
        cartesian_tolerance.orientation_error.y = pose_tolerance[4]
        cartesian_tolerance.orientation_error.z = pose_tolerance[5]
        cartesian_tolerance.twist_error.linear.x = twist_tolerance[0]
        cartesian_tolerance.twist_error.linear.y = twist_tolerance[1]
        cartesian_tolerance.twist_error.linear.z = twist_tolerance[2]
        cartesian_tolerance.twist_error.angular.x = twist_tolerance[3]
        cartesian_tolerance.twist_error.angular.y = twist_tolerance[4]
        cartesian_tolerance.twist_error.angular.z = twist_tolerance[5]
        cartesian_tolerance.acceleration_error.linear.x = accel_tolerance[0]
        cartesian_tolerance.acceleration_error.linear.y = accel_tolerance[1]
        cartesian_tolerance.acceleration_error.linear.z = accel_tolerance[2]
        cartesian_tolerance.acceleration_error.angular.x = accel_tolerance[3]
        cartesian_tolerance.acceleration_error.angular.y = accel_tolerance[4]
        cartesian_tolerance.acceleration_error.angular.z = accel_tolerance[5]

        # Check and override the tolerance values if input_cartesian_tolerance has non-zero values
        if input_cartesian_tolerance.position_error.x != 0:
            cartesian_tolerance.position_error.x = input_cartesian_tolerance.position_error.x
        if input_cartesian_tolerance.position_error.y != 0:
            cartesian_tolerance.position_error.y = input_cartesian_tolerance.position_error.y
        if input_cartesian_tolerance.position_error.z != 0:
            cartesian_tolerance.position_error.z = input_cartesian_tolerance.position_error.z
        if input_cartesian_tolerance.orientation_error.x != 0:
            cartesian_tolerance.orientation_error.x = input_cartesian_tolerance.orientation_error.x
        if input_cartesian_tolerance.orientation_error.y != 0:
            cartesian_tolerance.orientation_error.y = input_cartesian_tolerance.orientation_error.y
        if input_cartesian_tolerance.orientation_error.z != 0:
            cartesian_tolerance.orientation_error.z = input_cartesian_tolerance.orientation_error.z
        if input_cartesian_tolerance.twist_error.linear.x != 0:
            cartesian_tolerance.twist_error.linear.x = input_cartesian_tolerance.twist_error.linear.x
        if input_cartesian_tolerance.twist_error.linear.y != 0:
            cartesian_tolerance.twist_error.linear.y = input_cartesian_tolerance.twist_error.linear.y
        if input_cartesian_tolerance.twist_error.linear.z != 0:
            cartesian_tolerance.twist_error.linear.z = input_cartesian_tolerance.twist_error.linear.z
        if input_cartesian_tolerance.twist_error.angular.x != 0:
            cartesian_tolerance.twist_error.angular.x = input_cartesian_tolerance.twist_error.angular.x
        if input_cartesian_tolerance.twist_error.angular.y != 0:
            cartesian_tolerance.twist_error.angular.y = input_cartesian_tolerance.twist_error.angular.y
        if input_cartesian_tolerance.twist_error.angular.z != 0:
            cartesian_tolerance.twist_error.angular.z = input_cartesian_tolerance.twist_error.angular.z
        if input_cartesian_tolerance.acceleration_error.linear.x != 0:
            cartesian_tolerance.acceleration_error.linear.x = input_cartesian_tolerance.acceleration_error.linear.x
        if input_cartesian_tolerance.acceleration_error.linear.y != 0:
            cartesian_tolerance.acceleration_error.linear.y = input_cartesian_tolerance.acceleration_error.linear.y
        if input_cartesian_tolerance.acceleration_error.linear.z != 0:
            cartesian_tolerance.acceleration_error.linear.z = input_cartesian_tolerance.acceleration_error.linear.z
        if input_cartesian_tolerance.acceleration_error.angular.x != 0:
            cartesian_tolerance.acceleration_error.angular.x = input_cartesian_tolerance.acceleration_error.angular.x
        if input_cartesian_tolerance.acceleration_error.angular.y != 0:
            cartesian_tolerance.acceleration_error.angular.y = input_cartesian_tolerance.acceleration_error.angular.y
        if input_cartesian_tolerance.acceleration_error.angular.z != 0:
            cartesian_tolerance.acceleration_error.angular.z = input_cartesian_tolerance.acceleration_error.angular.z

        return cartesian_tolerance