#include <ros/ros.h>
#include <joint_to_cartesian_trajectory/joint_to_cartesian_service.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_to_cartesian_service_node");
    ros::NodeHandle nh;

    // Create a JointToCartesianService instance
    JointToCartesianService jointToCartesianService(nh);

    ROS_INFO("Joint to Cartesian service node is ready.");

    ros::spin();

    return 0;
}