#include <joint_to_cartesian_trajectory/joint_to_cartesian_service.h>

JointToCartesianService::JointToCartesianService(ros::NodeHandle& nh) : converter_(nh)
{
    service_ = nh.advertiseService("joint_to_cartesian_trajectory", &JointToCartesianService::jointToCartesianServiceCallback, this);
}

bool JointToCartesianService::jointToCartesianServiceCallback(joint_to_cartesian_trajectory::JointToCartesianTrajectory::Request& req,
                                                             joint_to_cartesian_trajectory::JointToCartesianTrajectory::Response& res)
{
    // Call the jointToCartesian method to convert the trajectory
    trajectory_msgs::JointTrajectory joint_trajectory = req.joint_trajectory;
    converter_.jointToCartesian(joint_trajectory, res.cartesian_trajectory);

    res.success = true;
    return true;
}
