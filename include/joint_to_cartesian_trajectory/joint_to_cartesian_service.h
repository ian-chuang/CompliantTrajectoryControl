#ifndef JOINT_TO_CARTESIAN_SERVICE_H
#define JOINT_TO_CARTESIAN_SERVICE_H

#include <ros/ros.h>
#include <joint_to_cartesian_trajectory/JointToCartesianTrajectory.h>
#include <joint_to_cartesian_trajectory/joint_to_cartesian_converter.h>

class JointToCartesianService
{
public:
    JointToCartesianService(ros::NodeHandle& nh);

private:
    bool jointToCartesianServiceCallback(joint_to_cartesian_trajectory::JointToCartesianTrajectory::Request& req,
                                        joint_to_cartesian_trajectory::JointToCartesianTrajectory::Response& res);

    ros::ServiceServer service_;
    JointToCartesianConverter converter_;
};

#endif  // JOINT_TO_CARTESIAN_SERVICE_H
