#ifndef JOINT_TO_CARTESIAN_CONVERTER_H
#define JOINT_TO_CARTESIAN_CONVERTER_H

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <unordered_map>

class JointToCartesianConverter
{
public:
    JointToCartesianConverter(ros::NodeHandle& nh);
    void jointToCartesian(trajectory_msgs::JointTrajectory& joint_trajectory, cartesian_control_msgs::CartesianTrajectory& cartesian_trajectory);

private:
    std::vector<int> createJointMap(const std::vector<std::string>& req_joint_names);

    ros::ServiceServer service_server_;
    boost::shared_ptr<KDL::ChainFkSolverVel_recursive> fk_solver_;
    std::string base_link_;
    std::string tip_link_;
    std::vector<std::string> joint_names_;
    KDL::Chain robot_chain_;
};

#endif  // JOINT_TO_CARTESIAN_CONVERTER_H
