#include <joint_to_cartesian_trajectory/joint_to_cartesian_converter.h>

JointToCartesianConverter::JointToCartesianConverter(ros::NodeHandle& nh)
{
    std::string robot_description;
    urdf::Model robot_model;
    KDL::Tree   robot_tree;

    // Load parameters from the parameter server
    if (!ros::param::search("robot_description", robot_description))
    {
        const std::string error = "Searched enclosing namespaces for 'robot_description' but nothing found";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }
    if (!nh.getParam(robot_description, robot_description))
    {
        const std::string error = "Failed to load '" + robot_description + "' from the parameter server";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }
    if (!nh.getParam("base", base_link_))
    {
        const std::string error = "Failed to retrieve 'base' parameter from the parameter server";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }

    if (!nh.getParam("tip", tip_link_))
    {
        const std::string error = "Failed to retrieve 'tip' parameter from the parameter server";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }

    if (!nh.getParam("joints", joint_names_))
    {
        const std::string error = "Failed to retrieve 'joints' parameter from the parameter server";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }

    // Build a kinematic chain of the robot
    if (!robot_model.initString(robot_description))
    {
        const std::string error = "Failed to parse urdf model from 'robot_description'";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }
    if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
    {
        const std::string error = "Failed to parse KDL tree from urdf model";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }
    if (!robot_tree.getChain(base_link_, tip_link_, robot_chain_))
    {
        const std::string error = "Failed to parse robot chain from urdf model. "
                                "Are you sure that both your 'robot_base_link' and 'end_effector_link' exist?";
        ROS_ERROR_STREAM(error);
        throw std::runtime_error(error);
    }


    // Add the segments to the chain (use your own code to define the robot's kinematic chain)
    fk_solver_.reset(new KDL::ChainFkSolverVel_recursive(robot_chain_));
}

void JointToCartesianConverter::jointToCartesian(trajectory_msgs::JointTrajectory& joint_trajectory, cartesian_control_msgs::CartesianTrajectory& cartesian_trajectory)
{
    // Initialize the CartesianTrajectory message
    cartesian_trajectory.header.frame_id = base_link_;  // Set your desired frame ID
    cartesian_trajectory.header.stamp = ros::Time::now();
    cartesian_trajectory.controlled_frame = tip_link_;  // Set your desired controlled frame
    cartesian_trajectory.points.clear();

    // Use the private method to create the joint mapping
    std::vector<int> joint_map = createJointMap(joint_trajectory.joint_names);

    for (size_t i = 0; i < joint_trajectory.points.size(); ++i)
    {
        // Convert joint positions to KDL JntArray
        KDL::JntArray joint_positions(joint_names_.size());
        KDL::JntArray joint_velocities(joint_names_.size());
        for (size_t j = 0; j < joint_names_.size(); ++j)
        {
            int k = joint_map[j];
            joint_positions(j) = joint_trajectory.points[i].positions[k];
            joint_velocities(j) = joint_trajectory.points[i].velocities[k];
        }
        
        // Calculate Cartesian position and velocity
        KDL::JntArrayVel jnt_array_vel(joint_positions, joint_velocities);
        KDL::FrameVel frame_vel;
        fk_solver_->JntToCart(jnt_array_vel, frame_vel);
        KDL::Frame frame = frame_vel.GetFrame();
        KDL::Twist twist = frame_vel.GetTwist();

        // Create a CartesianTrajectoryPoint
        cartesian_control_msgs::CartesianTrajectoryPoint cart_point;

        cart_point.time_from_start = joint_trajectory.points[i].time_from_start;

        cart_point.pose.position.x = frame.p.x();
        cart_point.pose.position.y = frame.p.y();
        cart_point.pose.position.z = frame.p.z();
        frame.M.GetQuaternion(cart_point.pose.orientation.x,
                                cart_point.pose.orientation.y,
                                cart_point.pose.orientation.z,
                                cart_point.pose.orientation.w);

        // Set Cartesian velocity
        cart_point.twist.linear.x = twist.vel.x();
        cart_point.twist.linear.y = twist.vel.y();
        cart_point.twist.linear.z = twist.vel.z();
        cart_point.twist.angular.x = twist.rot.x();
        cart_point.twist.angular.y = twist.rot.y();
        cart_point.twist.angular.z = twist.rot.z();

        // Add the CartesianTrajectoryPoint to the message
        cartesian_trajectory.points.push_back(cart_point);
    }
}

std::vector<int> JointToCartesianConverter::createJointMap(const std::vector<std::string>& req_joint_names)
{
    if (req_joint_names.size() != joint_names_.size())
    {
        const std::string error = "Mismatch in the number of joints. Expected " +
                                std::to_string(joint_names_.size()) +
                                " joints, but got " +
                                std::to_string(req_joint_names.size()) +
                                " joints.";
        ROS_ERROR_STREAM(error); // Assuming you have ROS logging
        throw std::runtime_error(error);
    }

    std::vector<int> joint_map;
    std::unordered_map<std::string, int> joint_name_to_index;

    for (int i = 0; i < static_cast<int>(req_joint_names.size()); ++i)
    {
        joint_name_to_index[req_joint_names[i]] = i;
    }

    for (int i = 0; i < static_cast<int>(joint_names_.size()); ++i)
    {
        const std::string& joint_name = joint_names_[i];
        auto it = joint_name_to_index.find(joint_name);
        if (it != joint_name_to_index.end())
        {
            joint_map.push_back(it->second);
        }
    }

    return joint_map;
}

