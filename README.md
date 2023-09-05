# CompliantTrajectoryControl

The CompliantTrajectoryControl ROS package is designed to facilitate joint trajectory planning in MoveIt while maintaining compliance and adhering to cartesian path and goal constraints. It provides an action server that accepts FollowCompliantTrajectory actions and controls the robot arm with compliance in cartesian space.

## Dependencies
Before using this package, make sure you have the following dependencies installed:

1. [cartesian_controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers)
2. [Universal_Robots_ROS_controllers_cartesian](https://github.com/UniversalRobots/Universal_Robots_ROS_controllers_cartesian.git)

## Overview
This package serves as a middleware layer, translating a joint trajectory into a cartesian trajectory, which is then published to a cartesian trajectory publisher. The cartesian compliance controller manages the target frames and ensures compliance with cartesian constraints.

## Configuration
To configure this package, refer to the following YAML file:

```yaml
cartesian_compliance_controller:
  type: "velocity_controllers/CartesianComplianceController"
  end_effector_link: "tcp_link"
  robot_base_link: "base_link"
  ft_sensor_ref_link: "wrist_3_link"
  compliance_ref_link: "tcp_link"
  target_frame_topic: "target_frame"
  joints: &robot_joints
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint 

cartesian_traj_publisher:
  type: cartesian_trajectory_publisher/CartesianTrajectoryPublisher
  base: base_link
  tip: tcp_link
  joints: *robot_joints

compliant_traj_action_server:
  controller: cartesian_traj_publisher
  path_tolerance:
    pose_error: {trans_x: 0.1, trans_y: 0.1, trans_z: 0.1, rot_x: 0.1, rot_y: 0.1, rot_z: 0.1}
    twist_error: {trans_x: 0.1, trans_y: 0.1, trans_z: 0.1, rot_x: 0.1, rot_y: 0.1, rot_z: 0.1}
    accel_error: {trans_x: 0.1, trans_y: 0.1, trans_z: 0.1, rot_x: 0.1, rot_y: 0.1, rot_z: 0.1}
  goal_tolerance:
    pose_error: {trans_x: 0.1, trans_y: 0.1, trans_z: 0.1, rot_x: 0.1, rot_y: 0.1, rot_z: 0.1}
    twist_error: {trans_x: 0.1, trans_y: 0.1, trans_z: 0.1, rot_x: 0.1, rot_y: 0.1, rot_z: 0.1}
    accel_error: {trans_x: 0.1, trans_y: 0.1, trans_z: 0.1, rot_x: 0.1, rot_y: 0.1, rot_z: 0.1}
  goal_time_tolerance: 3.0
```
