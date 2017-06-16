// Copyright 2017 Geoffrey Biggs (geoffrey.biggs@aist.go.jp)

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pickandplacer");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Set up the arm planning interface
  moveit::planning_interface::MoveGroupInterface arm("arm");
  // Specify end-effector positions in the "base_link" task frame
  arm.setPoseReferenceFrame("base_link");

  // Create a client to command the gripper
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper(
      "/crane_plus_gripper/gripper_command",
      "true");
  gripper.waitForServer();

  // Prepare
  ROS_INFO("Moving to prepare pose");
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.2;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.1;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707106;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.707106;
  // Plan a move to the pose
  arm.setPoseTarget(pose);
  // Execute the move
  if (!arm.move()) {
    ROS_WARN("Could not move to prepare pose");
    return 1;
  }

  ROS_INFO("Opening gripper");
  control_msgs::GripperCommandGoal goal;
  // Open the gripper to 10 cm wide
  goal.command.position = 0.1;
  // Send the gripper command
  gripper.sendGoal(goal);
  // Wait for the command to complete
  bool finishedBeforeTimeout = gripper.waitForResult(ros::Duration(30));
  if (!finishedBeforeTimeout) {
    ROS_WARN("Gripper open action did not complete");
    return 1;
  }

  // Approach
  ROS_INFO("Executing approach");
  // Move to 5 cm above the surface to get the gripper around the object
  pose.pose.position.z = 0.05;
  arm.setPoseTarget(pose);
  if (!arm.move()) {
    ROS_WARN("Could not move to grasp pose");
    return 1;
  }

  // Grasp
  ROS_INFO("Grasping object");
  // Close the gripper to 1.5 cm wide
  goal.command.position = 0.015;
  gripper.sendGoal(goal);
  finishedBeforeTimeout = gripper.waitForResult(ros::Duration(30));
  if (!finishedBeforeTimeout) {
    ROS_WARN("Gripper close action did not complete");
    return 1;
  }

  // Retreat
  ROS_INFO("Retreating");
  // Move to 10 cm above the surface to lift the object away
  pose.pose.position.z = 0.1;
  arm.setPoseTarget(pose);
  if (!arm.move()) {
    ROS_WARN("Could not move to retreat pose");
    return 1;
  }

  ros::shutdown();
  return 0;
}
