// Copyright 2017 Geoffrey Biggs (geoffrey.biggs@aist.go.jp)

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/Pose2D.h>


// Class to provide the node's behaviour and store its state between callbacks
class PickNPlacer {
 public:
  explicit PickNPlacer(ros::NodeHandle& node_handle)
      : arm_("arm"),
        gripper_("/crane_plus_gripper/gripper_command", "true") {
    // Specify end-effector positions in the "base_link" task frame
    arm_.setPoseReferenceFrame("base_link");
    // Start by moving to the vertical pose
    arm_.setNamedTarget("vertical");
    arm_.move();
    gripper_.waitForServer();

    // Subscribe to the "/block" topic to receive object positions; excecute
    // DoPickAndPlace() when one is received
    sub_ = node_handle.subscribe("/block", 1, &PickNPlacer::DoPickAndPlace, this);
  }

  void DoPickAndPlace(geometry_msgs::Pose2D::ConstPtr const& msg) {
    if (DoPick(msg)) {
      DoPlace();
    }
  }

  bool DoPick(geometry_msgs::Pose2D::ConstPtr const& msg) {
    // Prepare
    ROS_INFO("Moving to prepare pose");
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = msg->x;
    pose.pose.position.y = msg->y;
    pose.pose.position.z = 0.1;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.707106;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.707106;
    // Plan a move to the pose
    arm_.setPoseTarget(pose);
    // Execute the move
    if (!arm_.move()) {
      ROS_WARN("Could not move to prepare pose");
      return false;
    }

    ROS_INFO("Opening gripper");
    control_msgs::GripperCommandGoal goal;
    // Open the gripper to 10 cm wide
    goal.command.position = 0.1;
    // Send the gripper command
    gripper_.sendGoal(goal);
    // Wait for the command to complete
    bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
    if (!finishedBeforeTimeout) {
      ROS_WARN("Gripper open action did not complete");
      return false;
    }

    // Approach
    ROS_INFO("Executing approach");
    // Move to 5 cm above the surface to get the gripper around the object
    pose.pose.position.z = 0.05;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to grasp pose");
      return false;
    }

    // Grasp
    ROS_INFO("Grasping object");
    // Close the gripper to 1.5 cm
    goal.command.position = 0.015;
    gripper_.sendGoal(goal);
    finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
    if (!finishedBeforeTimeout) {
      ROS_WARN("Gripper close action did not complete");
      return false;
    }

    // Retreat
    ROS_INFO("Retreating");
    // Move to 10 cm above the surface to lift the object away
    pose.pose.position.z = 0.1;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to retreat pose");
      return false;
    }

    ROS_INFO("Pick complete");
    return true;
  }

  bool DoPlace() {
    // Prepare
    ROS_INFO("Moving to prepare pose");
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = 0.1;
    pose.pose.position.y = -0.2;
    pose.pose.position.z = 0.1;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.707106;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.707106;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to prepare pose");
      return false;
    }

    // Approach
    ROS_INFO("Executing approach");
    pose.pose.position.z = 0.05;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to place pose");
      return false;
    }

    // Release
    ROS_INFO("Opening gripper");
    control_msgs::GripperCommandGoal goal;
    goal.command.position = 0.1;
    gripper_.sendGoal(goal);
    bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
    if (!finishedBeforeTimeout) {
      ROS_WARN("Gripper open action did not complete");
      return false;
    }

    // Retreat
    ROS_INFO("Retreating");
    pose.pose.position.z = 0.1;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to retreat pose");
      return false;
    }

    // Rest
    goal.command.position = 0.015;
    gripper_.sendGoal(goal);
    arm_.setNamedTarget("vertical");
    arm_.move();

    ROS_INFO("Place complete");
    return true;
  }

 private:
  // Planning interface for the arm
  moveit::planning_interface::MoveGroupInterface arm_;
  // Gripper control client
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
  // Topic to receive object positions
  ros::Subscriber sub_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pickandplacer");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  // Create an instance of the class that implements the node's behaviour
  PickNPlacer pnp(nh);

  // Wait until the node is shut down
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
