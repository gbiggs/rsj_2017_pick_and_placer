// Copyright 2017 Geoffrey Biggs (geoffrey.biggs@aist.go.jp)

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/Pose2D.h>


class PickNPlacer {
 public:
  explicit PickNPlacer(ros::NodeHandle& node_handle)
      : arm_("arm"),
        gripper_("/crane_plus_gripper/gripper_command", "true") {
    arm_.setPoseReferenceFrame("base_link");
    arm_.setNamedTarget("vertical");
    arm_.move();
    gripper_.waitForServer();

    sub_ = node_handle.subscribe("/block", 1, &PickNPlacer::DoPick, this);
  }

  void DoPick(geometry_msgs::Pose2D::ConstPtr const& msg) {
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
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to prepare pose");
      return;
    }

    ROS_INFO("Opening gripper");
    control_msgs::GripperCommandGoal goal;
    goal.command.position = 0.1;
    gripper_.sendGoal(goal);
    bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
    if (!finishedBeforeTimeout) {
      ROS_WARN("Gripper open action did not complete");
      return;
    }

    // Approach
    ROS_INFO("Executing approach");
    pose.pose.position.z = 0.05;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to grasp pose");
      return;
    }

    // Grasp
    ROS_INFO("Grasping object");
    goal.command.position = 0.015;
    gripper_.sendGoal(goal);
    finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
    if (!finishedBeforeTimeout) {
      ROS_WARN("Gripper close action did not complete");
      return;
    }

    // Retreat
    ROS_INFO("Retreating");
    pose.pose.position.z = 0.1;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to retreat pose");
      return;
    }
  }

 private:
  moveit::planning_interface::MoveGroupInterface arm_;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
  ros::Subscriber sub_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pick_and_placer");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  PickNPlacer pnp(nh);

  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
