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
    ros::param::param<geometry_msgs::Pose2D>(
      "~place_pose",
      place_position_,
      geometry_msgs::Pose2D(0.1, -0.2, 0.0));
    ros::param::param<std::string>(
      "~task_frame",
      scene_task_frame_,
      "base_link");
    ros::param::param<float>("~pick_prepare_z", pick_prepare_z, 0.1);
    ros::param::param<float>("~pick_z", pick_z, 0.05);
    ros::param::param<float>("~place_prepare_z", place_prepare_z, 0.1);
    ros::param::param<float>("~place_z", place_z, 0.05);
    ros::param::param<float>("~gripper_open", gripper_open_, 0.1);
    ros::param::param<float>("~gripper_close", gripper_close_, 0.015);

    arm_.setPoseReferenceFrame(scene_task_frame_);
    arm_.setNamedTarget("vertical");
    arm_.move();
    gripper_.waitForServer();

    sub_ = node_handle.subscribe("/block", 1, &PickNPlacer::DoPickAndPlace, this);
  }

  void DoPickAndPlace(geometry_msgs::Pose2D::ConstPtr const& msg) {
    if (!DoPick(msg)) {
      return;
    }
    DoPlace();
  }

  bool DoPick(geometry_msgs::Pose2D::ConstPtr const& msg) {
    // Prepare
    ROS_INFO("Moving to prepare pose");
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = scene_task_frame_;
    pose.pose.position.x = msg->x;
    pose.pose.position.y = msg->y;
    pose.pose.position.z = pick_prepare_z;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.707106;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.707106;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to prepare pose");
      return false;
    }

    ROS_INFO("Opening gripper");
    control_msgs::GripperCommandGoal goal;
    goal.command.position = gripper_open_;
    gripper_.sendGoal(goal);
    bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
    if (!finishedBeforeTimeout) {
      ROS_WARN("Gripper open action did not complete");
      return false;
    }

    // Approach
    ROS_INFO("Executing approach");
    pose.pose.position.z = pick_z_;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to grasp pose");
      return false;
    }

    // Grasp
    ROS_INFO("Grasping object");
    goal.command.position = gripper_close_;
    gripper_.sendGoal(goal);
    finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
    if (!finishedBeforeTimeout) {
      ROS_WARN("Gripper close action did not complete");
      return false;
    }

    // Retreat
    ROS_INFO("Retreating");
    pose.pose.position.z = pick_prepare_z;
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
    pose.header.frame_id = scene_task_frame_;
    pose.pose.position.x = place_position_.x;
    pose.pose.position.y = place_position_.y;
    pose.pose.position.z = place_prepare_z_;
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
    pose.pose.position.z = place_z_;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to place pose");
      return false;
    }

    // Release
    ROS_INFO("Opening gripper");
    control_msgs::GripperCommandGoal goal;
    goal.command.position = gripper_open_;
    gripper_.sendGoal(goal);
    bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
    if (!finishedBeforeTimeout) {
      ROS_WARN("Gripper open action did not complete");
      return false;
    }

    // Retreat
    ROS_INFO("Retreating");
    pose.pose.position.z = pick_prepare_z_;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to retreat pose");
      return false;
    }

    // Rest
    goal.command.position = gripper_close_;
    gripper_.sendGoal(goal);
    arm_.setNamedTarget("vertical");
    arm_.move();

    ROS_INFO("Place complete");
    return true;
  }

 private:
  moveit::planning_interface::MoveGroupInterface arm_;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
  ros::Subscriber sub_;
  geometry_msgs::Pose2D place_position_;
  std::string scene_task_frame_;
  float pick_prepare_z_;
  float pick_z_;
  float place_prepare_z_;
  float place_z_;
  float gripper_open_;
  float gripper_close_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pickandplacer");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  PickNPlacer pnp(nh);

  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
