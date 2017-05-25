// Copyright 2017 Geoffrey Biggs (geoffrey.biggs@aist.go.jp)

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <string>
#include <vector>


class PickNPlacer {
 public:
  explicit PickNPlacer(ros::NodeHandle& node_handle)
      : arm_("arm"),
        gripper_group_("gripper"),
        gripper_("/crane_plus_gripper/gripper_command", "true") {
    arm_.setPoseReferenceFrame("base_link");
    gripper_.waitForServer();
    SetupPlanningScene();

    arm_.setNamedTarget("vertical");
    arm_.move();

    sub_ = node_handle.subscribe("/block", 1, &PickNPlacer::DoPickAndPlace, this);
  }

  void DoPickAndPlace(geometry_msgs::Pose2D::ConstPtr const& msg) {
    AddBoxToScene(msg);
    if (!DoPick(msg)) {
      return;
    }
    DoPlace();
    RemoveBoxFromScene();
  }

  void SetupPlanningScene() {
    ROS_INFO("Setting up planning scene");
    // Clear the planning scene
    std::vector<std::string> objs;
    for (auto o: scene_.getObjects()) {
      objs.push_back(o.first);
    }
    for (auto o: scene_.getAttachedObjects()) {
      objs.push_back(o.first);
    }
    scene_.removeCollisionObjects(objs);

    moveit_msgs::CollisionObject table;
    table.header.frame_id = "base_link";
    table.id = "table";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1;
    primitive.dimensions[1] = 1;
    primitive.dimensions[2] = 0.1;
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = -0.05;
    pose.orientation.w = 1;
    table.primitives.push_back(primitive);
    table.primitive_poses.push_back(pose);
    table.operation = table.ADD;
    std_msgs::ColorRGBA colour;
    colour.b = 0.5;
    colour.a = 1;
    scene_.applyCollisionObject(table, colour);

    arm_.setSupportSurfaceName("table");
  }

  void AddBoxToScene(geometry_msgs::Pose2D::ConstPtr const& msg) {
    moveit_msgs::CollisionObject sponge;
    sponge.header.frame_id = "base_link";
    sponge.id = "sponge";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.04;
    primitive.dimensions[1] = 0.04;
    primitive.dimensions[2] = 0.031;
    geometry_msgs::Pose pose;
    pose.position.x = msg->x;
    pose.position.y = msg->y;
    pose.position.z = 0.016;
    pose.orientation.w = 1;
    sponge.primitives.push_back(primitive);
    sponge.primitive_poses.push_back(pose);
    sponge.operation = sponge.ADD;
    scene_.applyCollisionObject(sponge);
    ros::Duration(1).sleep();
  }

  void RemoveBoxFromScene() {
    std::vector<std::string> objs;
    objs.push_back("sponge");
    scene_.removeCollisionObjects(objs);
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
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to prepare pose");
      return false;
    }

    ROS_INFO("Opening gripper");
    control_msgs::GripperCommandGoal goal;
    goal.command.position = 0.1;
    gripper_.sendGoal(goal);
    bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
    if (!finishedBeforeTimeout) {
      ROS_WARN("Gripper open action did not complete");
      return false;
    }

    // Approach
    ROS_INFO("Executing approach");
    pose.pose.position.z = 0.05;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to grasp pose");
      return false;
    }

    // Grasp
    ROS_INFO("Grasping object");
    goal.command.position = 0.015;
    gripper_.sendGoal(goal);
    finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
    if (!finishedBeforeTimeout) {
      ROS_WARN("Gripper close action did not complete");
      return false;
    }
    arm_.attachObject("sponge", "", gripper_group_.getLinkNames());

    // Retreat
    ROS_INFO("Retreating");
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
    arm_.detachObject("sponge");

    // Retreat
    ROS_INFO("Retreating");
    pose.pose.position.z = 0.15;
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
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  ros::Subscriber sub_;
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
