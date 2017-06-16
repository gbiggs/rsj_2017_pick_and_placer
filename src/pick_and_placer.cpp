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


// Class to provide the node's behaviour and store its state between callbacks
class PickNPlacer {
 public:
  explicit PickNPlacer(ros::NodeHandle& node_handle)
      : arm_("arm"),
        gripper_group_("gripper"),
        gripper_("/crane_plus_gripper/gripper_command", "true") {
    // Get the value for the configurable values from the parameter server, and
    // set sensible defaults for those values not specified on the parameter
    // server
    ros::param::param<float>(
      "~place_x",
      place_x_,
      0.1);
    ros::param::param<float>("~place_y", place_y_, -0.2);
    ros::param::param<std::string>(
      "~task_frame",
      scene_task_frame_,
      "base_link");
    ros::param::param<float>("~pick_prepare_z", pick_prepare_z_, 0.1);
    ros::param::param<float>("~pick_z", pick_z_, 0.05);
    ros::param::param<float>("~place_prepare_z", place_prepare_z_, 0.1);
    ros::param::param<float>("~place_z", place_z_, 0.05);
    ros::param::param<float>("~gripper_open", gripper_open_, 0.1);
    ros::param::param<float>("~gripper_close", gripper_close_, 0.015);

    // Specify end-effector positions in the configured task frame
    arm_.setPoseReferenceFrame(scene_task_frame_);
    gripper_.waitForServer();

    // Initialise the planning scene with known objects
    SetupPlanningScene();

    // Start by moving to the vertical pose
    arm_.setNamedTarget("vertical");
    arm_.move();

    // Subscribe to the "/block" topic to receive object positions; excecute
    // DoPickAndPlace() when one is received
    sub_ = node_handle.subscribe("/block", 1, &PickNPlacer::DoPickAndPlace, this);
  }

  void DoPickAndPlace(geometry_msgs::Pose2D::ConstPtr const& msg) {
    // Add the newly-detected object
    AddBoxToScene(msg);
    // Do the pick-and-place
    if (DoPick(msg)) {
      DoPlace();
    }
    // Remove the object now that we don't care about it any more
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

    // Add a table to the planning scene (the surface on which objects will be)
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

    // Let the planner know that this is the surface supporting things we will
    // be picking and placing, so collisions are allowed
    arm_.setSupportSurfaceName("table");
  }

  void AddBoxToScene(geometry_msgs::Pose2D::ConstPtr const& msg) {
    ROS_INFO("Adding box to planning scene at %f, %f", msg->x, msg->y);
    // Add a box to the scene to represent the object to be picked
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
    // Sleep a little to let the messages flow and be processed
    ros::Duration(1).sleep();
  }

  void RemoveBoxFromScene() {
    ROS_INFO("Removing box from planning scene");
    // Remove the box from the scene
    std::vector<std::string> objs;
    objs.push_back("sponge");
    scene_.removeCollisionObjects(objs);
  }

  bool DoPick(geometry_msgs::Pose2D::ConstPtr const& msg) {
    // Prepare
    ROS_INFO("Moving to prepare pose");
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = scene_task_frame_;
    pose.pose.position.x = msg->x;
    pose.pose.position.y = msg->y;
    pose.pose.position.z = pick_prepare_z_;
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
    // Open the gripper to the configuered open width
    goal.command.position = gripper_open_;
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
    // Move to the configured height above the surface to get the gripper
    // around the object
    pose.pose.position.z = pick_z_;
    arm_.setPoseTarget(pose);
    if (!arm_.move()) {
      ROS_WARN("Could not move to grasp pose");
      return false;
    }

    // Grasp
    ROS_INFO("Grasping object");
    // Close the gripper to the configured closed width
    goal.command.position = gripper_close_;
    gripper_.sendGoal(goal);
    finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
    if (!finishedBeforeTimeout) {
      ROS_WARN("Gripper close action did not complete");
      return false;
    }
    arm_.attachObject("sponge", "", gripper_group_.getLinkNames());

    // Retreat
    ROS_INFO("Retreating");
    // Move to the configuered height above the surface to lift the object away
    pose.pose.position.z = pick_prepare_z_;
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
    pose.pose.position.x = place_x_;
    pose.pose.position.y = place_y_;
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
    arm_.detachObject("sponge");

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
  // Planning interface for the arm
  moveit::planning_interface::MoveGroupInterface arm_;
  // Planning interface for the gripper (used for planning scene purposes here)
  moveit::planning_interface::MoveGroupInterface gripper_group_;
  // Gripper control client
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
  // Object to manage the planning scene
  moveit::planning_interface::PlanningSceneInterface scene_;
  // Topic to receive object positions
  ros::Subscriber sub_;
  // Variables to hold configured parameters
  float place_x_;
  float place_y_;
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
  // Create an instance of the class that implements the node's behaviour
  PickNPlacer pnp(nh);

  // Wait until the node is shut down
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
