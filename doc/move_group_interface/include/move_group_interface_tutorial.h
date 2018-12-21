#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/HomingAction.h>
#include <actionlib/client/simple_action_client.h>

#include <christmas_fair_common/FrankaStartTrajectorySrv.h>

#include <queue> 
#include <std_msgs/Bool.h>
#include <stdexcept>


bool startTrajectoryCallback(christmas_fair_common::FrankaStartTrajectorySrv::Request &req,
  christmas_fair_common::FrankaStartTrajectorySrv::Response &res);

bool chocolateMission(moveit::planning_interface::MoveGroupInterface &move_group, ros::Publisher pub_done);
bool strawberryMission(moveit::planning_interface::MoveGroupInterface &move_group, ros::Publisher pub_done);

void gripperControl(int actionID);
//std::vector<std::vector<double>> cJointPoses, sJointPoses;