/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

/*
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
*/
#include "move_group_interface_tutorial.h"
#include <geometry_msgs/PoseStamped.h>

std::vector<std::vector<double>> cJointPoses {
//zajednicka 1
{1.4232003220152436, -0.5563298991623761, 0.19412348143260433, -2.1881046153287484, 0.11685836605486746, 3.219219117537745, 0.7875993519055176},
//ispred 2
{1.8133583514472214, -0.17621650394115754, 0.04226603611380039, -1.729828427486239, 1.1918912311713699, 3.0980176103951713, -0.40866672065181575},
//okolo 3
{1.8133414141262458, -0.036419491822347994, 0.049905759174140606, -1.590799651792722, 1.1373396168284955, 3.0907553320621703, -0.3874780060869235},
//iznad 4
{1.8131860312077037, 0.06175369591798717, -0.06725812146588883, -1.2018739600466324, 0.6119598724643389, 2.7534607529201804, 0.23042863980716477},
//cookie 5
{0.8184173507313204, 1.2063623176206788, -0.15205178081974535, -0.9831984842210562, 0.35136860509174767, 3.7263726219440456, 0.6330407055047249}
};


//aula
std::vector<std::vector<double>> sJointPoses {
//zajednicka 1
{1.4232003220152436, -0.5563298991623761, 0.19412348143260433, -2.1881046153287484, 0.11685836605486746, 3.219219117537745, 0.7875993519055176},
// ispred
{1.4544374986902722, -0.24703246685784006, 0.08655435839327841, -1.969561095640264, 0.11636559269825618, 3.3083877694130237, 0.6732859551132159},
// okolo
{1.4544911263093614, -0.07828554675199241, 0.0962997266258037, -1.7882679256255742, 0.11578662215341286, 3.287372047889262, 0.6661623992198179},
// iznad 
{1.4544710948890534, -0.023156354995818047, 0.08341217712764495, -1.4151002559261892, 0.11932301374276479, 2.9899368475514945, 0.676556217767275},
// cookie
{0.8184173507313204, 1.2063623176206788, -0.15205178081974535, -0.9831984842210562, 0.35136860509174767, 3.7263726219440456, 0.6330407055047249}
};


std::vector<double> homePosition {
{1.553292975323242, -1.4182828076138525, -0.10834242502595812, -2.428669794684962, 0.09092836186232152, 2.5997016472169454, 0.6070526642792731}
};


std::queue<int> ordersQueue;


bool startTrajectoryCallback(christmas_fair_common::FrankaStartTrajectorySrv::Request &req,
  christmas_fair_common::FrankaStartTrajectorySrv::Response &res) {
  ordersQueue.push(req.sauce);
  return true;
}


/*
    -1  do nothing
    0 - home
    1 - move - hold it
    2 - move - release
    3 - grasp - squeeze - choco
    4 - grasp - squeeze - straw
*/
void gripperControl(int actionID) {

  if (actionID == -1) {
    return;
  }
  else if (actionID == 0) {
    actionlib::SimpleActionClient<franka_gripper::HomingAction> ac_home("/franka_gripper/homing", true);
    ac_home.waitForServer(); //will wait for infinite time
    franka_gripper::HomingAction goal_home;

    ac_home.sendGoal(goal_home.action_goal.goal);
  }
  else if (actionID < 3) {
    actionlib::SimpleActionClient<franka_gripper::MoveAction> ac_move("/franka_gripper/move", true);
    ac_move.waitForServer(); //will wait for infinite time
    franka_gripper::MoveAction goal_move;
    if (actionID == 1) {
      goal_move.action_goal.goal.width = 0.045;
      goal_move.action_goal.goal.speed = 0.08;
    }
    else if (actionID == 2) {
      goal_move.action_goal.goal.width = 0.08;
      goal_move.action_goal.goal.speed = 0.08;
    }
  
    ac_move.sendGoal(goal_move.action_goal.goal);

  }
  else  {
    actionlib::SimpleActionClient<franka_gripper::GraspAction> ac_grasp("/franka_gripper/grasp", true);
    ac_grasp.waitForServer(); //will wait for infinite time
    franka_gripper::GraspAction goal_grasp;
    goal_grasp.action_goal.goal.width = 0.025;
    goal_grasp.action_goal.goal.epsilon.inner = 0.06;
    goal_grasp.action_goal.goal.epsilon.outer = 0.06;
    goal_grasp.action_goal.goal.speed = 0.08;
    if (actionID == 3) {
      goal_grasp.action_goal.goal.force = 60.0;
    }
    else if (actionID == 4) {
      goal_grasp.action_goal.goal.force = 53.0;
    }
    ac_grasp.sendGoal(goal_grasp.action_goal.goal);
  }
}

bool moveGroupToPoint(moveit::planning_interface::MoveGroupInterface &move_group, 
  int sauceID, int i_point, double velocity) {
    
    std::vector<std::vector<double>> missionPoints;
    std::vector<double> jointPositionReference(7, 0);
  
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit_msgs::MoveItErrorCodes errorCode;

    if (i_point == -1) {          // HOME
      for(int i = 0; i < 7; i++){
        jointPositionReference[i] = homePosition[i];
      }
    }
    else {
      if (sauceID == 1) {
        missionPoints = cJointPoses;
      }
      else if (sauceID == 2) {
        missionPoints = sJointPoses;
      }
      else {
        std::cout << "INVALID SAUCE ID" << std::endl;
        return false;
      }
      for(int i = 0; i < 7; i++){
        jointPositionReference[i] = missionPoints[i_point][i];
      }
    }

    if (velocity >= 0) {
      move_group.setMaxVelocityScalingFactor(velocity);
    }
    move_group.setJointValueTarget(jointPositionReference);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    errorCode = move_group.move();
    
    if (errorCode.val < 0) {
      throw std::runtime_error( "Did not move!");
    }

    return success;
}


  /* mission: 
    1   - zajednicka tocka
    2   - ispred umaka
    3   - gripper oko umaka
    M1  - gripper move - primi umak
    4   - iznad posude za umak
    5   - home - tocka na pola puta da bi se odmaknula od stalka
    G1  - grasp - squeeze the sauce
    M1  - gripper move - primi umak 
    5   - home - tocka na pola puta da bi se odmaknula od stalka
    4   - iznad posude za umak
    3   - gripper oko umaka, spusten u posudu
    M2  - gripper move - pusti umak
    2   - ispred umaka
    1   - zajednicka tocka
    Gh   - gripper homing - recalibration!
  */

bool strawberryMission(moveit::planning_interface::MoveGroupInterface &move_group,
  ros::Publisher pub_done) {

  int sauceID = 2;
  bool success;
  double velScale = 0.2;
  std_msgs::Bool mission_done;
  mission_done.data = true;

  try {

    std::vector<double> velocities {0.5, 0.5, 0.9, 0.9};
    for(int i_point = 0; i_point < 4; i_point++){
      success = moveGroupToPoint(move_group, sauceID, i_point, velocities[i_point]);
      if (i_point == 2) {
        gripperControl(1); // move - hold it
        ros::Duration(1.0).sleep();
      }
    }

    success = moveGroupToPoint(move_group, sauceID, -1, 0.9);

    success = moveGroupToPoint(move_group, sauceID, 4, 0.9);

    gripperControl(4); //grasp
    ros::Duration(1.0).sleep();
    gripperControl(1); //move hold
    ros::Duration(0.5).sleep();

    success = moveGroupToPoint(move_group, sauceID, -1, 0.9);
    pub_done.publish(mission_done);

    for(int i_point = 3; i_point >= 0; i_point--){
      success = moveGroupToPoint(move_group, sauceID, i_point, velocities[i_point]);
      if (i_point == 2) {
        gripperControl(2); // move - release
        ros::Duration(0.5).sleep();
      }
    }

    gripperControl(0);
    ros::Duration(4.0).sleep();

  } catch (const std::runtime_error& e) {
    std::cout << "libfranka died. Aborting trajectory." << std::endl;
    mission_done.data = false;
    pub_done.publish(mission_done);
    return false;
  }

  return true;
}


  /* mission: 
    1   - zajednicka tocka
    2   - ispred umaka
    3   - gripper oko umaka
    M1  - gripper move - primi umak
    4   - iznad posude za umak
    5   - home - tocka na pola puta da bi se odmaknula od stalka
    G1  - grasp - squeeze the sauce
    M1  - gripper move - primi umak 
    G1  - grasp - squeeze the chocolate twice
    M1  - gripper move - primi umak 
    5   - home - tocka na pola puta da bi se odmaknula od stalka
    4   - iznad posude za umak
    3   - gripper oko umaka, spusten u posudu
    M2  - gripper move - pusti umak
    2   - ispred umaka
    1   - zajednicka tocka
    Gh   - gripper homing - recalibration!
  */

bool chocolateMission(moveit::planning_interface::MoveGroupInterface &move_group, 
  ros::Publisher pub_done) {
  
  int sauceID = 1;
  bool success;
  double velScale = 0.2;
  std_msgs::Bool mission_done;
  mission_done.data = true;

  try {

    std::vector<double> velocities {0.5, 0.5, 0.9, 0.9};
    for(int i_point = 0; i_point < 4; i_point++){
      success = moveGroupToPoint(move_group, sauceID, i_point, velocities[i_point]);
      if (i_point == 2) {
        gripperControl(1); // move - hold it
        ros::Duration(1.0).sleep();
      }
    }

    success = moveGroupToPoint(move_group, sauceID, -1, 0.9);

    success = moveGroupToPoint(move_group, sauceID, 4, 0.9);

    gripperControl(3); //grasp
    ros::Duration(1.0).sleep();
    gripperControl(1); //move hold
    ros::Duration(0.5).sleep();
    gripperControl(3); //grasp
    ros::Duration(1.0).sleep();
    gripperControl(1); //move hold
    ros::Duration(0.5).sleep();

    success = moveGroupToPoint(move_group, sauceID, -1, 0.9);
    pub_done.publish(mission_done);

    for(int i_point = 3; i_point >= 0; i_point--){
      success = moveGroupToPoint(move_group, sauceID, i_point, velocities[i_point]);
      if (i_point == 2) {
        gripperControl(2); // move - release
        ros::Duration(0.5).sleep();
      }
    }

    gripperControl(0);
    ros::Duration(4.0).sleep();

  } catch (const std::runtime_error& e) {
    std::cout << "libfranka died. Aborting trajectory." << std::endl;
    mission_done.data = false;
    pub_done.publish(mission_done);
    return false;
  }

  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::ServiceServer startTrajectoryService;
  startTrajectoryService = node_handle.advertiseService("frankaStartTrajectory", 
    &startTrajectoryCallback);

  //ros::Publisher pub_eef_pos = node_handle.advertise<geometry_msgs::PoseStamped>("end_effector_position", 5);
  ros::Publisher pub_done = node_handle.advertise<std_msgs::Bool>("franka_sauce_done", 1);

  // Setup
  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Getting Basic Information
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

  // MOVE TO HOME AT START; GRIPPER HOMING
  bool success = moveGroupToPoint(move_group, 1, -1, 0.9);
  gripperControl(0);
  ros::Duration(4.0).sleep();

  int sauce;
  bool a; 
  std_msgs::Bool mission_done;
  mission_done.data = true;

  while (ros::ok()) {
    if (!ordersQueue.empty()) {
      sauce = ordersQueue.front();
      ordersQueue.pop();
      if (sauce == 1) {
        bool a = chocolateMission(move_group, pub_done);
      }
      else if (sauce == 2) {
        bool a = strawberryMission(move_group, pub_done);
      }
      else {
        pub_done.publish(mission_done);
        std::cout << "Service called with invalid sauceID or no sauce." << std::endl;
      }
    }
    else {
      ros::Duration(0.1).sleep();
    }
    //pub_eef_pos.publish(move_group.getCurrentPose());
  }

  
  ros::shutdown();
  return 0;


}
