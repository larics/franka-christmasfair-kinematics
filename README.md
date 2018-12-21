# franka
Franka Catkin packages for the LARICS Christmas fair.

### Use

Connect to mission planner:

	roslaunch master_discovery_fkie master_discovery.launch 
    rosrun master_sync_fkie master_sync

Launch:

    roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=172.16.0.2
    rosrun franka_interface move_group_interface_cpp 

Trajectory can be triggered through service ```/frankaStartTrajectory "sauce: [int]" ```. ```sauce = 1``` chocolate sauce, ```sauce = 2``` strawberry sauce.

Returns ```true``` through ```/franka_sauce_done``` topic if mission was successful, ```false``` if not completed. Usually happens when communication constraints kill libfranka controller. In this case, rerun panda_control_moveit_rviz.launch and move_group_interface_cpp. 

### Recording waypoints

Make ```_scripts/franka_record.py``` executable. Run:

    roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=172.16.0.2
    rosrun franka_interface franka_record.py

Close the emergency stop button to enable manual control. Move robot to desired positions. Record each joint configuration calling

	rosservice call /waypoint_record "command: 1"

Store recorded joint positions to ```resources/waypoints.json``` calling

	rosservice call /waypoint_record "command: 3"

Copy the recorded positions into ```doc/move_group_interface/src/move_group_interface_tutorial.cpp into chocolate (cJointPoses), strawberry (sJointPoses), or home position (homePosition) waypoints array. Hardcoded procedure of joint and gripper movements explained in code.
