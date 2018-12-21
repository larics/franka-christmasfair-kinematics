#!/usr/bin/env python

import argparse
import json

import rospy
import rospkg

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from franka_interface.srv import *


class Waypoints(object):
    def __init__(self):

        # Recorded waypoints
        self._waypoints = []
        self._gripper_waypoints = []
        rospack = rospkg.RosPack()
        self.json_path = rospack.get_path('franka_interface') + '/resources/'

        print self.json_path

        # Recording state
        self._is_recording = False

        self._joint_states = JointState()



        rospy.Subscriber("/franka_state_controller/joint_states_desired", JointState, self.joint_states_callback)

        self.record_service = rospy.Service("/waypoint_record", waypoint_record, self._waypoint_record_callback)


        # Create publishers
        #self.gripper_pub = rospy.Publisher("/robot/end_effector/" + limb + "_gripper/command", EndEffectorCommand, queue_size=1)

    def _add_point(self):
        #print self._waypoints
        #print self.joint_states
        
        #self._add_point(self._joint_states)

        dikt = {}

        for i in range(0, len(self._joint_states.name)):
            dikt[self._joint_states.name[i]] = self._joint_states.position[i]

        print dikt

        self._waypoints.append(dikt) # = [self._waypoints, self._joint_states]

    def _waypoint_record_callback(self, msg):
        

        if (msg.command == 0):
            # COMAND 0
            print "Deleting Waypoint List!"
            self._waypoints = []
        elif (msg.command == 1):
            # COMMAND 1
            print "Add waypoint to List!"
            self._waypoints.append(self._joint_states.position)
            #self._add_point()
        elif (msg.command == 2):
            # COMMAND 1
            print "Temp waypoints: "
            print self._waypoints
        elif (msg.command == 3):
            # COMMAND 2
            print "Store waypoints to file."

            with open(self.json_path  + 'waypoints.json', 'w') as dump_file:
                for item in self._waypoints:
                    dump_file.write('{')
                    for (i, j1) in enumerate(item):
                        json.dump(j1, dump_file)
                        if i < 6:
                            dump_file.write(', ')
                    dump_file.write('},\n')
                #json.dump(self._waypoints, dump_file, indent=1)

        #msg.result = True

        return True

    def _stop_recording(self, value):
        pass
        """
        #Sets is_recording to false
        #Navigator 'Rethink' button callback
        """
        # On navigator Rethink button press, stop recording
        #if value:
        #    self._is_recording = False

        #with open(self.json_path + self._arm + '_waypoints.json', 'w') as dump_file:
        #    json.dump(self._waypoints, dump_file)

        #with open(self.json_path + self._arm + '_gripper.json', 'w') as dump_file:
        #    json.dump(self._gripper_waypoints, dump_file)

    def joint_states_callback(self, msg):
        self._joint_states = msg



    def _run(self):

       

        while not rospy.is_shutdown():
            try:

                #print "Running!"
                rospy.sleep(0.1)
            except KeyboardInterrupt:
                print "Interrupt"
                self._add_point()

                print self.waypoints
                ## ovdje pozvati spremanje tocke




if __name__ == '__main__':

    print("Initializing node... ")
    rospy.init_node("Franka_christmas_recorder")

    waypoints = Waypoints()
    waypoints._run()
