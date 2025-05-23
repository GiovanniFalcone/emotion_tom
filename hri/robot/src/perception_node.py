#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

import time
import os
import sys
# to access to config file
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from util.util import Util

# robot
from model.interface.robot_interface import RobotInterface
from model.robot_factory import RobotFactory

class PerceptionModule:
    def __init__(self, robot: RobotInterface):
        # get robot 
        self.robot = robot
        self.pub = rospy.Publisher('person_detected', Bool, queue_size=10)
        self.rate = rospy.Rate(5)  # 5 Hz
        self.user_found = False

    def run(self):
        """
        Check if a user is detected by the robot and publish the result on the topic '/person_detected'.
        The function will stop once a user has been engaged.
        """
        waiting_log_counter = 0
        waiting_log_interval = 10  # Log every 10 cycles when waiting

        time.sleep(2) # wait for subscriber to be ready

        while not rospy.is_shutdown():
            self.detect_person()
            if self.user_found:
                self.pub.publish(self.user_found)
                rospy.loginfo("[Perception] Writing on topic '/person_detected'!\n")
                break
            else:
                if waiting_log_counter % waiting_log_interval == 0:
                    rospy.logwarn("[Perception] Waiting for user...\n")
                waiting_log_counter += 1
            self.rate.sleep()

    def detect_person(self):
        # Get the users detected by the robot 
        users = self.robot.user_detection()
        if users == 'other':
            self.user_found = True
            #rospy.loginfo("User found!")
        elif len(users) > 0:
            self.user_found = True
            #rospy.loginfo("User found!")
        else:
            #rospy.loginfo("Waiting for user...")
            self.user_found = False

if __name__ == '__main__':
    try:
        rospy.init_node('perception_node', anonymous=True)
        # get robot from configuration file and create the instance 
        robot_type = Util.get_from_json_file("config")['robot_type']
        robot = RobotFactory.create_robot(robot_type)
        # connect to robot 
        robot.connect()
        rospy.loginfo(f"[Manager] Connection with '{robot_type}' successfully established!")
        node = PerceptionModule(robot)
        node.run()
    except rospy.ROSInterruptException:
        pass