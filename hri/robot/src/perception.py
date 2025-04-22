#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# robot
from model.interface.robot_interface import RobotInterface
from model.concrete.furhat import Furhat

class PerceptionModule:
    def __init__(self, robot: RobotInterface):
        # get robot 
        self.robot = robot
        self.pub = rospy.Publisher('person_detected', Bool, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.user_found = False

    def run(self):
        """
        Check if a user is detected by the robot and publish the result on the topic '/person_detected'.
        The function will write on the topic several times if a user is detected in order to give
        some time to the manager node to read the message.
        The function will stop after 10 iterations or if the node is stopped.
        """
        counter = 0
        while not rospy.is_shutdown() and counter < 10:
            self.detect_person()
            if self.user_found:
                self.pub.publish(self.user_found)
                if counter == 1: rospy.loginfo("[Perception] Writing on topic '/person_detected' several times!\n")
                counter += 1
            else:
                if counter == 1: rospy.loginfo("[Perception] Waiting for user...\n")
            self.rate.sleep()

    def detect_person(self):
        # Get the users detected by the robot 
        users = self.robot.user_detection()
        if users == 'demo':
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
        robot = Furhat()
        robot.connect()
        rospy.loginfo("[Perception] Connection with Furhat successfully established!")
        node = PerceptionModule(robot)
        node.run()
    except rospy.ROSInterruptException:
        pass