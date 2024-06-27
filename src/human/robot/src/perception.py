#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

from connection import RobotConnectionManager

class PerceptionModule:
    def __init__(self):
        self.robot = RobotConnectionManager.get_session()
        self.pub = rospy.Publisher('person_detected', Bool, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.user_found = False

    def run(self):
        while not rospy.is_shutdown():
            self.detect_person()
            if self.user_found:
                self.pub.publish(self.user_found)
                rospy.loginfo("Write on topic '/person_detected'!")
            self.rate.sleep()

    def run1(self):
        while not rospy.is_shutdown():
            if not self.user_found:
                self.detect_person()
                if self.user_found:
                    self.pub.publish(self.user_found)
                    rospy.loginfo("Write on topic!")
            else:
                rospy.loginfo("Still in camera!")
                self.rate = rospy.Rate(1)
            self.rate.sleep()

    def detect_person(self):
        # Get the users detected by the robot 
        users = self.robot.get_users()
        if len(users) > 0:
            # Attend the user closest to the robot
            self.robot.attend(user="CLOSEST")
            self.user_found = True
            rospy.loginfo("User found!")
        else:
            rospy.loginfo("Waiting for user...")
            self.user_found = False

if __name__ == '__main__':
    try:
        rospy.init_node('perception_node', anonymous=True)
        node = PerceptionModule()
        node.run()
    except rospy.ROSInterruptException:
        pass