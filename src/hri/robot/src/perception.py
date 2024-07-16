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
        self.robot.connect()
        self.pub = rospy.Publisher('person_detected', Bool, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.user_found = False

    def run(self):
        counter = 0
        while not rospy.is_shutdown() and counter < 10:
            self.detect_person()
            if self.user_found:
                self.pub.publish(self.user_found)
                rospy.loginfo("Write on topic '/person_detected'!")
                counter += 1
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
        users = self.robot.user_detection()
        if len(users) > 0:
            self.user_found = True
            rospy.loginfo("User found!")
        else:
            rospy.loginfo("Waiting for user...")
            self.user_found = False

if __name__ == '__main__':
    try:
        rospy.init_node('perception_node', anonymous=True)
        robot = Furhat()
        node = PerceptionModule(robot)
        node.run()
    except rospy.ROSInterruptException:
        pass