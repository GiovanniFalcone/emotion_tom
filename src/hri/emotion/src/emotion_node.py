#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from deepface import DeepFace
import cv2

import time

class EmotionModule:
    def __init__(self):
        self.last_emotion = ''
        self.person_found = False
        self.bridge = CvBridge()
        # Load face cascade classifier
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        rospy.Subscriber("/usb_cam/image_raw", Image, self.detect_emotion_callback, queue_size=1)
        self.emotion_publisher = rospy.Publisher('emotion', String, queue_size=10)

    
    def detect_emotion_callback(self, ros_image):
        frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        # Convert frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            start_time = time.time()
            result = DeepFace.analyze(frame, actions=['emotion'])
            end_time = time.time()
            print("Time to extract face: ",end_time-start_time)
            dominant_emotion = result[0]['dominant_emotion']
            self.handle_emotion(result)
        except Exception as e:
            pass


    def detect_emotion_callback_2(self, ros_image):
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            # Convert frame to grayscale
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Detect faces in the frame
            faces = self.face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            for (x, y, w, h) in faces:
                try:
                    start_time = time.time()
                    result = DeepFace.analyze(frame, actions=['emotion'])
                    end_time = time.time()
                    print("Time to extract face: ",end_time-start_time)
                    dominant_emotion = result[0]['dominant_emotion']
                    self.handle_emotion(result)
                    #cv2.putText(frame, dominant_emotion, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                except Exception as e:
                    pass
    
                # Draw rectangle around face and label with predicted emotion
                #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

            #cv2.imshow('Emotion Node - Face Emotion', frame)
            #cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def handle_emotion(self, emotion):
        dominant_emotion = emotion[0]['dominant_emotion']
        score_dominant_emotion = emotion[0]["emotion"][dominant_emotion]
        #if score_dominant_emotion > 50:
        print("Sending ", dominant_emotion)
        self.emotion_publisher.publish(dominant_emotion)

    def detect_emotion_callback_2(self, ros_image):
        if self.person_found:
            return
        
        self.person_found = True
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            result = DeepFace.analyze(frame, actions=['emotion'], enforce_detection=False)
            dominant_emotion = result[0]['dominant_emotion']
            score_dominant_emotion = result[0]["emotion"][dominant_emotion]

            if score_dominant_emotion > 50:
                if self.last_emotion == '':
                    self.last_emotion = dominant_emotion
                    print("Sending ", dominant_emotion)
                elif self.last_emotion != dominant_emotion:
                    print("Sending ", dominant_emotion)
                    self.last_emotion = dominant_emotion

            cv2.imshow('Emotion Node - Face Emotion', frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        # init ROS
        rospy.init_node('emotion_node', anonymous=True)
        node = EmotionModule()
        node.run()
    except rospy.ROSInterruptException:
        pass