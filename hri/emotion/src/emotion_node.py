#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from emotion.msg import emotion

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
        # used to save all emotion in the csv
        self.full_emotion_publisher = rospy.Publisher('full_emotion', emotion, queue_size=10)
        # this is synchronized with game information (if user has find a pair)
        self.filtered_emotion_publisher = rospy.Publisher('filtered_emotion', emotion, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.cap = cv2.VideoCapture(2)
        # Set camera parameters
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

    def run(self):
        #rospy.spin()
        frame_count = 0
        # how many frame should be analyzed for emotion recognition
        analyze_frequency = 1

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            if frame_count % analyze_frequency == 0:
                # if there is no face then handle ros message 
                if len(faces) == 0: 
                    self.handle_emotion(None)
                else:
                    # Find the face with the largest area (i.e the one more near to the robot)
                    largest_face = max(faces, key=lambda rect: rect[2] * rect[3])  # rect is (x, y, w, h)

                    # Unpack the coordinates of the largest face
                    x, y, w, h = largest_face
                    rgb_frame = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2RGB)
                    face_roi = rgb_frame[y:y + h, x:x + w]
                    
                    # Analyze the largest face
                    result = DeepFace.analyze(face_roi, actions=['emotion'], enforce_detection=False)
                    emotion = result[0]['dominant_emotion']
                    
                    # Handle the detected emotion
                    self.handle_emotion(result)
                    
                    # Draw rectangle and text around the largest face
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(frame, emotion, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

                """for (x, y, w, h) in faces:
                    rgb_frame = cv2.cvtColor(gray_frame, cv2.COLOR_GRAY2RGB)
                    face_roi = rgb_frame[y:y + h, x:x + w]
                    result = DeepFace.analyze(face_roi, actions=['emotion'], enforce_detection=False)
                    emotion = result[0]['dominant_emotion']
                    #print(f"Emotion: {emotion}")
                    self.handle_emotion(result)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    cv2.putText(frame, emotion, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)"""
            else:
                self.handle_emotion(None)

            self.rate.sleep()
            frame_count += 1
            cv2.imshow('Real-time Emotion Detection', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

    def handle_emotion(self, result):
        emotion_msg = emotion()
        if result is None:
            emotion_msg.face_found = False
            emotion_msg.dominant_emotion = ''
            emotion_msg.model_confidence = 0
            self.full_emotion_publisher.publish(emotion_msg)
            self.filtered_emotion_publisher.publish(emotion_msg)
            return
        
        # else a face was found and emotion was analyzed
        dominant_emotion = result[0]['dominant_emotion']
        score_dominant_emotion = result[0]["emotion"][dominant_emotion]
        # create ros msg
        emotion_msg.face_found = True
        emotion_msg.model_confidence = score_dominant_emotion
        emotion_msg.dominant_emotion = dominant_emotion

        #rospy.loginfo(f"Written emotion on topic: \n{emotion_msg} \n")
        self.full_emotion_publisher.publish(emotion_msg)
        self.filtered_emotion_publisher.publish(emotion_msg)

if __name__ == '__main__':
    try:
        # init ROS
        rospy.init_node('emotion_node', anonymous=True)
        node = EmotionModule()
        node.run()
    except rospy.ROSInterruptException:
        pass