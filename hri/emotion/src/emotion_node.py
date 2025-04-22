#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from emotion.msg import emotion

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import cv2
import time

# run in parallel deepface and headpose estimation
import concurrent.futures
from concurrent.futures import ProcessPoolExecutor
from detection import Detection
# prova
from collections import deque

class EmotionModule:
    def __init__(self):
        rospy.loginfo("[EmotionModule] Initializing camera...")
        self.last_emotion = ''
        self.person_found = False
        self.bridge = CvBridge()
        # used to save all emotion in the csv
        self.full_emotion_publisher = rospy.Publisher('full_emotion', emotion, queue_size=10)
        self.rate = rospy.Rate(30)  # Hz
        self.cap = cv2.VideoCapture(0)
        # Set camera parameters
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        rospy.loginfo("[EmotionModule] Camera initialized successfully.")
        
        # prova
        self.window_duration = 1.5
        self.emotion_window = deque()
        self.time_emotion = time.time()

    def add_emotion(self, emotion):
        now = time.time()
        #x = now - self.time_emotion
        self.emotion_window.append((now, emotion))
        #rospy.loginfo(f"Add {emotion} in q of len={len(self.emotion_window)} at {now} - difference is {now - self.emotion_window[0][0]} seconds - time passed: {x}\n")
        while self.emotion_window and (now - self.emotion_window[0][0] > self.window_duration):
            #rospy.loginfo(f"After {now - self.emotion_window[0][0]} seconds and queue has length {len(self.emotion_window)}")

            self.emotion_window.popleft()

            """rospy.loginfo(f"After pop:")
            for t, value in self.emotion_window:
                print(f"\t{now - t}, {value}")
            print("\n")"""

        #if x >= 1.5: self.time_emotion = time.time()

    def run(self):
        """
        This function does emotion recognition.
        First it detects faces using cascade classfier. 
        Then it consider the largest area only.
        If the face detected is at a reasonable distance then analyze that face to get the emotion.
        The emotion will be sended on ros topic: if a face is not detected or the distance is greater than 80cm emotion will be the empty string.
        """
        #rospy.spin()
        # Analysis variables
        start_time = time.time()    # Start the timer
        frame_counter = 0           # Initialize the frame counter
        thread_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        process_executor = ProcessPoolExecutor(max_workers=1) 
        detection = Detection()

        self.time_emotion = time.time()
        while not rospy.is_shutdown():
            # Increment the frame counter
            frame_counter += 1

            # Read a frame from the webcam
            ret, frame = self.cap.read()

            if not ret:
                continue

            # Mirror the frame for display (i.e mediapipe and deepface will analyze the original frame)
            mirrored_frame = cv2.flip(frame, 1)

            # check if a face is in rgb_frame and get it coordinates
            face_detected, face_with_padding, face_coordinates = detection.face_detection_mediapipe(frame)

            # when a face is recognized by mediapipe then it can be analyzed for emotion recognition
            if face_detected is not None:
                head_pose_t = thread_executor.submit(detection.head_pose, mirrored_frame, face_with_padding)
                emotion_t = process_executor.submit(Detection.analyze_emotion, face_detected, face_coordinates, 100)
                
                text_pose, pose_coordinates = head_pose_t.result()  
                result = emotion_t.result()   
                # get most probable emotion
                emotion = result[0]['dominant_emotion'] if result is not None else None
                # draw emotion on frame
                if emotion:
                    x, y, w, h = face_coordinates
                    # just to show clearly on the screen 
                    mirrored_x = frame.shape[1] - x - w
                    # show emotion on screen
                    cv2.putText(mirrored_frame, emotion, (mirrored_x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                    # draw rectangle around the face detected
                    cv2.rectangle(mirrored_frame, (mirrored_x, y), (mirrored_x + w, y + h), (0, 0, 255), 2)
                    #result = self.analyze_emotion(frame, mirrored_frame, face_detected, face_coordinates, 100)
                # rospy.loginfo(f"Time taken for DeepFace analysis: {analysis_time:.2f} seconds -> {emotion}")
                self.handle_emotion(result)

            # Calculate the FPS
            fps = frame_counter / (time.time() - start_time)

            # Display the FPS on the frame
            cv2.putText(mirrored_frame, f"FPS: {fps:.2f}", (30, 30), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 255), 2, cv2.LINE_AA)
            # Display the frame on the screen 
            cv2.imshow("Real-time Emotion Detection", mirrored_frame)

            # Check if the user has pressed the `q` key, if yes then close the program.
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
            emotion_msg.valence = 0 # if not face -> neutral
            emotion_msg.timestamp = time.time()
            # write on topic
            self.full_emotion_publisher.publish(emotion_msg)
            return
        
        # mapping of the emotion to valence
        valence_mapping = {
        'angry': -1,
        'disgust': -1,
        'fear': -1,
        'sad': -1,
        'happy': 1,
        'surprise': 1,
        'neutral': 0
        }
        
        # else a face was found and emotion was analyzed
        dominant_emotion = result[0]['dominant_emotion']
        score_dominant_emotion = result[0]["emotion"][dominant_emotion]
        # create ros msg
        emotion_msg.face_found = True
        emotion_msg.valence = valence_mapping.get(dominant_emotion, 0)
        emotion_msg.model_confidence = score_dominant_emotion
        emotion_msg.dominant_emotion = dominant_emotion
        emotion_msg.timestamp = time.time()

        # rospy.loginfo(f"Written emotion on topic: \n{emotion_msg}")
        self.add_emotion(dominant_emotion)

        # write on topic
        self.full_emotion_publisher.publish(emotion_msg)

if __name__ == '__main__':
    try:
        # init ROS
        rospy.loginfo("[EmotionModule] Initializing node...")
        rospy.init_node('emotion_node', anonymous=True)
        node = EmotionModule()
        node.run()
    except rospy.ROSInterruptException:
        pass