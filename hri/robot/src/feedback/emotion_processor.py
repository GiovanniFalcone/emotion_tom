"""
This module processes the detected emotions and valence values from the user.
It maintains a sliding window of the last 30 frames of emotions and valence values to compute the most probable emotion class (Negative, Neutral, Positive) 
and the most frequent emotion within that class. 

The module also calculates the probability of providing feedback based on the game state and user interactions.

It uses a deque to maintain the sliding window of emotions and valence values, ensuring thread safety with a lock.

This class is located in the feedback module (and not in the emotion module) since it is responsible for analyzing the user's emotional state 
over time and after the user has clicked a card.

In summary, this module is responsible for analyzing the user's emotional state over time and determining the most appropriate feedback to provide based on that analysis.
"""

import rospy
import time
import numpy as np

from collections import deque
from threading import Lock

class EmotionProcessor:
    def __init__(self):
        # Emotion window to analyze in order to get the most probable emotion
        self.emotion_window_names = deque(maxlen=30)    # 30 frames (as in the paper)
        self.emotion_window_valence = deque(maxlen=30)  # 30 frames (as in the paper)

        # Weights for each emotion (30 emotions)
        self.weights = np.array([np.arange(1, 31) / 465])  # 465 = sum of weights (1+2+3+...+30) where 30 is the number of frames

        # Lock for emotion window to ensure thread safety
        self.lock_window = Lock()                       

    def add_emotion_to_window(self, emotion, valence):
        """
        This function adds the detected emotion and valence to the emotion window.
        It uses a lock to ensure thread safety when accessing the emotion window.
        """
        with self.lock_window:
            self.emotion_window_names.append(emotion)
            self.emotion_window_valence.append(valence)

    def get_emotion(self):
        """
        Retrieves the most frequent emotion within a specific time window (30 frames) based on the formula provided in the paper.
        Determines the emotion class (Negative, Neutral, Positive) based on the score range defined in the paper.
        Then, identifies the discrete emotion within that class using weighted frequencies.
        """
        # wait some seconds after click
        rospy.loginfo(f"[Feedback] I'm gonna wait 1/2 second...\n")
        time.sleep(0.75)
        # during the wait, queue is updated with new emotions (the ones after the click), so now we can analyze it
        with self.lock_window:
            #rospy.loginfo(f"(Turn) queue is: {self.emotion_window_valence}")
            emotion_names = self.emotion_window_names
            emotion_valence = self.emotion_window_valence 

        # if 30 frames are not analyzed yet, return None
        if(len(emotion_names) < 30):    return None, 0
        
        emotion_score = np.dot(self.weights, emotion_valence)[0]
        
        emotion_class = ''
        if -1 <= emotion_score < -0.1:
            emotion_class = "Negative"
        elif -0.1 <= emotion_score <= 0.1:
            emotion_class =  "Neutral"
        elif 0.1 < emotion_score <= 1:
            emotion_class = "Positive"

        # Get the most frequent emotion in the determined class
        most_frequent_emotion = self._get_most_frequent_emotion(emotion_class, emotion_names, emotion_valence)
        print("\n")
        rospy.loginfo(f"[Feedback] Emotion score: {emotion_score}, classified as: {emotion_class}, most frequent emotion: {most_frequent_emotion}")
    
        return most_frequent_emotion, emotion_score

    def _get_most_frequent_emotion(self, emotion_class, emotion_names, emotion_valence):
        """
        Get the most frequent emotion (based on weights) within a specific class from emotion_names.
        """
        # get valance class
        class_val = {"Negative": -1, "Neutral": 0, "Positive": 1}[emotion_class]
        # dictionary to store the weights of each emotion
        emotion_weights = {}

        for idx, (name, val) in enumerate(zip(emotion_names, emotion_valence)):
            if val == class_val:
                weight = self.weights[0][idx]  # weight is a np.array([[...]])
                emotion_weights[name] = emotion_weights.get(name, 0) + weight

        # return emotion with the highest weight
        if emotion_weights: return max(emotion_weights.items(), key=lambda x: x[1])[0]
        else:               return None

    