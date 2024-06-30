#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool

import os
import random
import json
import requests
import sys
import time

# in order to use and save correctly in csv file
from threading import Lock

# robot
from model.interface.robot_interface import RobotInterface
from model.concrete.furhat import Furhat
# Emotion sentence
from sentences.emotion_sentences import EmotionGenerator
# emotion csv
from emotion_csv import EmotionCSV

class InteractionModule:
    def __init__(self, robot: RobotInterface, language='ita'):
        # initialize variable
        self.robot = robot
        self.language = language
        self.state = 'IDLE'
        self.emotional_condition = False
        self.person_detected = False
        self.last_emotion = ''
        # Get id player for csv: analysis for emotion
        self.id_player = self.get_player_id()
        # game info for csv file
        self.turn = 0
        self.match = False
        self.motivated = False
        self.lock = Lock()
        self.logger = EmotionCSV(self.id_player)
        # get sentences from interaction file (greetings, rules, goodbye)
        self.speech = self.load_interaction_sentences()
        # Ros topic
        rospy.Subscriber('/speech_hint', String, self.speech_callback)
        rospy.Subscriber('/game_data', String, self.game_callback)
        rospy.Subscriber("/emotion", String, self.emotion_callback)
        rospy.Subscriber('/person_detected', Bool, self.person_detected_callback)
        # connect to robot and do randomic movement with robot's head in order to look more natural
        self.robot.connect()
        self.robot.random_head_movements()
        # get motivational sentences
        self.emotion_sentence = EmotionGenerator('friendly', self.language)
        # get experimental condition
        self.get_emotion_condition()

    ###############################################################################################################
    #                                                   SETTINGS                                                  #
    ###############################################################################################################

    def get_player_id(self):
        """Returns id player from Flask server. Used for csv file."""
        url = "http://192.168.1.94:5000/get_id"
        try:
            response = requests.get(url)
            response.raise_for_status()  # Solleva un'eccezione per errori HTTP
            data = response.json()
            return data.get('id')
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"HTTP request error: {e}")
            return None
        
    def load_interaction_sentences(self):
        """Get sentences from interaction file."""
        filename = os.path.join(os.path.dirname(__file__), 'sentences', 'interaction', self.language, 'interaction.json')
        try:
            with open(filename, 'r', encoding='utf-8') as file:
                data = json.load(file)
            return data
        except FileNotFoundError:
            rospy.logerr(f"File {filename} not found.")
            return {}
        except json.JSONDecodeError:
            rospy.logerr(f"Error decoding JSON file {filename}.")
            return {}
    
    def get_emotion_condition(self):
        """Set experimental contion from ROS parameter."""
        try:
            self.emotional_condition = bool(rospy.get_param("emotion_condition"))
            print(f"Emotion condition: {self.emotional_condition}")
        except KeyError:
            rospy.logerr(
                "Usage: roslaunch robot controller.launch emotion_condition:=<value> (where value can be true or false)")
            sys.exit(1)

    ###############################################################################################################
    #                                                  CALLBACKS                                                  #
    ###############################################################################################################    
    
    def person_detected_callback(self, msg):
        """
        If this callback is triggered, it means that the user is in the robot's field of view, so the robot can initiate the interaction. 
        After finishing the conversation, it enters the game_state (by setting the corresponding variable).
        """
        if msg.data and self.state == 'IDLE': 
            self.person_detected = True
            #self.start_interaction()
            self.state = 'GAME'
        
    def speech_callback(self, data):
        """
        Robot will utter the suggestion.
        Once the robot has uttered the suggestion an http request is sent to the server in order to remove the pop-up.
        """
        rospy.loginfo(f"Hint Received: {data.data}")
        # utter the suggestion
        self.speak(data.data)
        # send to Flask
        json_data = ({"speech": "ended"})
        try:
            requests.post("http://192.168.1.94:5000/robot_speech", json=json_data)
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"HTTP error request: {e}")

    def emotion_callback(self, data):
        """Save the emotion received"""
        rospy.loginfo(f"Emotion Received: {data.data}")
        with self.lock:
            self.last_emotion = data.data
        self.logger.log_to_csv(rospy.get_time(), self.logger.csv_file_full, self.last_emotion, self.match, self.motivated)

    def game_callback(self, data):
        """
        If this callback is triggered, it means the user has made a move. 
        If the move is the last one of the game, the robot enters the final state (Goodbye) and says goodbye to the user. 
        Otherwise, if emotional condition is setted, the robot will motivate the user based on user's emotion.
        """
        rospy.loginfo(f"Game data Received: {data.data}")
        move = json.loads(data.data) 
        # get info game 
        n_pairs = move['game']['pairs']
        is_game_ended = n_pairs == 12
        if is_game_ended:
            self.state = 'END'
            self.goodbye()
        else:
            if self.emotional_condition:
                self.handle_turn(move)

    ###############################################################################################################
    #                                                INTERACTION                                                  #
    ###############################################################################################################

    def goodbye(self):
        """Ending state of the interaction."""
        rospy.loginfo(f"Goodbye state...")
        sentences = self.speech["end"]
        sentence = random.choice(sentences)
        self.speak(sentence)

    def handle_turn(self, move):
        """
        Depending on the outcome of the move (whether the user found a pair or not), 
        the robot will motivate the user based on user's emotion, 
        by uttering a motivational sentence, making a facial expression, and changing the LED color.
        """
        rospy.loginfo(f"Handle turn...")
        # get info about game
        n_pairs = move['game']['pairs']
        turn = move['game']['turn']
        match = move['game']['match']
        is_turn_even = turn % 2 == 0
        # motivate user only after the outcome of user's move
        if is_turn_even:
            # copy info
            with self.lock:
                # copy last emotion
                emotion = self.last_emotion
                self.turn = turn
                self.match = match
            # Set to 'neutral' if the emotion hasn't been detected.
            if emotion == '': emotion = 'neutral'
            # if emotion is 'fear' we take it as a classification error. Therefore, we set it to 'neutral'
            if emotion == 'fear': emotion = 'neutral'
            # if one of the current emotion is detected after a match, we consider the emotion detected as a classification error
            if match and emotion in ['sad', 'angry', 'fear']:
                emotion = 'neutral'
            # debug
            print(f"Emotion after match: {emotion}")
            # define probability for robot's motivational speech
            probability = 0.75 if match else 0.3
            probability = 1 if (match and emotion == 'happy' or n_pairs == 1) else 0.75
            # with 50% of chance the robot will motivate if user has found a pair (i.e match is True), 
            # otherwise the chance that robot will motivate are 25%
            if random.random() < probability:
                self.robot.change_led_color_based_on_emotion(emotion)
                self.handle_expression_based_on_emotion(emotion, n_pairs, match)
                motivational_sentence = self.emotion_sentence.get_sentence(emotion, n_pairs, match)
                print("Motivation: ", motivational_sentence)
                self.speak(motivational_sentence)
                self.robot.change_led_color_based_on_emotion("neutral")
                # Log to CSV
                with self.lock: 
                    self.motivated = True
                self.logger.log_to_csv(rospy.get_time(), self.logger.csv_file_filtered, emotion, match, 'yes')
            else:
                self.logger.log_to_csv(rospy.get_time(), self.logger.csv_file_filtered, emotion, match, 'no')
                with self.lock:
                    self.motivated = False

    def handle_expression_based_on_emotion(self, emotion, n_pairs, match):
        # naturalhappy -> 0
        # indefinitehappy -> 1
        is_first_pair = n_pairs == 1
        is_begin = n_pairs < 4
        is_middle = 3 < n_pairs < 8
        is_end = n_pairs >= 8
        # random condition in order to perform a different gesture
        prob = random.choice([0, 1]) == 0 

        if match:
            if emotion == "happy":
                if(is_begin or is_middle or is_first_pair):
                    if prob:
                        self.robot.do_facial_expression("happy_0")
                        time.sleep(0.5)
                        if random.choice([True, False]):
                            self.robot.do_facial_expression("Wink")
                        else:
                            self.robot.do_facial_expression("BigSmile")
                    else:
                        self.robot.do_facial_expression("happy_1")
                        time.sleep(0.5)
                        self.robot.do_facial_expression("BigSmile")
                else:
                    if random.choice([True, False]):
                        self.robot.do_facial_expression("happy_0")
                    else:
                        self.robot.do_facial_expression("happy_1")
                    self.robot.do_facial_expression("BigSmile")
            elif emotion == 'neutral':
                if is_first_pair:
                    if random.choice([True, False]):
                        self.robot.do_facial_expression("happy_0")
                    else:
                        self.robot.do_facial_expression("happy_1")
                    self.robot.do_facial_expression("BigSmile")
                else:
                    if random.choice([True, False]):
                        self.robot.do_facial_expression("happy_0")
                        self.robot.do_facial_expression("BigSmile")
                    else:
                        self.robot.do_facial_expression("happy_1")
                        if not is_end: time.sleep(0.5)
                        self.robot.do_facial_expression("Wink")
            elif emotion == 'surprise':
                self.robot.do_facial_expression("CustomSurprise")
        else:
            if emotion in ['happy', 'neutral']:
                if random.choice([True, False]):
                    self.robot.do_facial_expression("happy_0")
                else:
                    self.robot.do_facial_expression("happy_1")
                self.robot.do_facial_expression("BigSmile")
            elif emotion == 'sad':
                if(is_begin or is_end):
                    self.robot.do_facial_expression("CustomSad")
                    self.robot.do_facial_expression("BigSmile")
                    time.sleep(5)
                    self.robot.do_facial_expression("BigSmile")
                else:
                    self.robot.do_facial_expression("CustomSad")
                    self.robot.do_facial_expression("Wink")
            else:
                self.robot.do_facial_expression("CustomSad")
                time.sleep(5)
                self.robot.do_facial_expression("BigSmile")

    def start_interaction(self):
        """BEGIN state"""
        rospy.loginfo("Start interaction...")
        self.greetings()
        self.rules()

    def greetings(self):
        """The robot will start the interation."""
        rospy.loginfo("Greetings...")
        sentences = self.speech["greetings"]
        sentence = random.choice(sentences)
        self.speak(sentence)

    def rules(self):
        """Robot explain the rules to the user."""
        rospy.loginfo("Before game...")
        sentences = self.speech["before_rules"]
        sentence = random.choice(sentences)
        self.speak(sentence)
        sentences = self.speech["rules"]
        sentence = random.choice(sentences)
        self.speak(sentence)

    def speak(self, sentence):
        self.robot.say(sentence)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('interaction_node', anonymous=True)
    robot = Furhat()
    interaction_node = InteractionModule(robot)
    interaction_node.run()
