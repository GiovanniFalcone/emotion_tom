#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool

import os
import random
import json
import requests
import sys

# in order to use and save correctly in csv file
from threading import Lock

# to access to config file
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'util'))
from util import Util

# robot
from model.interface.robot_interface import RobotInterface
from model.concrete.furhat import Furhat
# Emotion sentence
from sentences.emotion_sentences import EmotionGenerator
# Emotion handler: handle facial expression based on emotion and game
from emotion_handler import EmotionHandler
# emotion csv
from emotion_csv import EmotionCSV

class InteractionModule:
    IP_ADDRESS = Util.get_from_json_file("config")['ip']

    def __init__(self, robot: RobotInterface, language='ita'):
        # initialize variable
        self.robot = robot
        self.language = language
        self.state = 'IDLE'
        self.emotional_condition = False
        self.person_detected = False
        self.last_emotion = ''
        # Get id player for csv: analysis for emotion
        self.id_player = ''
        # game info for csv file
        self.turn = 0
        self.match = False
        self.motivated = False
        self.lock = Lock()
        self.logger = None 
        # get sentences from interaction file (greetings, rules, goodbye)
        self.speech = self.load_interaction_sentences()
        # connect to robot and do randomic movement with robot's head in order to look more natural
        self.robot.connect()
        self.robot.random_head_movements()
        # get motivational sentences
        self.emotion_sentence = EmotionGenerator('friendly', self.language)
        # get experimental condition
        self.get_emotion_condition()
        # emotion object
        self.emotion_handler = EmotionHandler(self.robot)
        # Ros topic
        rospy.Subscriber('/speech_hint', String, self.speech_callback)
        rospy.Subscriber('/game_data', String, self.game_callback)
        rospy.Subscriber("/emotion", String, self.emotion_callback)
        rospy.Subscriber('/person_detected', Bool, self.person_detected_callback)

    ###############################################################################################################
    #                                                   SETTINGS                                                  #
    ###############################################################################################################

    def get_player_id(self):
        """Returns id player from Flask server. Used for csv file."""
        url = "http://" + InteractionModule.IP_ADDRESS + ":5000/get_id"
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
            self.start_interaction()
            self.id_player = self.get_player_id()
            self.logger = EmotionCSV(self.id_player, self.emotional_condition)
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
            requests.post("http://" + InteractionModule.IP_ADDRESS + ":5000/robot_speech", json=json_data)
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"HTTP error request: {e}")

    def emotion_callback(self, data):
        """Save the emotion received"""
        rospy.loginfo(f"Emotion Received: {data.data}")
        with self.lock:
            self.last_emotion = data.data
        if self.logger: 
            self.logger.log_to_csv(rospy.get_time(), "full", self.last_emotion, self.match, self.motivated)

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
            # if true, the robot will motivate the user based on their emotion (False -> ToM condition only)
            if self.emotional_condition:
                self.handle_turn(move)

    ###############################################################################################################
    #                                                INTERACTION                                                  #
    ###############################################################################################################

    def goodbye(self):
        """Ending state of the interaction."""
        rospy.loginfo(f"Goodbye state...")
        if self.emotional_condition:
            sentences = self.speech["end_etom"]
        else:
            sentences = self.speech["end_tom"]
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
            
            # set to 'neutral' if 
            #   - the emotion hasn't been detected 
            #   - the emotion is fear: we take it as classification error
            #   - the emotion is fear/angry/fear while user find a paid -> we take it as classification error
            if emotion in ['fear', ''] or (match and emotion in ['sad', 'angry', 'fear']):
                emotion = 'neutral'

            # debug
            rospy.loginfo(f"Emotion after match: {emotion}")

            # define probability for robot's motivational speech
            probability = 0.75 if match else 0.3
            probability = 1 if (match and emotion == 'happy' or n_pairs == 1) else 0.75

            # with 50% of chance the robot will motivate if user has found a pair (i.e match is True), 
            # otherwise the chance that robot will motivate are 30%
            if random.random() < probability:
                self.robot.change_led_color_based_on_emotion(emotion)
                self.emotion_handler.handle_expression_based_on_emotion(emotion, n_pairs, match)
                motivational_sentence = self.emotion_sentence.get_sentence(emotion, n_pairs, match)
                rospy.loginfo(f"Robot uttering: {motivational_sentence}...")
                self.speak(motivational_sentence)
                self.robot.change_led_color_based_on_emotion("neutral")

                # Log to CSV
                with self.lock: 
                    self.motivated = True
                self.logger.log_to_csv(rospy.get_time(), "filtered", emotion, match, 'yes')
            else:
                self.logger.log_to_csv(rospy.get_time(), "filtered", emotion, match, 'no')
                with self.lock:
                    self.motivated = False
                # Perform a facial expression based on match
                self.robot.do_facial_expression("Nod" if match else "Shake")        

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
    try:
        rospy.init_node('interaction_node', anonymous=True)
        robot = Furhat()
        interaction_node = InteractionModule(robot)
        interaction_node.run()
    except rospy.ROSInterruptException:
        pass