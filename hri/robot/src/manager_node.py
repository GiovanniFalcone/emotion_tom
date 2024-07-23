#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool, Int32
from emotion.msg import emotion
from message_filters import ApproximateTimeSynchronizer, Subscriber

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
# interaction functions
from interaction import InteractionModule
# Emotion handler: handle facial expression based on emotion and game
from emotion_handler import EmotionHandler
# emotion csv
from emotion_csv import EmotionCSV

class ManagerNode:
    IP_ADDRESS = Util.get_from_json_file("config")['ip']

    def __init__(self, robot: RobotInterface, language='ita'):
        # initialize variable
        self.robot = robot
        self.language = language
        self.state = 'IDLE'
        self.emotional_condition = False
        self.last_emotion = ''
        self.emotion_score = 0
        # if true than the robot can say motivational sentence
        self.is_hint_first_flip = False
        # id player for csv: analysis for emotion
        self.id_player = -1
        # game info for csv file
        self.turn = 1
        self.match = False
        self.motivated = 'no'
        self.lock = Lock()
        self.logger = None 
        # interaction module (must be initialized after robot connection!)
        self.interaction = InteractionModule(robot, language)
        # get experimental condition
        self.get_emotion_condition()
        # emotion object
        self.emotion_handler = EmotionHandler(self.robot)
        # Ros topic
        rospy.Subscriber('/speech_hint', String, self.speech_callback)
        rospy.Subscriber("/full_emotion", emotion, self.emotion_callback)
        rospy.Subscriber('/person_detected', Bool, self.person_detected_callback)
        rospy.Subscriber('/start', Int32, self.game_started)
        # Synchonized emotion topic with game info
        self.filtered_emotion = Subscriber("/filtered_emotion", emotion)
        self.game = Subscriber('/game_data', String)
        self.sync = ApproximateTimeSynchronizer([self.filtered_emotion, self.game], queue_size=100, slop=10, allow_headerless=True)
        self.sync.registerCallback(self.handle_emotional_intelligence)

    ###############################################################################################################
    #                                                   SETTINGS                                                  #
    ###############################################################################################################

    def get_emotion_condition(self):
        """Set experimental contion from ROS parameter."""
        try:
            self.emotional_condition = bool(rospy.get_param("emotion_condition"))
            print(f"Emotion condition: {self.emotional_condition}")
        except KeyError:
            rospy.logerr(
                "Usage: roslaunch robot controller.launch emotion_condition:=<value> (where value can be true or false)")
            sys.exit(1)

    def send_to_flask_robot_has_finished_to_speak(self, json_data):
        try:
            requests.post("http://" + ManagerNode.IP_ADDRESS + ":5000/robot_speech", json=json_data)
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"HTTP error request: {e}")

    ###############################################################################################################
    #                                                  CALLBACKS                                                  #
    ###############################################################################################################    
    
    def person_detected_callback(self, msg):
        """
        If this callback is triggered, it means that the user is in the robot's field of view, so the robot can initiate the interaction. 
        After finishing the conversation, it enters the game_state (by setting the corresponding variable).
        """
        if msg.data and self.state == 'IDLE': 
            self.state = 'GAME'
            #self.interaction.start_interaction(self.emotional_condition)
        
    def game_started(self, msg):
        """
        If this callback is trigger, it means that game page has been showed and so the user can starts to play.
        It is also used in order to understand when the emotion can be saved on csv.
        """
        self.id_player = msg.data
        rospy.loginfo(f"ID player received: {self.id_player}...")
        self.logger = EmotionCSV(self.id_player, self.emotional_condition)
        if self.logger:
            rospy.loginfo(f"Emotion csv initialized...\n\n")

    def speech_callback(self, data):
        """
        Robot will utter the suggestion.
        Once the robot has uttered the suggestion an http request is sent to the server in order to remove the pop-up.
        """
        # deserialize json
        json_data = json.loads(data.data)
        rospy.loginfo(json_data)
        sentence = json_data["action"]["sentence"]
        flip_type = json_data["action"]["flip_type"]
        # if hint is provided for first flip 
        with self.lock:
            self.is_hint_first_flip = (flip_type == "firstCard")
        # if hint is provided then utter it
        if sentence != '':
            rospy.loginfo(f"Hint Received: {sentence}")
            # utter the suggestion
            self.interaction.speak(sentence)
            # send to Flask
            json_data = ({"speech": "ended"})
            self.send_to_flask_robot_has_finished_to_speak(json_data)
            

    def emotion_callback(self, data):
        """Save the emotion received"""
        #rospy.loginfo(f"Emotion Received: {data.dominant_emotion}")
        emotion = data.dominant_emotion
        emotion_score = data.model_confidence
        if emotion != '':
            with self.lock:
                if self.logger and self.state != 'END': 
                    self.logger.log_to_csv(self.id_player, "full", emotion, 
                                           emotion_score, self.match, self.turn, self.motivated)

    def handle_emotional_intelligence(self, emotion_data, game_data):
        """
        If this callback is triggered, it means the user has made a move. 
        If the move is the last one of the game, the robot enters the final state (Goodbye) and says goodbye to the user. 
        Otherwise, if emotional condition is setted, the robot will motivate the user based on user's emotion.
        """
        rospy.loginfo(f"Game data Received: {game_data.data}")
        rospy.loginfo(f"Emotion Received: {emotion_data.dominant_emotion}")

        move = json.loads(game_data.data) 
        # get info game 
        n_pairs = move['game']['pairs']
        is_game_ended = n_pairs == 12
        if is_game_ended:
            self.state = 'END'
            self.interaction.goodbye(self.emotional_condition)
            self.interaction.player_name = ''
        else:
            self.handle_turn(move, emotion_data)

    ###############################################################################################################
    #                                                   HANDLER                                                   #
    ###############################################################################################################

    def handle_turn(self, move, emotion_data):
        """
        Depending on the outcome of the move (whether the user found a pair or not), 
        the robot will motivate the user based on user's emotion, 
        by uttering a motivational sentence, making a facial expression, and changing the LED color.
        """
        rospy.loginfo(f"Handle move with emotion...")
        # get info about game
        n_pairs = move['game']['pairs']
        turn = move['game']['turn']
        match = move['game']['match']
        is_turn_even = turn % 2 == 0

        # copy info
        with self.lock:
            self.turn = turn
            self.match = match

        # do not speak if user has not found a pair in the first 4 turns
        if not match and turn < 3: 
            print("\n")
            return

        # motivate user only after the outcome of user's move
        if is_turn_even:
            emotion = emotion_data.dominant_emotion
            emotion_score = emotion_data.model_confidence
            
            # set to 'neutral' if 
            #   - the emotion hasn't been detected 
            #   - the emotion is fear: we take it as classification error
            #   - the emotion is fear/angry/fear while user find a paid -> we take it as classification error
            if emotion in ['fear', ''] or (match and emotion in ['sad', 'angry', 'fear']):
                emotion = 'neutral'

            # debug
            rospy.loginfo(f"Emotion after move: {emotion}")

            # define probability for robot's motivational speech
            probability = 0.75 if match else 0.25
            probability = 1 if (match and (emotion == 'happy' or emotion == 'surprise') or n_pairs == 1) else 0.75

            # checking if user has received a suggestion on first flip
            with self.lock:
                is_hint_provided_on_first_flip = self.is_hint_first_flip
            if is_hint_provided_on_first_flip: rospy.loginfo("Robot can't motivate because of hint provided on first flip...")

            # if true, the robot will motivate the user based on their emotion (False -> ToM condition only)
            if not self.emotional_condition:
                with self.lock:
                    self.motivated = 'no'
                # Log to CSV
                self.logger.log_to_csv(self.id_player, "filtered", emotion, emotion_score, match, turn, 'yes')
                return 

            # if probability and hint is not provided on first flip then motivate user
            if random.random() < probability and not is_hint_provided_on_first_flip:
                self.robot.change_led_color_based_on_emotion(emotion)
                self.emotion_handler.handle_expression_based_on_emotion(emotion, n_pairs, match)
                motivational_sentence = self.interaction.get_motivational_sentence(emotion, n_pairs, match)
                rospy.loginfo(f"Robot uttering: '{motivational_sentence}'...")
                self.interaction.speak(motivational_sentence)
                self.robot.change_led_color_based_on_emotion("")

                with self.lock:
                    self.motivated = 'yes'
                # Log to CSV
                self.logger.log_to_csv(self.id_player, "filtered", emotion, emotion_score, match, turn, 'yes')
            else:
                with self.lock:
                    self.motivated = 'no'
                self.logger.log_to_csv(self.id_player, "filtered", emotion, emotion_score, match, turn, 'no')
                # Perform a facial expression based on match
                self.robot.do_facial_expression("Nod" if match else "Shake")   
            self.is_hint_first_flip = False  
            print("\n")
        else:
            print("\n")

    def sync_emotion_game_on_csv(self, move, emotion_data):
        rospy.loginfo(f"Handle move with emotion...")
        turn = move['game']['turn']
        match = move['game']['match']
        is_turn_even = turn % 2 == 0

        # copy info
        with self.lock:
            self.turn = turn
            self.match = match

        # do not speak if user has not found a pair in the first 4 turns
        if not match and turn < 3: 
            print("\n")
            return

        # motivate user only after the outcome of user's move
        if is_turn_even:
            emotion = emotion_data.dominant_emotion
            emotion_score = emotion_data.model_confidence
            
            # set to 'neutral' if 
            #   - the emotion hasn't been detected 
            #   - the emotion is fear: we take it as classification error
            #   - the emotion is fear/angry/fear while user find a paid -> we take it as classification error
            if emotion in ['fear', ''] or (match and emotion in ['sad', 'angry', 'fear']):
                emotion = 'neutral'


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('interaction_node', anonymous=True)
        robot = Furhat()
        # connect to robot 
        robot.connect()
        # if connection is ok than start
        manager_node = ManagerNode(robot)
        manager_node.run()
    except rospy.ROSInterruptException:
        pass