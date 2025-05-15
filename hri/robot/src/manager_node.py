#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool, Int32
from emotion.msg import emotion

import os
import json
import requests
import sys

# to access to config file
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))
from util.util import Util

# robot
from model.interface.robot_interface import RobotInterface
from model.robot_factory import RobotFactory
# interaction functions
from interaction.interaction import InteractionModule
# feedback module
from feedback.feedback import Feedback
# emotion csv
from emotion_csv import EmotionCSV

class ManagerNode:
    IP_ADDRESS = Util.get_from_json_file("config")['ip']
    RECORD_ALL_GAME = Util.get_from_json_file("config")["record_all_game"]

    def __init__(self, robot: RobotInterface, language='ita'):
        # Initialize variables
        self.robot = robot                              # Robot object
        self.language = language                        # Language used by the robot
        self.emotional_condition = False                # Experimental condition
        self.id_player = -1                             # ID player for CSV: analysis for emotion

        # Emotion CSV object
        self.logger = None                              

        # Interaction module (must be initialized after robot connection!)
        self.interaction = InteractionModule(robot, language)

        # Get experimental condition
        self._get_emotion_condition()

        # ROS topics
        self._initialize_ros_subscribers()

        # feedback module
        self.feedback = Feedback(ManagerNode.IP_ADDRESS, ManagerNode.RECORD_ALL_GAME, self.robot, self.emotional_condition, self.interaction)

    ###############################################################################################################
    #                                                   SETTINGS                                                  #
    ###############################################################################################################

    def _initialize_ros_subscribers(self):
        """Initialize ROS subscribers."""
        # the following topics are used for all the conditions
        rospy.Subscriber('/speech_hint', String, self.speech_callback)
        rospy.Subscriber('/person_detected', Bool, self.person_detected_callback)
        rospy.Subscriber('/start', Int32, self.game_started)
        # the following topics are used only for emotional condition (compare E-ToM and ToM)
        rospy.Subscriber('/full_emotion', emotion, self.emotion_callback)
        rospy.Subscriber('/game_data', String, self.game_callback)

    def _get_emotion_condition(self):
        """Set experimental contion from ROS parameter."""
        try:
            self.emotional_condition = bool(rospy.get_param("emotion_condition"))
            rospy.loginfo(f"[Manager] Emotion condition: {self.emotional_condition}")
        except KeyError:
            rospy.logerr(
                "Usage: roslaunch robot controller.launch emotion_condition:=<value> (where value can be true or false)")
            sys.exit(1)

    def _send_to_flask_robot_has_finished_to_speak(self, json_data):
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
        if msg.data:
            self.interaction.start_interaction(self.emotional_condition)
        
    def game_started(self, msg):
        """
        If this callback is trigger, it means that game page has been showed and so the user can starts to play.
        It is also used in order to understand when the emotion can be saved on csv.
        """
        # get player id from topic
        self.id_player = msg.data

        # debug
        print("\n")
        rospy.loginfo(f"[Init] ID player received: {self.id_player}...")

        # initialize csv file for current player
        self.logger = EmotionCSV(self.id_player, self.emotional_condition)
        self.logger.set_starting_time()

        # update feedback module with the id player and logger
        self.feedback.set_logger(self.logger)
        self.feedback.set_id_player(self.id_player)

        # debug
        if self.logger: rospy.loginfo(f"[Init] Emotion csv initialized...\n")

    def speech_callback(self, data):
        """
        Robot will utter the suggestion.
        Once the robot has uttered the suggestion an http request is sent to the server in order to remove the pop-up.
        This functions works wheater the condition is emotional or not.
        """
        # deserialize json
        json_data = json.loads(data.data)
        # rospy.loginfo(f"[Hint] {json_data}")
        sentence = json_data["action"]["sentence"]
        flip_type = json_data["action"]["flip_type"]
        # if hint is provided for first flip 
        self.feedback.handle_first_flip(flip_type == "firstCard")
        # if hint is provided then utter it
        if sentence != '':
            rospy.loginfo(f"[Hint] Hint Received: {sentence}")
            # utter the suggestion
            self.interaction.speak(sentence)
            # send to Flask
            json_data = ({"speech": "ended"})
            self._send_to_flask_robot_has_finished_to_speak(json_data)
    
    def emotion_callback(self, data):
        """
        Process the received emotion data and delegate handling to the Feedback module.
        This function ensures emotions are saved and processed correctly based on the game state 
        and the logger's initialization status.

        Args:
            data: ROS message containing emotion data (dominant_emotion, valence, model_confidence).
        """
        emotion = data.dominant_emotion
        
        # Skip processing if no emotion is detected
        if not emotion:
            return

        # Get timestamp and game time if logger is initialized
        if not self.logger:
            rospy.logwarn("[Emotion callback] Logger not initialized. Skipping emotion processing.")
            return

        # handle emotion msg for feedback
        self.feedback.handle_emotion(data)
        
    def game_callback(self, game_data):
        """
        Triggered when the user makes a move in the game.
        """
        self.feedback.handle_game(game_data)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('interaction_node', anonymous=True)
        # get robot from configuration file and create the instance 
        robot_type = Util.get_from_json_file("config")['robot_type']
        robot = RobotFactory.create_robot(robot_type)
        # connect to robot 
        robot.connect()
        rospy.loginfo(f"[Manager] Connection with '{robot_type}' successfully established!")
        # if connection is ok than start
        manager_node = ManagerNode(robot)
        manager_node.run()
    except rospy.ROSInterruptException:
        pass