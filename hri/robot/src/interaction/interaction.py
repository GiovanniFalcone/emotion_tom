#!/usr/bin/env python3

"""
Flow of the interaction module:
    1. The robot greets the user and asks for their name. 
    2. The robot explains the rules of the game.
    3. The robot interacts with the user during the game, providing motivational sentences based on the user's emotional state.
    4. The robot ends the interaction with a goodbye message.   
"""

import rospy
from std_msgs.msg import String

import os
import random
import time
import json
import sys

# to access to config file
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from util.util import Util

# robot
from model.interface.robot_interface import RobotInterface
from model.robot_factory import RobotFactory
# Emotion sentence
from sentences.emotion_sentences import EmotionGenerator

class InteractionModule:
    IP_ADDRESS = Util.get_from_json_file("config")['ip']
    FEEDBACK_TYPE = Util.get_from_json_file("config")['feedback_type']
    SKIP_INTRO = Util.get_from_json_file("config")['skip_intro']

    def __init__(self, robot: RobotInterface, language='ita'):
        # initialize variable
        self.robot = robot
        self.language = language
        self.player_name = Util.get_from_json_file("config")['player_name']
        # do randomic movement with robot's head in order to look more natural
        self.robot.random_head_movements()
        # get sentences from interaction file (greetings, rules, goodbye)
        self.speech = self.load_interaction_sentences()
        # get motivational sentences
        self.emotion_sentence = EmotionGenerator(InteractionModule.FEEDBACK_TYPE, self.language)

    ###############################################################################################################
    #                                                   SETTINGS                                                  #
    ###############################################################################################################

    def load_interaction_sentences(self):
        """Get sentences from interaction file."""
        filename = os.path.join(os.path.dirname(__file__), '../sentences', 'interaction', self.language, 'interaction.json')
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

    ###############################################################################################################
    #                                                INTERACTION                                                  #
    ###############################################################################################################

    def start_interaction(self, emotional_condition):
        """BEGIN state"""
        rospy.loginfo(f"[Start] User detected, starting interaction...")
        # if skip_intro is True, the robot will not greet the user 
        if not InteractionModule.SKIP_INTRO:
            self.greetings(emotional_condition)
            self.rules()

    def greetings(self, emotional_condition):
        """The robot will start the interaction."""
        rospy.loginfo("[Greetings] ...")
        sentences = self.speech["greetings_etom"] if emotional_condition else self.speech["greetings"]
        sentence = random.choice(sentences)
        self.speak(sentence)

        # Ask for the user's name if emotional condition is true
        if emotional_condition:
            self._ask_for_player_name()

    def _ask_for_player_name(self):
        """Ask for the player's name and confirm it."""
        while self.player_name in ['', None]:
            rospy.loginfo("[Greetings] Asking for the player's name...")
            self.player_name = self.robot.listen()
            rospy.loginfo(f"[Greetings] Player's name received: {self.player_name}")

            if self.player_name not in [None, '']:
                confirmation_sentence = self.speech["asking_name"] % self.player_name
                self.speak(confirmation_sentence)
                answer = self.robot.listen()
                rospy.loginfo(f"[Greetings] Confirmation answer: {answer}")

                # answer of user is yes/yep/si ...
                if answer.lower() in self.speech["yes"]:
                    break
                else:
                    # otherwise, ask for the name again
                    self.speak(self.speech["repeating_name"])
                    self.player_name = None
                    print("\n")

        greeting_name_sentence = self.speech["greeting_name"] % self.player_name
        self.speak(greeting_name_sentence)

    def rules(self):
        """Robot explain the rules to the user."""
        rospy.loginfo("[Before rules] Robot talking...")
        sentences = self.speech["before_rules"]
        sentence = random.choice(sentences)
        self.speak(sentence)

        rospy.loginfo("[Rules] Robot uttering rules...")
        sentences = self.speech["rules"]
        sentence = random.choice(sentences)
        self.speak(sentence)

    def goodbye(self, emotional_condition):
        """Ending state of the interaction."""
        rospy.loginfo(f"[Goodbye] Waiting a moment before saying goodbye...")
        time.sleep(1.5)

        # based on the condition, choose the appropriate set of sentences
        if emotional_condition: sentences = self.speech["end_etom"]
        else:                   sentences = self.speech["end_tom"]
        
        # get one of the sentences randomly
        sentence = random.choice(sentences)

        # check for placeholders to replace with player name (if any it will be replaced with the player name) 
        placeholders = sentence.count('%s')
        if placeholders > 0:
            sentence = sentence % (self.player_name)

        # speak the sentence
        self.speak(sentence)

    def get_motivational_sentence(self, emotion, n_pairs, match, board_changed=False):
        return self.emotion_sentence.get_sentence(emotion, n_pairs, match, self.player_name, board_changed)

    def speak(self, sentence, **kwargs):
        self.robot.say(sentence, **kwargs)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('interaction_node', anonymous=True)
        # get robot from configuration file and create the instance 
        robot_type = Util.get_from_json_file("config")['robot_type']
        robot = RobotFactory.create_robot(robot_type)
        interaction_node = InteractionModule(robot)
        interaction_node.run()
    except rospy.ROSInterruptException:
        pass