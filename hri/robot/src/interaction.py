#!/usr/bin/env python3

import rospy

import os
import random
import json
import sys

# to access to config file
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'util'))
from util import Util

# robot
from model.interface.robot_interface import RobotInterface
from model.concrete.furhat import Furhat
# Emotion sentence
from sentences.emotion_sentences import EmotionGenerator

class InteractionModule:
    IP_ADDRESS = Util.get_from_json_file("config")['ip']

    def __init__(self, robot: RobotInterface, language='ita'):
        # initialize variable
        self.robot = robot
        self.language = language
        self.player_name = 'gio'
        # do randomic movement with robot's head in order to look more natural
        self.robot.random_head_movements()
        # get sentences from interaction file (greetings, rules, goodbye)
        self.speech = self.load_interaction_sentences()
        # get motivational sentences
        self.emotion_sentence = EmotionGenerator('friendly', self.language)

    ###############################################################################################################
    #                                                   SETTINGS                                                  #
    ###############################################################################################################

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

    ###############################################################################################################
    #                                                INTERACTION                                                  #
    ###############################################################################################################

    def goodbye(self, emotional_condition):
        """Ending state of the interaction."""
        rospy.loginfo(f"Goodbye state...")
        if emotional_condition:
            sentences = self.speech["end_etom"]
        else:
            sentences = self.speech["end_tom"]
        sentence = random.choice(sentences)
        # check for placeholders to replace with player name
        placeholders = sentence.count('%s')
        if placeholders > 0:
            sentence = sentence % (self.player_name)
        self.speak(sentence)

    def start_interaction(self, emotional_condition):
        """BEGIN state"""
        rospy.loginfo("Start interaction...")
        self.greetings(emotional_condition)
        self.rules()

    def greetings(self, emotional_condition):
        """The robot will start the interation."""
        rospy.loginfo("Greetings...")
        if emotional_condition:
            sentences = self.speech["greetings_etom"]
        else:
            sentences = self.speech["greetings"]
        sentence = random.choice(sentences)
        self.speak(sentence)
        # ask name to user if emotional condition
        if emotional_condition:
            while self.player_name in ['', None]:
                rospy.loginfo("Asking name...")
                self.player_name = self.robot.listen()
                rospy.loginfo(f"Player's name is {self.player_name}...")
                if self.player_name not in ['', None]:
                    sentence = self.speech["asking_name"]
                    self.speak(sentence % self.player_name)
                    answer = self.robot.listen()
                    print(f"Answer is {answer}")
                    if answer in ["yes", "si", "certo", "yep", "si si"]:
                        break
                    else:
                        self.speak(self.speech["repeating_name"])
                        self.player_name = None

            self.speak(self.speech["greeting_name"] % self.player_name)

    def rules(self):
        """Robot explain the rules to the user."""
        rospy.loginfo("Before game...")
        sentences = self.speech["before_rules"]
        sentence = random.choice(sentences)
        self.speak(sentence)
        sentences = self.speech["rules"]
        sentence = random.choice(sentences)
        self.speak(sentence)

    def get_motivational_sentence(self, emotion, n_pairs, match):
        return self.emotion_sentence.get_sentence(emotion, n_pairs, match, self.player_name)

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