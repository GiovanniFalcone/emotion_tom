#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool

import os
import random
import json
import requests
import sys
import csv
from datetime import datetime

# robot
from model.interface.robot_interface import RobotInterface
from model.concrete.furhat import Furhat
# Emotion sentence
from sentences.emotion_sentences import EmotionGenerator

class InteractionModule:
    def __init__(self, robot: RobotInterface, language = 'ita'):
        def __init__(self, robot: RobotInterface, language='ita'):
            # initialize variable
            self.robot = robot
            self.language = language
            self.state = 'IDLE'
            self.emotional_condition = False
            self.person_detected = False
            self.last_emotion = ''
            # initialize csv file
            self.init_csv()
            # get sentences from interaction file (greetings, rules, goodbye)
            self.speech = self.load_interaction_sentences()
            # Get id player for csv: analysis for emotion
            self.id_player = self.get_player_id()
            # Ros topic
            rospy.Subscriber('/speech_hint', String, self.speech_callback)
            rospy.Subscriber('/game_data', String, self.game_callback)
            rospy.Subscriber("/emotion", String, self.handle_emotion)
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
            print(f"Errore nella richiesta HTTP: {e}")
            return None
        
    def load_interaction_sentences(self):
        """Get sentences from interaction file."""
        filename = os.path.join(os.path.dirname(__file__), 'sentences', 'interaction', self.language, 'interaction.json')
        try:
            with open(filename, 'r', encoding='utf-8') as file:
                data = json.load(file)
            return data
        except FileNotFoundError:
            print(f"File {filename} not found.")
            return {}
        except json.JSONDecodeError:
            print(f"Error decoding JSON file {filename}.")
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

    def init_csv(self):
        """Initialize the CSV file with headers."""
        # get current dir
        script_dir = os.path.dirname(__file__) 
        # get path to data directory
        user_dir = "user_" + self.id_player
        data_dir = os.path.abspath(os.path.join(script_dir, '..', '..', 'app', 'src', 'data', user_dir))
        if not os.path.exists(data_dir):
            raise FileNotFoundError(f"The directory {data_dir} does not exist. Run the app.launch first!")
        # save the csv in the player directory
        self.csv_file = os.path.join(data_dir, 'emotion_data.csv')
        # Stampa del percorso assoluto del file CSV
        print(f"Absolute path of the CSV file: {self.csv_file}")
        headers = ['id', 'timestamp', 'emotion', 'match', 'motivated', 'condition']
        with open(self.csv_file, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(headers)
        rospy.loginfo(f"CSV file initialized at {self.csv_file}")

    def log_to_csv(self, emotion, action_robot, result_action):
        """Log the interaction data to the CSV file."""
        timestamp = rospy.get_time()
        dt_object = datetime.fromtimestamp(timestamp)
        formatted_time = dt_object.strftime('%Y-%m-%d %H:%M:%S.%f') 
        condition = "E-ToM" if self.emotional_condition else "ToM"
        with open(self.csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([formatted_time, emotion, action_robot, result_action, condition])

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
        requests.post("http://192.168.1.94:5000/robot_speech", json=json_data)

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
            # copy last emotion
            emotion = self.last_emotion
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
            # with 50% of chance the robot will motivate if user has found a pair (i.e match is True), 
            # otherwise the chance that robot will motivate are 25%
            if random.random() < probability:
                self.robot.change_led_color_based_on_emotion(emotion)
                if emotion != 'happy':
                    self.robot.do_facial_expression("ExpressSad")
                else:
                    self.robot.do_facial_expression("BigSmile")
                motivational_sentence = self.emotion_sentence.get_sentence(emotion, n_pairs, match)
                print("Motivation: ", motivational_sentence)
                self.speak(motivational_sentence)
                self.robot.change_led_color_based_on_emotion("neutral")
                # Log to CSV
                self.log_to_csv(emotion, match, 'yes')
            else:
                self.log_to_csv(emotion, match, 'no')

    def handle_emotion(self, data):
        """Save the emotion received"""
        rospy.loginfo(f"Emotion Received: {data.data}")
        self.last_emotion = data.data

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
