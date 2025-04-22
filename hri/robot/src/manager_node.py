#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool, Int32
from emotion.msg import emotion

import os
import random
import json
import requests
import sys
import time
import numpy as np

# in order to use and save correctly in csv file, ecc
from threading import Lock

# for emotion 
from collections import deque

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
#from util import Util

class ManagerNode:
    IP_ADDRESS = Util.get_from_json_file("config")['ip']

    def __init__(self, robot: RobotInterface, language='ita'):
        # Initialize variables
        self.robot = robot                              # Robot object
        self.language = language                        # Language used by the robot
        self.state = 'IDLE'                             # State of the interaction
        self.emotional_condition = False                # Experimental condition
        self.id_player = -1                             # ID player for CSV: analysis for emotion

        # Game info for CSV file
        self.turn = 1
        self.match = False
        self.motivated = 'no'
        self.logger = None                              # Emotion CSV object

        # Interaction module (must be initialized after robot connection!)
        self.interaction = InteractionModule(robot, language)

        # Get experimental condition
        self.get_emotion_condition()

        # Emotion handler
        self.emotion_handler = EmotionHandler(self.robot)

        # ROS topics
        self._initialize_ros_subscribers()

        # Emotion window to analyze in order to get the most probable emotion
        self.emotion_window_names = deque(maxlen=30)    # 30 frames (as in the paper)
        self.emotion_window_valence = deque(maxlen=30)  # 30 frames (as in the paper)

        # Weights for each emotion (30 emotions)
        self.weights = np.array([np.arange(1, 31) / 465])  # 465 = sum of weights (1+2+3+...+30)

        # Locks
        self._initialize_locks()

        # Time used to save the emotion in the CSV when the robot is not uttering (both conditions)
        self.last_saved_time = time.time()
        self.time_to_wait = random.choice([3, 10])

    def _initialize_ros_subscribers(self):
        """Initialize ROS subscribers."""
        rospy.Subscriber('/speech_hint', String, self.speech_callback)
        rospy.Subscriber('/full_emotion', emotion, self.emotion_callback)
        rospy.Subscriber('/person_detected', Bool, self.person_detected_callback)
        rospy.Subscriber('/start', Int32, self.game_started)
        rospy.Subscriber('/game_data', String, self.game_callback)

    def _initialize_locks(self):
        """Initialize threading locks."""
        self.game_data_lock = Lock()
        self.first_flip_lock = Lock()                   # Lock for first flip hint
        self.is_hint_first_flip = False                 # If true, the robot can say a motivational sentence
        self.lock_window = Lock()                       # Lock for emotion window
        self.utter_lock = Lock()                        # Lock when emotions are saved in the CSV while the robot is talking
        self.has_uttering = False                       # Variable used by locks while the robot is talking
        self.speaking_lock = Lock()                     # Lock when the robot speaks and the user clicks a card

    ###############################################################################################################
    #                                                   SETTINGS                                                  #
    ###############################################################################################################

    def get_emotion_condition(self):
        """Set experimental contion from ROS parameter."""
        try:
            self.emotional_condition = bool(rospy.get_param("emotion_condition"))
            rospy.loginfo(f"[Manager] Emotion condition: {self.emotional_condition}")
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
            # rospy.loginfo(f"[Start] User detected, starting interaction...")
            # self.interaction.start_interaction(self.emotional_condition)
        
    def game_started(self, msg):
        """
        If this callback is trigger, it means that game page has been showed and so the user can starts to play.
        It is also used in order to understand when the emotion can be saved on csv.
        """
        self.id_player = msg.data
        print("\n")
        rospy.loginfo(f"[Init] ID player received: {self.id_player}...")
        self.logger = EmotionCSV(self.id_player, self.emotional_condition)
        self.logger.set_starting_time()
        if self.logger:
            rospy.loginfo(f"[Init] Emotion csv initialized...\n")

    def speech_callback(self, data):
        """
        Robot will utter the suggestion.
        Once the robot has uttered the suggestion an http request is sent to the server in order to remove the pop-up.
        """
        # deserialize json
        json_data = json.loads(data.data)
        # rospy.loginfo(f"[Hint] {json_data}")
        sentence = json_data["action"]["sentence"]
        flip_type = json_data["action"]["flip_type"]
        # if hint is provided for first flip 
        with self.first_flip_lock: self.is_hint_first_flip = (flip_type == "firstCard")
        # if hint is provided then utter it
        if sentence != '':
            rospy.loginfo(f"[Hint] Hint Received: {sentence}")
            # utter the suggestion
            self.interaction.speak(sentence)
            # send to Flask
            json_data = ({"speech": "ended"})
            self.send_to_flask_robot_has_finished_to_speak(json_data)
    
    def emotion_callback(self, data):
        """
        Save the emotion received and update the CSV file accordingly.
        Handles saving emotions based on whether the robot is uttering, the game state, 
        and the time elapsed since the last save.

        Args:
            data: ROS message containing emotion data (dominant_emotion, valence, model_confidence).
        """
        emotion = data.dominant_emotion
        emotion_valence = data.valence
        emotion_score = data.model_confidence

        # Skip processing if no emotion is detected
        if not emotion:
            return

        # Get timestamp and game time if logger is initialized
        if self.logger:
            timestamp = self.logger.get_time()
            game_time = self.logger.get_game_time()
        else:
            rospy.logwarn("[Emotion callback] Logger not initialized. Skipping emotion processing.")
            return

        # Copy game state variables under lock
        with self.game_data_lock:
            condition = self.logger and self.state != 'END'
            is_turn_even = self.turn % 2 == 0
            match_copy = self.match
            turn_copy = self.turn
            motivated_copy = self.motivated

        # Process emotion only if the game is ongoing and logger is initialized
        if condition:
            # Update emotion windows
            with self.lock_window:
                self.emotion_window_names.append(emotion)
                self.emotion_window_valence.append(emotion_valence)

            # Handle emotion saving for even turns (end of a move)
            if is_turn_even:
                with self.utter_lock:
                    utter_copy = self.has_uttering

                if motivated_copy == 'no':
                    # Save emotions in a temporary window if not motivating
                    self._save_emotions_in_csv_if_not_motivating(
                        timestamp, game_time, emotion, emotion_score, 
                        match_copy, turn_copy, motivated_copy, utter_copy
                    )
                else:
                    # Save emotions directly to CSV while the robot is talking
                    rospy.loginfo(
                        "[Emotion callback]\n"
                        f"  - Game time (pop-up included): {game_time}\n"
                        f"  - Adding emotion: {emotion}\n"
                        f"  - Has Uttering: {utter_copy}\n"
                        f"  - Turn: {turn_copy}\n"
                        f"  - Motivated: {motivated_copy}"
                    )
                    self.logger.log_to_csv(
                        timestamp, game_time, self.id_player, "full", emotion, 
                        emotion_score, match_copy, turn_copy, motivated_copy
                    )

    def _save_emotions_in_csv_if_not_motivating(self, timestamp, game_time, emotion, emotion_score, match_copy, turn_copy, motivated_copy, utter_copy):
        """
        Saves emotion data to a CSV file if the robot is not in a motivating state 
        and the specified time interval has not elapsed. If the time interval has 
        elapsed, increments the turn counter instead.

        Args:
            timestamp (float): The timestamp of the emotion data.
            game_time (float): The total game time, including any pop-up delays.
            emotion (str): The detected emotion to be logged.
            emotion_score (float): The confidence score of the detected emotion.
            match_copy (bool): Indicates whether a match was found.
            turn_copy (int): The current turn number.
            motivated_copy (bool): Indicates whether the robot is in a motivating state.
            utter_copy (bool): Indicates whether the robot is uttering a response.

        Behavior:
            - If the time since the last save is less than or equal to the specified 
              waiting time (`self.time_to_wait`), logs the emotion data to a CSV file.
            - If the time interval has elapsed, increments the turn counter to ensure 
              the callback processes updated data when the user interacts again.

        Logging:
            - Logs information about the game time, timestamp, emotion, and other 
              relevant details for debugging purposes.
        """
        current_time = time.time()
        time_since_last_save = current_time - self.last_saved_time
        # if the time limit has not elapsed save the emotion
        # this means: when the robot doesn't provide a feedback we wait some seconds 
        # in this time interval we save all the emotion analyzed
        # N.B: the emotion is saved until time interval has elasped or user has clicked another card before time limit has elapsed
        if time_since_last_save <= self.time_to_wait:
            rospy.loginfo(
                "[Emotion callback]\n"
                f"  - Game time (pop-up included): {game_time}\n"
                f"  - Timestamp: {time_since_last_save:.2f} seconds\n"
                f"  - Adding emotion: {emotion}\n"
                f"  - Has Uttering: {utter_copy}\n"
                f"  - Turn: {turn_copy}\n"
                f"  - Motivated: {motivated_copy}"
            )
            #if not utter_copy: rospy.loginfo(f"(Call) queue is: {self.emotion_window_valence}\n")
            # update csv
            self.logger.log_to_csv(timestamp, game_time, self.id_player, "full", emotion, 
                                    emotion_score, match_copy, turn_copy, motivated_copy)
            return
        else:
            # otherwise, time limit has elpased -> don't save emotions 
            # until the user clicks a new card, the callback does not receive updated data. Therefore increase turn to be odd
            # e.g: user finds a pair in a turn 4; wait before click another card -> callback will wait and so the turn will still be 4
            with self.game_data_lock: self.turn += 1 
            rospy.loginfo(f"[Emotion callback] ROS turn is: '{self.turn}'...")

    def game_callback(self, game_data):
        """
        If this callback is triggered, it means the user has made a move. 
        If the move is the last one of the game, the robot enters the final state (Goodbye) and says goodbye to the user. 
        Otherwise, if emotional condition is setted, the robot will motivate the user based on user's emotion.
        """

        # If the user clicks the card before the robot finishes speaking,
        # the callback will be blocked (the lock will only be released once the robot finishes speaking).
        print("\n")
        rospy.loginfo(f"[Game callback] Aquiring lock since the user has clicked a card...")
        with self.speaking_lock:
            rospy.loginfo(f"[Game callback] Robot was not uttering or it has already done...")
            self.has_uttering = False

        # debug
        # rospy.loginfo(f"Game data Received: {game_data.data}")

        # get data
        move = json.loads(game_data.data) 
        n_pairs = move['game']['pairs']
        is_game_ended = n_pairs == 12

        # when the game ends, the end of the interaction is handled
        if is_game_ended:
            self.state = 'END'
            self.interaction.goodbye(self.emotional_condition)
            self.interaction.player_name = ''
        else:
            # otherwise, the turn is handled -> save info on log, eventually provide feedback
            self.handle_turn(move)

    ###############################################################################################################
    #                                             EMOTION LOGIC                                                   #
    ###############################################################################################################

    def get_emotion(self):
        """
        Ottiene l'emozione più frequente in un certo intervallo di tempo (30 frame) basandosi sulla formula del paper.
        Ottiene la classe sulla base del range (fornito dal paper) e si riottiene l'emozione discreta di quella classe in base ai pesi.
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
        most_frequent_emotion = self.get_most_frequent_emotion(emotion_class, emotion_names, emotion_valence)
        print("\n")
        rospy.loginfo(f"[Feedback] Emotion score: {emotion_score}, classified as: {emotion_class}, most frequent emotion: {most_frequent_emotion}")
    
        return most_frequent_emotion, emotion_score

    def get_most_frequent_emotion(self, emotion_class, emotion_names, emotion_valence):
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

    ###############################################################################################################
    #                                                   HANDLER                                                   #
    ###############################################################################################################

    def handle_turn(self, move):
        """
        Depending on the outcome of the move (whether the user found a pair or not), 
        the robot will motivate (with a certain probability) the user based on user's emotion, 
        by uttering a motivational sentence, making a facial expression, and changing the LED color.
        """

        # get info about game
        n_pairs = move['game']['pairs']
        turn = move['game']['turn']
        match = move['game']['match']
        time_js = move['game']['time_game']
        is_turn_even = turn % 2 == 0

        # acquire lock and save it for emotion_callback ()
        with self.game_data_lock:
            self.turn = turn
            self.match = match

        # debug 
        rospy.loginfo(f"[Feedback] Card clicked in ros time {self.logger.get_game_time()} - js time {time_js}")

        # if the turn is odd (first card of pair is clicked) update values
        #   - motivated: no, since the robot can only motivate after the outcome of a move
        #   - has_uttering: false, since the robot isn't talking
        if not is_turn_even:
            with self.game_data_lock:   self.motivated = 'no'
            with self.utter_lock:       self.has_uttering = False
            return
        
        # otherwise, the turn is even and robot can, eventually, provide a feedback
        # however, if user has received a suggestion on first flip robot will not motivate 
        # example: 
        #   - turn 'n' (even) user do not finds a pair; 
        #   - the next action (based on RL policy) is provide a hint on first flip (turn n + 1) using ToM
        #   - therefore, the robot will provide the hint
        with self.first_flip_lock:
            is_hint_provided_on_first_flip = self.is_hint_first_flip
        if is_hint_provided_on_first_flip: rospy.loginfo("[Feedback] Robot can't motivate because of hint provided on first flip...")

        # get current time in order to save the emotion for some seconds when the robot should not provide a feedback 
        self.last_saved_time = time.time()

        # get emotion (it will wait 0.75 seconds before to compute, in order to have new emotions after the click)
        emotion, emotion_score = self.get_emotion()
        # set to 'neutral' if 
        #   - the emotion hasn't been detected 
        #   - the emotion is fear: we take it as classification error
        #   - the emotion is fear/angry/fear while user find a paid -> we take it as classification error
        if emotion in ['fear', '', None] or (match and emotion in ['sad', 'angry', 'fear']):
            emotion = 'neutral'

        # if true, the robot will motivate the user based on their emotion (False -> ToM condition only)
        if not self.emotional_condition:
            timestamp = self.logger.get_time()
            game_time = self.logger.get_game_time()
            with self.game_data_lock: self.motivated = 'no'
            # in order to save the emotion for some seconds when the robot should not provide a feedback 
            self.time_to_wait = random.choice([3, 10])
            rospy.loginfo(f"[Feedback] Time to wait until time interval expires or new card has been clicked: {self.time_to_wait} seconds...\n")
            # Log to CSV
            self.logger.log_to_csv(timestamp, game_time, self.id_player, "filtered", emotion, emotion_score, match, turn, 'no')
            return 

        # define probability for robot's motivational speech
        probability = ManagerNode.get_probability_of_feedback(match, n_pairs, turn)

        # get time for csv (time when the feedback is provided)
        # (it is not the same as the one in the game, since it is based on the time when the user clicked the card)
        timestamp = self.logger.get_time()
        game_time = self.logger.get_game_time()

        # if probability and hint is not provided on first flip then motivate user
        if random.random() < probability and not is_hint_provided_on_first_flip:
            rospy.loginfo(f"[Feedback] Waiting ...")
            self._provide_feedback(timestamp, game_time, emotion, emotion_score, n_pairs, match, turn)
        else:
            rospy.loginfo(f"[Feedback] No feedback provided...")
            # in order to save the emotion for some seconds when the robot should not provide a feedback 
            self.time_to_wait = random.choice([3, 10])
            rospy.loginfo(f"[Feedback] Time to wait until time interval expires or new card has been clicked: {self.time_to_wait} seconds...\n")
            # the robot will not motivate the user
            with self.game_data_lock: self.motivated = 'no'
            # Saves the emotion that would be used for the feedback in the csv file
            self.logger.log_to_csv(timestamp, game_time, self.id_player, "filtered", emotion, emotion_score, match, turn, 'no')
            # Perform a facial expression based on match
            self.robot.do_facial_expression("Nod" if match else "Shake")   

        # if it was true, reset to false in order to motivate after a move
        self.is_hint_first_flip = False  

    def _provide_feedback(self, timestamp, game_time, emotion, emotion_score, n_pairs, match, turn):
        """
        This function make the robot utter a motivational sentence based on the emotion of the user.
        """
        # set robot appearance
        self.robot.change_led_color_based_on_emotion(emotion)
        self.emotion_handler.handle_expression_based_on_emotion(emotion, n_pairs, match)

        # get sentence that robot will utter
        motivational_sentence = self.interaction.get_motivational_sentence(emotion, n_pairs, match)

        # set to 'yes' for full emotion csv, since the robot will provide a feedback
        with self.game_data_lock: self.motivated = 'yes'
        rospy.loginfo(f"[Feedback] Robot uttering: '{motivational_sentence}'...")

        # lock used for emotion_callback
        with self.utter_lock: self.has_uttering = True

        # block until robot has not finished to talk
        # (used when user click a card to fast)
        with self.speaking_lock:
            self.interaction.speak(motivational_sentence)
            rospy.loginfo(f"[Feedback] Robot ended uttering at: '{self.logger.get_game_time()}'...")

        # robot ended uttering -> update variable for emotion_callback (full emotion csv)
        with self.utter_lock: self.has_uttering = False

        # until the user clicks a new card, the callback does not receive updated data. Therefore increase turn to be odd
        # e.g: user finds a pair in a turn 4; wait before click another card -> callback will wait and so the turn will still be 4
        with self.game_data_lock: self.turn += 1 
        rospy.loginfo(f"[Feedback] ROS turn is: '{self.turn}'...\n")
        
        # remove LED
        self.robot.change_led_color_based_on_emotion("")
        # Log to CSV ('yes' because is 'filtered' csv - it only save the emotion on each move)
        self.logger.log_to_csv(timestamp, game_time, self.id_player, "filtered", emotion, emotion_score, match, turn, 'yes')

    @ staticmethod
    def get_probability_of_feedback(match, n_pairs, turn):   
        """
        Calculate the probability of providing feedback based on the game state.

        Args:
            match (bool): Whether the user found a pair.
            n_pairs (int): Number of pairs found so far.
            turn (int): Current turn number.

        Returns:
            float: Probability of providing feedback.
        """
        # Higher probability for the first pair found
        if match and n_pairs == 1:
            return 1.0
        # No feedback in the first few turns to allow the user to get familiar with the game
        if turn <= 4:
            return 0.0
        # Higher probability if a match is found
        if match:
            return 0.75
        # Lower probability if no match is found
        return 0.25
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('interaction_node', anonymous=True)
        robot = Furhat()
        # connect to robot 
        robot.connect()
        rospy.loginfo("[Manager] Connection with Furhat successfully established!")
        # if connection is ok than start
        manager_node = ManagerNode(robot)
        manager_node.run()
    except rospy.ROSInterruptException:
        pass