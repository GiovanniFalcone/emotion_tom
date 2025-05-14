import rospy
from std_msgs.msg import String, Bool, Int32
from emotion.msg import emotion

import random
import json
import requests
import time

# in order to use and save correctly in csv file, ecc
from threading import Lock

# robot
from model.interface.robot_interface import RobotInterface
# interaction functions
from interaction.interaction import InteractionModule
# emotion csv
from emotion_csv import EmotionCSV
# emotion logic
from feedback.emotion_processor import EmotionProcessor
# Emotion handler: handle facial expression based on emotion and game
from feedback.robot_emotion_controller import RobotEmotionController

class Feedback:
    # for how many seconds the emotion should be save in the csv file when robot is not motivating the user 
    TIME_TO_WAIT_OPTIONS = [4, 9]
    
    def __init__(self, IP_ADDRESS, RECORD_ALL_GAME, robot: RobotInterface, emotional_condition, interaction: InteractionModule):
        """
        Initialize the Feedback class.  
        """
        self.IP_ADDRESS = IP_ADDRESS
        self.RECORD_ALL_GAME = RECORD_ALL_GAME
        self.robot = robot
        self.id_player = -1
        self.emotional_condition = emotional_condition
        self.interaction = interaction

        # Game info for CSV file
        self.turn = 1
        self.match = False
        self.motivated = 'no'
        self.is_game_ended = False

        # emotion logic (how to handle frame, classify based on turn, ecc)
        self.emotion_processor = EmotionProcessor()
        # Emotion handler
        self.robot_emotion_controller = RobotEmotionController(self.robot)

        # Head pose estimation for csv
        self.head_pose = None
        self.x_pose = None
        self.y_pose = None
        self.z_pose = None

        # Locks
        self._initialize_locks()

        # Time used to save the emotion in the CSV when the robot is not uttering (both conditions)
        self.board_changed = False
        self.last_saved_time = time.time()
        self.time_to_wait = random.choice(Feedback.TIME_TO_WAIT_OPTIONS)

    def _initialize_locks(self):
        """Initialize threading locks."""
        self.game_data_lock = Lock()
        self.first_flip_lock = Lock()                   # Lock for first flip hint
        self.is_hint_first_flip = False                 # If true, the robot can say a motivational sentence
        self.utter_lock = Lock()                        # Lock when emotions are saved in the CSV while the robot is talking
        self.has_uttering = False                       # Variable used by locks while the robot is talking
        self.speaking_lock = Lock()                     # Lock when the robot speaks and the user clicks a card
        self.pose_lock = Lock()                         # Lock for head pose estimation

    def set_logger(self, logger: EmotionCSV):
        self.logger = logger

    def set_id_player(self, id_player):
        self.id_player = id_player

    ###############################################################################################################
    #                                                    UTILS                                                    #
    ###############################################################################################################

    def _extract_head_pose(self, emotion_msg):
        """
        Extract head pose estimation from the emotion message.
        It saves the head pose estimation in the class variables and returns them.
        
        Args: 
            emotion_msg: ROS message containing head pose estimation (head_pose, x, y, z).

        Returns:
            head_pose: Head pose estimation.
            x_pose: X coordinate of the head pose.
            y_pose: Y coordinate of the head pose.
            z_pose: Z coordinate of the head pose.
        """
        # save head pose estimation for "filtered" csv
        with self.pose_lock:
            self.head_pose = emotion_msg.head_pose
            self.x_pose = emotion_msg.x
            self.y_pose = emotion_msg.y
            self.z_pose = emotion_msg.z
        
        # copy estimated head pose
        head_pose = emotion_msg.head_pose
        x_pose = emotion_msg.x
        y_pose = emotion_msg.y
        z_pose = emotion_msg.z

        return head_pose, x_pose, y_pose, z_pose

    def _extract_game_info(self):
        """
        Extract game information for emotion handling.
        It use the game_data_lock to ensure thread safety when accessing game state variables.
        It use the first_flip_lock to check if the robot is providing a hint on the first flip.
        
        Returns:
            condition: Boolean indicating if the logger is initialized and the game is not ended.
            is_turn_even: Boolean indicating if the current turn is even.
            match_copy: Copy of the match state.
            turn_copy: Copy of the current turn number.
            motivated_copy: Copy of the motivated state.
            is_hint_provided_on_first_flip: Boolean indicating if a hint is provided on the first flip.
        """
        # Copy game state variables under lock
        with self.game_data_lock:
            condition = self.logger and self.is_game_ended == False # if logger is initialized and game is not ended -> True
            is_turn_even = self.turn % 2 == 0
            match_copy = self.match
            turn_copy = self.turn
            motivated_copy = self.motivated

        # when the robot is providing a hint on the first flip, we don't save the emotion
        # (this is done since the robot could motivate the user but the hint has higher priority)
        with self.first_flip_lock:
            is_hint_provided_on_first_flip = self.is_hint_first_flip

        return condition, is_turn_even, match_copy, turn_copy, motivated_copy, is_hint_provided_on_first_flip
    
    def _update_game_when_record_frames_in_even_turns(self, callback_name):
        """
        Updates the turn count to ensure it is odd when recording frames during even turns.
        Then, it set the 'match' variable to False, since the turn will be odd.

        This method increments the turn count by 1 to make it odd. This is necessary because
        the callback does not receive updated data until the user interacts (e.g., clicks a new card).
        For example, if the user finds a pair during turn 4 and waits before clicking another card,
        the callback will still consider the turn as 4. Incrementing the turn ensures proper
        synchronization.
        Regarding the 'match' variable, once the turn is increased by 1, it must be set to False.
        For example, if the user finds a pair during turn 4, this function will increase the turn by 1,
        so turn is now 5. This means that the user must uncover the first card of the pair. 

        Args:
            callback_name (str): The name of the callback function invoking this method, used for logging.

        Side Effects:
            - Increments the `self.turn` attribute by 1.
            - Sets the `self.match` to False.
            - Logs the updated turn value using ROS logging.
        """
        # until the user clicks a new card, the callback does not receive updated data. Therefore increase turn to be odd
        # e.g: user finds a pair in a turn 4; wait before click another card -> callback will wait and so the turn will still be 4
        with self.game_data_lock: 
            self.turn += 1 
            self.match = False
        rospy.loginfo(f"[{callback_name}] ROS turn is: '{self.turn}'...")

    def _should_skip_feedback_due_to_hint(self):
        """
        Check if the robot should skip the feedback since user has received a hint on first flip
        
        Example: 
           - turn 'n' (even) user do not finds a pair; 
           - the next action (based on RL policy) is provide a hint on first flip (turn n + 1) using ToM
           - therefore, the robot will provide the hint

        Returns:
            bool: True if the robot should not provide a feedback, False otherwise
        """
        with self.first_flip_lock:
            is_hint_provided_on_first_flip = self.is_hint_first_flip

        if is_hint_provided_on_first_flip:
            rospy.loginfo("[Feedback] Robot can't motivate because of hint provided on first flip...")
            # uncomment the next line if you want to save the emotion once the user has made a move
            # example: user don't find a pair in turn 4, so we should save the emotion
            # but in the turn 5 the robot will provide a hint on first flip
            return True

        return False

    def handle_first_flip(self, value):
        """
        Save the value into is_hint_first_flip variable.
        If value is True, the robot can't motivate the user, since it is providing a hint on the first flip.
        """
        with self.first_flip_lock: 
            self.is_hint_first_flip = value

    ###############################################################################################################
    #                                              Emotion callback                                               #
    ###############################################################################################################

    def handle_emotion(self, emotion_msg):
        """
        Save the emotion received and update the CSV file accordingly.
        Handles saving emotions based on whether the robot is uttering, the game state, 
        and the time elapsed since the last save.

        Args:
            data: ROS message containing emotion data (dominant_emotion, valence, model_confidence).
        """

        # extract emotion data
        emotion = emotion_msg.dominant_emotion
        emotion_valence = emotion_msg.valence
        emotion_score = emotion_msg.model_confidence

        # extract head pose estimation
        head_pose, x_pose, y_pose, z_pose = self._extract_head_pose(emotion_msg)

        # Get timestamp and game time for csv
        timestamp = self.logger.get_time()
        game_time = self.logger.get_game_time()

        # get game info
        condition, is_turn_even, match_copy, turn_copy, motivated_copy, is_hint_provided_on_first_flip = self._extract_game_info()

        # since surprise can have any valence, we set to positive if match, negative otherwise
        # if emotion == "surprise": emotion_valence = 1 if match_copy else -1

        # Process emotion only if the game is ongoing and logger is initialized
        if not condition: return

        # Update emotion windows
        self.emotion_processor.add_emotion_to_window(emotion, emotion_valence)

        # Save emotion on file
        self._save_emotions_in_csv(is_turn_even, is_hint_provided_on_first_flip, 
                                   timestamp, game_time, emotion, emotion_score, 
                                   head_pose, x_pose, y_pose, z_pose, 
                                   match_copy, turn_copy, motivated_copy)
        
    def _save_emotions_in_csv(self, is_turn_even, is_hint_provided_on_first_flip,
                                timestamp, game_time, emotion, emotion_score,
                                head_pose, x_pose, y_pose, z_pose, match_copy, turn_copy, motivated_copy):
        """
        Saves the detected emotion into the CSV file "full_emotion.csv".
        The amount of emotions saved depends on the value of RECORD_ALL_GAME. 
        If RECORD_ALL_GAME is True, all emotions during the game will be saved. 
        Otherwise, only emotions during even turns (where the robot could provide feedback) will be saved. 
        Specifically:
        - If the robot is providing feedback, all emotions while the robot is speaking will be saved.
        - If the robot is not providing feedback, emotions will be saved during a random time interval 
          determined by the value of TIME_TO_WAIT_OPTIONS (This is done since the robot can't motivate when condition is ToM).
        """
        # if true it will save all emotions during the game
        if self.RECORD_ALL_GAME:
            with self.utter_lock:
                utter_copy = self.has_uttering
            if utter_copy:
                rospy.loginfo(
                    "[Emotion callback]\n"
                    f"  - Game time (pop-up included): {game_time}\n"
                    f"  - Adding emotion: {emotion}\n"
                    f"  - Has Uttering: {utter_copy}\n"
                    f"  - Turn: {turn_copy}\n"
                    f"  - Motivated: {motivated_copy}"
                )

            self.logger.log_to_csv(
                timestamp, game_time, self.id_player, "full", emotion, emotion_score, 
                head_pose, x_pose, y_pose, z_pose, match_copy, turn_copy, motivated_copy
            )
            return 

        if is_turn_even or self.board_changed:
            # don't save emotion when robot is providing a hint on the first flip 
            # (this is done since the robot could motivate the user bu the hint has higher priority)
            if is_hint_provided_on_first_flip:
                rospy.loginfo(f"[Emotion callback] Robot is providing a hint on first flip -> no emotion saved...")
                return
            
            self._save_emotions_based_on_motivation(
                timestamp, game_time, emotion, emotion_score, 
                head_pose, x_pose, y_pose, z_pose,
                match_copy, turn_copy, motivated_copy
            )

    def _save_emotions_based_on_motivation(self, timestamp, game_time, emotion, emotion_score, 
                                     head_pose, x_pose, y_pose, z_pose, match_copy, turn_copy, motivated_copy):
        """
        Saves the detected emotion to the CSV file based on the robot's motivation status.
        """
        with self.utter_lock:
            utter_copy = self.has_uttering

        if motivated_copy == 'no' and not self.board_changed:
            # Save emotions in a temporary window if not motivating
            self._save_emotions_in_csv_if_not_motivating(
                timestamp, game_time, emotion, emotion_score, 
                head_pose, x_pose, y_pose, z_pose,
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
                timestamp, game_time, self.id_player, "full", emotion, emotion_score, 
                head_pose, x_pose, y_pose, z_pose, match_copy, turn_copy, motivated_copy
            )

    def _save_emotions_in_csv_if_not_motivating(self, timestamp, game_time, emotion, emotion_score, 
                                                head_pose, x_pose, y_pose, z_pose, match_copy, turn_copy, motivated_copy, utter_copy):
        """
        Saves emotion data to a CSV file if the robot is not in a motivating state 
        and the specified time interval has not elapsed. If the time interval has 
        elapsed, increments the turn counter instead.

        Args:
            logger (EmotionCSV): The logger instance to log the data.
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
                f"  - Timestamp: {time_since_last_save:.2f} seconds (time limit is {self.time_to_wait} seconds)\n"
                f"  - Adding emotion: {emotion}\n"
                f"  - Has Uttering: {utter_copy}\n"
                f"  - Turn: {turn_copy}\n"
                f"  - Motivated: {motivated_copy}"
            )
            #if not utter_copy: rospy.loginfo(f"(Call) queue is: {self.emotion_window_valence}\n")
            # update csv
            self.logger.log_to_csv(timestamp, game_time, self.id_player, "full", emotion, emotion_score, 
                                   head_pose, x_pose, y_pose, z_pose, match_copy, turn_copy, motivated_copy)
            return
        else:
            # otherwise, time limit has elpased -> don't save emotions 
            self._update_turn_when_record_frames_in_even_turns("Emotion callback")

    ###############################################################################################################
    #                                                Game callback                                                #
    ###############################################################################################################

    def handle_game(self, game_data):
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
        self.is_game_ended = n_pairs == 12
        board_changed = move['game']['board_changed']

        # if robot has E-ToM it can provide a feedback when board changed
        if board_changed and self.emotional_condition:
            self.board_changed = True
            self._handle_board_change(move)
            return 
        
        # otherwise, the turn is handled -> save info on log, eventually provide feedback
        self._handle_turn(move)

        # when the game ends, robot will say goodbye to the user
        if self.is_game_ended:
            self.interaction.goodbye(self.emotional_condition)
            self.interaction.player_name = ''

    def _handle_turn(self, move):
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
        
        # If the turn is even, the robot can potentially provide feedback.
        # However, if the user has received a hint on the first flip, the robot will skip motivation.
        # (Refer to the documentation of the following function for more details.)
        if self._should_skip_feedback_due_to_hint():
            return

        # get current time in order to save the emotion for some seconds when the robot should not provide a feedback 
        self.last_saved_time = time.time()

        # get emotion (it will wait 0.75 seconds before to compute, in order to have new emotions after the click)
        emotion, emotion_score = self.emotion_processor.get_emotion()
        # set to 'neutral' if 
        #   - the emotion hasn't been detected 
        #   - the emotion is fear: we take it as classification error
        #   - the emotion is fear/angry/fear while user find a paid -> we take it as classification error
        if emotion in ['fear', 'disgust', '', None] or (match and emotion in ['sad', 'angry', 'fear']):
            emotion = 'neutral'

        # get estimated head pose
        head_pose, x_pose, y_pose, z_pose = None, None, None, None
        with self.pose_lock:
            head_pose = self.head_pose
            x_pose = self.x_pose
            y_pose = self.y_pose
            z_pose = self.z_pose

        # if true, the robot will motivate the user based on their emotion (False -> ToM condition only)
        if not self.emotional_condition:
            return self._save_emotion_when_condition_is_false(
                    emotion, emotion_score, 
                    head_pose, x_pose, y_pose, z_pose, match, turn
                )
            
        # define probability for robot's motivational speech
        probability = Feedback.get_probability_of_feedback(match, n_pairs, turn)

        # get time for csv (time when the feedback is provided)
        # (it is not the same as the one in the game, since it is based on the time when the user clicked the card)
        timestamp = self.logger.get_time()
        game_time = self.logger.get_game_time()

        # if probability and hint is not provided on first flip then motivate user
        if random.random() < probability:
            self._handle_when_feedback_is_provided(timestamp, game_time, emotion, emotion_score, 
                                                   head_pose, x_pose, y_pose, z_pose, n_pairs, match, turn
                                                   )
        else:
            self._handle_when_no_feedback_provided(
                timestamp, game_time, emotion, emotion_score, 
                head_pose, x_pose, y_pose, z_pose, match, turn
            )  

        # if it was true, reset to false in order to motivate after a move
        self.is_hint_first_flip = False  

    def _save_emotion_when_condition_is_false(self, emotion, emotion_score,
                                                head_pose, x_pose, y_pose, z_pose, match, turn):
        """
        Save the emotion in the csv file when the condition is false (ToM condition).
        """
        # get current time
        timestamp = self.logger.get_time()
        game_time = self.logger.get_game_time()

        # motivated is set to 'no' since the robot will not provide a feedback
        with self.game_data_lock: self.motivated = 'no'
        
        # in order to save the emotion for some seconds when the robot should not provide a feedback 
        self.time_to_wait = random.choice(Feedback.TIME_TO_WAIT_OPTIONS)
        rospy.loginfo(f"[Feedback] Time to wait until time interval expires or new card has been clicked: {self.time_to_wait} seconds...\n")
        
        # Log to CSV
        self.logger.log_to_csv(timestamp, game_time, self.id_player, "filtered", emotion, emotion_score, 
                                head_pose, x_pose, y_pose, z_pose, match, turn, 'no')
        return 

    def _handle_when_feedback_is_provided(self, timestamp, game_time, emotion, emotion_score, 
                                          head_pose, x_pose, y_pose, z_pose, n_pairs, match, turn):
        """
        This function make the robot utter a motivational sentence based on the emotion of the user.
        """
        # send to Flask that the robot is providing a feedback
        requests.post("http://" + self.IP_ADDRESS + ":5000/robot_feedback")
        rospy.loginfo(f"[Feedback] Waiting ...")
        
        # set robot appearance
        self.robot_emotion_controller.change_led_color_based_on_emotion(emotion)
        self.robot_emotion_controller.handle_expression_based_on_emotion(emotion)

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

        # set to 'no' for full emotion csv, since the robot has finished talking
        with self.game_data_lock: self.motivated = 'no'

        # robot ended uttering -> update variable for emotion_callback (full emotion csv)
        with self.utter_lock: self.has_uttering = False

        # Once the user has clicked a card in a certain turn (e.g turn 4), 
        # until a new card is clicked (i.e callback activated) the turn will still be 4.
        # Therefore, increase the turn to avoid saving the emotion in the CSV during odd turns.
        self._update_game_when_record_frames_in_even_turns("Feedback")
        
        # remove LED
        self.robot_emotion_controller.change_led_color_based_on_emotion("")
        # Log to CSV ('yes' because is 'filtered' csv - it only save the emotion on each move)
        self.logger.log_to_csv(timestamp, game_time, self.id_player, "filtered", emotion, emotion_score, 
                               head_pose, x_pose, y_pose, z_pose, match, turn, 'yes')

    def _handle_board_change(self, move):
        rospy.loginfo(f"[Feedback] Game board is changed -> robot should provide feedback!")

        # get info about game
        n_pairs, turn, match = move['game']['pairs'], move['game']['turn'], move['game']['match']

        # acquire lock and save it for emotion_callback ()
        with self.game_data_lock:
            self.turn = turn
            self.match = match

        # emotion to use for feedback
        emotion, emotion_score = self.emotion_processor.get_emotion()
        # get estimated head pose
        head_pose, x_pose, y_pose, z_pose = None, None, None, None
        with self.pose_lock:
            head_pose = self.head_pose
            x_pose = self.x_pose
            y_pose = self.y_pose
            z_pose = self.z_pose
        # time of emotion
        timestamp = self.logger.get_time()
        game_time = self.logger.get_game_time()

        # set robot appearance
        self.robot_emotion_controller.change_led_color_based_on_emotion(emotion)
        self.robot_emotion_controller.handle_expression_based_on_emotion(emotion)

        # get sentence that robot will utter
        motivational_sentence = self.interaction.get_motivational_sentence(emotion, n_pairs, None, board_changed=True)

        # set to 'yes' for full emotion csv, since the robot will provide a feedback
        with self.game_data_lock: self.motivated = 'yes'
        rospy.loginfo(f"[Feedback] Robot uttering once the board is changed: '{motivational_sentence}'...")

        # lock used for emotion_callback
        with self.utter_lock: self.has_uttering = True

        # block until robot has not finished to talk
        # (used when user click a card to fast)
        with self.speaking_lock:
            self.interaction.speak(motivational_sentence)
            rospy.loginfo(f"[Feedback] Robot ended uttering at: '{self.logger.get_game_time()}'...")
        
        # if emotion is happy or neutral do another facial expression
        if emotion in ['happy, neutral']: self.robot.do_facial_expression("BigSmile" if random.choice([True, False]) else "Wink")

        # set to 'no' for full emotion csv, since the robot has finished talking
        with self.game_data_lock: self.motivated = 'no'

        # robot ended uttering -> update variable for emotion_callback (full emotion csv)
        with self.utter_lock: self.has_uttering = False
        
        # remove LED
        self.robot_emotion_controller.change_led_color_based_on_emotion("")
        # Once the user has clicked a card in a certain turn (e.g turn 4), 
        # until a new card is clicked (i.e callback activated) the turn will still be 4.
        # Therefore, increase the turn to avoid saving the emotion in the CSV during odd turns. 
        # (similarly for match that is set to False)
        self._update_game_when_record_frames_in_even_turns("Feedback")
        # Log to CSV ('yes' because is 'filtered' csv - it only save the emotion on each move)
        self.board_changed = False
        self.logger.log_to_csv(timestamp, game_time, self.id_player, "filtered", emotion, emotion_score, 
                               head_pose, x_pose, y_pose, z_pose, False, turn, 'yes')

    def _handle_when_no_feedback_provided(self, timestamp, game_time, emotion, emotion_score,
                                        head_pose, x_pose, y_pose, z_pose, match, turn):
        """
        Handle the case when the robot don't provides feedback to the user.
        """
        rospy.loginfo(f"[Feedback] No feedback provided...")
            
        # in order to save the emotion for some seconds when the robot should not provide a feedback
        # (only when we want save emotions during even turns)
        if not self.RECORD_ALL_GAME:
            self.time_to_wait = random.choice(Feedback.TIME_TO_WAIT_OPTIONS)
            rospy.loginfo(f"[Feedback] Time to wait until time interval expires or new card has been clicked: {self.time_to_wait} seconds...\n")

        # update turn and match since this data will not be updated until the callback is called (i.e. the user does not click a new card)
        self._update_game_when_record_frames_in_even_turns("Feedback")
        
        # the robot will not motivate the user
        with self.game_data_lock: self.motivated = 'no'
        
        # Saves the emotion that would be used for the feedback in the csv file
        self.logger.log_to_csv(timestamp, game_time, self.id_player, "filtered", emotion, emotion_score, 
                                head_pose, x_pose, y_pose, z_pose, match, turn, 'no')
        
        # Perform a facial expression based on match
        self.robot.do_facial_expression("Nod" if match else "Shake") 

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
        # Probability if a match is found
        if match:
            return 0.85 
        # No feedback in the first few turns to allow the user to get familiar with the game
        if turn <= 4:
            return 0.0
        # Lower probability when outcome of move is not a match
        return 0.65