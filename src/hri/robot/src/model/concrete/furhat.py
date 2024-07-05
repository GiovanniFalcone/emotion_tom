# furhat movements
from model.concrete.automatic_movements import AutomaticMovements
# Furhat connection
from model.concrete.connection import RobotConnectionManager

# create a separate thread to run automatic movements of furhat's head
import threading
# load custom gestures
import json

class Furhat:
    def __init__(self):
        self.robot = None
        self._gestures_api = None

    def connect(self):
        self.robot = RobotConnectionManager.get_session()
        # load gestures
        expressions = self.robot.get_gestures()
        self._gestures_api = [expression.name for expression in  expressions]

    def say(self, sentence):
        self.robot.say(text=sentence, blocking=True)

    def user_detection(self):
        users = self.robot.get_users()
        # Attend the user closest to the robot
        self.robot.attend(user="CLOSEST")
        return users

    def random_head_movements(self):
        threading.Thread(target=AutomaticMovements.random_head_movements, args=(self.robot, )).start()

    def do_facial_expression(self, expression):
        if expression in self._gestures_api:
            self.robot.gesture(name=expression)
        else:
            # custom
            if expression == "happy_0":
                expression = self._get_custom_expression("happy_gesture")
            elif expression == "happy_1":
                expression = self._get_custom_expression("indefinitesmile_gesture")
            elif expression == "CustomSad":
                expression = self._get_custom_expression("sadtohappy_gesture")
            elif expression == "CustomSurprise":
                expression = self._get_custom_expression("surprise_gesture")
            else:
                raise ValueError(f"Gesture '{expression}' not defined!")
            self.robot.gesture(body=expression)

    def _get_custom_expression(self, filename):
        file_path = "../emotion_tom/src/hri/robot/src/gestures/" + filename + ".json"
        gesture = ''
        try:
            with open(file_path, "r") as f:
                gesture = json.load(f)
        except Exception as e:
            import os
            print(f"Error: {e}\n Current path is: {os.getcwd()}")
        return gesture

    def change_led_color_based_on_emotion(self, emotion):
        if emotion == 'happy' or emotion == 'surprise':
            # orange 
            self.robot.set_led(red=255, green=165, blue=0)
        else:
            # green in order to calm
            self.robot.set_led(red=0, green=255, blue=0)