# furhat movements
from model.concrete.automatic_movements import AutomaticMovements
# Furhat connection
from model.concrete.connection import RobotConnectionManager

import threading

class Furhat:
    def __init__(self):
        self.robot = None

    def connect(self):
        self.robot = RobotConnectionManager.get_session()

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
        self.robot.gesture(name=expression)

    def change_led_color_based_on_emotion(self, emotion):
        if emotion == 'happy':
            # green
            self.robot.set_led(red=144, green=238, blue=144)
        elif emotion == 'anger':
            # blue
            self.robot.set_led(red=0, green=0, blue=255)
        elif emotion == 'sad':
            # light yellow
            self.robot.set_led(red=255, green=255, blue=102)
        elif emotion == 'neutral':
            # white
            self.robot.set_led(red=255, green=255, blue=255)
        else:
            self.robot.set_led(red=0, green=0, blue=0)