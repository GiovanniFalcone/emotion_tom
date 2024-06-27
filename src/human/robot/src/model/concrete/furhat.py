# furhat movements
from human.robot.src.model.concrete.automatic_movements import AutomaticMovements
# Furhat connection
from connection import RobotConnectionManager

import threading

class Furhat:
    def __init__(self):
        self.robot = None

    def connect(self):
        self.robot = RobotConnectionManager.get_session()

    def say(self, sentence):
        self.robot.say(text=sentence, blocking=True)

    def random_head_movements(self):
        threading.Thread(target=AutomaticMovements.random_head_movements, args=(self.robot, )).start()