from abc import ABC, abstractmethod

class RobotInterface(ABC):
    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def say(self, sentence):
        pass

    @abstractmethod
    def user_detection(self):
        pass

    @abstractmethod
    def random_head_movements(self):
        pass

    @abstractmethod
    def do_facial_expression(self, expression):
        pass

    @abstractmethod
    def change_led_color_based_on_emotion(self, emotion):
        pass