from abc import ABC, abstractmethod

class RobotInterface(ABC):
    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def say(self, sentence):
        pass

    @abstractmethod
    def random_head_movements(self):
        pass