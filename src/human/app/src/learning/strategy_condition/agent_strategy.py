from abc import ABC, abstractmethod

class AgentStrategy(ABC):
    def __init__(self, env):
        self.env = env

    @abstractmethod
    def _handle_first_flip(self):
        pass
    
    @abstractmethod
    def _handle_second_flip(self, action):
        pass

    @abstractmethod
    def generate_sentence(self, hint_type, flip_type, flag_ToM, card, position):
        pass