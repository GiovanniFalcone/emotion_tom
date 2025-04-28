from learning.strategy_condition.agent_strategy import AgentStrategy
from sentences.sentences import SuggestionGenerator

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))

from util import Util
from util import constants

class SuperficialStrategy(AgentStrategy):
    def __init__(self, env):
        super().__init__(env)
        language = Util.get_from_json_file("config")["language"]
        self.sentence_generator = SuggestionGenerator(Util.get_experimental_condition(constants.SUPERFICIAL_DECEPTION), language)

    def _handle_first_flip(self):
        """
        Choose a random card from the available set of card.

        Returns:
        -------
        Tuple of (str, Tuple of (int, int))
            The name of the card chosen and its other location on the game board.
        """

        print("Choose random card")
        card_name, other_pos = self.env.get_random_card()
        
        return (card_name, other_pos)
    
    def _handle_second_flip(self, action):
        """
        Returns a random card and its position.

        Parameters:
        ----------
        action: str
            The agent's chosen action for the current turn.

        Returns:
        -------
        Tuple of (str, Tuple of (int, int))
            The name of the card chosen and its other location on the game board.
        """
        
        card_name, position = self.env.get_random_card()

        return (card_name, position)
    
    def generate_sentence(self, hint_type, flip_type, flag_ToM, card, position):
        """
        Generate a sentence based on the provided hint for both first and second flip.
        """

        # if none then say nothing
        if hint_type == 'none': 
            return None

        # else return the correct sentence
        return self.sentence_generator.get_sentence(hint_type, flip_type, flag_ToM, card, position)