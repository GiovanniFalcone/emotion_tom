from learning.strategy_condition.agent_strategy import AgentStrategy
from sentences.sentences import SuggestionGenerator

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))

from util import Util
from util import constants

class TomStrategy(AgentStrategy):
    def __init__(self, env):
        super().__init__(env)
        language = Util.get_from_json_file("config")["language"]
        self.sentence_generator = SuggestionGenerator(Util.get_experimental_condition(constants.TOM), language)

    def _handle_first_flip(self):
        """
        Choose the most clicked location of a card and return the other location of the pair
        (e.g: if most clicked is (shark, (0, 0)), returns (shark, (1, 1)))

        Returns:
        -------
        Tuple of (str, Tuple of (int, int))
            The name of the card chosen and its other location on the game board.
        """

        # choose randomly a card from the list of most clicked cards
        card_name, other_pos = self.env.get_least_clicked_location_of_most_clicked_card()
        
        return (card_name, other_pos)
    
    def _handle_second_flip(self, action):
        """
        Choose a card and its other location to flip for the second time.

        Parameters:
        ----------
        action: str
            The agent's chosen action for the current turn.

        Returns:
        -------
        Tuple of (str, Tuple of (int, int))
            The name of the card chosen and its other location on the game board.
        """
        current_open, current_pos = self.env.get_current_open_card()
        
        card_name = current_open
        position = self.env.get_other_location(current_open, current_pos)

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