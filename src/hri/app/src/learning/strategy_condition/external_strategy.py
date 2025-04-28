from learning.strategy_condition.agent_strategy import AgentStrategy
from sentences.sentences import SuggestionGenerator

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))

from util import Util
from util import constants
from game.card import Card

class ExternalStrategy(AgentStrategy):
    def __init__(self, env):
        super().__init__(env)
        language = Util.get_from_json_file("config")["language"]
        self.sentence_generator_first_flip = SuggestionGenerator(Util.get_experimental_condition(constants.TOM), language)
        self.sentence_generator_second_flip = SuggestionGenerator(Util.get_experimental_condition(constants.EXTERNAL_DECEPTION), language)

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
    
    def _handle_deceptive_behavior(self, robot_type, card_name, position, action):
        """
        This function returns a different card name from the one passed as parameter
        (e.g if card_name = 'pelican' the function will return any other available card but 'pelican')

        Parameters:
        ----------
            robot_type (int): The robot's deceptive behavior chosen.
            card_name (string): The card name that the robot chosen to suggests.

        Returns:
        ---------
            string: The card's name chosen randomly.
        """
        open_card, _ = self.env.get_current_open_card()
        is_card_open = open_card != ''
        has_suggest = action != constants.SUGGEST_NONE

        if is_card_open and has_suggest:
            # select a different card name (the position suggested in first place is unchanged)
            game_board = self.env.get_board()
            available_cards = Card.get_available_cards_and_position(game_board)
            # remove the card name from available
            if card_name in available_cards and len(available_cards) > 1:
                del available_cards[card_name]
            # now choose a random card
            card_name, _ , _ = Card.get_random_card(available_cards)

        return card_name, position
    
    def generate_sentence(self, hint_type, flip_type, flag_ToM, card, position):
        """
        Generate a sentence based on the provided hint for both first and second flip.
        If flip_type == "firstCard" it will use ToM sentences, hidden otherwise.
        """
        is_first_flip = flip_type == "firstCard"

        # if none then say nothing
        if hint_type == 'none': 
            return None

        if is_first_flip:
            # Use ToM sentences if hint is provided for the first flip and mode is 'external'
            return self.sentence_generator_first_flip.get_sentence(hint_type, flip_type, flag_ToM, card, position)
        else:
            return self.sentence_generator_second_flip.get_sentence(hint_type, flip_type, flag_ToM, card, position)