from learning.strategy_condition.agent_strategy import AgentStrategy
from sentences.sentences import SuggestionGenerator

import random
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))

from util import constants
from util import Util
from game.card import Card

class DeceptionStrategy(AgentStrategy):
    def __init__(self, env):
        super().__init__(env)
        language = Util.get_from_json_file("config")["language"]
        # Deceptive agent will use same sentences of ToM agent, but with different card location
        self.sentence_generator = SuggestionGenerator(Util.get_experimental_condition(constants.TOM), language)

    def _handle_first_flip(self):
        """
        Choose the least clicked location from the available set of card.

        Returns:
        -------
        Tuple of (str, Tuple of (int, int))
            The name of the card chosen and its other location on the game board.
        """

        card_name, other_pos = self.env.get_least_clicked_location_from_visitated()
        
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
        This function returns, with a 50%, a different card location from the one passed as parameter
        (e.g if card_name = 'pelican' the function will return any other available card location but 'pelican')

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
        has_suggest_a_card = action == constants.SUGGEST_CARD

        if is_card_open and has_suggest_a_card:
            # with 50% the agent can provide the wrong card
            if random.choice([0, 1]) == 0:
                _, position = self.__get_wrong_card()
                row, col = position
                return card_name, (row + 1, col + 1)
            
        return card_name, position
    
    def __get_wrong_card(self):
        """
        With this function tha agent suggests the wrong card based on the card currently open.

        Parameters:
        ----------
            None
        
        Returns:
        -------
            card_name (str): the name of the selected card to suggest
            other_pos (tuple) the position of selected card
        """
        
        print("Choosing wrong card (if possible)...") # choose neighbour card

        open_card_name, open_card_pos = self.env.get_current_open_card()
        game_board = self.env.get_board()

        # get other location of current open card
        other_pos = self.env.get_other_location(open_card_name, open_card_pos)
        
        # choose what suggest (row or column)
        suggest, position = self.__get_which_position_to_suggest(other_pos)
        index = 0 if suggest == "row" else 1
        
        # select a random card from row/column of the other location of the card currently open
        available_cards = Card.get_available_cards_by_suggest(suggest, position[index], game_board)
        
        # remove the other location if it is present: it must provide a wrong position
        if open_card_name in available_cards and len(available_cards) > 1:
            del available_cards[open_card_name]
        
        # now choose one (wrong) card from available
        card_name, position, _ = Card.get_random_card(available_cards)
        
        # in this case the agent can't provide anymore a wrong card: there is only one available who's also a match
        # (this because the card was not removed from available)
        if open_card_name == card_name:
            return card_name, other_pos

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