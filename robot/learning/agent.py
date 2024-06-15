import random

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))

from util import constants

class Agent:
    def __init__(self, env):
        """
        Initializes an Agent object.

        Parameters
        ----------
            env (Environment): the environment the agent operates in
        """
        self.env = env

    def take_action(self, action):
        """
        Executes an action based on the current game state and returns a tuple with the suggested action.

        Parameters
        ----------
            action (int): The action to be taken.

        Returns
        ----------
            Tuple of strings and integers:
                The first element indicates the type of the suggested action ('row'/'col' for suggesting row and column,
                'card' for suggesting a card and 'none' for not suggesting).
                The second element is a string rapresenting the name of the card suggested for 'card' suggestion, None otherwise.
                The third element is an integer representing coordinates of suggested card.
        """
        
        if action != constants.SUGGEST_NONE:
            current_open = self.env.get_current_open_card()

            if current_open == '':
                name_card, other_location = self.env.get_least_clicked_location_of_most_clicked_card()
            else:
                name_card = current_open
                other_location, _ = self.env.get_other_location(current_open)

            if action == constants.SUGGEST_ROW_COLUMN:
                which_index, index_pos = self.__get_which_position_to_suggest(other_location)
                #print("Card to suggest:", name_card, other_location, which_index, index_pos)
                return (which_index, None, index_pos)
            else:
                return ("card", name_card, other_location)
            
        return ("none", None, None)

    # used only when |actions| = 3
    def __get_which_position_to_suggest(self, other_location):
        """
        Determines which position to suggest for the next action based on the number of face-up cards in each row and column.

        Parameters
        ----------
            other_location (Tuple of integers): A tuple with two integers representing the location of the current open card.

        Returns
        ----------
            Tuple of a string and a tuple of integers:
                - The first element indicates whether to suggest a row or a column ('row' or 'column').
                - The second element is a tuple of two integers representing the suggested row and column indexes. 
        """
        row, col = other_location
        # rimuovere len se non funziona
        face_up_card_rows = (self.env.get_cards_by_index("row", row))
        face_up_card_cols = (self.env.get_cards_by_index("column", col))

        if face_up_card_cols > 2 and face_up_card_rows < 5:
            # suggest the row cause the column has already 2/4 face up card
            return ('row', (row, -1))
        elif face_up_card_cols <= 2 and face_up_card_rows > 3:
            # suggest the column cause the row has already 4/6 face up card
            return ('column', (-1, col))
        else:
            # the suggest can be random
            choice = random.randint(0, 1)  # 0: suggest row, 1: suggest column
            if choice == 0:
                return ('row', (row, -1))
            else:
                return ('column', (-1, col)) 

    