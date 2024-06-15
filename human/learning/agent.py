import random

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))

from util import constants

class Agent:
    def __init__(self, env, type, strategy):
        """
        Initializes an Agent object.

        Parameters
        ----------
            env (Environment): the environment the agent operates in
            type (int): The type of robot that's gonna help the user. 
        """
        self.env = env
        self.type = type
        self.strategy = strategy

    def take_action(self, action):
        """
        Executes an action based on the current game state and returns a tuple with the suggested action.

        Parameters
        ----------
            action (int): The action to be taken.

        Returns
        ----------
            Tuple of strings and integers:
                - The first element indicates the type of the suggested action ('row'/'col' for suggesting row and column,
                  'card' for suggesting a card and 'none' for not suggesting).
                - The second element is a string rapresenting the name of the card suggested.
                - The third element is an integer representing coordinates of suggested card. 
                  The agent returns the coordinates of the position increased by 1 to facilitate the human.
        """
        if action != constants.SUGGEST_NONE:
            current_open, _ = self.env.get_current_open_card()

            if current_open == '':
                name_card, other_location = self.strategy._handle_first_flip()
            else:
                name_card, other_location = self.strategy._handle_second_flip(action)

            if action == constants.SUGGEST_ROW_COLUMN:
                which_index, index_pos = self.__get_which_position_to_suggest(other_location)
                # increase position to make it clear for human
                row, col = index_pos
                if row == -1:   col += 1
                else:           row += 1
                other_location = (row, col)
                return (which_index, name_card, other_location)
            else:
                # increase position to make it clear for human
                row, col = other_location
                return ("card", name_card, (row + 1, col + 1))

        return ("none", None, None)

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

    ##################################################################################################################
    #                                                    Speech                                                      #
    ##################################################################################################################

    def generate_sentence(self, hint_type, flip_type, flag_ToM, card, position):
        """
        Generate a sentence based on the provided hint.
        """
        return self.strategy.generate_sentence(hint_type, flip_type, flag_ToM, card, position)
    

    ##################################################################################################################
    #                                                   Deception                                                    #
    ##################################################################################################################

    def handle_deceptive_behavior(self, robot_type, card_name, position, action):
        """
        This function returns a different card location or name, based on exp.condition, from the one passed as parameter.

        Parameters:
        ----------
            robot_type (int): The robot's deceptive behavior chosen.
            card_name (string): The card name that the robot chosen to suggests.

        Returns:
        ---------
            string: The card's name chosen randomly.
        """
        if hasattr(self.strategy, '_handle_deceptive_behavior'):
            return self.strategy._handle_deceptive_behavior(robot_type, card_name, position, action)
        
        return card_name, position
    
    def has_provided_wrong_card(self, action, suggested_position):
        """
        This function returns a boolean if the position provided by the agent is not the position that makes match with card currently open.

        Parameters:
        ---------
            suggested_position (tuple): the card position provided by the agent

        Returns:
        -------
            True if the position suggested and position that make a match are different, False otherwise
        """
        open_card_name, open_card_pos = self.env.get_current_open_card()

        # if there is no open card then return (since the wrong card is provived only in even turns)
        if open_card_name == '':
            return False
        
        # get other location of current open card
        other_pos = self.env.get_other_location(open_card_name, open_card_pos)
        # increment by 1 both coordinates
        other_pos = tuple([x + 1 for x in other_pos])

        wrong_hint = ''
        if action == 'card':
            wrong_hint = suggested_position != other_pos
        elif action == 'row':
            wrong_hint = suggested_position[0] != other_pos[0]
        else:
            wrong_hint = suggested_position[1] != other_pos[1]

        return  wrong_hint

