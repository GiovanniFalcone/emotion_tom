import sys
import os
import math

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))
from util import constants

from game.card import Card
from game.player import Player
from game.game import Game

class Environment:
    def __init__(self):
        self.__game = Game()
        self.__player = Player(self.__game)
        self.__reward_first_card = 0.1
        self.__has_suggests_for_the_first_card = False
        self._agent = None

    def __reset(self):
        self.__game.reset()
        self.__player.reset()
        self.__player.game = self.__game
        # in order to create the player history game must be initialized first
        self.__player.create_history()
        self.__reward_first_card = 0.1
        self.__has_suggests_for_the_first_card = False
        #self._agent = None

    def start(self):
        self.__reset()

    def get_player(self):
        return self.__player
    
    def set_agent(self, agent):
        self._agent = agent
    
    def get_game(self):
        return self.__game

    def get_turn(self):
        return self.__game.turns
    
    def get_pairs_found(self):
        return self.__player.pairs_found
    
    def get_flip_number(self):
        return self.__player.flip_number
    
    def is_Begin_state(self):
        pairs_found = self.get_pairs_found()

        return pairs_found <= 3
    
    def is_Middle_state(self):
        pairs_found = self.get_pairs_found()

        return 3 < pairs_found < 8
    
    def is_End_state(self):
        pairs_found = self.get_pairs_found()

        return pairs_found >= 8
    
    def play(self, suggestion, turn):
        return self.__player.play(suggestion, turn)
    
    def get_current_open_card(self):
        return self.__game.current_open_card_name
    
    def get_other_location(self, card):
        game_board = self.__game.board
        return Card.get_other_location_of_open_card(card, game_board)
    
    def get_cards_by_index(self, type, index):
        game_board = self.__game.board
        return Card.get_face_up_cards_by_index(type, index, game_board)
    
    def was_last_move_a_match(self):
        return self.__player.last_match
    
    def is_game_finished(self):
        return self.__game.is_game_ended()
    
    def print_player_history(self):
        self.__player.print_history()
    
    def get_least_clicked_location_of_most_clicked_card(self):
        history = self.__player.history

        return self.__game.get_least_clicked_location_of_most_clicked_pair(history)
    
    ##################################################################################################################
    #                                                 Step                                                           #
    ##################################################################################################################

    def step(self, state, action, action_string, state_string):
        """
        Takes a step in the environment using the selected action.
        
        Parameters:
            state (int): The current state.
            action (int): The selected action.
            action_string (str): The string associated with the selected action.
            state_string (str): The string associated with the current state.
        
        Returns:
            tuple: The next state, reward, and done flag.
        """
        # get suggestion based on action selected
        suggestion = self._agent.take_action(action)

        # click a card and follow the hint provided
        move = self.play(suggestion, self.get_turn())
        
        next_state = self.get_next_state(action)

        reward = self.get_reward(action)
        
        # debug
        """print(f"Turn: {self.get_turn() - 1}\n"
              f"Current action: {action_string}\n"
              f"Current state: {state_string}\n"
              f"Suggestion: {suggestion}\n"
              f"Attemps: {self.__player.flip_number//2}\n"
              f"Clicked: {move}\n"
              f"Reward: {reward}\n")"""

        return next_state, reward

    ##################################################################################################################
    #                                                 State                                                          #
    ##################################################################################################################

    def get_next_state(self, action):
        """
        Returns the next state based on the current state and the agent's action.

        Parameters
        ----------
            action (int): the action the agent takes, represented by an integer code

        Returns
        ----------
            str: the name of the next state

        Raises
        ----------
            AttributeError: if the specified constant name doesn't exist in the constants module
        """

        is_turn_odd = ((self.get_turn() - 1) % 2) != 0
        is_turn_less_than_six = self.get_turn() - 1 < 6
        attemps_number = self.__player.flip_number//2

        if is_turn_less_than_six and self.get_pairs_found() == 0:
            return getattr(constants, 'INIT_STATE')

        state_suffix = "CORRECT" if self.was_last_move_a_match() else "WRONG"

        attempts_suffix = ''
        if state_suffix == "WRONG":
            if attemps_number == 1:
                attempts_suffix = '_LOW'
            elif attemps_number == 2:
                attempts_suffix = '_MEDIUM'
            else:
                attempts_suffix = '_HIGH'
        
        if self.is_Begin_state():
            if is_turn_odd:
                return self.__which_state([
                    self.get_constant('NO_HELP_BEG_S_', state_suffix, attempts_suffix),
                    self.get_constant('SUGG_ROW_BEG_S_', state_suffix, attempts_suffix),
                    self.get_constant('SUGG_CARD_BEG_S_', state_suffix, attempts_suffix)],
                    action)
            else:
                return self.__which_state([
                    self.get_constant('NO_HELP_BEG_F_', state_suffix, attempts_suffix),
                    self.get_constant('SUGG_ROW_BEG_F_', state_suffix, attempts_suffix),
                    self.get_constant('SUGG_CARD_BEG_F_', state_suffix, attempts_suffix)],
                    action)
            

        elif self.is_Middle_state():
            if is_turn_odd:
                return self.__which_state([
                    self.get_constant('NO_HELP_MID_S_', state_suffix, attempts_suffix),
                    self.get_constant('SUGG_ROW_MID_S_', state_suffix, attempts_suffix),
                    self.get_constant('SUGG_CARD_MID_S_', state_suffix, attempts_suffix)],
                    action)
            else:
                return self.__which_state([
                    self.get_constant('NO_HELP_MID_F_', state_suffix, attempts_suffix),
                    self.get_constant('SUGG_ROW_MID_F_', state_suffix, attempts_suffix),
                    self.get_constant('SUGG_CARD_MID_F_', state_suffix, attempts_suffix)],
                    action)

        else:
            if is_turn_odd:
                return self.__which_state([
                    self.get_constant('NO_HELP_END_S_', state_suffix, attempts_suffix),
                    self.get_constant('SUGG_ROW_END_S_', state_suffix, attempts_suffix),
                    self.get_constant('SUGG_CARD_END_S_', state_suffix, attempts_suffix)],
                    action)
            else:
                return self.__which_state([
                    self.get_constant('NO_HELP_END_F_', state_suffix, attempts_suffix),
                    self.get_constant('SUGG_ROW_END_F_', state_suffix, attempts_suffix),
                    self.get_constant('SUGG_CARD_END_F_', state_suffix, attempts_suffix)],
                    action)

    @staticmethod
    def get_constant(constant_name, state_suffix, attempts_suffix):
        """
        Returns the value of the specified constant from the constants module, with the given suffix.

        Parameters
        ----------
            constant_name (str): the name of the constant
            state_suffix (str): the suffix to append to the constant name

        Returns
        ----------
            object: the value of the constant

        Raises
        ----------
            AttributeError: if the specified constant name doesn't exist in the constants module
        """
        return getattr(constants, constant_name + state_suffix + attempts_suffix)

    @staticmethod
    def __which_state(states, action):
        """
        Returns the correct state to transition to based on the last action.

        Parameters
        ----------
            states (List of strings): The possible states to transition to.
            action (int): The last action taken.

        Returns
        ----------
            String: The state to transition to based on the last action.
        """

        if action == constants.SUGGEST_NONE:
            return states[0]
        elif action == constants.SUGGEST_ROW_COLUMN:
            return states[1]
        else:
            return states[2]

    ##################################################################################################################
    #                                                 Reward                                                         #
    ##################################################################################################################
        
    def get_reward(self, action):
        """
        Returns a reward for the specified action, based on the current state of the environment.

        Parameters:
        ----------
            action: An integer representing the action taken by the agent.

        Returns:
        --------
            A floating point number representing the reward for the specified action.
        """
        num_pairs_found = self.get_pairs_found()
        has_found_pair = self.was_last_move_a_match()
        is_turn_even = (self.get_turn() - 1) % 2 == 0

        # for the first six turns the agent can't provide any suggestion, thus the reward is 0
        # n.b the turn is not the current turn, but the next one
        if self.get_turn() < 7 and num_pairs_found == 0:
            return 0
        
        # suggestion for the first card
        if not is_turn_even and self.__should_suggest_for_first_card():
            return self.__get_reward_for_first_card(action)

        # reset reward for the first card if previous suggestion was not useful
        if is_turn_even and not has_found_pair and self.__has_suggests_for_the_first_card:
            self.__reset_reward_for_first_card()
        
        # if the player has found a pair
        if has_found_pair and is_turn_even:
            self.__has_suggests_for_the_first_card = False
            self.__reset_reward_for_first_card()
            return self.__get_reward_for_second_card(action)
        
        return 0
    
    def __should_suggest_for_first_card(self):
        """
        Determines if the agent should suggest a card for the first flip.

        Returns:
        --------
            A boolean indicating whether the agent should suggest a card for the first flip.
        """
        clicks_until_match = self.get_flip_number()
        pairs_found = self.get_pairs_found()
        threshold_with_pair = constants.CLICKS_UNTIL_MATCH_THRESHOLD_WITH_PAIR
        threshold_without_pair = constants.CLICKS_UNTIL_MATCH_THRESHOLD_WITHOUT_PAIR

        return (clicks_until_match > threshold_with_pair and pairs_found > 0) or\
                (clicks_until_match > threshold_without_pair and pairs_found == 0)
    
    def __reset_reward_for_first_card(self):
        """
        Resets the reward for the first card to its default value.
        """
        self.__reward_first_card = 0.1

    def __get_reward_for_first_card(self, action):
        """
        Calculates the reward for suggesting the first card.

        Parameters:
        -----------
            action (int): The action taken by the agent.

        Returns:
        -------
            float: The calculated reward.
        """
        self._set_has_suggested_first_card(action != constants.SUGGEST_NONE)

        # get player's info
        moves_attemps = self.__player.flip_number//2    # number of moves before match
        clicks = self.__player.flip_number              # number of flip before math
        begin = self.is_Begin_state()
        middle = self.is_Middle_state()
        end = self.is_End_state()

        attempts_state = self._calculate_attempts_state('first', moves_attemps)    # constant based on number of moves
        game_state = self._calculate_game_state_constant_of_first_flip()           # constant based on game state
        a = self._get_constant_based_on_action(action)                             # constant based on action

        if middle and action == constants.SUGGEST_NONE:
            self.__reward_first_card += (10/(attempts_state * clicks) * game_state)
        elif (end or begin) and moves_attemps > 2 and action != constants.SUGGEST_NONE:
            self.__reward_first_card += -1
        else:
            self.__reward_first_card += (a * (attempts_state * clicks) / game_state)

        return self.__reward_first_card

    def _set_has_suggested_first_card(self, boolean):
        """
        Sets the hint status for the first flip.

        Parameters:
        ----------
            boolean (bool): True if a none hint was provived, False otherwise
        """
        self.__has_suggests_for_the_first_card = boolean

    @staticmethod
    def _get_constant_based_on_action(action):
        """
        Returns a constant based on provided action.

        Parameters:
        -----------
            action (int): The action taken by the agent.

        Returns:
        -------
            float: a constant.
        """
        if action == constants.SUGGEST_ROW_COLUMN:  return 0.01
        elif action == constants.SUGGEST_CARD:      return 0.35
        else:                                       return 0.55

    def _calculate_game_state_constant_of_first_flip(self):
        """
        Returns a constant based on game state.

        Parameters:
        -----------
            action (int): The action taken by the agent.

        Returns:
        -------
            float: a constant.
        """
        begin = self.is_Begin_state()
        end = self.is_End_state()

        if begin or end:    return 3       
        else:               return 0.75

    @staticmethod
    def _calculate_attempts_state(flip_type, moves_attempts):
        """
        Returns a constant based on number of errors.

        Parameters:
        -----------
            flip_type (str): The flip type (first or second)
            moves_attempts (int): The current number of moves (before match)

        Returns:
        -------
            float or tuple: a constant when the flip is the first one, a tuple otherwise (constant error and a multiplicative constant).
        """

        if flip_type not in ['first', 'second']:
            raise ValueError('Flip type must be "first" or "second"!')
        
        if flip_type == 'first':
            if moves_attempts == 1:         return 0.1 # low
            elif moves_attempts == 2:       return 0.4 # medium
            else:                           return 0.8 # high
        else:
            if moves_attempts == 1:         return (constants.LOW_STATE, 0)
            elif moves_attempts == 2:       return (constants.MEDIUM_STATE, 0) 
            else:                           return (constants.HIGH_STATE, 1)

    def __get_reward_for_second_card(self, action):
        """
        Calculates the reward for suggesting the second card.

        Returns:
        ---------
            float: The calculated reward.
        """
        clicks_until_match = self.get_flip_number()     # clicks before match
        flip_number = self.__player.flip_number // 2    # moves before match
        # to report [correct, low, medium, high] = 4 to [low, medium, high] = 3 attempts
        moves_attempts = flip_number if flip_number == 1 else (flip_number - 1)
        # get constants in order to calculate the reward
        attempts_errors = self._calculate_attempts_state('second', moves_attempts)
        constants_game_state = self._calculate_game_state_constant_of_second_flip(moves_attempts)

        return self.__get_reward_by_state_action_pair(action, clicks_until_match, attempts_errors, 
                                                      constants_game_state, self.__has_suggests_for_the_first_card)

    def _calculate_game_state_constant_of_second_flip(self, moves_attempts):
        """
        Returns a set of contants based on game state.

        Parameters:
        -----------
            moves_attempts (int): The current number of moves (before match)

        Returns:
        -------
            tuple
        """

        begin = self.is_Begin_state()
        middle = self.is_Middle_state()

        k, alpha, beta = 0, 0, 0
        if begin:
            k = moves_attempts * constants.BEGIN_STATE 
            alpha, beta = 0.3, 1
        elif middle:
            k = moves_attempts * constants.MIDDLE_STATE 
            alpha, beta = 0, 0
        else:
            k = moves_attempts * constants.END_STATE 
            alpha, beta = 0, 0

        return k, alpha, beta

    @staticmethod
    def __get_reward_by_state_action_pair(action, clicks_until_match, attempts_errors, constants_game_state, has_suggests_for_the_first_card):
        """
        Calculates the reward for the current state-action pair.

        Parameters:
        ----------
            action (int): The action taken by the agent.
            clicks_until_match (int): The number of clicks before finding a pair.
            attempts_errors (tuple): the tuple of constants based on number of errors
            constants_game_state (tuple): the tuple of constants based on game state
            has_suggests_for_the_first_card (bool): a boolean that identifies whether a not-none action was provided at the previous turn

        Returns:
        ---------
            float: The calculated reward.
        """
        k, alpha, beta = constants_game_state
        attempts, gamma = attempts_errors

        if action == constants.SUGGEST_ROW_COLUMN:
            return math.log(constants.REWARD_SUGGEST_RC * k * attempts) + (-alpha * clicks_until_match * beta * gamma)
        
        elif action == constants.SUGGEST_CARD and has_suggests_for_the_first_card:
            return -0.5
        
        elif action == constants.SUGGEST_CARD:
            return math.log(constants.REWARD_SUGGEST_CARD * k * attempts) + (alpha * clicks_until_match * beta * gamma)
        
        else:
            return math.log(constants.REWARD_SUGGEST_NONE / (k * attempts))