import json, sys, os, requests
from game.game import Game
from game.player import Player
from game.card import Card

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))
from util import constants
from util import Util

class Environment:
    def __init__(self, SERVER_IP, ID_PLAYER, instance_flask):
        """
        Initialize the game environment.

        Args:
            socket: Socket used for the communication.
            SERVER_IP (str): Nome del server.
        """
        self._SERVER_IP = SERVER_IP
        self._game = Game()
        self._player = Player(self._game)
        self._agent = None
        self._ID_PLAYER = ID_PLAYER
        self.classification = ''
        self.instance_flask = instance_flask

    def set_agent(self, agent):
        self._agent = agent

    def start(self):
        """Starts the game."""
        self._reset()

    def _reset(self):
        """Resets the game state."""
        
        while not self.instance_flask.board_changed.wait(timeout=1):
            if self.instance_flask.stop_thread.is_set():
                Util.formatted_debug_message("Stopping update_environment due to stop_event.", level='INFO')
                return "stopped" 
        self.instance_flask.board_changed.clear()
        decoded_board = self.instance_flask.board
        if 'matrix' in decoded_board:
            game_board = decoded_board['matrix']
        else:
            print("Error: 'matrix' not found, refresh to game page!")

        self._game.reset(game_board)
        self._player.reset()
        self._player.game = self._game
        self._player.create_history()

    def update_environment(self):
        """Updates the game environment state."""
        while not self.instance_flask.board_changed.wait(timeout=1):
            if self.instance_flask.stop_thread.is_set():
                Util.formatted_debug_message("Stopping update_environment due to stop_event.", level='INFO')
                return "stopped"
        self.instance_flask.board_changed.clear()
        data = self.instance_flask.move

        # if user has reloaded web page it could cause an expcetion
        if data is None:
            Util.formatted_debug_message("Killing script since page has been reloaded by " + str(self._ID_PLAYER), level='INFO')
            return

        clicked_card_name = data['game']['open_card_name']
        clicked_card_position = data['game']['position']
        match = data['game']['match']

        self._game.update_state_of_game(clicked_card_name, clicked_card_position, match)
        self._player.update_data_player(clicked_card_name, clicked_card_position, match)

    def get_server_IP(self) -> str:
        """Returns the server IP."""
        return self._server_name
    
    def get_turn(self):
        """Returns the current turn."""
        return self._game.turns
    
    def get_pairs_found(self):
        """Returns the number of pairs found."""
        return self._player.pairs_found
    
    def get_flip_number(self):
        """Returns the flip number."""
        return self._player.flip_number
    
    def set_history(self, dict):
        """set player history"""
        self._player.set_history(dict)

    def set_board(self, dict):
        """set board game"""
        self._game.set_board(dict)

    def play(self, suggestion):
        """Plays the given suggestion."""
        return self._player.play(suggestion)
    
    def get_current_open_card(self):
        """
        Returns the name and position of the current open card.

        Returns:
            tuple: A tuple containing the name of the current open card (str) and its position (list [x, y]).
        """
        return self._game.current_open_card_name, self._game.current_open_card_position

    def get_board(self):
        """Returns the game board."""
        return self._game.board
    
    def get_other_location(self, card, pos):
        """Returns the other location of the open card."""
        game_board = self._game.board
        return Card.get_other_location_of_open_card(card, pos, game_board)
    
    def get_cards_by_index(self, type, index):
        """Returns cards by row/column index."""
        game_board = self._game.board
        current_open_pos = self._game.current_open_card_position
        return Card.get_face_up_cards_by_index(type, index, game_board, current_open_pos)
    
    def was_last_move_a_match(self):
        """Checks if the last move was a match."""
        return self._player.last_match
    
    def is_game_finished(self):
        """Checks if the game is finished."""
        return self._game.is_game_ended()
    
    def get_history(self):
        """Returns the player's history."""
        return self._player.history
    
    def print_player_history(self):
        """Prints the player's history."""
        self._player.print_history()
    
    def get_least_clicked_location_of_most_clicked_card(self):
        """Returns the least clicked location of the most clicked card."""
        history = self._player.history
        return self._game.get_least_clicked_location_of_most_clicked_pair(history)
    
    def get_random_card(self):
        """Returns a random card."""
        return self._game.get_random_card()
        
    def get_least_clicked_location_from_visitated(self):
        """Returns the least clicked location from the visited locations."""
        return self._game.get_least_clicked_location_from_visitated(self._player.history)
    
    def set_board(self, board):
        self._game.set_board(board)

    def set_current_open_card_name(self, card_name):
        self._game.set_current_open_card_name(card_name)

    def set_current_open_card_position(self, position):
        self._game.set_current_open_card_position(position)

    def set_turns(self, turn):
        self._game.set_turns(turn)
    
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
        # debug
        print(f"Turn: {self.get_turn()}\n"
              f"Current action: {action_string}\n"
              f"Current state: {state_string}")

        # the agent select the correct suggestion based on the action chosen greedly
        suggestion = self._agent.take_action(action)
        print("suggestion", suggestion)

        # unpack tupla in order to send it to Flask
        suggest, card, position = suggestion

        # determine ToM flag
        flag_ToM, flip_type = self._determine_TOM_and_flip_type(suggest, card, position)
        #if flag_ToM != None: print("flag tom: ", flag_ToM)

        # for deceptive mode
        card, position = self._agent.handle_deceptive_behavior(self._agent.type, card, position, action)

        # get sentence based on provided hint
        sentence = self._agent.generate_sentence(suggest, flip_type, flag_ToM, card, position)
        print(sentence)

        # Check if the agent provided a wrong card
        wrong_hint = self.check_wrong_hint(action, suggest, position)
        if wrong_hint: print("Wrong hint: ", (card, position))

        # send suggestion to flask server
        self.send_suggest_through_http(
            state_string,       # send current state (for log file)
            suggest,            # send which suggestion (card, row, column)
            card,               # send card name (if suggestion is card, otherwise None)
            position,           # send card position ([n,n] if suggest card, [n, -1] if row, [-1, n] otherwise)
            self._agent.type,   # send agent type (tom, no_tom, deception, ...)
            wrong_hint,         # send flag in order to save if the agent lied to the user
            flag_ToM,           # in order to understand which sentence robot should say
            flip_type,          # which flip (firstCard or secondCard)
            sentence            # sentence that robot will provide
        )

        # update player's history and game state, it's not None if page has been reloaded (this means the current Q-learning thread must be stopped)
        reloaded = self.update_environment()

        # move to next state
        next_state = self.get_next_state(action)

        return next_state, reloaded

    def _determine_TOM_and_flip_type(self, suggest, card, position):
        """
        Determine the ToM flag and flip type.

        Returns:
            tuple: the flag (str), flip_type (str).
        """
        flag_ToM = None       
        flip_type = None
        is_turn_even = self.get_turn() % 2 == 0

        no_tom_modes = [constants.NO_TOM, constants.SUPERFICIAL_DECEPTION]
        if self._agent.type not in no_tom_modes:
            flag_ToM = self._get_tom_flag(suggest, card, position)

        # we don't need tom sentences when deception mode is "hidden" (i.e., the robot will use different set sentences during even turns)
        if self._agent.type == constants.HIDDEN_DECEPTION and is_turn_even:
            flag_ToM = None

        # The JSON file 'no_tom' is the only one without keys for the first and second card since the set of sentences is valid for both flip
        if self._agent.type != constants.NO_TOM:
            flip_type = "secondCard" if is_turn_even else "firstCard"
        else:
            flip_type = None

        return flag_ToM, flip_type

    def check_wrong_hint(self, action, suggest, position):
        """
        Check if the agent provided a wrong card.
        """
        deceptive_modes = [constants.DECEPTION, constants.SUPERFICIAL_DECEPTION]
        is_action_not_none = action != constants.SUGGEST_NONE

        if is_action_not_none and self._agent.type in deceptive_modes:
            wrong_hint = self._agent.has_provided_wrong_card(suggest, position)
            print("Has provided wrong card? ", wrong_hint)
            return wrong_hint
        
        return False

    def send_suggest_through_http(self, state, suggestion, card, position, experimental_condition, wrong_hint, flag_ToM, flip_type, sentence):
        experimental_condition_str = Util.get_experimental_condition(experimental_condition)
            
        json_data = ({
                "action": {
                    "state": state,
                    "suggestion": suggestion,
                    "card": card,
                    "position": position,
                    "experimental_condition": experimental_condition_str,
                    "flip_type": flip_type,
                    "wrong_hint": wrong_hint,
                    "flagToM": flag_ToM,
                    "sentence": sentence
                }
            })
        requests.post("http://" + self._SERVER_IP + ":5000/hint_data/" + str(self._ID_PLAYER), json=json_data)

    ##################################################################################################################
    #                                                   State                                                        #
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

        pairs = self.get_pairs_found()
        is_turn_odd = ((self.get_turn() - 1) % 2) != 0
        is_turn_less_than_six = self.get_turn() - 1 < 6
        attemps_number = self._player.flip_number//2

        if is_turn_less_than_six and pairs == 0:
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
        
        if pairs <= 3:
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
            

        elif 3 < pairs < 8:
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
    #                                              Theory of mind                                                    #
    ##################################################################################################################
    
    def _get_tom_flag(self, suggestion, card, position):
        """
        This function will return a flag to understand what the robot should say in theory of mind case.

        Parameters:
        ----------
        suggestion (str): the type of suggestion ("card", "row" or "column")
        card (str): the name of the card suggested
        position (tuple): the coordinates of suggested card. Both positive if suggestion is card, negative for the row or the column.
        
        Returns:
        --------
        flag_prova: int
            - None if there is no suggestion
            - flag indicating the state of the suggested card or position based on the Theory of Mind considerations
        """
        if suggestion == "none": return None

        player_history = self.get_history()
        open_card, open_position = self.get_current_open_card()
        tom_flag = None

        # if hint is row/column we need the exact coordinates of suggested card and not only one
        if suggestion != "card":
            position = self.get_other_location(card, open_position)
        else:
            position = [position[0] - 1, position[1] - 1]

        which_flip = "secondCard" if open_card else "firstCard"

        if which_flip == "secondCard" or (which_flip == "firstCard" and suggestion == "card"):
            which_to_open = Card.get_which_to_open(card, position, player_history)
        else:
            # row/column case for first flip
            # in order to understand which coordinate [x,y] we need (e.g if 'row' then we neeed 'x')
            index = 0 if suggestion == "row" else 1
            # get a coordinate of card's first location
            first_pos_coordinate = player_history[card]['first_pos'][index]
            # update the coordinate of suggested position
            suggested_pos_coordinate = position[index]
            print(first_pos_coordinate, suggested_pos_coordinate)
            # if they are the same it means that the user should open the first position to made a match
            which_to_open = "is_first_opened" if first_pos_coordinate == suggested_pos_coordinate else "is_second_opened"

        # get flag to understand which case is
        tom_flag = self._player.analyze_click_counts(card, which_to_open, which_flip)
        
        return tom_flag