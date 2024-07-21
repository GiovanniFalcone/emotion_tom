import json

from game.card import Card

class Player:
    def __init__(self, game):
        """
        Initialize the Player object.

        Parameters:
        ----------
        game (Game): a Game object to associate with the player.

        Attributes:
        history (dict): A dictionary that tracks the player's card history.
            The key is the name of the card, while the value is a dictionary with the following keys:
                - card (str): the name of the card.
                - first_pos (int): the number of times the first position of the card has been clicked.
                - second_pos (int): the number of times the second position of the card has been clicked.
        last_match (bool): True if the previous move was a match, False otherwise.
        flip_number (int): The number of flip made by the player before they find a pair.
        pairs_found (int): The number of pairs found by the player.
        previous_card_name (str): The name of the card selected in the previous move.
        """
        self.game = game
        self.history = {}
        self.last_match = False
        self.flip_number = 0
        self.pairs_found = 0
        self.previous_card_name = ''
        self.create_history()

    def reset(self):
        """
        Reset the player's attributes to their initial values.
        """
        self.game = None
        self.history = {}
        self.last_match = False
        self.flip_number = 0
        self.pairs_found = 0
        self.previous_card_name = ''
        self.has_received_hint_on_first_flip = False
    
    def play(self, suggestion, turn):
        """
        Play a move suggested by the agent.

        Parameters:
        ----------
        suggestion (tuple): A tuple that represents the suggested move.
            The tuple has three elements:
                - suggestion_type (str): The type of suggestion ('none', 'card', or 'row'/'column').
                - card (str or None): The name of the card suggested, if any.
                - position (tuple): The position suggested, as a tuple of integers (row, column).

        Returns:
        --------
        tuple: A tuple that represents the played move.
            The tuple has three elements:
                - clicked_card_name (str): The name of the card clicked.
                - clicked_card_position (tuple): The position of the card clicked, as a tuple of integers (row, column).
                - match (bool): True if the clicked card matches the previous card clicked, False otherwise.
        """
        suggestion_type, card, position = suggestion

        if turn % 2 != 0 and suggestion_type != "none":
            self.has_received_hint_on_first_flip = True

        # get a card
        if suggestion_type == 'none':
            # get "random" card (as the turns progress, the probability of making matches increases, so it is not entirely random) 
            clicked_card_name, clicked_card_position, match = self.game.select_card(self.pairs_found, self.flip_number, self.has_received_hint_on_first_flip)
        elif suggestion_type == 'card':
            # take the card suggested
            clicked_card_name, clicked_card_position, match = self.game.make_move_by_card(card, position)
        else:
            # take random card based on specific row or col
            row = position[0]
            column = position[1]

            number_suggested = row if column == -1 else column
            clicked_card_name, clicked_card_position, match = self.game.make_move_by_grid(suggestion_type, number_suggested)

        # update player info turn by turn
        self.__update_player_info(match)
        self.__update_history(clicked_card_name, clicked_card_position, match)

        return clicked_card_name, clicked_card_position, match

    def __update_player_info(self, match):
        """
        This function modifies the player's data by updating their pair-finding progress and click counter.

        Parameters:
        -----------
            match (bool): A boolean value indicating whether the two flipped cards in the current turn form a matching pair.

        Returns:
        ----------
            None
        """
        
        is_turn_even = (self.game.turns - 1) % 2 == 0

        if is_turn_even:
            self.has_received_hint_on_first_flip = False

        # reset counter if a pair was found in the previous turn
        if self.last_match and is_turn_even is False:
            self.flip_number = 0

        # update match only after move(2 turn) and not turn by turn
        if is_turn_even and match:
            self.pairs_found += 1
            self.flip_number += 1
            self.last_match = True
        elif is_turn_even and match is False:
            self.last_match = False
            self.flip_number += 1
        else:
            self.flip_number += 1

    def create_history(self):
        deck = self.game.shuffled

        k = 0
        for i in range(self.game.num_rows):
            for j in range(self.game.num_cols):
                card = deck[k]
                if card in self.history:
                    self.history[card]['second_pos'] = [i, j]
                else:
                    self.history[card] = {
                        'first_pos': [i, j],          
                        'is_first_opened': False,           
                        'times_that_first_was_clicked': 0,  
                        'second_pos': [-1, -1],
                        'is_second_opened': False,
                        'times_that_second_was_clicked': 0,
                        'founded': False                      
                    }
                k += 1

    def __update_history(self, clicked_card_name, clicked_card_position, match):
        """
        This method updates the history of the game with the information of the clicked card.

        Parameters:
        ----------
            clicked_card_name (str): The name of the card that was clicked.
            clicked_card_position (tuple): A tuple containing the row and column indices of the clicked card.
            match (bool): A boolean value indicating whether the two flipped cards in the current turn form a matching pair.
        """
        if self.history[clicked_card_name]['first_pos'] == clicked_card_position:
            # update first location
            self.history[clicked_card_name]['times_that_first_was_clicked'] += 1
            self.history[clicked_card_name]['is_first_opened'] = True
            which_is_open = 'is_first_opened'
        else:
            # update first location
            self.history[clicked_card_name]['times_that_second_was_clicked'] += 1
            self.history[clicked_card_name]['is_second_opened'] = True
            which_is_open = 'is_second_opened'

        is_turn_even = (self.game.turns - 1) % 2 == 0

        if match and is_turn_even:
            self.history[clicked_card_name]['founded'] = True

        # after a move turn down previous card
        if is_turn_even:
            # get the card that is still open (the first one)
            which_flag = Card.get_which_is_open(self.previous_card_name, self.history)
            self.history[self.previous_card_name][which_flag] = False
            self.history[clicked_card_name][which_is_open] = False
            self.previous_card_name = ''

        self.previous_card_name = clicked_card_name

    def print_history(self):
        print(json.dumps(self.history, indent=4))
