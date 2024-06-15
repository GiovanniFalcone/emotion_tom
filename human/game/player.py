import json

from game.card import Card

class Player:
    # for debug
    _MSG_TOM = {
        "bothLocationsOnce":                        "Hai visto entrambe le locazioni una volta sola",
        "oneLocationClickedZeroOtherMultipleTimes": "Hai cliccato spesso la locazione {key_A}, prova a cliccare la locazione {key_B}",
        "bothClickedMultipleTimes":                 "Hai cliccato entrambe le locazioni spesso",
        "currentLocationClickedMultipleTimes":      "Hai cliccato spesso la locazione corrente {key_A}, prova a cliccare l'altra {key_B}",
        "oneCardClickedMultipleTimes":              "L'altra l'hai vista spesso mentre questa una volta sola",
        "otherCases":                               "È la prima volta che scopri questa, ricordi l'altra?",
        "withoutHistory":                           "Prima volta"
    }

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

    def create_history(self):
        k = 0
        for i in range(self.game.num_rows):
            for j in range(self.game.num_cols):
                card = self.game.shuffled[k]

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

    def set_history(self, dict):
        self.history = dict

    def update_data_player(self, card, position, match):
        """
        This function modifies the player's data by updating their pair-finding progress and click counter.

        Parameters:
        -----------
            card (str): The name of clicked card
            position (tuple): The coordinates of clicked card
            match (bool): A boolean value indicating whether the two flipped cards in the current turn form a matching pair.

        Returns:
        ----------
            None
        """

        is_turn_even = (self.game.turns - 1) % 2 == 0

        if self.last_match and is_turn_even is False:
             self.flip_number = 1
        else:
             self.flip_number += 1
        
        if is_turn_even:
            self.last_match = True if match else False

            if self.last_match:
                self.pairs_found += 1

        self.__update_history(card, position, match)

    def __update_history(self, clicked_card_name, clicked_card_position, match):
        """
        This method updates the history of the game with the information of the clicked card.

        Parameters:
        ----------
            clicked_card_name (str): The name of the card that was clicked.
            clicked_card_position (tuple): A tuple containing the row and column indices of the clicked card.
            match (bool): A boolean value indicating whether the two flipped cards in the current turn form a matching pair.
        """
        is_turn_even = (self.game.turns - 1) % 2 == 0

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

        if match and is_turn_even:
            self.history[clicked_card_name]['founded'] = True

        # after a move turn down previous card
        if is_turn_even:
            which_flag = Card.get_which_is_open(self.previous_card_name, self.history)
            self.history[self.previous_card_name][which_flag] = False
            self.history[clicked_card_name][which_is_open] = False
            self.previous_card_name = ''

        self.previous_card_name = clicked_card_name

    def print_history(self):
        print(json.dumps(self.history, indent=4))

    def analyze_click_counts(self, suggested_card, suggested_position, which_flip):
        """
        Returns an integer in order to understand if the user has already seen a card.

        Returns
            - bothLocationsOnce: if it has been clicked only once
            - oneLocationClickedZeroOtherMultipleTimes: it the card to open has never been clicked while the other one has been clicked multiple times
            - bothClickedMultipleTimes: if both had been clicked multiple times
            - currentLocationClickedMultipleTimes: if the clicked card has been clicked multiple times while the other one has been clicked 0 times
            - oneCardClickedMultipleTimes: if the clicked card has been clicked only once while the other one multiple times
            - otherCases: if the clicked card has been clicked for the first time 
            - withoutHistory otherwise
        """
        flag = None

        # get keys
        suggested_clicks_key = 'times_that_first_was_clicked' if suggested_position == "is_first_opened" else "times_that_second_was_clicked"
        # other key (regardless of whether it is face up or not)
        other_clicks_key =  "times_that_second_was_clicked" if suggested_clicks_key == "times_that_first_was_clicked" else "times_that_first_was_clicked" 
        # Get the number of clicks for the suggested card and the other card
        suggested_click_count = self.history[suggested_card][suggested_clicks_key]
        other_click_count = self.history[suggested_card][other_clicks_key]
        # debug
        # print("card suggested: ", suggested_clicks_key, suggested_click_count, "\nother location: ", other_clicks_key, other_click_count)
        # get flag
        if which_flip == "firstCard":
            flag = self._get_flag(other_click_count, suggested_click_count)
        else:
            # in this case the other card is opened and suggested is the one that made match
            flag = self._get_flag(suggested_click_count, other_click_count)
        
        #self._print_debug_msg(flag, other_clicks_key, suggested_clicks_key)

        return flag

    def _get_flag(self, count_A, count_B):
        if count_A == 0:
            if count_B > 1:     return "oneLocationClickedZeroOtherMultipleTimes"
        elif count_A > 1:
            if count_B > 1:     return "bothClickedMultipleTimes"
            elif count_B == 0:  return "currentLocationClickedMultipleTimes"
            elif count_B == 1:  return "otherCases"
        elif count_A == 1:
            if count_B == 1:    return "bothLocationsOnce"
            if count_B > 1:     return "oneCardClickedMultipleTimes"
        
        return "withoutHistory"

    def _print_debug_msg(self, msg_code, key_A, key_B):
        msg = self._MSG_TOM.get(msg_code, "Message not found")
        print(msg.format(key_A=key_A, key_B=key_B))