import random

from game.card import Card

class Game:
    # deck to be shuffled
    __cards = ['goose', 'koala', 'bird', 'tiger', 'panda', 'pelican',
               'penguin', 'walrus', 'flamingo', 'shark', 'horse', 'duck',
               'goose', 'koala', 'bird', 'tiger', 'panda', 'pelican',
               'penguin', 'walrus', 'flamingo', 'shark', 'horse', 'duck']
    
    def __init__(self):
        """
        This is the constructor method. It initializes the object with the following attributes:
        shuffled - the array that contains the board's card shuffled
        board - a dictionary with card name as key and positions and flags as value
        turns - the number of turns played in the game
        pairs_left - the number of pairs of cards remaining to be found
        finished - a boolean indicating if the game has finished
        current_open_card_name - the current open card in the game
        current_open_card_position - the position of the current open card in the game
        num_cols - the number of columns in the game board
        num_rows - the number of rows in the game board
        """
        self.shuffled = []
        self.board = {}
        self.turns = 1
        self.pairs_left = 12
        self.finished = False
        self.current_open_card_name = ''
        self.current_open_card_position = []
        self.num_cols = 6
        self.num_rows = 4
        self.__create_board()

    def reset(self):
        self.shuffled = []
        self.board = {}
        self.turns = 1
        self.pairs_left = 12
        self.finished = False
        self.current_open_card_name = ''
        self.current_open_card_position = []
        self.num_cols = 6
        self.num_rows = 4
        self.__create_board()

    def __create_board(self):
        """
        Create a new dictionary which contains the board data.
        """
        self.shuffled = random.sample(self.__cards, len(self.__cards))
        
        k = 0
        for i in range(self.num_rows):
            for j in range(self.num_cols):
                card = self.shuffled[k]
                if card in self.board:
                    self.board[card]['second_pos'] = [i, j]
                else:
                    self.board[card] = {
                        'first_pos': [i, j],        # the first location of a card
                        'is_first_opened': False,   # is the card currently face up?
                        'second_pos': [-1, -1],
                        'is_second_opened': False,
                        'founded': False            # true if the pair has been found    
                    }
                k += 1

    def __update_game(self, clicked_card_name, clicked_card_position, match, which_is_clicked):
        is_turn_even = self.turns % 2 == 0

        if is_turn_even:
            # click second card
            previous_card = self.current_open_card_name  # it still contain the card's name opened in the previous turn
            which_flag = Card.get_which_is_open(previous_card, self.board)
            self.board[previous_card][which_flag] = False
            self.current_open_card_name = ''
            self.current_open_card_position = []
        else:
            # click first card
            self.current_open_card_name = clicked_card_name
            self.current_open_card_position = clicked_card_position
            self.board[clicked_card_name][which_is_clicked] = True

        if is_turn_even and match:
            self.board[clicked_card_name]['founded'] = True
            self.pairs_left -= 1

        self.turns += 1
    
    def make_move_by_card(self, card, position):
        """
        This function makes a move based on the card that agent has suggested.

        Parameters:
        ----------
            card (str): the name of the card suggested.
            position (int): the position of the card clicked suggested.

        Returns:
        --------
            A tuple containing the card name, position and whether it was a match (True/False).
        """
        # Get the current state of the game board and the name of the currently open card
        game_board = self.board
        open_card_name = self.current_open_card_name

        # in order to figure out which card's location has been face up
        which_is_clicked = Card.get_which_to_open(card, position, game_board)

        # if the agent suggests when there is already a face up card then it's 100% a match
        match = False if  open_card_name == '' else True

        # update game data
        self.__update_game(card, position, match, which_is_clicked)

        return (card, position, match)

    def make_move_by_grid(self, suggest, number_suggested):
        """
        Makes a move in the game based on a suggestion of a row or column and a number in that row or column.

        Parameters:
        ----------
            suggest (str): A string indicating whether the suggestion is for a row or column.
            number_suggested (int): The number of the row or column suggested.

        Returns:
        ---------
            tuple: A tuple containing the card selected, its position on the game board, and a boolean indicating whether it matches with the currently open card.
        """

        # Get the current state of the game board and the name of the currently open card
        game_board = self.board
        open_card_name = self.current_open_card_name

        # Get the available cards to select from based on the suggestion and number suggested
        available_cards = Card.get_available_cards_by_suggest(suggest, number_suggested, game_board)
        # Select a random card from the available cards and get its position on the game board and which location was clicked
        random_card, position, which_is_clicked = Card.get_random_card(available_cards)

        # if there is already an open card then check if it is a match, false otherwise
        if open_card_name != '':
            match = self.__check_if_match(random_card, open_card_name)
        else:
            match = False

        # Update the game data with the selected card, its position, whether it matches, and which location was clicked
        self.__update_game(random_card, position, match, which_is_clicked)

        return (random_card, position, match)

    def select_card(self, pairs_found, click_before_match, has_received_hint_on_first_flip):
        """
        This function selects a card to play based on the number of pairs already found and the number of clicks 
        made since the last match. It calculates the probability of making a match and chooses a card randomly 
        or based on the probability calculated.

        Parameters:
        ----------
            pairs_found (int): The number of pairs already found.
            click_before_match (int): The number of clicks made since the last match.

        Returns:
        ---------
            tuple: A tuple containing the name of the card, the position of the card, and a boolean value indicating 
            whether the card is a match or not.
        """
        pairs_to_found = 12 - pairs_found
        is_turn_even = self.turns % 2 == 0

        # Calculate the probability of not making a match after a certain number of clicks
        probability_of_no_match = (1 - 1 / (2 * pairs_to_found - 1))
        probability_of_no_match_after_n_clicks = pow(probability_of_no_match, click_before_match)
        probability_of_match = (1 - probability_of_no_match_after_n_clicks) * 100

        if has_received_hint_on_first_flip and is_turn_even and pairs_to_found > 4:
            return self._select_card_based_on_dataset_probability(pairs_found)
        elif is_turn_even and probability_of_match > 50 and random.randint(0, 1) == 1:
        #elif is_turn_even and probability_of_match > 5 and random.randint(0, 1) == 1:
            return self._select_correct_card()
        else:
            return self._select_random_card()
        
    def _select_card_based_on_dataset_probability(self, pairs_found):
        """
        The function returns the correct card if a random integer is >= of probability computed on dataset, i.e
        the probability of a match after the agent has provided a hint on first flip.
        If the integer is < of this probability then the virtual user will select a random card.

        Parameters:
        ----------
            pairs_found (int): The number of pairs already found.

        Returns:
        ---------
            tuple: A tuple containing the name of the card, the position of the card, and a boolean value indicating 
            whether the card is a match or not.
        """

        # if last turn user has received hint on first card, they have x% of chance to find a pair
        probability_of_match_given_hint_on_first_card = 0
        if pairs_found <= 3:
            probability_of_match_given_hint_on_first_card = 59
        elif 3 < pairs_found < 8:
            probability_of_match_given_hint_on_first_card = 67

        if random.randint(0, 100) >= probability_of_match_given_hint_on_first_card:
            random_card = self.current_open_card_name
            position, which_is_clicked = Card.get_other_location_of_open_card(random_card, self.board)
            self.__update_game(random_card, position, True, which_is_clicked)
            return (random_card, position, True)
        else:
            return self._select_random_card()
        
    
    def _select_correct_card(self):
        """
            If the current turn is even and the probability of making a match is greater than 50%, 
            the robot chooses the card that allows it to make a match
            N.B after a match the counter is not zero yet, but since we check if the turn is even we don't have any problem
            In order to have a player with perfect memory decrease probability and remove the randint check

            Returns:
            ---------
                tuple: A tuple containing the name of the card, the position of the card, and a boolean value indicating 
                whether the card is a match or not.
        """
        random_card = self.current_open_card_name
        position, which_is_clicked = Card.get_other_location_of_open_card(random_card, self.board)
        self.__update_game(random_card, position, True, which_is_clicked)
        return (random_card, position, True)

    def _select_random_card(self):
        """
        It returns a card selected randomly-

        Returns:
        ---------
            tuple: A tuple containing the name of the card, the position of the card, and a boolean value indicating 
            whether the card is a match or not.
        """
        # Random otherwise
        available_cards = Card.get_available_cards_and_position(self.board)
        random_card, position, which_is_clicked = Card.get_random_card(available_cards)
        match = self.__check_if_match(random_card, self.current_open_card_name)
        self.__update_game(random_card, position, match, which_is_clicked)

        return random_card, position, match

    @staticmethod
    def __check_if_match(first_card, second_card):
        return True if first_card == second_card else False

    def is_game_ended(self):
        return self.pairs_left == 0

    def get_least_clicked_location_of_most_clicked_pair(self, history):
        """
        This function will find the most clicked card. Then it will returns the least clicked location of that card.

        Parameters:
        ----------
            history (dict): the player's history which contains the card name as key, and the 
            locations and how many times they have clicked as values.

        Returns:
        ---------
            tuple: a tuple containing the name of the card and another tuple which contains the coordinates of the least clicked location
        """

        card = Card()
        # get all most clicked cards
        available_cards = card.get_most_clicked_cards(history)
        # choose randomly one of them
        card_name, other_pos, which_is_clicked = Card.get_random_card(available_cards)
        # get least clicked location of chosen card
        if which_is_clicked == 'is_first_opened':
            other_pos = history[card_name]['second_pos']
        else:
            other_pos = history[card_name]['first_pos']

        return card_name, other_pos