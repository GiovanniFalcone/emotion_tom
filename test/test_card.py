import unittest

import sys 
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'robot'))

from card import Card

class CardTestCase(unittest.TestCase):

    game_board = {
        "panda": {
            "first_pos": [
                1,
                2
            ],
            "times_that_first_was_clicked": 10,
            "is_first_opened": False,
            "second_pos": [
                2,
                2
            ],
            "times_that_second_was_clicked": 5,
            "is_second_opened": False,
            "founded": True
        },
        "tiger": {
            "first_pos": [
                0,
                0
            ],
            "times_that_first_was_clicked": 1,
            "is_first_opened": False,
            "second_pos": [
                0,
                3
            ],
            "times_that_second_was_clicked": 1,
            "is_second_opened": False,
            "founded": True
        },
        "horse": {
            "first_pos": [
                0,
                4
            ],
            "times_that_first_was_clicked": 7,
            "is_first_opened": False,
            "second_pos": [
                1,
                1
            ],
            "times_that_second_was_clicked": 3,
            "is_second_opened": False,
            "founded": False
        },
        "duck": {
            "first_pos": [
                2,
                0
            ],
            "times_that_first_was_clicked": 2,
            "is_first_opened": False,
            "second_pos": [
                2,
                3
            ],
            "times_that_second_was_clicked": 3,
            "is_second_opened": False,
            "founded": True
        },
        "penguin": {
            "first_pos": [
                2,
                5
            ],
            "times_that_first_was_clicked": 4,
            "is_first_opened": False,
            "second_pos": [
                3,
                1
            ],
            "times_that_second_was_clicked": 0,
            "is_second_opened": False,
            "founded": False
        },
        "koala": {
            "first_pos": [
                0,
                5
            ],
            "times_that_first_was_clicked": 7,
            "is_first_opened": False,
            "second_pos": [
                3,
                3
            ],
            "times_that_second_was_clicked": 7,
            "is_second_opened": False,
            "founded": False
        },
        "bird": {
            "first_pos": [
                1,
                4
            ],
            "times_that_first_was_clicked": 1,
            "is_first_opened": False,
            "second_pos": [
                3,
                2
            ],
            "times_that_second_was_clicked": 7,
            "is_second_opened": False,
            "founded": True
        },
        "shark": {
            "first_pos": [
                0,
                1
            ],
            "times_that_first_was_clicked": 4,
            "is_first_opened": False,
            "second_pos": [
                3,
                5
            ],
            "times_that_second_was_clicked": 2,
            "is_second_opened": False,
            "founded": False
        },
        "flamingo": {
            "first_pos": [
                1,
                0
            ],
            "times_that_first_was_clicked": 7,
            "is_first_opened": True,
            "second_pos": [
                1,
                3
            ],
            "times_that_second_was_clicked": 1,
            "is_second_opened": False,
            "founded": False
        },
        "goose": {
            "first_pos": [
                1,
                5
            ],
            "times_that_first_was_clicked": 0,
            "is_first_opened": False,
            "second_pos": [
                2,
                4
            ],
            "times_that_second_was_clicked": 0,
            "is_second_opened": False,
            "founded": False
        },
        "walrus": {
            "first_pos": [
                0,
                2
            ],
            "times_that_first_was_clicked": 0,
            "is_first_opened": False,
            "second_pos": [
                2,
                1
            ],
            "times_that_second_was_clicked": 0,
            "is_second_opened": False,
            "founded": False
        },
        "pelican": {
            "first_pos": [
                3,
                0
            ],
            "times_that_first_was_clicked": 0,
            "is_first_opened": False,
            "second_pos": [
                3,
                4
            ],
            "times_that_second_was_clicked": 0,
            "is_second_opened": False,
            "founded": False
        }
    }


    def test_check_if_card_can_be_clicked_2_turn_in_a_row(self):
        available = Card.get_available_cards_and_position(self.game_board)
        # if card is already clicked then the position clicked will be "None"
        card = available['flamingo']['first_pos']
        self.assertEqual(card, None)

    
    def test_if_only_pairs_not_found_are_clickable(self):
        available = Card.get_available_cards_and_position(self.game_board)
        result = "tiger" in available
        self.assertFalse(result)


    def test_check_if_get_other_location_of_card_returns_other_position_of_card(self):
        dict = {
            "flamingo": 
            {
                "first_pos": [1, 0], "is_first_opened": True,
                "second_pos": [1, 3], "is_second_opened": False,
                "founded": False
            }
        }

        position, _ = Card.get_other_location_of_open_card('flamingo', dict)
        self.assertEqual(position, [1, 3])


    def test_check_if_returns_correct_cards_by_row_suggest(self):
        number_of_row = 1
        available = Card.get_available_cards_by_suggest("row", number_of_row, self.game_board)
        number_of_available = len(available)
        self.assertEqual(number_of_available, 3)
        result = 'flamingo' in available and 'horse' in available and 'goose' in available
        self.assertTrue(result)


    def test_if_returns_most_clicked_cards(self):
        card = Card() 
        available = card.get_most_clicked_cards(self.game_board)
        # pairs found are not considered, so the max is 7
        result = "horse" in available and "koala" in available and "flamingo" in available
        self.assertTrue(result)

if __name__ == '__main__':
    unittest.main()

