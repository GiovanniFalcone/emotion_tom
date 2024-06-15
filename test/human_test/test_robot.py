import unittest

import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'human')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'util')))

from learning.agent import Agent
from game.environment import Environment
import constants

class RobotTestCase(unittest.TestCase):
    def setUp(self):
        self.env = Environment(None, None)
        self.cards = {
            "goose": {
                "first_pos": [0, 0],
                "is_first_opened": False,
                "times_that_first_was_clicked": 1,
                "second_pos": [3, 4],
                "is_second_opened": False,
                "times_that_second_was_clicked": 0,
                "founded": False
            },
            "flamingo": {
                "first_pos": [0, 1],
                "is_first_opened": False,
                "times_that_first_was_clicked": 0,
                "second_pos": [2, 5],
                "is_second_opened": False,
                "times_that_second_was_clicked": 0,
                "founded": False
            },
            "pelican": {
                "first_pos": [0, 2],
                "is_first_opened": False,
                "times_that_first_was_clicked": 0,
                "second_pos": [2, 0],
                "is_second_opened": False,
                "times_that_second_was_clicked": 0,
                "founded": False
            },
            "panda": {
                "first_pos": [0, 3],
                "is_first_opened": False,
                "times_that_first_was_clicked": 0,
                "second_pos": [2, 1],
                "is_second_opened": False,
                "times_that_second_was_clicked": 0,
                "founded": False
            },
            "duck": {
                "first_pos": [0, 4],
                "is_first_opened": False,
                "times_that_first_was_clicked": 0,
                "second_pos": [3, 0],
                "is_second_opened": False,
                "times_that_second_was_clicked": 0,
                "founded": False
            },
            "horse": {
                "first_pos": [0, 5],
                "is_first_opened": False,
                "times_that_first_was_clicked": 0,
                "second_pos": [2, 4],
                "is_second_opened": False,
                "times_that_second_was_clicked": 0,
                "founded": False
            },
            "bird": {
                "first_pos": [1, 0],
                "is_first_opened": False,
                "times_that_first_was_clicked": 0,
                "second_pos": [3, 5],
                "is_second_opened": False,
                "times_that_second_was_clicked": 0,
                "founded": False
            },
            "tiger": {
                "first_pos": [1, 1],
                "is_first_opened": False,
                "times_that_first_was_clicked": 0,
                "second_pos": [3, 2],
                "is_second_opened": False,
                "times_that_second_was_clicked": 0,
                "founded": False
            },
            "koala": {
                "first_pos": [1, 2],
                "is_first_opened": False,
                "times_that_first_was_clicked": 0,
                "second_pos": [3, 3],
                "is_second_opened": False,
                "times_that_second_was_clicked": 0,
                "founded": False
            },
            "walrus": {
                "first_pos": [1, 3],
                "is_first_opened": False,
                "times_that_first_was_clicked": 0,
                "second_pos": [1, 4],
                "is_second_opened": False,
                "times_that_second_was_clicked": 0,
                "founded": False
            },
            "shark": {
                "first_pos": [1, 5],
                "is_first_opened": False,
                "times_that_first_was_clicked": 0,
                "second_pos": [3, 1],
                "is_second_opened": False,
                "times_that_second_was_clicked": 0,
                "founded": False
            },
            "penguin": {
                "first_pos": [2, 2],
                "is_first_opened": False,
                "times_that_first_was_clicked": 0,
                "second_pos": [2, 3],
                "is_second_opened": False,
                "times_that_second_was_clicked": 0,
                "founded": False
            }
        }

    def test_checks_if_the_suggestion_provided_is_the_same_as_the_action_passed_as_the_argument(self):
        tom_mode = 0
        self.agent = Agent(self.env, tom_mode)
        ACTION_SUGGEST_CARD = 2
        self.env.set_history(self.cards)
        self.env.set_board(self.cards)
        suggest, _, _ = self.agent.take_action(ACTION_SUGGEST_CARD)
        self.assertEqual("card", suggest)

    def test_check_if_agent_suggest_other_position_of_opened_card(self):
        tom_mode = 0
        self.agent = Agent(self.env, tom_mode)
        ACTION_SUGGEST_CARD = 2
        self.cards["goose"]["is_first_opened"] = True
        self.env.set_history(self.cards)
        self.env.set_board(self.cards)
        self.env.set_current_open_card_name("goose")
        self.env.set_current_open_card_position([0, 0])
        suggest, card, position = self.agent.take_action(ACTION_SUGGEST_CARD)
        position = [position[0] - 1, position[1] - 1]
        self.assertEqual(suggest, "card")
        self.assertEqual(card, "goose")
        self.assertEqual(position, [3, 4])


    def test_check_if_external_mode_returns_different_card_name_but_right_position_of_other_opened_card(self):
        external_mode = constants.EXTERNAL_DECEPTION
        self.agent = Agent(self.env, external_mode)
        ACTION_SUGGEST_CARD = 2
        self.cards["goose"]["is_first_opened"] = True
        self.env.set_history(self.cards)
        self.env.set_board(self.cards)
        self.env.set_current_open_card_name("goose")
        self.env.set_current_open_card_position([0, 0])
        suggest, card, position = self.agent.take_action(ACTION_SUGGEST_CARD)
        card, position = self.agent.handle_deceptive_behavior(self.agent.type, card, position, ACTION_SUGGEST_CARD)
        position = [position[0] - 1, position[1] - 1]
        self.assertEqual(suggest, "card")
        print(card)
        self.assertNotEqual(card, "goose")
        self.assertEqual(position, [3, 4])

if __name__ == '__main__':
    unittest.main()