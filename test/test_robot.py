import unittest

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'robot'))

from agent import Agent
from environment import Environment

class RobotTestCase(unittest.TestCase):

    def setUp(self):
        self.env = Environment()
        self.agent = Agent(self.env)
        self.cards = {
            'flamingo': 
            {
                'times_that_first_was_clicked': 10, 'first_pos': [1, 0], 'is_first_opened': False,
                'times_that_second_was_clicked': 0, 'second_pos': [1, 3], 'is_second_opened': False,
                'founded': False
            },
            'panda': 
            {
                'times_that_first_was_clicked': 0, 'first_pos': [1, 2], 'is_first_opened': False, 
                'times_that_second_was_clicked': 0, 'second_pos': [2, 2], 'is_second_opened': False, 
                'founded': True
            },
            'tiger': 
            {
                'times_that_first_was_clicked': 0, 'first_pos': [0, 0], 'is_first_opened': False, 
                'times_that_second_was_clicked': 0, 'second_pos': [0, 3], 'is_second_opened': False, 
                'founded': True
            },
            'pelican': 
            {
                'times_that_first_was_clicked': 0, 'first_pos': [3, 0], 'is_first_opened': False, 
                'times_that_second_was_clicked': 0, 'second_pos': [3, 4], 'is_second_opened': False, 
                'founded': False
            }
        }

    def test_checks_if_the_suggestion_provided_is_the_same_as_the_action_passed_as_the_argument(self):
        ACTION_SUGGEST_CARD = 2
        suggest, _, _ = self.agent.take_action(ACTION_SUGGEST_CARD)
        self.assertEqual("card", suggest)

    def test_check_if_agent_suggest_other_position_of_most_clicked_card(self):
        # this is the suggestion on the first card
        ACTION_SUGGEST_CARD = 2
        player = self.env.get_player()
        player.history = self.cards
        suggest, card, position = self.agent.take_action(ACTION_SUGGEST_CARD)
        self.assertEqual(position, [1, 3])

if __name__ == '__main__':
    unittest.main()