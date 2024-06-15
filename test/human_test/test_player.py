import unittest

import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'human')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'util')))

from game.player import Player
from util import constants

class PlayerTestCase(unittest.TestCase):
    def setUp(self):
        self.player = Player(None)
        self.cards = {
            "goose": {
                "first_pos": [0, 0],
                "is_first_opened": False,
                "times_that_first_was_clicked": 0,
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

    def test_check_flag_is_withoutHistory_when_firstCard(self):
        which_flip = "secondCard"
        self.cards["goose"]["times_that_first_was_clicked"] = 1
        suggested_card = "goose"
        suggested_position = "is_first_opened"
        self.player.set_history(self.cards)
        flag = self.player.analyze_click_counts(suggested_card, suggested_position, which_flip)
        self.assertEqual(flag, "withoutHistory")

    def test_check_flag_is_bothLocationsOnce_when_firstCard(self):
        # ha cliccato entrambe una volta
        which_flip = "firstCard"
        self.cards["goose"]["times_that_first_was_clicked"] = 1
        self.cards["goose"]["times_that_second_was_clicked"] = 1
        suggested_card = "goose"
        suggested_position = "is_second_opened"
        self.player.set_history(self.cards)
        flag = self.player.analyze_click_counts(suggested_card, suggested_position, which_flip)
        self.assertEqual(flag, "bothLocationsOnce")

    def test_check_flag_is_bothClickedMultipleTimes_when_firstCard(self):
        # hai cliccato spesso entrambe le locazioni
        which_flip = "firstCard"
        self.cards["goose"]["times_that_first_was_clicked"] = 2
        self.cards["goose"]["times_that_second_was_clicked"] = 2
        suggested_card = "goose"
        suggested_position = "is_second_opened"
        self.player.set_history(self.cards)
        flag = self.player.analyze_click_counts(suggested_card, suggested_position, which_flip)
        self.assertEqual(flag, "bothClickedMultipleTimes")

    def test_check_flag_is_otherCases_when_firstCard(self):
        # hai cliccato spesso entrambe le locazioni
        which_flip = "firstCard"
        self.cards["goose"]["times_that_first_was_clicked"] = 5
        self.cards["goose"]["times_that_second_was_clicked"] = 1
        suggested_card = "goose"
        suggested_position = "is_second_opened"
        self.player.set_history(self.cards)
        flag = self.player.analyze_click_counts(suggested_card, suggested_position, which_flip)
        self.assertEqual(flag, "otherCases")

    ############################################################################################
    #                                    second card                                           #
    ############################################################################################

    def test_check_flag_is_withoutHistory_when_secondCard(self):
        which_flip = "secondCard"
        self.cards["goose"]["is_first_opened"] = True
        self.cards["goose"]["times_that_first_was_clicked"] = 1
        suggested_card = "goose"
        # the card suggest is the second one since the first is already opened
        suggested_position = "is_second_opened"
        self.player.set_history(self.cards)
        flag = self.player.analyze_click_counts(suggested_card, suggested_position, which_flip)
        self.assertEqual(flag, "withoutHistory")

    def test_check_flag_is_bothLocationsOnce_when_secondCard(self):
        # ha cliccato entrambe una volta
        which_flip = "secondCard"
        self.cards["goose"]["is_first_opened"] = True
        self.cards["goose"]["times_that_first_was_clicked"] = 1
        self.cards["goose"]["times_that_second_was_clicked"] = 1
        suggested_card = "goose"
        # the card suggest is the second one since the first is already opened
        suggested_position = "is_second_opened"
        self.player.set_history(self.cards)
        flag = self.player.analyze_click_counts(suggested_card, suggested_position, which_flip)
        self.assertEqual(flag, "bothLocationsOnce")

    def test_check_flag_is_oneLocationClickedZeroOtherMultipleTimes_when_secondCard(self):
        # ha cliccato spesso una locazione mentre l'altra no
        which_flip = "secondCard"
        self.cards["goose"]["is_first_opened"] = True
        self.cards["goose"]["times_that_first_was_clicked"] = 2
        suggested_card = "goose"
        # the card suggest is the second one since the first is already opened
        suggested_position = "is_second_opened"
        self.player.set_history(self.cards)
        flag = self.player.analyze_click_counts(suggested_card, suggested_position, which_flip)
        self.assertEqual(flag, "oneLocationClickedZeroOtherMultipleTimes")

    def test_check_flag_is_bothClickedMultipleTimes_when_secondCard(self):
        # hai cliccato spesso entrambe le locazioni
        which_flip = "secondCard"
        self.cards["goose"]["is_first_opened"] = True
        self.cards["goose"]["times_that_first_was_clicked"] = 2
        self.cards["goose"]["times_that_second_was_clicked"] = 2
        suggested_card = "goose"
        # the card suggest is the second one since the first is already opened
        suggested_position = "is_second_opened"
        self.player.set_history(self.cards)
        flag = self.player.analyze_click_counts(suggested_card, suggested_position, which_flip)
        self.assertEqual(flag, "bothClickedMultipleTimes")

    def test_check_flag_is_oneCardClickedMultipleTimes_when_secondCard(self):
        # hai cliccato spesso una locazione rispetto all'altra
        which_flip = "secondCard"
        self.cards["goose"]["is_first_opened"] = True
        self.cards["goose"]["times_that_first_was_clicked"] = 2
        self.cards["goose"]["times_that_second_was_clicked"] = 1
        suggested_card = "goose"
        # the card suggest is the second one since the first is already opened
        suggested_position = "is_second_opened"
        self.player.set_history(self.cards)
        flag = self.player.analyze_click_counts(suggested_card, suggested_position, which_flip)
        self.assertEqual(flag, "oneCardClickedMultipleTimes")

    def test_check_flag_is_otherCases_when_secondCard(self):
        # hai cliccato spesso una locazione rispetto all'altra
        which_flip = "secondCard"
        self.cards["goose"]["is_second_opened"] = True
        self.cards["goose"]["times_that_first_was_clicked"] = 2
        self.cards["goose"]["times_that_second_was_clicked"] = 1
        suggested_card = "goose"
        # the card suggest is the second one since the first is already opened
        suggested_position = "is_first_opened"
        self.player.set_history(self.cards)
        flag = self.player.analyze_click_counts(suggested_card, suggested_position, which_flip)
        self.assertEqual(flag, "otherCases")