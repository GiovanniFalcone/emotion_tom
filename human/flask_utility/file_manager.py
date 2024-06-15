"""
Class used to manage csv file
"""
import sys
import os
import numpy as np

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'util'))

from util import Util

class FileManager:

    def __init__(self):
        """
        Initializes a FileManager object.

        Attributes:
        ----------
            id_player (int): ID of the player.
            n_game (int): Number of times that player has played
            CSV_FIELDS (list): List of field names for the CSV data.
            csv_data (dict): Dictionary to store CSV data fields.
        """
        self.id_player = None
        self.n_game = 1
        self.CSV_FIELDS = ['id_player', 'turn_number', 'position_clicked', 'time_game',
                           'time_until_match', 'suggestion_type', 'position_suggested', 
                           'experiment_condition', 'match', 'game_ended', 'wrong_hint']
        self.csv_data = {field: [] for field in self.CSV_FIELDS}

    def _write_hint_data_on_file(self, data, current_experimental_condition):
        """
        Write on file the hint provided by the agent.
        If the experimental condition is deception or hidden, it will also indicate whether the provided hint is correct or not.
        """
        state = data['action']['state']
        hint_type = data['action']['suggestion']
        hint_position = data['action']['position']
        hint_card = data['action']['card']

        wrong_hint = f"Has provided wrong card? {data['action']['wrong_hint']}"

        hint_info = ''
        if hint_type == 'none':
            hint_info = 'none, None, None'
        elif hint_type == 'row':
            hint_info = f'{hint_type}, {hint_card}, ({hint_position[0] - 1}, -1)'
        elif hint_type == 'column':
            hint_info = f'{hint_type}, {hint_card}, (-1, {hint_position[1] - 1})'
        else:
            hint_info = f'{hint_type}, {hint_card}, ({hint_position[0] - 1}, {hint_position[1] - 1})'
        
        self._update_csv_data(data)
        deception_conditions = ['deception', 'superficial_deception']
        if current_experimental_condition in deception_conditions: 
            Util.update_log_file(f"\n\nHint: ({hint_info})\n{state}\n{wrong_hint}", self.id_player, self.n_game)
        else:
            Util.update_log_file(f"\n\nHint: ({hint_info})\n{state}", self.id_player, self.n_game)

    def _write_game_data_on_file(self, data):
        """
        Write on file the user's info and update the structure for csv data.
        """
        self._update_csv_data(data)
        self._update_log_file_by_game_data(data)

    def _update_log_file_by_game_data(self, data):
        """
        Write on log file the user's info.
        """
        game_data = data["game"]
        Util.update_log_file(f"\nTurn: {game_data['turn']}\nPosition_clicked: {game_data['position']}\nCard_clicked: {game_data['open_card_name']}\nTime_game: {game_data['time_game']}\nTime_before_match: {game_data['time_until_match']}\nMatch: {game_data['match']}", self.id_player, self.n_game)

    def _write_board_on_file(self, shuffle_cards):
        """
        Print game board as matrix and write it on log-file.

        Args:
            shuffle_cards (list): List of cards.
        """
        output_lines = []
        board = np.zeros((6, 4))
        board = np.reshape(shuffle_cards, (4, 6))
        mx = len(max((sub[0] for sub in board), key=len))
        for row in board:
            output_lines.append(" ".join(["{:<{mx}}".format(ele, mx=mx) for ele in row]))
        print("\n")

        # write board on file
        if self.id_player != -1: Util.update_log_file("\n".join(output_lines) + "\n", self.id_player, self.n_game)

    def _clear_csv_struct(self):
        """
        Clear csv structure for other users.
        """
        for field in self.CSV_FIELDS:
            if isinstance(self.csv_data[field], list):
                self.csv_data[field].clear()
            else:
                self.csv_data[field] = ''
    
    def _update_csv_data(self, data):
        """
        Update structure for csv data.
        """

        if 'game' in data:
            self.csv_data["id_player"].append(self.id_player)
            #self.csv_data["experiment_condition"].append(self.experimental_condition_str)

            self.csv_data["turn_number"].append(data["game"]["turn"])
            self.csv_data["position_clicked"].append(data["game"]["position"])
            self.csv_data["time_game"].append(data["game"]["time_game"])
            self.csv_data["time_until_match"].append(data["game"]["time_until_match"])
            self.csv_data["match"].append(data["game"]["match"])
            self.csv_data["game_ended"].append(data["game"]["pairs"] == 12)

            # if game is finished, write the csv file and clear the csv structure
            if data["game"]["pairs"] == 12:
                Util.formatted_debug_message("Saving csv...", level='INFO')
                Util.put_data_in_csv(self.csv_data, self.id_player, self.n_game)
                Util.formatted_debug_message("Data saved on csv file. Clear csv struct...", level='INFO')
                self._clear_csv_struct()

        if 'action' in data:
            action_data = data["action"]
            self.csv_data["experiment_condition"].append(data["action"]["experimental_condition"])

            suggestion_type = action_data["suggestion"]
            
            self.csv_data["suggestion_type"].append(suggestion_type)
            self.csv_data["wrong_hint"].append(action_data["wrong_hint"])

            if suggestion_type == "none":
                self.csv_data["position_suggested"].append("none")
            else:
                if suggestion_type in ["row", "column"]:
                    position_index = 0 if suggestion_type == "row" else 1
                    suggested_position = action_data["position"][position_index] - 1
                else:
                    suggested_position = [pos - 1 for pos in action_data["position"]]

                self.csv_data["position_suggested"].append(suggested_position)