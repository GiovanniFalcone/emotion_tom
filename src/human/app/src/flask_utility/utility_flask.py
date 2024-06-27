from flask import jsonify
from flask_socketio import emit

import threading
import json
import multiprocessing

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'util'))

from util import Util
from play import run_q_learning

from flask_utility.file_manager import FileManager

class UtilityFlask:
    """
    Utility class for Flask application management.

    Attributes:
        IP_ADDRESS (str): IP address of the server.
        POP_UP (bool): Flag indicating whether pop-up windows are enabled (used before start the game).
        LEARNING_PORT (int): Port number for the learning socket.
        ROBOT_PORT (int): Port number for the robot socket.
        robot_socket (int): Socket object for the robot connection.
    """

    # Constants
    IP_ADDRESS = Util.get_from_json_file("config")['ip'] 
    LEARNING_PORT = 6000
    ROBOT_PORT = 7000
    robot_socket = 0

    def __init__(self):
        """
        Initializes a UtilityFlask object.

        Attributes:
        ----------
            id_player (int): ID of the player.
            experiment_condition (str): Current experimental condition.
            isRobotConnected (bool): Flag indicating if the robot is connected: 
                                     if false the client will not show a pop-up before highlight the position suggested.
            received_hint (dict): Dictionary containing received hints.
            csv_data (dict): Dictionary to store CSV data fields.
        """
        self.id_player = -1
        self.experimental_condition = None
        self.experimental_condition_str = ''
        self.isRobotConnected = False
        self.board = None
        self.move = None
        self.board_lock = multiprocessing.Lock()
        self.board_changed = multiprocessing.Event()
        self.n_game = 1
        self.file_manager = FileManager()
        self.thread = None
        self.stop_thread = threading.Event()

    def handle_id_player(self, id_session, instance, expertiment_condition):
        """
        Handle the player ID received in the request.

        Args:
        - request: The request object containing player ID information.

        Returns:
        - Response indicating success or failure.
        """
        self.id_player = id_session

        if self.id_player is not None:
            self.file_manager.id_player = id_session
            self.experimental_condition = expertiment_condition

            # Reset the stop event
            self.stop_thread.clear()

            # run script
            self.thread = threading.Thread(target=run_q_learning, args=(self.id_player, instance, self.experimental_condition, self.n_game, self.stop_thread))
            self.thread.start()

            # update log file
            Util.update_log_file(f"id_player: {self.id_player}", self.id_player, self.n_game)
            Util.formatted_debug_message("Init for player with ID " + str(self.id_player), level='INFO')
        
    def handle_game_board(self, request):
        """
        Receive and handle the game board sent in the request.

        Args:
        - request: The request object containing the game board matrix.

        Returns:
        - Response indicating success or failure.
        """
        data = request.get_json()
        # get id from json data
        game_board = data.get('matrix')

        # handle game board (received from js and send it to python program)
        if game_board is not None:
            # send game board to Q-learning algorithm
            with self.board_lock:
                self.board = data
                self.board_changed.set()
            # print game board (if not local) and write it on log file
            self.file_manager._write_board_on_file(data['matrix'])
            return jsonify({'message': 'Game board received'}), 200
        else:
            return jsonify({'error': 'Game board not passed not provided in the request'}), 400
        
    def handle_player_move(self, request):
        """
        Receive and handle the player's move (card clicket, position, ...) sent in the request.

        Args:
        - request: The request object containing the game data.

        Returns:
        - Response indicating success or failure.
        """
        # get json data
        data = request.get_json()
        string = json.dumps(data)
        dictionary = json.loads(string)
        # get id from json data
        player_move = data.get('game')

        # received user's info (js)
        if player_move is not None:
            with self.board_lock:
                self.move = data
                self.board_changed.set()

            # write log file and update csv file
            self.file_manager._write_game_data_on_file(dictionary)
                    
            return jsonify({'message': 'User move received'}), 200
        else:
            return jsonify({'error': 'User move not passed not provided in the request'}), 400
        
    def handle_robot_hint(self, request, socketio):
        """
        Handle the hint received from the robot.

        Args:
        - request: The request object containing the hint data.
        - socketio: The SocketIO instance for emitting events.

        Returns:
        - Response indicating success or failure.
        """
        # get json data
        data = request.get_json()
        string = json.dumps(data)
        dictionary = json.loads(string)
        # get id from json data
        agent_hint = data.get('action')

        # received user's info (js)
        if agent_hint is not None:
            # if not none send to js in order to highlight the card/row/col and send it to robot app     
            if agent_hint["suggestion"] != "none":
                agent_hint["isRobotConnected"] = self.isRobotConnected
                socket_address = "robot_hint_" + str(self.id_player)
                #print("address", socket_address)
                socketio.emit(socket_address, json.dumps(data))

            # write hint on log file and csv
            self.file_manager._write_hint_data_on_file(dictionary, self.experimental_condition)
            # save hint object (could be used in order to avoid websocket)
            return jsonify({'message': 'Hint agent received'}), 200
        else:
            return jsonify({'error': 'Hint agent not passed not provided in the request'}), 400
        
    def handle_cheater(self):
        """
        This function will delate the files associated to the user in case they refreshed the page
        without finishing the game.
        Then it restart the Q-learning script.
        """
        Util.formatted_debug_message(f"Deleting csv file of player with ID={self.id_player}...", level='INFO')
        Util.delete_files(self.id_player, self.n_game)
        Util.formatted_debug_message("File deleted...", level='INFO')
        self.file_manager._clear_csv_struct()
        Util.formatted_debug_message("CSV cleaned, restarting...", level='INFO')
        # stop old q-learning script
        self.stop_running_thread()

    def stop_running_thread(self):
        if self.thread and self.thread.is_alive():
            # Set the stop event
            self.stop_thread.set()
            self.thread.join()