import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))

from util import constants
from util import Util

from learning.qlearning_elements import actions, states, epsilon, alpha, gamma
from learning.learning import Qlearning
from game.environment import Environment

def run_q_learning(id_player, instance, experimental_condition, n_game, stop_event):
    Util.formatted_debug_message("Running Q-learning script...", level='INFO')

    # get ip: the q-learning thread will send HTTP request to flask
    SERVER_IP = Util.get_from_json_file("config")['ip'] 

    # get string and send it to server
    # if experimental_condition is None:
        # Choose one of the following experimental conditions:
        # - TOM (currently selected)
        # - NO_TOM
        # - DECEPTION
        # - SUPERFICIAL_DECEPTION
        # - HIDDEN_DECEPTION
        # - EXTERNAL_DECEPTION
        # experimental_condition = constants.EXTERNAL_DECEPTION
    experimental_condition_str = Util.get_experimental_condition(experimental_condition)
    Util.update_log_file("\nExperimental condition:" + experimental_condition_str + "\n\n", id_player, n_game)
    Util.formatted_debug_message("Experiment condition:" + experimental_condition_str, level='INFO')

    # define the number of games that the agent will plays
    EPISODES = constants.EPISODES_WITH_HUMAN

    # Read from file the Q-table of robot training
    Q_table = Util.get_Q_table_from_file()

    # init environment
    env = Environment(SERVER_IP, id_player, instance)

    # play
    Q = Qlearning(Q_table, env, experimental_condition, len(states), len(actions), states, actions, epsilon, alpha, gamma)
    Q.training(EPISODES)
    if stop_event.is_set():
        Util.formatted_debug_message("Thread for player ID " + str(id_player) + " stopped.", level='INFO')
    else:
        Util.formatted_debug_message("Script ended for " + str(id_player), level='INFO')