import numpy as np

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))

from learning.learning import Qlearning
from game.environment import Environment

from util import constants
from util import Util
from plotting import Plotting

# define robot's action
actions = ['none', 'suggest_row_col', 'suggest_card']

# define states: {robot_last_action X game_state X user_state}
states = [
    'INIT_STATE',

    '(no_help, beg, f_correct)',
    '(no_help, beg, f_wrong, low_attempts)',
    '(no_help, beg, f_wrong, medium_attempts)',
    '(no_help, beg, f_wrong, high_attempts)',
    
    '(no_help, mid, f_correct)',
    '(no_help, mid, f_wrong, low_attempts)',
    '(no_help, mid, f_wrong, medium_attempts)',
    '(no_help, mid, f_wrong, high_attempts)',
    
    '(no_help, end, f_correct)',
    '(no_help, end, f_wrong, low_attempts)',
    '(no_help, end, f_wrong, medium_attempts)',
    '(no_help, end, f_wrong, high_attempts)',
    
    '(sugg_row, beg, f_correct)',
    '(sugg_row, beg, f_wrong, low_attempts)',
    '(sugg_row, beg, f_wrong, medium_attempts)',
    '(sugg_row, beg, f_wrong, high_attempts)',
    
    '(sugg_row, mid, f_correct)',
    '(sugg_row, mid, f_wrong, low_attempts)',
    '(sugg_row, mid, f_wrong, medium_attempts)',
    '(sugg_row, mid, f_wrong, high_attempts)',
    
    '(sugg_row, end, f_correct)',
    '(sugg_row, end, f_wrong, low_attempts)',
    '(sugg_row, end, f_wrong, medium_attempts)',
    '(sugg_row, end, f_wrong, high_attempts)',
    
    '(sugg_card, beg, f_correct)',
    '(sugg_card, beg, f_wrong, low_attempts)',
    '(sugg_card, beg, f_wrong, medium_attempts)',
    '(sugg_card, beg, f_wrong, high_attempts)',
    
    '(sugg_card, mid, f_correct)',
    '(sugg_card, mid, f_wrong, low_attempts)',
    '(sugg_card, mid, f_wrong, medium_attempts)',
    '(sugg_card, mid, f_wrong, high_attempts)',
    
    '(sugg_card, end, f_correct)',
    '(sugg_card, end, f_wrong, low_attempts)',
    '(sugg_card, end, f_wrong, medium_attempts)',
    '(sugg_card, end, f_wrong, high_attempts)',
    
    '(no_help, beg, s_correct)',
    '(no_help, beg, s_wrong, low_attempts)',
    '(no_help, beg, s_wrong, medium_attempts)',
    '(no_help, beg, s_wrong, high_attempts)',
    
    '(no_help, mid, s_correct)',
    '(no_help, mid, s_wrong, low_attempts)',
    '(no_help, mid, s_wrong, medium_attempts)',
    '(no_help, mid, s_wrong, high_attempts)',
    
    '(no_help, end, s_correct)',
    '(no_help, end, s_wrong, low_attempts)',
    '(no_help, end, s_wrong, medium_attempts)',
    '(no_help, end, s_wrong, high_attempts)',
    
    '(sugg_row, beg, s_correct)',
    '(sugg_row, beg, s_wrong, low_attempts)',
    '(sugg_row, beg, s_wrong, medium_attempts)',
    '(sugg_row, beg, s_wrong, high_attempts)',
    
    '(sugg_row, mid, s_correct)',
    '(sugg_row, mid, s_wrong, low_attempts)',
    '(sugg_row, mid, s_wrong, medium_attempts)',
    '(sugg_row, mid, s_wrong, high_attempts)',
    
    '(sugg_row, end, s_correct)',
    '(sugg_row, end, s_wrong, low_attempts)',
    '(sugg_row, end, s_wrong, medium_attempts)',
    '(sugg_row, end, s_wrong, high_attempts)',
    
    '(sugg_card, beg, s_correct)',
    '(sugg_card, beg, s_wrong, low_attempts)',
    '(sugg_card, beg, s_wrong, medium_attempts)',
    '(sugg_card, beg, s_wrong, high_attempts)',
    
    '(sugg_card, mid, s_correct)',
    '(sugg_card, mid, s_wrong, low_attempts)',
    '(sugg_card, mid, s_wrong, medium_attempts)',
    '(sugg_card, mid, s_wrong, high_attempts)',
    
    '(sugg_card, end, s_correct)',
    '(sugg_card, end, s_wrong, low_attempts)',
    '(sugg_card, end, s_wrong, medium_attempts)',
    '(sugg_card, end, s_wrong, high_attempts)',
]

# define the number of games that the agent will plays
EPISODES = constants.NUMBER_EPISODES

# define number of pair that user need to find
PAIRS = 12

# Keeps track of useful statistics
stats = Plotting.EpisodeStats(
    episode_lengths = np.zeros(EPISODES),
    episode_rewards = np.zeros(EPISODES),
    episode_click_until_match = np.zeros(PAIRS),
    avg_of_moves_until_match = np.zeros(PAIRS),
    avg_of_suggests_in_specific_episode = np.zeros(PAIRS),
    avg_of_suggests_after_some_episode = np.zeros(PAIRS),
    avg_of_suggests_first_card_over_time = np.zeros(PAIRS)
)

# create directory for plots if it doesn't exists
Util.create_directory_for_plots()

#create environment 
env = Environment()

# parameters setting
epsilon = 1.0
alpha = 0.1
gamma = 0.8

# start with training
Q = Qlearning(stats, env, len(states), len(actions), states, actions, epsilon, alpha, gamma)
Q.training(EPISODES)