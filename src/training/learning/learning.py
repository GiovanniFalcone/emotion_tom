import numpy as np
import pandas as pd

import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))

from util import constants
from util import Util
from plotting import Plotting

from learning.agent import Agent

class Qlearning:
    def __init__(self, stats, env, state_space, action_space, states, actions, epsilon, alpha, gamma, player_type):
        """
        Initializes the Q table and Q-Learning parameters.
        
        Parameters:
            stats (object): Object for plotting
            env (object): The game environment.
            state_space (int): The number of states in the environment.
            action_space (int): The number of actions available in the environment.
            states (array of string): The name of states
            actions (array of string): The name of actions
            epsilon (float): The probability of choosing a random action during exploration.
            alpha (float): The learning rate.
            gamma (float): The discount factor.
            player_type (string): the player that agent will simulate
        """
        self.action_space = action_space
        self.state_space = state_space
        self.Q = np.zeros((state_space, action_space))
        self.epsilon = epsilon
        self.gamma = gamma
        self.alpha = alpha
        self.states = states
        self.actions = actions
        self.stats = stats
        self.env = env
        self.agent = Agent(env)
        env.set_agent(self.agent)
        self.player_type = player_type
    
    def training(self, max_episode):
        """
        Trains the Q-Learning agent.

        Parameters:
            max_episode (int): how many games the agent will play
        """

        for episode in range(max_episode):
            # restart the game
            self.env.start()

            # init S
            state = constants.INIT_STATE

            # for each step of episode
            while self.env.is_game_finished() is False:
                # Choose A from S using policy derived from Q
                action = self.select_action(state)

                # Take action A, observe R, S'
                next_state, reward = self.env.step(state, action, self.actions[action], self.states[state])

                # update statistics
                Plotting.update_stats(self.stats, episode, reward, action, self.env, self.states[state])

                # update Q
                self.update_Q_table(state, action, reward, next_state)

                # S <- S'
                state = next_state

            # epsilon decay
            self.epsilon = self.update_epsilon(episode)
            # learning rate increase
            self.alpha = self.update_alpha(self.alpha, episode)

            # save plot
            self.print_stats(episode)

            # for debug; print q-table with pandas every 500 times
            Util.print_Q_table(self.Q, self.states, self.actions, 1000)

        # print final Q
        print("\n", pd.DataFrame(self.Q, self.states, self.actions))
        # save matrix into a file in order to use that with human player
        Util.save_Q_table_into_file(self.Q)
        # save epsilon for the same reason
        Util.save_epsilon_into_file(self.epsilon)
        
    def select_action(self, state):
        """
        Selects the best action for the given state based on the Q table.
        
        Parameters:
            state (int): The current state.
        
        Returns:
            int: The selected action.
        """
        #return constants.SUGGEST_NONE
    
        is_turn_less_than_seven = self.env.get_turn() < 7
        is_turn_odd = self.env.get_turn() % 2 != 0
        clicks_until_match = self.env.get_flip_number()
        pairs = self.env.get_pairs_found()

        # the number of attempts is not yet updated because it is needed for the reward
        # therefore, if the user has found a pair we set the copy of the number of attempts to zero 
        if self.env.was_last_move_a_match() and is_turn_odd:
            clicks_until_match = 0

        if is_turn_less_than_seven:
            return constants.SUGGEST_NONE
        elif is_turn_odd and clicks_until_match < 4 and pairs > 0:
            return constants.SUGGEST_NONE
        elif is_turn_odd and clicks_until_match < 10 and pairs == 0:
            return constants.SUGGEST_NONE
        else:
            return self.epsilon_greedy(state)

    def epsilon_greedy(self, state):
        """
        Selects an action using the epsilon-greedy algorithm.
        
        Parameters:
            state (int): The current state.
        
        Returns:
            int: The selected action.
        """
        if np.random.random() < self.epsilon:
            return np.random.randint(self.action_space)
        else:
            return np.argmax(self.Q[state, :])

    def update_Q_table(self, state, action, reward, next_state):
        """
        Updates the Q table using the Bellman equation.
        
        Parameters:
            state (int): The current state.
            action (int): The selected action.
            reward (float): The received reward.
            next_state (int): The next state.
        """
        self.Q[state, action] = self.Q[state, action] + self.alpha * (
            reward + self.gamma * np.max(self.Q[next_state, :]) - self.Q[state, action]
            )

    def update_epsilon(self, episode):
        """
        Updates the value of epsilon during training.
        
        Parameters:
            episode (int): The current episode number.
        """
        START_EPSILON = 1.0   # Probability of random action
        EPSILON_TAPER = 0.01  # How much epsilon is adjusted each step
    
        return START_EPSILON / (1.0 + episode * EPSILON_TAPER)

    def update_alpha(self, current_alpha, episode):
        """
        Updates the value of alpha during training.
        
        Parameters:
            current_alpha (int): The current alpha value.
            episode (int): The current episode number.
        """
        START_LR = 0.1
        TAPER_LR = 0.001

        if current_alpha < 0.96:
            return START_LR * (1.0 + episode * TAPER_LR)
    
        return current_alpha

    def print_stats(self, episode):
        if (episode != 0 and episode % 1000 == 0) or episode == (constants.NUMBER_EPISODES - 1):
            # total moves for each episode
            avg_moves = Util.get_cumulative_avg(self.stats.episode_lengths, episode)
            Plotting.save_stats(avg_moves, Plotting.AVERAGE_EPISODE_LENGTH, episode)

            # average of moves until pair is founded considering n episodes
            avg_moves_after_episode = Util.get_avarage(self.stats.avg_of_moves_until_match, 12, episode)
            Plotting.save_stats(avg_moves_after_episode, Plotting.AVERAGE_OF_TURNS_FOR_MATCH, episode)

            # total reward for each episode
            avg_rewards = Util.get_cumulative_avg(self.stats.episode_rewards, episode)
            Plotting.save_stats(avg_rewards, Plotting.AVERAGE_EPISODE_REWARDS, episode)

            file_path = "data/" + self.player_type + ".npy"
            with open(file_path, 'wb') as f:
                np.save(f, self.stats.episode_lengths)

            # number of mistakes
            Plotting.save_stats(None, Plotting.MISTAKES, episode)

            # percentage for each moves over time: should be commented if condition=game_perfect
            Plotting.save_stats(None, Plotting.PERCENTAGE_MOVES, episode)


