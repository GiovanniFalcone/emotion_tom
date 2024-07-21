from collections import namedtuple

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from util import Util

class Plotting:
    
    EpisodeStats = namedtuple(
        "Stats", [
            "episode_lengths",                     # How many steps it took to finish an episode
            "episode_rewards",                     # Reward of an episode
            "episode_click_until_match",           # How many clicks it took to find a pair in a specific episode
            "avg_of_moves_until_match",            # Average of number of moves it took to find a pair - after n episodes
            "avg_of_suggests_in_specific_episode", # Average of suggestions provided by the agent in a specific episode for each pair
            "avg_of_suggests_after_some_episode",  # Average of suggestions provided by the agent after n episodes for each pair
            "avg_of_suggests_first_card_over_time" # Average of suggestions provided by the agent for the first card for each pair
        ]
    )

    # constants used to plot data
    AVERAGE_EPISODE_REWARDS = 0
    AVERAGE_EPISODE_LENGTH = 1
    AVERAGE_OF_TURNS_FOR_MATCH = 2
    PERCENTAGE_MOVES = 7
    MISTAKES = 8

    state_counts_first = {}
    state_counts_second = {}
    mistakes = np.zeros(100000)                                 # to count all the moves that doesn't end in a match

    @staticmethod
    def save_stats(stats, flag, episode, smoothing_window=10):
        """
        This function will create plot and it will save them in specific directory.
        It will save them as png and pdf extensions.

        Parameters
        ----------
            stats (array): The array which contains the data to plot
            flag (int): In order to save the data in the correct path and with the correct name
            episode (int): The current number of episode
        """

        if flag == Plotting.PERCENTAGE_MOVES:
            figure = plt.figure(figsize=(5,5))
        else:
            figure = plt.figure(figsize=(12,5))

        match flag:
            case Plotting.AVERAGE_EPISODE_REWARDS:
                plt.plot(stats[0:episode])
                plt.xlabel("Episode")
                plt.ylabel("Episode Reward")
                plt.title("Episode Reward over Time".format(smoothing_window))
                
                plt.savefig('plot/png/Rewards/Rewards_after_' + str(episode) + '.png')
                plt.savefig('plot/pdf/Rewards/Rewards_after_' + str(episode) + '.pdf')

            case Plotting.AVERAGE_EPISODE_LENGTH:
                plt.plot(stats[0:episode])

                plt.xlabel("Episode")
                plt.ylabel("Moves number")
                plt.title("Game Length over Time".format(smoothing_window))
                
                plt.savefig('plot/png/Episode_length/Number_of_moves_after_' + str(episode) + '.png')
                plt.savefig('plot/pdf/Episode_length/Number_of_moves_after_' + str(episode) + '.pdf')

            case Plotting.AVERAGE_OF_TURNS_FOR_MATCH:
                plt.plot(stats, 'o-')
                plt.xlabel("Pairs", fontsize = 14)
                plt.ylabel("Number of clicks", fontsize = 14)
                plt.title("Average click for each card (for episode)".format(smoothing_window))
                
                plt.savefig('plot/png/Avg_of_moves_until_match/AVG_of_moves_episode_after_' + str(episode) + '.png')
                plt.savefig('plot/pdf/Avg_of_moves_until_match/AVG_of_moves_episode_after_' + str(episode) + '.pdf')

            case Plotting.PERCENTAGE_MOVES:
                # Calculate percentages for the first and second flips
                percent_actions_first = {}
                percent_actions_second = {}

                # Calculate percentages for the first flip
                for state, action_counts in Plotting.state_counts_first.items():
                    total_state_actions = sum(action_counts.values())
                    percent_actions_first[state] = {action: (count / total_state_actions) * 100 for action, count in action_counts.items()}
                    #print(Plotting.state_counts_first, "\n", state, "\n", total_state_actions, "\n", percent_actions_first[state], "\n")

                # Calculate percentages for the second flip
                for state, action_counts in Plotting.state_counts_second.items():
                    total_state_actions = sum(action_counts.values())
                    percent_actions_second[state] = {action: (count / total_state_actions) * 100 for action, count in action_counts.items()}

                # variables
                state_strings = ['low', 'medium', 'high']
                actions = ['none', 'row', 'card']

                # Plot creation (subplot for each state)
                fig, axes = plt.subplots(1, len(state_strings), figsize=(15, 5), sharey=True)

                # Subplot
                for i, state in enumerate(state_strings):
                    # get percentage
                    percent_first = {action: percent_actions_first[state].get(action, 0) for action in actions}
                    percent_second = {action: percent_actions_second[state].get(action, 0) for action in actions}

                    # dataframe creation
                    df_first = pd.DataFrame(percent_first, index=['1st flip'])
                    df_second = pd.DataFrame(percent_second, index=['2nd flip'])

                    # combine them
                    df_combined = pd.concat([df_first, df_second])

                    # .T because we want action as column
                    df_combined = df_combined.T

                    # create graph
                    df_combined.plot(kind='bar', ax=axes[i], rot=0)

                    # labels
                    axes[i].set_xlabel("Actions")
                    axes[i].set_ylabel("Percentage of times selected (%)")
                    axes[i].set_title(f"Percentage of hint provided - {state.capitalize()}")

                    # percentage label
                    for container in axes[i].containers:
                        axes[i].bar_label(container, labels=[f'{v:.1f}%' for v in container.datavalues], label_type='edge')
                
                plt.tight_layout()
                # Save the plot
                fig.savefig('plot/png/Percent/Percent_after_' + str(episode) + '.png')
                fig.savefig('plot/pdf/Percent/Percent_after_' + str(episode) + '.pdf')

            case Plotting.MISTAKES:
                cumulative_mistakes = Util.get_cumulative_avg(Plotting.mistakes, episode)

                plt.plot(cumulative_mistakes)
                
                plt.xlabel("Episode", fontsize = 14)
                plt.ylabel("Mistakes number", fontsize = 14)
                plt.title("Number of mistakes over Time".format(smoothing_window))
                plt.savefig('plot/png/Mistakes/Mistakes_after_' + str(episode) + '.png')
                plt.savefig('plot/pdf/Mistakes/Mistakes_after_' + str(episode) + '.pdf')

        plt.close(figure)

    @staticmethod
    def update_stats(stats, episode, reward, action, env, state_string):
        is_turn_even = (env.get_turn() - 1) % 2 == 0     # turn - 1 because the turn has already been increased
        is_game_ended = env.get_pairs_found() == 12      # the turn doesn't increase when game is ended
        moves_until_match = env.get_flip_number()/2

        if 'correct' in state_string:
            state_string = 'correct'
        elif 'low' in state_string:
            state_string = 'low'
        elif 'medium' in state_string:
            state_string = 'medium'
        elif 'high' in state_string:
            state_string = 'high'
        else:
            state_string = 'init'

        # in order to get the average of suggestions for the first card only
        if is_turn_even is False:
            if state_string not in Plotting.state_counts_first:
                Plotting.state_counts_first[state_string] = {'none': 0, 'row': 0, 'card': 0}
            if action == 0:
                Plotting.state_counts_first[state_string]['none'] +=1
            elif action == 1:
                Plotting.state_counts_first[state_string]['row'] +=1
            else:
                Plotting.state_counts_first[state_string]['card'] +=1
        else:
            if state_string not in Plotting.state_counts_second:
                Plotting.state_counts_second[state_string] = {'none': 0, 'row': 0, 'card': 0}
            if action == 0:
                Plotting.state_counts_second[state_string]['none'] +=1
            elif action == 1:
                Plotting.state_counts_second[state_string]['row'] +=1
            else:
                Plotting.state_counts_second[state_string]['card'] +=1

            if env.was_last_move_a_match() is False:
                Plotting.mistakes[episode] += 1

        # if a new pair has been found
        if (is_turn_even and env.was_last_move_a_match()) or is_game_ended:
            pair = env.get_pairs_found() - 1
            # average number of moves until pair is found, considering n episodes
            stats.avg_of_moves_until_match[pair] += moves_until_match

        # cumulative reward 
        stats.episode_rewards[episode] += reward
        # cumulative moves
        stats.episode_lengths[episode] = (env.get_turn() - 1)/2  # 2 turns = 1 move
        