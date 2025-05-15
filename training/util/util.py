import json
import os
import numpy as np
import pandas as pd

from socket import *

class Util:
    __counter = 0                       # in order to print the Q-table only n times

    @staticmethod
    def get_avarage(array, index, episode):
        tmp = np.zeros(index)
        for i in range(index):
            tmp[i] = array[i] / episode

        return tmp

    @staticmethod
    def get_cumulative_avg(array, index):
        return np.cumsum(array[0:index]) / (np.arange(index) + 1)

    @staticmethod
    def get_percentage(percent, number):
        return (percent * number) / 100

    @staticmethod
    def save_epsilon_into_file(epsilon):
        with open('../rl_utils/env/greedy.json', 'w') as greedy_file:
            greedy_file.write(json.dumps({'epsilon': epsilon}))

    @staticmethod
    def save_Q_table_into_file(Q):
        with open('../rl_utils/policy/matrix.npy', 'wb') as matrix_file:
            np.save(matrix_file, Q)  
            
    @staticmethod
    def get_from_json_file(filename):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(current_dir, filename + ".json")
        with open(file_path, "r") as configfile:
            data_file = json.load(configfile)
            
        return data_file

    @staticmethod
    def get_Q_table_from_file():
        # get path
        current_file_dir = os.path.dirname(os.path.abspath(__file__))
        config_file_path = os.path.join(current_file_dir, "../../rl_utils/policy/matrix.npy")
        if not os.path.exists(config_file_path):
            raise FileNotFoundError(f"Q-table file not found at: {config_file_path}")
        
        with open(config_file_path, 'rb') as matrix_file:
            Q = np.load(matrix_file)

        return Q

    @staticmethod
    def create_directory_for_plots():
        """
        This function will create a directory for each stats to plot. (training part)
        """

        # create directory if it doesn't exists
        training_path_plot = "../training/plot"
        if os.path.exists(training_path_plot) is False:
            os.mkdir(training_path_plot)

        # create png folder
        training_path_png = "../training/plot/png"
        if os.path.exists(training_path_png):
            return
        os.mkdir(training_path_png)
        os.mkdir(training_path_png + "/Avg_of_moves_until_match")
        os.mkdir(training_path_png + "/Rewards")
        os.mkdir(training_path_png + "/Episode_length")
        os.mkdir(training_path_png + "/Mistakes")
        os.mkdir(training_path_png + "/Percent")
        
        # create pdf folder
        training_path_pdf = "../training/plot/pdf"
        if os.path.exists(training_path_pdf):
            return
        os.mkdir(training_path_pdf)
        os.mkdir(training_path_pdf + "/Avg_of_moves_until_match")
        os.mkdir(training_path_pdf + "/Rewards")
        os.mkdir(training_path_pdf + "/Episode_length")
        os.mkdir(training_path_pdf + "/Mistakes")
        os.mkdir(training_path_pdf + "/Percent")

        # npy folder for player's type
        training_path_npy = "../training/data"
        if os.path.exists(training_path_npy):
            return
        os.mkdir(training_path_npy)
    
    @staticmethod
    def print_Q_table(Q, states, actions, module):
        if Util.__counter % module == 0:
            print("\n", pd.DataFrame(Q, states, actions))
            print("end of episode", Util.__counter)
        Util.__counter += 1