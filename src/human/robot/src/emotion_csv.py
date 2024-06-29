import os
import csv

from datetime import datetime

class EmotionCSV:
    def __init__(self, id_player):
        # get current dir
        script_dir = os.path.dirname(__file__) 
        # get path to data directory
        user_dir = "user_" + id_player
        data_dir = os.path.abspath(os.path.join(script_dir, '..', '..', 'app', 'src', 'data', user_dir))
        if not os.path.exists(data_dir):
            raise FileNotFoundError(f"The directory {data_dir} does not exist. Run the app.launch first!")
        # save the csv in the player directory: emotion only when robot speaks
        self.csv_file_filtered = os.path.join(data_dir, 'emotion_filtered.csv')
        # save the csv in the player directory: all emotion during the game
        self.csv_file_full = os.path.join(data_dir, 'emotion_full.csv')
        # initialize csv file
        self.init_csv(self.csv_file_filtered)
        self.init_csv(self.csv_file_full)
        
    def init_csv(self):
        """Initialize the CSV file with headers."""
        headers = ['id', 'timestamp', 'emotion', 'match', 'motivated', 'condition']
        # file containing emotion only when robot speaks
        with open(self.csv_file_filtered, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(headers)
        # file containing all emotion
        with open(self.csv_file_full, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(headers)

    def log_to_csv(self, timestamp, filename, emotion, match, motivated):
        """Log the interaction data to the CSV file."""
        dt_object = datetime.fromtimestamp(timestamp)
        formatted_time = dt_object.strftime('%Y-%m-%d %H:%M:%S.%f') 
        condition = "E-ToM" if self.emotional_condition else "ToM"
        if "filtered" in filename:
            with open(self.csv_file_filtered, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([formatted_time, emotion, match, motivated, condition])
        elif "full" in emotion:
            with open(self.csv_file_full, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([formatted_time, emotion, match, motivated, condition])
        else:
            raise FileNotFoundError(f"The file {filename} does not exist. \
                                    File must be: \n\t - {self.csv_file_filtered} \n\t - {self.csv_file_full}")