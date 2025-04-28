import os
import csv

from datetime import datetime

class EmotionCSV:
    def __init__(self, id_player, condition):
        self.emotional_condition = condition
        # get current dir
        script_dir = os.path.dirname(__file__) 
        # get path to data directory
        user_dir = "user_" + str(id_player)
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
        # start time will be initialized only after game is started
        self.starting_time = None
        
    def init_csv(self, filename):
        """Initialize the CSV file with headers."""
        headers = ['id', 'timestamp', 'game_time', 'emotion', 'model_confidence', 'head_pose', 
                   'x_pose', 'y_pose', 'z_pose', 'match', 'turn', 'motivated', 'condition']
        # file containing emotion only when robot speaks
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(headers)

    def set_starting_time(self):
        self.starting_time = datetime.now()

    def get_time(self):
        # getting the current date and time
        current_datetime = datetime.now()
        # getting the date and time from the current date and time in the given format
        current_date_time = current_datetime.strftime("%m/%d/%Y, %H:%M:%S")
        return current_date_time
    
    def get_game_time(self):
        current_datetime = datetime.now()
        elapsed_time = current_datetime - self.starting_time
        minutes = elapsed_time.seconds // 60
        seconds = elapsed_time.seconds % 60
        
        return f"{minutes}:{seconds}"

    def log_to_csv(self, timestamp, game_time, id, filename, emotion, model_confidence, head_pose, x_pose, y_pose, z_pose, match, turn, motivated):
        """Log the interaction data to the CSV file."""
        # tempo non calcolato qui perchè a causa dei lock (ed eventuali overhead) c'è uno sfasamento di qualche secondo
        # n.b il tempo sarà comunque sfasato: 
        #   - i pop-up bloccano il tempo di gioco, ma il tempo serve anche per capire in quale momento 
        #     l'emozione è stata registrata (ergo i due momenti sono necessari)
        #   - anche se non ci fossero pop-up (per esmepio all'inizio), lo sfasamento *potrebbe* comunque esserci 
        #     (1-2 secondi, in quanto vengono calcolati in momenti diversi)
        
        #timestamp = self.get_time()
        #game_time2 = self.get_game_time()
        condition = "E-ToM" if self.emotional_condition else "ToM"
        if "filtered" == filename:
            #print(f'Game tim2 received: {game_time2}...')
            with open(self.csv_file_filtered, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([id, timestamp, game_time, emotion, model_confidence, 
                                 head_pose, x_pose, y_pose, z_pose, match, turn, motivated, condition])
        elif "full" == filename:
            with open(self.csv_file_full, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([id, timestamp, game_time, emotion, model_confidence, 
                                head_pose, x_pose, y_pose, z_pose, match, turn, motivated, condition])
        else:
            raise FileNotFoundError(f"The file {filename} does not exist. \
                                    File must be: \n\t - {self.csv_file_filtered} \n\t - {self.csv_file_full}")