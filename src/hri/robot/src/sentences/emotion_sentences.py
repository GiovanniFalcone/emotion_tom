import json
import random
import os

class EmotionGenerator:
    def __init__(self, filename, language='ita'):
        self.language = language
        script_dir = os.path.dirname(__file__)  
        # mode sentences
        filename = os.path.join(script_dir, 'emotions', language, filename + ".json")
        self.sentences = self.load_json_file(filename)

    def load_json_file(self, filename):
        try:
            with open(filename, 'r', encoding='utf-8') as file:
                data = json.load(file)
            return data
        except FileNotFoundError:
            print(f"File {filename} not found.")
            return None
        except json.JSONDecodeError:
            print(f"Error decoding JSON file {filename}.")
            return None
        
    def get_sentence(self, emotion, n_pairs, match, player_name):
        
        if match:
            if emotion in ['happy', 'neutral']:
                if n_pairs == 1:
                    key = emotion + '_first_pair'
                elif n_pairs == 12:
                    key = emotion + '_last_pair'
                elif n_pairs >= 8:
                    key = emotion + '_end'
                else:
                    key = emotion + '_beg_mid'
                sentences = self.sentences['match'][emotion][key]
            else:
                # surprise
                sentences = self.sentences['match'][emotion]
        else:
            if emotion not in ['angry', 'sad', 'neutral', 'happy']:
                emotion = 'other'
            
            if emotion == 'sad':
                if n_pairs < 4:
                    emotion = emotion + '_beg'
                elif 3 < n_pairs < 8:
                    emotion = emotion + '_mid'
                else:
                    emotion = emotion + '_end'

            if emotion == 'neutral':
                if n_pairs <= 4:
                    emotion = emotion + '_begin'
                else:
                    emotion = emotion + '_mid_end'

            sentences = self.sentences['unmatch'][emotion]

        # get a random sentence
        sentence = random.choice(sentences)
        # check for placeholders to replace with player name
        placeholders = sentence.count('%s')
        
        if placeholders == 0:
            return sentence
        else:
            return sentence % (player_name)