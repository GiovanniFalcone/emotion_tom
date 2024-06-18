from socket import  *

import requests
import json
import random

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'util'))
from util import Util

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from sentences.emotion_sentences import EmotionGenerator

SERVER_IP = Util.get_from_json_file("config")['ip'] 
SERVER_PORT = 7000

emotion_sentence = EmotionGenerator('friendly', 'ita')

client_socket = socket(AF_INET, SOCK_STREAM) 
connected = False
while not connected:
    try:
        # connect socket to remote server at (serverName, serverPort)
        client_socket.connect((SERVER_IP, SERVER_PORT))
        connected = True
    except Exception as e:
        print("catch exception: ", e)

    Util.formatted_debug_message("Connected to: " + str(client_socket.getsockname()), level='INFO')

while True:
    data = client_socket.recv(1024).decode()
    #print(data)

    try:
        data_dict = json.loads(data)
    except json.JSONDecodeError:
        print("Received data is not valid JSON")
        continue
    
    if "action" in data:
        sentence = data_dict['action']['sentence']
        if sentence: print(sentence)
    
    if "game" in data:
        turn = data_dict['game']['turn']
        if turn % 2 == 0:
            result = requests.post("http://" + SERVER_IP + ":5000/get_emotion", json=None)
            response_data = result.json()
            emotion = response_data.get('emotion', 'unknown')
            match = data_dict['game']['match']

            if emotion == '': 
                continue

            if emotion == 'fear': emotion = 'neutral'
            if match:
                if emotion in ['sad', 'angry', 'fear']: emotion = 'neutral'
            print("Emotion: ", emotion)

            if random.choice([0, 1]) == 1:
                n_pairs = data_dict['game']['pairs']
                sentence = emotion_sentence.get_sentence(emotion, n_pairs, match)
                print(sentence)
            else:
                print("********")

    
