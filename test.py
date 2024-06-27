import json 
import random
import threading

from furhat_remote_api import FurhatRemoteAPI
from automatic_movements import AutomaticMovements

# Create an instance of the FurhatRemoteAPI class, providing the address of the robot or the SDK running the virtual robot
furhat = FurhatRemoteAPI("localhost")

# Get the voices on the robot
voices = furhat.get_voices()

# Set the voice of the robot
furhat.set_voice(name='Giorgio')

# Chiamata alla funzione per i movimenti casuali della testa
threading.Thread(target=AutomaticMovements.random_head_movements, args=(furhat, )).start()

with open('interaction.json', encoding='utf-8') as f:
    data = json.load(f)

# Get the users detected by the robot 
users = furhat.get_users()

while len(users) == 0:
    users = furhat.get_users()

before_game_phrases = data['beforeGameStart']
end_game_phrases = data['endGame']
furhat_intro_phrases = data['intro']
memory_game_intro_phrases = data['memoryGameIntro']

# Attend the user closest to the robot
furhat.attend(user="CLOSEST")
furhat.say(text=random.choice(furhat_intro_phrases), blocking=True)
furhat.say(text=memory_game_intro_phrases)
furhat.say(text=before_game_phrases, blocking=True)

# Perform a named gesture
furhat.gesture(name="BrowRaise")

# Attend a user with a specific id
furhat.attend(userid="virtual-user-2")

# Set the LED lights
furhat.set_led(red=200, green=50, blue=50)

import os
os._exit(0)