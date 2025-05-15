import json
import random
import os
import sys

# for txt prompt
from string import Template

# to access to config file
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
from util.util import Util


class EmotionGenerator:
    GPT = Util.get_from_json_file("config")['gpt']

    def __init__(self, filename, language='ita'):
        self.language = language
        self.script_dir = os.path.dirname(__file__)  
        # mode sentences
        if EmotionGenerator.GPT:
            self._init_gpt()
        else:
            filename = os.path.join(self.script_dir, 'emotions', language, filename + ".json")
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

    def get_sentence(self, emotion, n_pairs, match, player_name, board_changed=False):
        """
        Generate a sentence based on the current game state, emotion, and player information.
        This function selects a sentence from a predefined set of sentences stored in a JSON file.
        The selection is determined by the game event (shuffle, match, or unmatch), the player's 
        emotional state, and the number of pairs remaining in the game. If the sentence contains 
        placeholders for the player's name, they are replaced with the provided `player_name`.
        Args:
            emotion (str): The emotional state of the player (e.g., "happy", "sad").
            n_pairs (int): The number of pairs remaining in the game.
            match (bool): Whether the last action resulted in a match.
            player_name (str): The name of the player to personalize the sentence.
            board_changed (bool, optional): Whether the game board has changed (e.g., shuffled). 
                                            Defaults to False.
        Returns:
            str: A randomly selected sentence, optionally personalized with the player's name.
        """
        # This line determines which key to use to access a specific section of a JSON file.
        event_key = 'shuffle' if board_changed else ('match' if match else 'unmatch')
        if EmotionGenerator.GPT:
            return self._get_sentences_using_gpt(event_key, emotion, n_pairs, match, player_name)
        # get all sentences based on event (shuffle, match, unmatch), emotion and game state (beg, mid, end)
        sentences = self._get_sentences_by_event(event_key, emotion, n_pairs)
        # get a random sentence
        sentence = random.choice(sentences)
        # check for placeholders to replace with player name
        placeholders = sentence.count('%s')
        
        # if it's 0 there is no name to utter
        if placeholders == 0:
            return sentence
        else:
            return sentence % (player_name)
        
    def _get_sentences_by_event(self, event_key, emotion, n_pairs):
        # there are different sentences when user finds the first (or the last) pair and the emotion are happy/neutral 
        if emotion in ['happy', 'neutral'] and n_pairs in [1, 12] and event_key not in ['unmatch', 'shuffle']:
            if n_pairs == 1:
                key = emotion + '_first_pair'
            elif n_pairs == 12:
                key = emotion + '_last_pair'
        else:
            # for all emotion 
            if n_pairs < 4:
                key = emotion + '_beg'
            elif 3 < n_pairs < 8:
                key = emotion + '_mid'
            else:
                key = emotion + '_end'
        
        return self.sentences[event_key][emotion][key]

    ###############################################################################################################
    #                                                     GPT                                                     #
    ###############################################################################################################

    def _init_gpt(self):
        """
        Initializes the GPT-related configurations and templates for generating sentences
        based on the emotion and game context. This method sets up the OpenAI client,
        reads configuration files, and initializes various sentence templates for use
        during gameplay.

        The method performs the following tasks:
        - Imports necessary modules (`configparser` and `OpenAI`).
        - Initializes the OpenAI client using the API key from the environment variable.
        - Reads a configuration file specific to the language and emotion prompts.
        - Sets up templates and variables for:
            - Player name and gender-related sentences.
            - Game state sentences (initial, middle, final).
            - Move outcome sentences (match, unmatch, shuffle).
            - Pairs-related sentences.
            - Board shuffle sentences.

        Attributes initialized:
        - `self.name_gender_sentence`: Template for sentences including the player's name and gender.
        - `self.gender_sentence`: Template for sentences excluding the player's name but including gender.
        - `self.must_say_name`: Configuration for whether to include the user's name in the prompt.
        - `self.dont_say_name`: Configuration for excluding the user's name in the prompt.
        - `self.game_state_init`: Sentence for the initial game state.
        - `self.game_state_middle`: Sentence for the middle game state.
        - `self.game_state_final`: Sentence for the final game state.
        - `self.match_outcome`: Template for sentences describing a successful match.
        - `self.unmatch_outcome`: Template for sentences describing an unsuccessful match.
        - `self.shuffle_outcome`: Template for sentences describing a shuffle action.
        - `self.pairs_sentence`: Template for sentences related to pairs.
        - `self.shuffle_sentence`: Template for sentences describing board changes.

        Raises:
        - KeyError: If required keys are missing in the configuration file.
        - FileNotFoundError: If the configuration file is not found.
        - Exception: For any other issues during initialization.
        """
        import configparser        # prompt 
        from openai import OpenAI  # Import only if GPT is true

        self.client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])
        filename = os.path.join(self.script_dir, 'emotions/gpt', "prompt_" + self.language + ".txt")
        config = configparser.ConfigParser()
        config.read(filename, encoding='utf-8')

        ### variable initialization (in order to open the file only once) ###
        # name
        self.name_gender_sentence = Template(config["name"]["true"])             # player name is ... (figure out gender by name)
        self.gender_sentence = Template(config["name"]["false"])                 # player name is ... figure out gender by name, but don't utter the name
        self.must_say_name = config["if_sentence_contains_the_name"]["true"]     # (in the prompt) say user's name and their emotion
        self.dont_say_name = config["if_sentence_contains_the_name"]["false"]    # (in the prompt) say user's emotion

        # game state
        self.game_state_init = config["game_state"]["initial"]
        self.game_state_middle = config["game_state"]["middle"]
        self.game_state_final = config["game_state"]["final"]

        # move outcome
        self.match_outcome = Template(config["game_context"]["match"])
        self.unmatch_outcome = Template(config["game_context"]["unmatch"])
        self.shuffle_outcome = Template(config["game_context"]["shuffle"])

        # pairs sentence
        self.pairs_sentence = Template(config["pairs"]["sentence"])

        # shuffle sentence 
        self.shuffle_sentence = Template(config["board_changed"]["sentence"])


    def _get_sentences_using_gpt(self, event_key, emotion, n_pairs, match, player_name):
        """
        Generates sentences using the GPT-4.1-nano model based on the provided parameters.
        This function is invoked only if the GPT setting in the configuration file (`config.json`) is set to `true`.
        It constructs a prompt based on the event type and emotion, sends it to the GPT-4.1-nano model, and retrieves
        a generated sentence in the language specified in the configuration file.
        Args:
            event_key (str): The key representing the type of event (e.g., 'shuffle').
            emotion (str): The emotion to be used for generating sentences.
            n_pairs (int): The number of sentence pairs to generate.
            match (str): Additional context or matching criteria for the sentence generation.
            player_name (str): The name of the player to personalize the generated sentences.
        Returns:
            str: The generated sentence in the language specified in the configuration file.
        """
        prompt = self._get_prompt(event_key, emotion, n_pairs, match, player_name)

        response = self.client.responses.create(
            model="gpt-4.1-nano",
            input=[
                {
                    "role": "system",
                    "content": [
                        {
                        "type": "input_text",
                        "text": prompt
                        }
                    ]
                }
            ],
            text={
                "format": {
                "type": "text"
                }
            },
            temperature=0.8,
            max_output_tokens=40
        )
        
        return response.output_text

    def _get_prompt(self, event_key, emotion, n_pairs, match, player_name):
        # get game state sentence
        game_state = (
            self.game_state_init if n_pairs < 4 else
            self.game_state_middle  if n_pairs < 8 else
            self.game_state_final
        )

        # get context based on outcome
        which_game_context_sentence = self.shuffle_outcome if event_key == 'shuffle' else (self.match_outcome if match else self.unmatch_outcome)
        game_context_sentence = which_game_context_sentence.substitute(
            found_pairs=n_pairs,
            remaining_pairs=12 - n_pairs,
            game_state=game_state
        )

        # 50% of times robot will utter the user's name
        name_prob = random.choice([True, False])
        # if true, get "player name is '...'", 
        # else name is given to gpt to figure out the gender of the user, but it is made explicit that he cannot pronounce his name
        name_sentence = f"{self.name_gender_sentence.substitute(player_name=player_name)}\n" if name_prob else \
            f"{self.gender_sentence.substitute(player_name=player_name)}\n"
        # if true, we make explicit in the prompt that the robot will have to say the name of the user
        what_robot_must_say = self.must_say_name if name_prob else self.dont_say_name

        # get sentence based on what happend (user has (has not) found a pair, or the board is changed)
        sentence = self.shuffle_sentence if event_key == 'shuffle' else self.pairs_sentence
        # get complete prompt
        prompt = sentence.substitute(
                say_name=name_sentence,                        # player name is ...
                emotion=emotion,                               # player emotion is ...
                game_context = game_context_sentence,          # include name in the sentence (sometimes)
                what_robot_must_say=what_robot_must_say        # user has (has not) found a pairs ...
            )

        return prompt