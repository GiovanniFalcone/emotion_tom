import json
import random
import os

class SuggestionGenerator:
    def __init__(self, filename, language='ita'):
        self.language = language
        script_dir = os.path.dirname(__file__)  
        # mode sentences
        filename = os.path.join(script_dir, 'conditions', language, filename + ".json")
        self.data = self.load_json_file(filename)
        # card and hint location sentences
        filename = os.path.join(script_dir, 'conditions', language, "common.json")
        self.common = self.load_json_file(filename)

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
    
    def get_random_suggestion(self, which_flip, suggestion_type):
        try:
            if which_flip in ("firstCard", "secondCard"):
                if suggestion_type is not None:
                    suggestions = self.data[which_flip][suggestion_type]
                else:
                    suggestions = self.data[which_flip]
            elif which_flip == None:
                suggestions = self.data["hint"]
            else:
                print(f"Invalid card type: {which_flip}")
                return None

            if isinstance(suggestions, list):
                return random.choice(suggestions)
            elif isinstance(suggestions, str):
                return suggestions
            else:
                return None
        except KeyError as e:
            print(f"Error: {e}")
            return None
        
    def replace_placeholders(self, sentence, *values):
        # Search all '%s'
        placeholders = sentence.count('%s')
        
        # Replace them with *values
        if placeholders == 0:
            return sentence
        elif placeholders == 1:
            if len(values) >= 1:
                return sentence % (values[1])
            else:
                print("Error: you need to pass at least one value")
                return None
        elif placeholders == 2:
            if len(values) >= 2:
                return sentence % (values[0], values[1])
            else:
                print("Error: you need to pass at least two values")
                return None
        else:
            print("Error: there is a sentence with 3 or more placeholders")
            return None
        
    def __generate_common_suggestion(self, hint_type, suggested_card, position):
        if hint_type not in {"card", "row", "column"}:
            raise ValueError("Hint type must be 'card', 'row' o 'column'.")
        
        # If it's english, we don't need to translate because the names of the cards are already in english
        if self.language != 'en':
            suggested_card = self.common["card"][suggested_card]
        
        suggestions = self.common["hint_location"][hint_type]
        if isinstance(suggestions, list):
            if hint_type == "card":
                return suggested_card, random.choice(suggestions) % (position[0], position[1])
            elif hint_type == "row":
                return suggested_card, random.choice(suggestions) % (position[0])
            else:
                return suggested_card, random.choice(suggestions) % (position[1])
        else:
            return None
    
    def __generate_suggestion(self, hint_type, flip_type, tom_flag, card_suggested, position):
        """
        Returns
            - card_name the translated card's name
            - the sentence to provide the location (e.g 'in row X and column Y')
            - the correct key based on robot condition
        """
        card_name, suggestion_text = self.__generate_common_suggestion(hint_type, card_suggested, position)
        
        # case when robot say same sentences for both first and second flip
        suggestion_key = None if tom_flag == None else tom_flag

        if flip_type == "firstCard" and suggestion_key in ["oneLocationClickedZeroOtherMultipleTimes", "bothClickedMultipleTimes", 
                                                           "currentLocationClickedMultipleTimes", "oneCardClickedMultipleTimes"]:
            suggestion_key = "otherCases"

        return card_name, suggestion_text, suggestion_key

    def get_sentence(self, hint_type, flip_type, tom_flag, card_suggested, position):
        """
        Generates a sentence based on the provided parameters.

        Parameters:
        - hint_type (str): The type of hint to be generated.
        - flip_type (str): The type of card flip.
        - tom_flag (str): A flag indicating which tom sentence get.
        - card_suggested (str): The suggested card name.
        - position (Tuple[int, int]): The position of the suggested card.

        Returns:
        - str or None: The generated sentence or None if no suggestion is available.
        """
        if hint_type not in ['card', 'row', 'column']:
            raise ValueError("Invalid hint_type. Must be one of 'card', 'row', or 'column'.")
        if flip_type not in ['firstCard', 'secondCard', None]:
            raise ValueError("Invalid flip_type. Must be either 'firstCard' or 'secondCard'.")
        if tom_flag is not None and not isinstance(tom_flag, str):
            raise ValueError("Invalid tom_flag. Must be a string or None.")
        if not card_suggested:
            raise ValueError("Invalid card_suggested. Must be a non-empty string.")
        if not isinstance(position, tuple) or len(position) != 2:
            raise ValueError("Invalid position. Must be a tuple of two integers.")

        card_name, suggestion_text, suggestion_key = self.__generate_suggestion(hint_type, flip_type, tom_flag, card_suggested, position)
        random_suggestion = self.get_random_suggestion(flip_type, suggestion_key)
        if random_suggestion:
            return self.replace_placeholders(random_suggestion, card_name, suggestion_text)
        return None

# esempio d'uso
if __name__ == '__main__':
    filename = 'tom' 
    generator = SuggestionGenerator(filename)
    suggest = "card"
    which_flip = "firstCard"
    flag_ToM = 5
    card = "cavallo"
    position = (1, 3)
    sentence = generator.get_sentence(suggest, which_flip, flag_ToM, card, position)
    print(sentence,"\n")

    filename = 'no_tom' 
    generator = SuggestionGenerator(filename)
    suggest = "card"
    which_flip = None
    flag_ToM = -1
    card = "cavallo"
    position = (1, 3)
    sentence = generator.get_sentence(suggest, which_flip, flag_ToM, card, position)
    print(sentence,"\n")

    filename = 'superficial_deception' 
    generator = SuggestionGenerator(filename)
    suggest = "card"
    which_flip = "firstCard"
    flag_ToM = -1
    card = "cavallo"
    position = (1, 3)
    sentence = generator.get_sentence(suggest, which_flip, flag_ToM, card, position)
    print(sentence,"\n")