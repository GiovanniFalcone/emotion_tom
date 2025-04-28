import time
import random

class EmotionHandler:
    def __init__(self, robot):
        self.robot = robot

    def change_led_color_based_on_emotion(self, emotion):
        if emotion == '':
            self.robot.set_color_led(red=0, green=0, blue=0)
        elif emotion == 'happy' or emotion == 'surprise':
            # yellow 
            self.robot.set_color_led(red=255, green=255, blue=0)
        else:
            # green in order to calm
            self.robot.set_color_led(red=0, green=0, blue=255)

    def handle_expression_based_on_emotion(self, emotion, n_pairs, match, board_changed=False):
        if board_changed:
            self._handle_emotion_when_board_change(emotion)
            return
        
        is_first_pair = n_pairs == 1
        is_begin = n_pairs < 4
        is_middle = 3 < n_pairs < 8
        is_end = n_pairs >= 8
        
        if match:
            self.handle_expression_given_match(emotion, is_first_pair, is_begin, is_middle, is_end)
        else:
            self.handle_expression_given_unmatch(emotion, is_begin, is_end)

    def handle_expression_given_match(self, emotion, is_first_pair, is_begin, is_middle, is_end):
        if emotion == "happy":
            self.handle_happy_expression_given_match(is_first_pair, is_begin, is_middle)
        elif emotion == "neutral":
            self.handle_neutral_expression_given_match(is_first_pair, is_end)
        elif emotion == "surprise":
            self.robot.do_facial_expression("CustomSurprise")

    def handle_happy_expression_given_match(self, is_first_pair, is_begin, is_middle):
        if(is_begin or is_middle or is_first_pair):
            self.robot.do_facial_expression("happy_1")
            time.sleep(0.5)
            self.robot.do_facial_expression("BigSmile")
        else:
            # end
            self.robot.do_facial_expression("happy_1")
            self.robot.do_facial_expression("BigSmile")

    def handle_neutral_expression_given_match(self, is_first_pair, is_end):
        if is_first_pair:
            self.robot.do_facial_expression("happy_1")
            self.robot.do_facial_expression("BigSmile")
        else:
            self.robot.do_facial_expression("happy_1")
            if not is_end: time.sleep(0.5)
            self.robot.do_facial_expression("Wink" if random.choice([True, False]) else "BigSmile")

    def handle_expression_given_unmatch(self, emotion, is_begin, is_end):
        if emotion in ['happy', 'neutral']:
            self.robot.do_facial_expression("happy_1")
            self.robot.do_facial_expression("BigSmile")
        elif emotion == 'sad':
            if(is_begin or is_end):
                self.robot.do_facial_expression("CustomSad")
                self.robot.do_facial_expression("BigSmile")
            else:
                # middle
                self.robot.do_facial_expression("CustomSad")
                self.robot.do_facial_expression("Wink")
        else:
            # angry, other
            self.robot.do_facial_expression("CustomSad")
            time.sleep(0.5)
            self.robot.do_facial_expression("BigSmile")

    def _handle_emotion_when_board_change(self, emotion):
        if emotion in ["happy", "neutral"]: 
            self.robot.do_facial_expression("happy_1")
            time.sleep(0.5)
            self.robot.do_facial_expression("BigSmile")
        elif emotion == "sad":
            self.robot.do_facial_expression("CustomSad")
            self.robot.do_facial_expression("BigSmile")
        elif emotion == "surprise":
            self.robot.do_facial_expression("CustomSurprise")
        else:
            # angry, sad
            self.robot.do_facial_expression("CustomSad")
            time.sleep(0.5)
            self.robot.do_facial_expression("BigSmile")