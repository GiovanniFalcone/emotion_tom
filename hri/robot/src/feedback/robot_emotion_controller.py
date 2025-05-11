"""
This module handles the robot's emotional responses based on the detected emotions.
It includes methods for changing LED colors, handling facial expressions, and managing emotional states.
It is used to provide feedback to the user during the interaction.
It is designed to work with a robot interface, allowing for easy integration with different robot platforms.
"""

class RobotEmotionController:
    def __init__(self, robot):
        self.robot = robot

    def change_led_color_based_on_emotion(self, emotion):
        """
        Changes the LED color of the robot based on the given emotion.

        Parameters:
        emotion (str): The emotion to determine the LED color. 
                Possible values are:
                   - '' (empty string): Turns off the LED.
                   - 'happy' or 'surprise': Sets the LED to yellow.
                   - Any other value: Sets the LED to blue (calming effect).
        """
        # turn off
        if emotion == '':
            self.robot.set_color_led(red=0, green=0, blue=0)
        # positive valence and neutral
        elif emotion in ['neutral', 'surprise']:
            # yellow 
            self.robot.set_color_led(red=255, green=255, blue=0)
        else:
            # green in order to calm (i.e used for negative valence)
            self.robot.set_color_led(red=0, green=0, blue=255)

    def handle_expression_based_on_emotion(self, emotion):
        """"
        Adjusts the robot's facial expression based on the input emotion.
        The robot will display specific facial expressions corresponding to the given emotion:
        - For 'happy' and 'neutral', the robot will display a happy facial expression.
        - For 'surprise', the robot will display a surprised facial expression.
        - For all other emotions (i.e., 'angry', 'sad', 'fear', 'disgust'), the robot will display a sad facial expression.

        Args:
            emotion (str): The input emotion.
        """
        if emotion in ['happy', 'neutral']:
            self.robot.do_facial_expression("happy_1")
        elif emotion == 'surprise':
            self.robot.do_facial_expression("CustomSurprise")
        else:
            # all negative emotion
            self.robot.do_facial_expression("CustomSad")