import sys
import os
import threading

from flask import jsonify

from util import Util

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'util'))

class Menu:
    MULTITHREADING = Util.get_from_json_file("config")['multithreading'] 

    @staticmethod
    def clean_shell():
        """
        Clears the terminal shell.
        """
        os.system('clear' if os.name == 'posix' else 'cls')

    @staticmethod
    def debug_print(message):
        """
        Prints a debug message if multithreading is enabled.
        
        Args:
            message (str): Debug message to print.
        """
        if Menu.MULTITHREADING: print(message)

    @staticmethod
    def _handle_admin_menu():
        """
        Handle the admin menu options.

        This function displays a menu to the administrator, allowing them to restart the Q-learning script
        or close the server. The administrator can input 'ok' to restart the Q-learning script or 'exit' to 
        close the server. If 'ok' is entered it also displays a menu to change experimental condition.
        """
        Util.formatted_debug_message("Write 'ok' to restart Q-learning script or press anything to close the server...", level='INFO')
        
        while True:
            user_input = input("Choice:").lower()
            
            return user_input == "ok"

    @staticmethod
    def _handle_admin_menu_experimental_condition():
        """
        Handle the experimental conditions in the admin menu.

        This function prompts the administrator to change the experimental condition. It displays the available
        experimental conditions and allows the administrator to choose one. If the administrator chooses to change 
        the experimental condition, the program is restarted with the new condition. Otherwise, the program restarts 
        with the same experimental condition.

        Returns
        -------
            The experimental condition chosen (int), None otherwise
        """
        Util.formatted_debug_message("Do want to choose the experimental condition?", level='INFO')
        new_experimental_condition = input("Y/n? ").lower()
        
        if new_experimental_condition in ['yes', 'y']:
            Util.formatted_debug_message("Experimental conditions:\n" +
                                        "0: Theory of Mind\n" +
                                        "1: No-Theory of Mind\n" +
                                        "2: Deception\n" +
                                        "3: External\n" +
                                        "4: Superficial\n" +
                                        "5: Hidden\n" +
                                        "Other: you will restart with the same experimental condition setted at the beginning",
                                        level='INFO')
            experimental_condition = input("Choose mode: ")
            
            if experimental_condition == '' or (not experimental_condition.isdigit() or int(experimental_condition) not in range(6)):
                experimental_condition = None
                Menu.clean_shell()
            return int(experimental_condition)
        else:
            return None