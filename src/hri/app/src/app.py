#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32

import json
import logging
import signal
import threading

from threading import Lock
from flask import Flask, render_template, request, session, jsonify, redirect, url_for
from flask_socketio import SocketIO

from flask_utility.utility_flask import UtilityFlask
from flask_utility.menu import Menu

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'util'))

from util import Util

# remove debug messages
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# constants
IP_ADDRESS = Util.get_from_json_file("config")['ip'] 

# Creazione dell'app Flask
app = Flask(__name__, template_folder="./animal_game/template", static_folder="./animal_game/static")
app.config['SECRET_KEY'] = 'secret!'
app.config["SESSION_PERMANENT"] = False 
socketio = SocketIO(app)

# global
client_instances = {}
lock = Lock()
expertiment_condition = None    # new experimental condition received from menu
first_start = True              # in order to not have a menu at the very beginning
exit_pressed = False            # when return to home page after pressing exit in the menu, don't show again the menu
cleanup_flag = False            # clear session after CTRL+C
id_player = -1                  # only used for HRI

# init ROS
rospy.init_node('flask_to_ros_node')
game_publisher = rospy.Publisher('game_data', String, queue_size=10)
speech_publisher = rospy.Publisher('speech_hint', String, queue_size=10)
start_publisher = rospy.Publisher('start', Int32, queue_size=10)

try:
    expertiment_condition = int(rospy.get_param("condition"))
except KeyError:
    rospy.logerr(
        "Usage: roslaunch app app.launch condition:=<condition>")
    sys.exit(1)

def get_id():
    return session.get('id')

def get_utility_flask(id, log_context):
    utility_flask = client_instances.get(id)
    if utility_flask is None:
        print(f"[{log_context}] No client yet")
        return None
    return utility_flask

def create_new_session(data):
    # get id for user
    lock.acquire()
    id = Util.create_dir_for_current_user() 
    lock.release()
    # save it into session
    session['id'] = id
    session['language'] = data.get('language')
    return id

def clear_session():
    id = session.get('id')
    session.clear()
    client_instances.pop(id, None)

def send_to_ros(request, flag):
    # send on correct topic
    if flag == 'start':
        ros_message = Int32()
        ros_message.data = request
        start_publisher.publish(ros_message)
    elif flag == 'speech':
        # unpack and get only the sentence
        data = request.get_json()
        sentence = data['action']['sentence']
        if sentence == None:
            return
        # create new ros massage
        ros_message = String()
        ros_message.data = sentence
        print(f"sending through ros {ros_message.data}")
        speech_publisher.publish(ros_message)
    else:
        data_dict = request.json
        json_data = json.dumps(data_dict)
        # create ros message
        ros_message = String()
        ros_message.data = json_data
        game_publisher.publish(ros_message)

@app.route('/get_id', methods=["GET", "POST"])
def provide_id():
    return jsonify({"id": id_player})

@app.route('/', methods=["GET", "POST"])
@app.route('/index', methods=["GET", "POST"])
def index():
    global expertiment_condition, first_start, exit_pressed

    if 'menu_handled' not in session and not exit_pressed:
        if not first_start: 
            res = Menu._handle_admin_menu()
            session['menu_handled'] = True 
            if not res:
                Util.formatted_debug_message("Exit...", level='INFO')
                session.clear()
                exit_pressed = True
                # run a thread in order to show home page before closing the server
                threading.Timer(2.0, os._exit, args=[0]).start() 
            # if exit was pressed it's useless show the other menu
            if len(session) != 0:
                expertiment_condition = Menu._handle_admin_menu_experimental_condition()
        else: 
            first_start = False
        
        if not exit_pressed:
            if 'id' in session:
                Util.formatted_debug_message("Welcom back!", level='INFO')
                return render_template('home_page.html')
            else:
                Util.formatted_debug_message("New user", level='INFO')

    return render_template('home_page.html')

@app.route('/set_settings', methods=["POST"])
def set_setting():
    global id_player
    # unpack request
    data = request.get_json()

    # create a session for new user, otherwise delete it and create a new one
    if 'id' not in session:
        Util.formatted_debug_message("Creating new session...", level='Settings')
        id = create_new_session(data)
    else:
        clear_session()
        Util.formatted_debug_message("Session cleared...", level='Settings')
        id = create_new_session(data)
    
    id_player = id
    Menu.clean_shell()
    # return response
    response = jsonify({"message": "ok"})
    response.status_code = 200
    return redirect(url_for("show_game", _external=True), Response=response)

@app.route("/game", methods=["POST", "GET"])
def show_game():
    if request.method == "GET":
        id = get_id()
        Util.formatted_debug_message(f"Showing game page to user with ID={id}", level='INFO')
        if id not in client_instances:
            # write on topic player's id in order to understand that the game is started (using to start writing emotions)
            send_to_ros(id, "start")
            # create instance for user
            client_instances[id] = UtilityFlask() 
            # handle player and run Q-learning
            client_instances[id].handle_id_player(id, client_instances.get(id), expertiment_condition)
            return render_template("index.html", session_id=session.get('id'), session_language=session.get('language'))

    return render_template("index.html", session_id=session.get('id'), session_language=session.get('language'))
      
@app.route('/exit', methods=['GET'])
def exit():
    Util.formatted_debug_message(f"User with ID={session.get('id')} has pressed 'exit'!", level='INFO')
    clear_session()
    # only to have a cleaner shell, you can delete it
    Menu.clean_shell()
    return redirect(url_for("index"))

@app.route('/game_board/<int:id>', methods=["POST"])
def receive_game_board(id):
    Util.formatted_debug_message(f"Received game board for user with ID={id}", level='INFO')
    # get instance for current user
    utility_flask = get_utility_flask(id, "Game Board")
    if utility_flask is None:
        return render_template('index.html')
    
    # handle game board and return response
    return utility_flask.handle_game_board(request)

@app.route('/player_move/<int:id>', methods=["POST"])
def receive_player_move_data(id):
    # get instance for current user
    utility_flask = get_utility_flask(id, "Player move")
    if utility_flask is None:
        return render_template('index.html')
    
    send_to_ros(request, "game")
    
    return utility_flask.handle_player_move(request)

@app.route('/hint_data/<int:id>', methods=["POST"])
def receive_hint_data(id):
    
    utility_flask = get_utility_flask(id, "Agent")
    if utility_flask is None:
        return render_template('index.html')
    
    send_to_ros(request, "speech")
    
    return utility_flask.handle_robot_hint(request, socketio)

@app.route('/cheating/<int:id>', methods=["GET", "POST"])
def def_cheater(id):
    id = session.get('id')
    Menu.clean_shell()
    Util.formatted_debug_message("Page reloaded during the game...", level='INFO')
    utility_flask = client_instances.get(id)
    if utility_flask is not None:
        client_instances[id].handle_cheater()
        client_instances[id].handle_id_player(id, client_instances.get(id), expertiment_condition)
    
    return redirect(url_for("show_game"))

@app.route('/robot_speech', methods=["POST"])
def handle_pop_up():
    data = request.get_json()
    # send to the client a message in order to remove the pop-up since the robot has finished uttering the suggestion
    socketio.emit('Speech', json.dumps(data))
    
    return jsonify({'message': 'hint uttered'}), 200

@app.before_request
def before_request():
    global cleanup_flag
    if cleanup_flag:
        Util.formatted_debug_message("Cleaning session before exiting...", level='INFO')
        session.clear()
        os._exit(0)

def handle_exit(*args):
    global cleanup_flag
    Util.formatted_debug_message('(Server) CTRL-C pressed!', level='INFO')
    Util.formatted_debug_message(f"Close web page and open it again if you want to play again!", level='INFO')
    Util.formatted_debug_message("Exit...", level='INFO')
    cleanup_flag = True
    os._exit(0)

if __name__ == '__main__':
    # handle CTRL+C
    signal.signal(signal.SIGINT, handle_exit)
    # run app
    print("* Running on http://" + IP_ADDRESS + ":5000/ (Press CTRL+C to quit)")
    socketio.run(app, host=IP_ADDRESS, port=5000, debug=True, use_reloader=False, log_output=False)