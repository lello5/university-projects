import sys
from pepper_cmd_classes import *
sys.path.append('tablet/scripts')
from modim_classes import *
from classify_evaluate_executor import run_executor

dialogue = Dialogue() 
touch = Touch()
database = Database(filename = 'registered_users', timeout = 10)
database.create_database(['Name', 'Family Name', 'Classification Result'])
# use absolute path to music to connect to a real robot
# music = Music(music_path = '/home/nao/audio/whatever it takes.wav', music_path_wave = '/home/nao/audio/whatever it takes.wav')
music = Music(music_path = '/home/olga/playground/music/whatever it takes.wav', music_path_wave = 'music/whatever it takes.wav')
game = Game()

def ask_start():
    dialogue.say("Hello! I am Pepper, and I can teach you more about the basketball.", sleeping_time = 1.0)
    dialogue.say("Touch my head if you would like to start the interaction.")
    return touch.monitor_touch(monitoring_time = 20.0) # waiting for python touch_sim.py --sensor HeadMiddle

def check_new():
    global res, name, surname, classification_result
    res, name, surname, classification_result = database.detect_user()
    return res

def classify_evaluate_user():
    global name, surname, classification_result
    classification_result = run_executor()
    database.update_result(name, surname, classification_result)

def ask_players():
    AskPlayers()

def warm_up():
    WarmUp()

def start_lessons(allow_choice = False):
    global res, classification_result
    #res, classification_result = False, 'BEGINNER'
    if not res:
        allow_choice = True
    else:
        dialogue.say("I am launching for you a basketball course of the {} level.".format(classification_result))
    print("Starting lessons...")
    f = open('/home/robot/playground/status.out', 'w')
    f.write('')      
    f.close()
    lessons = Lessons(classification_result, allow_choice = allow_choice)
    f = open('/home/robot/playground/status.out', 'r')
    button = f.read()
    while not button in ['game', 'music', 'exit']:
        button = f.read()
    print("Button: ", button)  
    f.close()
    if button == 'music':
        music.play()
        start_lessons(allow_choice = True)
    elif button == 'game':
        game.interaction_handler()
        start_lessons(allow_choice = True)
    else: return 0

def terminate():
    dialogue.say("I will be glad to see you next time! Goodbye!")
    time.sleep(10)
    CleanScreen()

