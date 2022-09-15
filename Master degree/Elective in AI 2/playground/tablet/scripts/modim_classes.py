import sys, os
import time

try:
    sys.path.insert(0, os.getenv('MODIM_HOME')+'/src/GUI')
except Exception as e:
    print "Please set MODIM_HOME environment variable to MODIM folder."
    sys.exit(1)

import ws_client
from ws_client import *

class Quiz:

    def __init__(self, mode = ''):
        mws = ModimWSClient()
        mws.setDemoPathAuto(__file__)
        if mode == 'easy1':
            mws.run_interaction(self.i1)
        elif mode == 'easy2':
            mws.run_interaction(self.i2)
        elif mode == 'easy3':
            mws.run_interaction(self.i3)
        elif mode == 'hard1':
            mws.run_interaction(self.i4)    
        elif mode == 'hard2':
            mws.run_interaction(self.i5)
        elif mode == 'hard3':
            mws.run_interaction(self.i6)
        elif mode == 'good':
            mws.run_interaction(self.i7)
        elif mode == 'medium':
            mws.run_interaction(self.i8)
        elif mode == 'bad':
            mws.run_interaction(self.i9)
        else:
            mws.run_interaction(self.i0)    

    def i0(self):
        im.init()
        im.ask('welcome_basketball', timeout = -1)  # wait for button
    def i1(self):
        a = im.ask('easy1', timeout = -1)
        f = open('/home/robot/playground/quiz_answer.out','w')
        f.write(a)
        f.close()
    def i2(self):
        a = im.ask('easy2', timeout = -1)
        f = open('/home/robot/playground/quiz_answer.out','w')
        f.write(a)
        f.close()
    def i3(self):
        a = im.ask('easy3', timeout = -1)
        f = open('/home/robot/playground/quiz_answer.out','w')
        f.write(a)
        f.close()
    def i4(self):
        a = im.ask('hard1', timeout = -1)
        f = open('/home/robot/playground/quiz_answer.out', 'w')
        f.write(a)
        f.close()
    def i5(self):
        a = im.ask('hard2', timeout = -1)
        f = open('/home/robot/playground/quiz_answer.out', 'w')
        f.write(a)
        f.close()
    def i6(self):
        a = im.ask('hard3', timeout = -1)
        f = open('/home/robot/playground/quiz_answer.out', 'w')
        f.write(a)
        f.close()
    def i7(self):
        im.display.loadUrl('good.html')
    def i8(self):
        im.display.loadUrl('medium.html')
    def i9(self):
        im.display.loadUrl('bad.html')

class AskPlayers:

    def __init__(self):
        mws = ModimWSClient()
        mws.setDemoPathAuto(__file__)
        mws.run_interaction(self.i1)

    def i1(self):
        time.sleep(2.0)
        im.init()
        time.sleep(1.0)
        im.ask('continue', timeout = -1)  # wait for button

        a = im.ask('players', timeout = -1)

        if (a!='timeout'):
            im.execute(a)
            im.execute('goodbye_players')

class WarmUp:

    def __init__(self):
        mws = ModimWSClient()
        mws.setDemoPathAuto(__file__)
        mws.run_interaction(self.i1)

    def i1(self):
        im.ask('continue', timeout = -1)  # wait for button
        im.ask('warmup', timeout = 68)
        im.execute('goodbye_warmup')

class Lessons:

    def __init__(self, mode = 'BEGINNER', allow_choice = False):
        mws = ModimWSClient()
        mws.setDemoPathAuto(__file__)
        if mode == 'BEGINNER':
            if allow_choice:
                mws.run_interaction(self.i0)
            else:
                mws.run_interaction(self.i1)
        elif mode == 'INTERMEDIATE':
            if allow_choice:
                mws.run_interaction(self.i2)
            else:
                mws.run_interaction(self.i3)
        else:
            if allow_choice:
                mws.run_interaction(self.i4)
            else:
                mws.run_interaction(self.i5)

    def i0(self):
        im.display.loadUrl('beginner_choice.html')

    def i1(self):
        a = im.ask('start_lessons', timeout = -1)
        if (a != 'timeout'):
            im.display.loadUrl('beginner_lesson1.html')

    def i2(self):
        im.display.loadUrl('intermediate_choice.html')

    def i3(self):
        a = im.ask('start_lessons', timeout = -1)
        if (a != 'timeout'):
            im.display.loadUrl('intermediate_lesson1.html')

    def i4(self):
        im.display.loadUrl('expert_choice.html')

    def i5(self):
        a = im.ask('start_lessons', timeout = -1)
        if (a != 'timeout'):
            im.display.loadUrl('expert_lesson1.html')

class CleanScreen:

    def __init__(self):
        mws = ModimWSClient()
        mws.setDemoPathAuto(__file__)
        mws.run_interaction(self.i1)

    def i1(self):
        im.init()
