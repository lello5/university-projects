import os, sys
import time
import csv
import wave, contextlib
import math
from numpy.random import choice

sys.path.append(os.getenv('PEPPER_TOOLS_HOME')+'/cmd_server')

import pepper_cmd
from pepper_cmd import *

begin()

class Configure():

	def __init__(self, alive = True, speed = 80):
		pepper_cmd.robot.setAlive(alive)
		pepper_cmd.robot.tts_service.setParameter("speed", 80)
		
class Sonar:

	def __init__(self):
		Configure()

	def listen(self, personHere = False, threshold = 1.0, curr_time = 0.0, wait_time = 3.0): 

		pepper_cmd.robot.startSensorMonitor()

		if not personHere:
			print("Waiting people...")
			try:
				while not personHere:
					p = pepper_cmd.robot.sensorvalue()
					personHere =  (0.0 < p[1] < threshold) or (0.0 < p[2] < threshold)
					curr_time = time.time()
			except KeyboardInterrupt:
				pepper_cmd.robot.stopSensorMonitor()
				sys.exit(0)
			return self.listen(True, 1.0, curr_time, 3.0)
		else:
			print("Person approached. Measuring time...")
			while personHere:
				p = pepper_cmd.robot.sensorvalue()
				personHere =  (0.0 < p[1] < threshold) or (0.0 < p[2] < threshold)
				if time.time() - curr_time >= wait_time:
					print("Person stayed for more than {} seconds.".format(wait_time))
					pepper_cmd.robot.stopSensorMonitor()
					return 'front' if (0.0 < p[1] < threshold) else 'back'
			print("Person approached, but then left.")
			return self.listen(False, 1.0, 0.0, 3.0)

# sonar = Sonar()
# sonar.listen()

class Dialogue:

	def __init__(self, speed = 50):
		Configure(speed = speed)

	def say(self, sentence, require_answer = False, sleeping_time = 0.0):
		pepper_cmd.robot.say(sentence)
		if require_answer:
			return self.listen(timeout = 30)
		if sleeping_time:
			time.sleep(sleeping_time)

	def listen(self, vocabulary = ['olga', 'giorgia', 'lorenzo', 'leandro', 'luca', 'fabio', 'sorokoletova', 'natalizia', 
								   'nicoletti', 'maglianella', 'iocchi', 'patrizi', 'yes', 'no'], timeout = 30):
		answer = pepper_cmd.robot.asr(vocabulary = vocabulary, timeout = timeout)
		while not answer:
			answer = self.say(sentence = "Sorry, I did not hear you, repeat please.", require_answer = True)
		return answer

# dialogue = Dialogue()
# name = dialogue.say(sentence = 'what is your name?', require_answer = True)
# dialogue.say(sentence = name)

class Touch:
	def __init__(self, touched = False):
		Configure(speed = 200)
		# joint angles should be given in radians
		# 0 -  HeadYaw,        1 -  HeadPitch
		# 2 -  LShoulderPitch, 3 -  LShoulderRoll, 4 -  LElbowYaw, 5 -  LElbowRoll, 6 -  LWristYaw
		# 7 -  RShoulderPitch, 8 -  RShoulderRoll, 9 -  RElbowYaw, 10 - RElbowRoll, 11 - RWristYaw
		# 12 - LHand,          13 - RHand, 		 14 - HipRoll, 	 15 - HipPitch,   16 - KneePitch
		self.jointNames = ["HeadYaw", "HeadPitch",
               "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
               "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw",
               "LHand", "RHand", "HipRoll", "HipPitch", "KneePitch"]
		self.touched = touched

	def change_pose(self, indices, values, pose, sleeping_time = 0.5):

		joint_list = []
		for i, v in zip(indices, values):
			pose[i] = v
			joint_list.append(self.jointNames[i])

		pepper_cmd.robot.setPosture(pose)
		time.sleep(sleeping_time)
		print("Pose changed in joints: {}".format(joint_list))

		return pose

	def monitor_touch(self, monitoring_time = 20.0):

		curr_time = time.time() #start timer

		pepper_cmd.robot.startSensorMonitor()
		print("Waiting a touch to start during {} seconds...".format(monitoring_time))
		while not self.touched and (time.time() - curr_time < monitoring_time):
			p = pepper_cmd.robot.sensorvalue()
			self.touched = (p[3] > 0) # p[3] is a head touch sensor
		pepper_cmd.robot.stopSensorMonitor()

		if self.touched:
			print("A touch is detected.")
			pose = pepper_cmd.robot.getPosture()
			self.change_pose([0, 1], [0.0, -0.5], pose, sleeping_time = 1.0)
			pepper_cmd.robot.normalPosture()
			print("Pose is back to normal.")

		return self.touched

class Database:

	def __init__(self, filename = "", timeout = 30):
		Configure()
		self.filename = filename
		self.timeout = timeout
		self.dialogue = Dialogue()

	def create_database(self, header):
		if not os.path.exists(self.filename):
			with open(self.filename, 'a') as f:
				writer = csv.DictWriter(f, delimiter = ",", fieldnames = header)
				writer.writeheader()
			print("Create the database with name {} and header {}.".format(self.filename, header))
		else:
			print("The database with this name already exists.")
		self.header = header

	def interview_user(self):
		name = self.dialogue.say(sentence = "Right decision! What is your name?", require_answer = True, sleeping_time = 0.0)
		print("Name is " + name)
		surname = self.dialogue.say(sentence = "Perfect. Tell me also your family name please.", require_answer = True, sleeping_time = 0.0)
		return name.lower().capitalize(), surname.lower().capitalize()

	def detect_user(self, register = True):
		is_new, classification_result = False, ''
		name, surname = self.interview_user()
		if os.path.exists(self.filename):
			id_data, results_data = [], {}
			with open(self.filename, 'r') as f:
				reader = csv.DictReader(f)
				for row in reader:
					id_data.append((row[self.header[0]], row[self.header[1]]))
					results_data[(row[self.header[0]], row[self.header[1]])] = row[self.header[2]]
			if (name, surname) in id_data:
				print("Record {} {} exists.".format(name, surname))
				self.dialogue.say("I am glad to see you again {}!".format(name))
				classification_result = results_data[(name, surname)]
			else:
				print("Record {} {} doesn't exist.".format(name, surname))
				self.dialogue.say("Nice to meet you {}".format(name))
				is_new = True
				if register:
					self.register_user(name, surname)
		else: 
			print("Cannot find database.")
		return is_new, name, surname, classification_result

	def register_user(self, name, surname):
		if os.path.exists(self.filename):
			with open(self.filename, 'a') as f:
				writer = csv.DictWriter(f, delimiter = ",", fieldnames = self.header)
				writer.writerow({self.header[0]: name, self.header[1]: surname})
				print("User {} {} has been registered succesfully.".format(name, surname))
		else: 
			raise Exception("Cannot find the database.")

	def update_result(self, name, surname, classification_result):
		
		if os.path.exists(self.filename):
			data = []
			with open(self.filename, 'r') as f:
				reader = csv.DictReader(f)
				for row in reader:
					if (row[self.header[0]], row[self.header[1]]) == (name, surname):
						data.append([row[self.header[0]], row[self.header[1]], classification_result])
					else:
						data.append([row[self.header[0]], row[self.header[1]], row[self.header[2]]])

			with open(self.filename, 'w') as f:
				writer = csv.DictWriter(f, delimiter = ",", fieldnames = self.header)
				writer.writeheader()
				for row in data:
					writer.writerow({self.header[0]: row[0], self.header[1]: row[1], self.header[2]: row[2]})
		else: 
			print("Cannot find database.")

#database = Database(filename = 'registered_users', timeout = 10)
#database.create_database(['Name', 'Family Name', 'Classification Result'])
#database.register_user('giorgia', 'natalizia')
# is_new, name, surname, classification_result = database.detect_user()
# print(is_new, name, surname, classification_result)
# is_new, name, surname, classification_result = database.detect_user()
# print(is_new, name, surname, classification_result)
#database.update_result('olga', 'sorokoletova', 'EXPERT')
# is_new, name, surname, classification_result = database.detect_user()
# print(is_new, name, surname, classification_result)

class Music:

	def __init__(self, music_path = '', music_path_wave = ''):

		self.dialogue = Dialogue()
		self.music_path = music_path
		self.music_path_wave = music_path_wave
		self.get_duration()

	def get_duration(self):
	
		with contextlib.closing(wave.open(self.music_path_wave, 'r')) as f:
			frames = f.getnframes()
			rate = f.getframerate()
			self.duration = int(frames / float(rate)) - 4

	def play(self):

		self.dialogue.say("Alright! Let us listen some nice music together.")

	 	ap_service = pepper_cmd.robot.session.service("ALAudioPlayer")
		ap_service.playFile(self.music_path, _async = True)
		#ap_service.playFile('/home/nao/audio/whatever it takes.wav', _async = True)

		cur_time = time.time()
	 	touch = Touch()
	 	pose = pepper_cmd.robot.getPosture()
	 	indices = [2, 3, 5, 7, 8, 10, 14, 15, 16]
	 	moves = [[1.5, 1.5, 0, 0, -1.5, 0, -0.2, 0.3, -0.2],
		 		 [0, 0, 1.5, 0, 0, 1.5, 0.2, -0.3, 0.2],
		 		 [0, 1.5, -1.5, 1.5, -1.5, 1.5, 0.2, -0.3, 0.2],
		 		 [0, 0, 0, 0, 0, 0, -0.2, 0.3, -0.2]]

	 	while time.time() - cur_time < self.duration:
		 	for values in moves:
		 		touch.change_pose(indices, values, pose)

		pepper_cmd.robot.normalPosture()
		self.dialogue.say("You have to believe in yourself! Now it is time to continue exercising.")

# music = Music(music_path = '/home/olga/playground/music/whatever it takes.wav', music_path_wave = 'music/whatever it takes.wav')
# music.play()

class Bounce:

	def __init__(self, hand = "left", num_bounces = 3):

		self.a = math.pi/180
		self.touch = Touch()
		self.pose = pepper_cmd.robot.getPosture()
		self.num_bounces = num_bounces

		if hand == "left":
			self.bounce_left()
		elif hand == "right":
			self.bounce_right()
		else:
			self.bounce_both()

	def bounce_right(self):

		self.touch.change_pose([7, 10, 11], 
							   [self.a*50, self.a*60, -self.a*90], 
							   self.pose, 
							   0.1)

		for _ in range(self.num_bounces):
			self.touch.change_pose([10], [self.a*50], self.pose, 0.1)
			self.touch.change_pose([10], [self.a*60], self.pose, 0.1)

	def bounce_left(self):

		self.touch.change_pose([2, 5, 6], 
							   [self.a*50, -self.a*60, self.a*90],
							   self.pose, 
							   0.1)

		for _ in range(self.num_bounces):
			self.touch.change_pose([5], [-self.a*50], self.pose, 0.1)
			self.touch.change_pose([5], [-self.a*60], self.pose, 0.1)

	def bounce_both(self):

		self.touch.change_pose([7, 10, 11, 2, 5, 6], 
							   [self.a*50,  self.a*60, -self.a*90, self.a*50, -self.a*60, self.a*90], 
							   self.pose,
							   0.1)

		for _ in range(self.num_bounces):
			self.touch.change_pose([10, 5], [self.a*50, -self.a*50], self.pose, 0.1)
			self.touch.change_pose([10, 5], [self.a*60, -self.a*60], self.pose, 0.1)

# bounce = Bounce(hand = "left", num_bounces = 2)

class Game:

	def __init__(self):
		
		self.dialogue = Dialogue()
		self.a = math.pi/180
		self.vocabulary = [['yes', 'no'], ['left', 'right']]

	def play(self):

		self.dialogue.say("Guess which hand hides an invisible candy? Say left or right.")

		# hand moves
		touch = Touch()
		pose = pepper_cmd.robot.getPosture()
		touch.change_pose([7, 10, 11, 2, 5, 6], 
						  [self.a*50,  self.a*60, -self.a*90, self.a*50, -self.a*60, self.a*90], 
						  pose, 
						  sleeping_time = 1.0)
		pepper_cmd.robot.motion_service.stopMove()

		# getting correct answer
		correct_answer  = choice(self.vocabulary[1], 1)[0]
		print("\nPepper hides candy in a {} hand.\n".format(correct_answer.upper()))

		# getting user answer and comparing with correct
		user_answer = self.dialogue.listen(vocabulary = self.vocabulary[1]).lower()
		if user_answer != correct_answer:
			if user_answer == "left":
				touch.change_pose([7, 10, 11, 2, 5, 6], 
								  [self.a*50, self.a*60, -self.a*90, self.a*50, -self.a*60, -self.a*90], 
								  pose)
			else:
				touch.change_pose([7, 10, 11, 2, 5, 6], 
								  [self.a*50,  self.a*60, self.a*90, self.a*50, -self.a*60, self.a*90], 
								  pose)
			self.dialogue.say("Oops, there is no candy, let us try again!")
			self.play()
		else: return 0

	def interaction_handler(self):
		# game handler
		self.dialogue.say("Perfect! Let us play the game.")
		self.play()
		self.dialogue.say("Yes! Candy was hidden here.")
		time.sleep(2)
		# bounce handler
		self.dialogue.say("Now it is time to exercise some more. Set you arms as me and bounce the ball.")
		hand = choice(["left", "right", "both"], 1)[0]
		bounce = Bounce(hand = hand, num_bounces = 3)
		# normalizing configuration
		pepper_cmd.robot.normalPosture()
		self.dialogue.say("Good job! We can return to the lessons now.")

# game = Game()
# game.interaction_handler()

end()
