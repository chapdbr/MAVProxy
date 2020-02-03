# Libraries
import nidaqmx.stream_readers
import numpy as np
import time
import datetime
import subprocess
import os
import msvcrt
import math

###################################################################################################
calibration_ati45 = np.array([[0.15596, 0.05364, -0.29616, -22.76401, 0.71258, 22.77619],
						[0.17628, 25.86714, -0.34444, -13.07013, -0.42680, -13.39725],
						[29.42932, -1.68566, 29.50395, -0.09508, 29.75563, -1.01253],
						[0.00754, 0.22624, -0.47721, -0.11727, 0.47877, -0.12927],
						[0.53998, -0.02617, -0.25730, 0.19768, -0.29711, -0.19153],
						[0.00107, -0.33260, -0.00357, -0.34021, -0.01832, -0.33980]])
# for debug
#calibration_ati45 = np.array([[0,1], [2,2]])
###################################################################################################

class sensor(object):
	def __init__(self, name, calibration, r):
		self.data = np.empty([6,])
		self.name = name
		self.calibration = calibration
		self.r = r # length of the cable
		self.time_interval =[]
		self.elapsed_time = 0.0
		self.fx = 0.0
		self.fy = 0.0
		self.fz = 0.0
		self.fr = 0.0
		self.theta = 0.0
		self.phi = 0.0
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
	def set_radius(self, radius):
		self.r = radius
	def tare(self):
		self.tare_input = np.empty([6,])
		#print("Press ENTER to tare")
		#while input() != '':
		#	pass
		self.my_multi_channel_ai_reader.read_one_sample(self.tare_input, timeout=10)
		print("Tared")
	def init_stream(self):
		self.task = nidaqmx.Task()
		self.task.ai_channels.add_ai_voltage_chan("Dev1/ai0:5","analog_input",nidaqmx.constants.TerminalConfiguration.RSE)
		#self.channel1 = nidaqmx.system.PhysicalChannel('Dev1/ai0')
		#print(self.channel1.ai_term_cfgs)
		self.my_stream = nidaqmx._task_modules.in_stream.InStream(self.task)
		self.my_multi_channel_ai_reader = nidaqmx.stream_readers.AnalogMultiChannelReader(self.my_stream)

	def init_time(self):
		self.last_time = time.time()
		self.start_time = self.last_time
	def read_data(self):
		self.my_multi_channel_ai_reader.read_one_sample(self.data, timeout=10)
		self.data_tared = np.array(self.data - self.tare_input)
		#self.data_tared = self.data_tared[:, np.newaxis] # create column vector from row
		self.data_tared = np.reshape(self.data_tared,(6, 1))
		self.data_tared_cal = np.matmul(self.calibration,self.data_tared)
		self.now = time.time()
		#self.time_interval.append(self.now-self.last_time) # reading_freq
		#self.last_time = self.now
		#self.last_time = time.time()
		# Log data
		# Calculate time
		self.elapsed_time = str(self.now - self.start_time)
		# Define forces
		self.fx = self.data_tared_cal[0, 0]
		self.fy = self.data_tared_cal[1, 0]
		self.fz = self.data_tared_cal[2, 0]
		self.fr = math.sqrt(self.fx**2+self.fy**2+self.fz**2)
		# Calculate position
		self.theta = math.atan2(self.fy, self.fx)
		self.phi = math.acos(self.fz/self.fr)
		self.x = self.r*math.cos(self.theta)*math.sin(self.phi)
		self.y = self.r*math.sin(self.theta)*math.sin(self.phi)
		self.z = self.r*math.cos(self.phi)

		return self.data_tared_cal

	#def reading_freq(self):
		#self.mean_delay = np.average(self.time_interval[2:len(self.time_interval)])
		#print(str(int(round(1 / self.mean_delay))) + ' Hz')

	def close(self):
		self.task.close()
		self.log.close()
def uinput_plot_forces():
	global plotting_forces, p1
	repeat = True
	while repeat:
		uinput = input()
		if uinput == 'y':
			dir_path = os.path.dirname(os.path.realpath(__file__))
			plot_forces_target = dir_path + '\plot_forces.exe'
			repeat = False
			plotting_forces = True
			print('Starting the plotting process...')
			p1 = subprocess.Popen(['python', 'plot_forces.py', 'ati45.filename'])
			#p1 = subprocess.Popen([plot_forces_target, 'ati45.filename']) #if building exe file
		elif uinput == 'n':
			repeat = False
			plotting_forces = False
		else:
			print("Please press the right key.")
def uinput_plot_pos():
	global plotting_pos, p2
	print("Do you want to plot the position? <y/n>: ")
	repeat = True
	while repeat:
		uinput = input()
		if uinput == 'y':
			dir_path = os.path.dirname(os.path.realpath(__file__))
			plot_pos_target = dir_path + '\plot_position.exe'
			repeat = False
			plotting_pos = True
			print('Starting the plotting process...')
			p2 = subprocess.Popen(['python', 'plot_position.py', 'ati45.filename'])
			#p2 = subprocess.Popen([plot_pos_target, 'ati45.filename']) #if building exe file
		elif uinput == 'n':
			repeat = False
			plotting_pos = False
		else:
			print("Please press the right key.")
def read_n_log_data():
	print('Reading and logging data')
	print('Press T to tare and ESC key to exit...')
	#for x in range(0, 3):
	while True:
		ati45.read_data()
		time.sleep(0.01)
		if msvcrt.kbhit():
			key = ord(msvcrt.getch())
			if key == 27: # escape key
				break
			elif key == 116: # t key
				ati45.tare_quick()
if __name__ == '__main__':
	global plotting_forces, plotting_pos, p1, p2
	ati45 = sensor('ATI45', calibration_ati45, 10)
	ati45.init_savefile()
	ati45.init_stream()
	ati45.tare()
	ati45.init_time()
	ati45.read_data()

	uinput_plot_forces()
	uinput_plot_pos()

	read_n_log_data()

	if plotting_forces:
		p1.kill()
		p1.terminate()
	if plotting_pos:
		p2.kill()
		p2.terminate()
	#ati45.reading_freq()
	ati45.close() # end task