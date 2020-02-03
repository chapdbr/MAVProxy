#!/usr/bin/env python
'''

Module to send the position from a FT sensor.
Bruno Chapdelaine, June 2018

'''

import Tkinter as tk
import tkFileDialog
from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
import ATI45
from ATI45 import sensor
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

class position_ft_sensor_ati45_module(mp_module.MPModule):
	def __init__(self, mpstate):
		# The callable module name is defined in the second __init__ arg
		super(position_ft_sensor_ati45_module, self).__init__(mpstate, "position_ft_sensor_ati45", "ati45 position ft sensor support")
		self.add_command('radius', self.cmd_radius, "tether length",
						 ['tether length (m)'])
		self.add_command('tare', self.cmd_tare, "tare load cell",
						 ['tare load cell'])
		self.radius = 10 # initial value
		self.ati45 = sensor('ATI45', calibration_ati45, self.radius)
		self.init_savefile()
		self.ati45.init_stream()
		self.ati45.tare()
		self.ati45.init_time()
		#p1 = subprocess.Popen(['python', 'plot_forces.py', 'self.filename'])
		#p2 = subprocess.Popen(['python', 'plot_position.py', 'self.filename'])
		self.save_period = mavutil.periodic_event(20)

		# init variables
		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0
		self.rollspeed = 0.0
		self.pitchspeed = 0.0
		self.yawspeed = 0.0
		self.airspeed = 0.0
		self.voltage = 0.0
		self.current = 0.0
		self.rpm = 0.0
		self.xacc = 0.0
		self.yacc = 0.0
		self.zacc = 0.0
		self.throttle = 0.0
		self.now = 0.0
		self.time_us = 0.0

		self.q = np.array([1.0,0.0,0.0,0.0])

		#root = tk.Tk()
		#root.withdraw()

		#self.file_path = tkFileDialog.askopenfilename(filetypes=(("Text Files", ".txt"),
		#														 ("All Files", "*.*")))

	def cmd_tare(self, args):
		'''tether length'''
		usage = 'tare <start>'
		if len(args) < 1:
			print(usage)
		elif args[0] == "start":
			self.ati45.tare()
			print("Load cell tared")
		else:
			print(usage)

	def cmd_radius(self, args):
		'''tether length'''
		usage = 'Usage: radius <value (m)>'
		if len(args) < 1:
			print(usage)
		else:
			value = float(args[0])
			if value < 0:
				print("Tether length cannot be negative")
			else:
				self.radius = value
				self.ati45.set_radius(self.radius)
				print("Tether length set to %f m" % (self.radius))

	def idle_task(self):
		'''called in idle time'''
		if self.save_period.trigger():
			self.ati45.read_data()
			self.log.write(str(self.ati45.elapsed_time) + ',' +
						   str(self.ati45.fx) + ',' +
						   str(self.ati45.fy) + ',' +
						   str(self.ati45.fz) + ',' +
						   str(self.ati45.fr) + ',' +
						   str(self.ati45.x) + ',' +
						   str(self.ati45.y) + ',' +
						   str(self.ati45.z) + ',' +
						   str(self.roll) + ',' +
						   str(self.pitch) + ',' +
						   str(self.yaw) + ',' +
						   str(self.rollspeed) + ',' +
						   str(self.pitchspeed) + ',' +
						   str(self.yawspeed) + ',' +
						   str(self.current) + ',' +
						   str(self.voltage) + ',' +
						   str(self.rpm) + ',' +
						   str(self.airspeed) + ',' +
						   str(self.xacc) + ',' +
						   str(self.yacc) + ',' +
						   str(self.zacc) + ',' +
						   str(self.throttle) + '\n')

			try:
				#self.master.mav.position_ft_sensor_send(
				#	self.ati45.x,
				#	self.ati45.y,
				#	self.ati45.z,
				#	)
				self.now = time.time()
				self.time_us = int(self.now * 1.0e6)
				#self.master.mav.att_pos_mocap_send(self.time_us, self.q, self.ati45.fx, self.ati45.fy, self.ati45.fz, force_mavlink1=True)
				self.master.mav.vicon_position_estimate_send(self.time_us, self.ati45.fx, self.ati45.fy, self.ati45.fz, self.ati45.fx, self.ati45.fy, self.ati45.fz, force_mavlink1=True)
			except Exception as e:
				print("Position output failed:", e)
	def init_savefile(self):
		self.filename = self.name + "_savefile_" + datetime.datetime.now().strftime("%Y-%B-%d_%I-%M-%p") + ".txt"
		self.log = open(self.filename, 'w+',1)
		self.log.write('elapsed_time (s)' + ',' +
					   'fx (N)' + ',' +
					   'fy (N)' + ',' +
					   'fz (N)' + ',' +
					   'fr (N)' + ',' +
					   'x (m)' + ',' +
					   'y (m)' + ',' +
					   'z (m)' + ',' +
					   'roll (rad)' + ',' +
					   'pitch (rad)' + ',' +
					   'yaw (rad)' + ',' +
					   'rollspeed (rad/s)' + ',' +
					   'pitchspeed (rad/s)' + ',' +
					   'yawspeed (rad/s)' + ',' +
					   'battery_current (centiamp)' + ',' +
					   'battery_voltage (milivolts)' + ',' +
					   'RPM' + ',' +
					   'airspeed (m/s)' + ',' +
					   'xacc' + ',' +
					   'yacc' + ',' +
					   'zacc' + ',' +
					   'throttle%' + '\n')
	def mavlink_packet(self, m):
		'''handle a mavlink packet'''
		#if m.get_type() == 'POSITION_FT_SENSOR':
		#	print('Received message:')
		#	print(m)
		if m.get_type() == 'ATTITUDE':
			self.roll = m.roll
			self.pitch = m.pitch
			self.yaw = m.yaw
			self.rollspeed = m.rollspeed
			self.pitchspeed = m.pitchspeed
			self.yawspeed = m.yawspeed
			#print(m)
		if m.get_type() == 'VFR_HUD':
			self.airspeed = m.airspeed
			self.throttle = m.throttle
		if m.get_type() == 'SYS_STATUS':
			self.voltage = m.voltage_battery
			self.current = m.current_battery
		if m.get_type() == 'RPM':
			self.rpm = m.rpm1
		if m.get_type() == 'SCALED_IMU2':
			self.xacc = m.xacc
			self.yacc = m.yacc
			self.zacc = m.zacc
def init(mpstate):
	'''initialise module'''
	return position_ft_sensor_ati45_module(mpstate)