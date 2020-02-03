#!/usr/bin/env python
'''

Module to send the position from a FT sensor.
Bruno Chapdelaine, June 2018

'''

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
import time
import socket, errno
import json
import math

class position_ft_sensor_module(mp_module.MPModule):
	def __init__(self, mpstate):
		# The callable module name is defined in the second __init__ arg
		super(position_ft_sensor_module, self).__init__(mpstate, "position_ft_sensor", "position ft sensor support")

		self.add_command('radius', self.cmd_radius, "tether length", ['tether length (m)'])
		self.add_command('port', self.cmd_port, 'port selection', ['<5006>'])

		self.radius = 10 # initial value
		# self.send_period = mavutil.periodic_event(1)
		self.last_time = time.clock()
		self.ip = "127.0.0.1" # localhost
		self.portnum = 5010 # udp port
		self.buffer_size = 1024
		self.port = socket.socket(socket.AF_INET,  # Internet
								  socket.SOCK_DGRAM)  # UDP
		self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.port.bind((self.ip, self.portnum))
		mavutil.set_close_on_exec(self.port.fileno())
		self.port.setblocking(0)
		print("Listening for GPS Input packets on UDP://%s:%s" % (self.ip, self.portnum))

	def idle_task(self):
		'''called in idle time'''
		try:
			'''Receive data from UDP socket'''
			datagram, addr = self.port.recvfrom(self.buffer_size)
			data = json.loads(datagram)
			now = time.time()
			time_us = int(now * 1.0e6)
			fx = data[0]
			fy = data[1]
			fz = data[2]
			fr = math.sqrt(fx**2 + fy**2 + fz**2)
			'''Calculate position'''
			theta = math.atan2(fy, fx)
			phi = math.acos(fz / fr)
			x = self.radius * math.cos(theta) * math.sin(phi)
			y = self.radius * math.sin(theta) * math.sin(phi)
			z = self.radius * math.cos(phi)
			'''Evaluate frequency'''
			now = time.clock()
			freq = 1 / (now - self.last_time)
			self.last_time = now
			print('freq:%s' % (freq))
			'''Send MAVLink message'''
			try:
				self.master.mav.vicon_position_estimate_send(time_us, x, y, z, fx, fy, fz, force_mavlink1=True)
			except Exception as e:
				print("Position inject failed:", e)
		except socket.error as e:
			if e.errno in [errno.EAGAIN, errno.EWOULDBLOCK]:
				return
			raise

	# def mavlink_packet(self, m):
	# 	'''handle a mavlink packet'''
	def cmd_port(self, args):
		'handle port selection'
		if len(args) != 1:
			print('Usage: port <number>')
			return

		self.port.close()
		self.portnum = int(args[0])
		self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.port.bind((self.ip, self.portnum))
		self.port.setblocking(0)
		mavutil.set_close_on_exec(self.port.fileno())
		print("Listening for GPS INPUT packets on UDP://%s:%s" % (self.ip, self.portnum))

	def cmd_radius(self, args):
		'''tether length'''
		if len(args) < 1:
			print('Usage: radius <value (m)>')
		else:
			value = float(args[0])
			if value < 0:
				print("Tether length cannot be negative")
			else:
				self.radius = value
				print("Tether length set to %f m" % (self.radius))

def init(mpstate):
	'''initialise module'''
	return position_ft_sensor_module(mpstate)