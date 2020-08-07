"""
Module that sends force & position data from a FT sensor connected via UDP to a MAVLink vehicle.
Bruno Chapdelaine, February 2020
"""

from MAVProxy.modules.lib import mp_module
from pymavlink import mavutil
import time
import socket, errno
import json

class position_ft_sensor_module(mp_module.MPModule):
	"""Transmits data from a UDP port to a MAVLink vehicle."""
	def __init__(self, mpstate):
		super(position_ft_sensor_module, self).__init__(mpstate, "position_ft_sensor", "position ft sensor support") # the callable module name is defined in the second init arg
		self.add_command("port", self.cmd_port, "port selection", ["<5006>"]) # UDP port selection
		self.last_time = time.clock()
		# UDP setup
		self.ip = "127.0.0.1" # localhost
		self.portnum = 5010 # default UDP port
		self.buffer_size = 1024
		self.port = socket.socket(socket.AF_INET,  # internet
								  socket.SOCK_DGRAM)  # UDP
		self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.port.bind((self.ip, self.portnum)) # bind port
		mavutil.set_close_on_exec(self.port.fileno())
		self.port.setblocking(0)
		print("Listening for input packets on UDP://%s:%s" % (self.ip, self.portnum))

	def idle_task(self):
		"""Called in idle time."""
		# Try to read data from UDP socket
		try:
			datagram, addr = self.port.recvfrom(self.buffer_size)
			data = json.loads(datagram)
			now = time.time()
			time_us = int(now * 1.0e6)
			fx = data[0]
			fy = data[1]
			fz = data[2]
			x  = data[3]
			y  = data[4]
			z  = data[5]

			# Try to send MAVLink message
			try:
				self.master.mav.vicon_position_estimate_send(time_us, x, y, z, fx, fy, fz, force_mavlink1=True)
			except Exception as e:
				print("Position inject failed:", e)
		except socket.error as e:
			if e.errno in [errno.EAGAIN, errno.EWOULDBLOCK]:
				return
			raise

	def cmd_port(self, args):
		"""Handle port selection."""
		if len(args) != 1:
			print("Usage: port <number>")
			return
		self.port.close() # close current port
		self.portnum = int(args[0])
		self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.port.bind((self.ip, self.portnum)) # bind new port
		self.port.setblocking(0)
		mavutil.set_close_on_exec(self.port.fileno())
		print("Listening for INPUT packets on UDP://%s:%s" % (self.ip, self.portnum))

def init(mpstate):
	"""Initialise module."""
	return position_ft_sensor_module(mpstate)