#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division

import rospy
import roslib
roslib.load_manifest('saetta_energy')

import numpy as np

from saetta_energy.msg import GlobalEnergy, SubsystemEnergy
from auv_msgs.msg import ThrusterForce


# constants
DEFAULT_SEED = 42
DEFAULT_MA = 10					# samples
DEFAULT_RATE = 1.0				# sec
DEFAULT_BAT = 1800.0 * 3600		# Wh * sec/h = Joules

# topics
TOPIC_ENERGY_GLOBAL = 'saetta/energy/global'
TOPIC_SIM_THRUST = 'motors/thrust_req'

# init random number generator
np.random.seed(DEFAULT_SEED)


class EnergySimulator(object):

	def __init__(self, sim_rate=DEFAULT_RATE, bat_init=DEFAULT_BAT):

		# simulation params
		self.mu_v = np.array([24.0, 24.0, 24.0, 12.0, 12.0])
		self.sigma_v = np.array([0.02, 0.02, 0.04, 0.05, 0.05])
		self.mu_a = np.array([0.6, 0.6, 1.5, 1.1, 1.1])
		self.sigma_a = np.array([0.1, 0.1, 0.5, 0.2, 0.2])

		# simulation config
		self.sim_dt = sim_rate						# integration time (sec)
		self.sim_sensors = np.size(self.mu_a)		# simulated sensors

		# simulation data
		self.V_sim = np.zeros((self.sim_sensors, 1))		# voltages calculated
		self.A_sim = np.zeros((self.sim_sensors, 1))		# currents calculated

		# thruster commands
		self.v_thrusters = 0.0
		self.a_thrusters = 0.0
		self.thrust_time = 0.0

		# power averaging
		self.W_sim = np.zeros((self.sim_sensors, 1))

		# energy tracking
		self.E_sim = np.zeros((self.sim_sensors, 1))		# energy counters
		self.E_thrusters = 0.0
		self.E_total = 0.0

		# battery info
		self.bat_full = bat_init		# joule
		self.bat_real = bat_init		# joule
		self.bat_level = 0.0			# percentage

		# ros interface
		self.pub_report = rospy.Publisher(TOPIC_ENERGY_GLOBAL, GlobalEnergy, tcp_nodelay=True, latch=True)
		self.sub_thrust = rospy.Subscriber(TOPIC_SIM_THRUST, ThrusterForce, self.handle_thrust)

		# timers
		self.t_sim = rospy.Timer(rospy.Duration(1, 0), self.handle_simulated)
		self.t_report = rospy.Timer(rospy.Duration(1,0), self.handle_report)
		self.t_status = rospy.Timer(rospy.Duration(5, 0), self.print_status)


	def print_status(self, event):
		# update battery levels
		self.bat_real = self.bat_full - self.E_total
		self.bat_level = (self.bat_real / self.bat_full) * 100

		rospy.loginfo('[%s]: te: %.2f J, re: %.2f J, bl: %.2f %%', rospy.get_name(), self.E_total, self.bat_real, self.bat_level)


	def handle_thrust(self, data):
		# calculate thrusters time and current usage
		t = rospy.Time.now().to_sec()
		self.thrust_dt = t - self.thrust_time		# integration time
		self.thrust_time = t

		if self.thrust_dt > 10.0:
			rospy.logerr('dt: %s', self.thrust_dt)
			return

		# thruster linear model
		self.thrust_amps = np.abs(np.array(data.forces)) / 10.0

		# simulate readings
		self.v_thrusters = 28.0
		self.a_thrusters = np.sum(self.thrust_amps)

		# calculate thruster energy
		self.E_thrusters += self.v_thrusters * self.a_thrusters * self.thrust_dt


	def handle_simulated(self, event):
		"""Generate simulated current and voltage offsets using Gaussian model
		"""
		for s in xrange(self.sim_sensors):
			self.V_sim[s] = (self.sigma_v[s] * np.random.randn() + self.mu_v[s])
			self.A_sim[s] = (self.sigma_a[s] * np.random.randn() + self.mu_a[s])

			# calculate energy
			self.E_sim[s] += self.V_sim[s] * self.A_sim[s] * self.sim_dt


	def handle_report(self, event):
		# update average power
		#np.roll(self.W_thrusters, -1)	# shift array
		#self.W_thrusters[-1] = self.v_thrusters * self.a_thrusters

		# update status
		self.E_total = self.E_thrusters #+ np.sum(self.E_sim, axis=0)

		# publish message
		gb = GlobalEnergy()
		gb.header.stamp = rospy.Time.now()
		gb.energy_used = self.E_total
		gb.subsystems = []

		# add subsystems
		se = SubsystemEnergy(sid='thrusters', energy_used=self.E_thrusters)
		gb.subsystems.append(se)

		for s in xrange(self.sim_sensors):
			se = SubsystemEnergy()
			se.sid = 'sensor-{0}'.format(s)
			se.energy_used = self.E_sim[s]
			gb.subsystems.append(se)

		self.pub_report.publish(gb)


if __name__ == '__main__':
	rospy.init_node('saetta_simulator')
	rospy.loginfo('%s is spinning ...', rospy.get_name())

	# load parameters
	rate = rospy.get_param('saetta/simulator/rate', DEFAULT_RATE)		# simulated sensors rate (sec)
	level = rospy.get_param('saetta/simulator/battery', DEFAULT_BAT)	# simulated battery level (joules)

	# run the simulation
	try:
		es = EnergySimulator(sim_rate=rate, bat_init=level)
		rospy.spin()
	except rospy.ROSInterruptException as ri:
		rospy.logerr('[%s]: caught exception: %s', rospy.get_name(), str(ri))
	else:
		rospy.loginfo('[%s]: clean shutdown ...', rospy.get_name())
