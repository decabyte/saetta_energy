#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('saetta_energy')

# import thrusters_config as tc
# import thrusters_model as tm

from vehicle_interface.msg import ThrusterFeedback, FloatArrayStamped

# constants
TOPIC_MODEL = 'thrusters/model'
TOPIC_REAL = 'thrusters/status'
TOPIC_DIAG = 'thrusters/diagnostics'

TOPIC_ENERGY_REAL = 'thrusters/energy/real'
TOPIC_ENERGY_MODEL = 'thrusters/energy/model'
TS_ENERGY = 1

PUBLISH_RATE = 5            # Hz
NOMINAL_VOLTAGE = 28.0      # assumed constant

# configs
SAMPLE_WINDOW = 20  # samples
SAMPLE_TIME = 0.1   # secs


class ThrustersMonitor(object):
    def __init__(self, name, topic_input_model, topic_input_real, sample_window):
        self.name = name

        self.sample_window = sample_window
        self.real_current = np.zeros((6, sample_window))
        self.model_current = np.zeros((6, sample_window))
        self.integrated_error = np.zeros((6, 1))

        # subscribers
        self.sub_model = rospy.Subscriber(topic_input_model, ThrusterFeedback, self.handle_model, queue_size=1)
        self.sub_status = rospy.Subscriber(topic_input_real, ThrusterFeedback, self.handle_status, queue_size=1)

        # publishers
        self.pub_error = rospy.Publisher(TOPIC_DIAG, FloatArrayStamped, queue_size=1)

        # rates
        self.l_rate = rospy.Rate(PUBLISH_RATE)

        self.t_energy = rospy.Timer(rospy.Duration(TS_ENERGY), self.callback_energy)
        self.pub_real_en = rospy.Publisher(TOPIC_ENERGY_REAL, FloatArrayStamped, queue_size=1)
        self.pub_model_en = rospy.Publisher(TOPIC_ENERGY_MODEL, FloatArrayStamped, queue_size=1)
        self.real_energy = np.zeros((6, 1))
        self.model_energy = np.zeros((6, 1))


    def send_error_msg(self):
        msg = FloatArrayStamped()
        msg.header.stamp = rospy.Time.now()
        msg.values = self.integrated_error.flatten().tolist()
        self.pub_error.publish(msg)


    def run(self):
        # find the maximum possible error in current so it can be used as a reference
        #max_throttle = np.ones(6) * tc.MAX_THROTTLE
        #max_error = SAMPLE_TIME * self.sample_window * tm.thruster_model_current(max_throttle)

        while not rospy.is_shutdown():
            # compensate for time delays (warning)
            self.integrated_error = np.trapz(self.model_current[:, 0:-2] - self.real_current[:, 1:-1], dx=SAMPLE_TIME)  # absolute error
            #self.integrated_error = (self.integrated_error / max_error) * 100                         # normalised error

            self.send_error_msg()
            self.l_rate.sleep()


    # it is assumed that the messages will come synchronously
    def handle_model(self, msg):
        self.model_current = np.roll(self.model_current, -1, axis=1)
        self.model_current[:, -1] = msg.current

        # calculate the energy for model (using the last two samples)
        self.model_energy[0:6, 0] += np.trapz(self.model_current[:, -2:], dx=SAMPLE_TIME) * NOMINAL_VOLTAGE

    def handle_status(self, msg):
        self.real_current = np.roll(self.real_current, -1, axis=1)
        self.real_current[:, -1] = msg.current

        # calculate the energy for real (using the last two samples)
        self.real_energy[0:6, 0] += np.trapz(self.real_current[:, -2:], dx=SAMPLE_TIME) * NOMINAL_VOLTAGE


    def callback_energy(self, event):
        # calculate the total energy usage for the thrusters subsystem
        #   adding an extra field at the end of the energy array
        #self.model_energy[6, 0] = np.sum(self.model_energy[0:6, 0])
        #self.real_energy[6, 0] = np.sum(self.real_energy[0:6, 0])

        # send messages
        msg = FloatArrayStamped()
        msg.header.stamp = rospy.Time.now()
        msg.values = self.real_energy.flatten().tolist()
        self.pub_real_en.publish(msg)

        msg = FloatArrayStamped()
        msg.header.stamp = rospy.Time.now()
        msg.values = self.model_energy.flatten().tolist()
        self.pub_model_en.publish(msg)



def main():
    rospy.init_node('thrusters_monitor')
    name = rospy.get_name()
    rospy.loginfo('%s initializing ...', name)

    topic_input_model = rospy.get_param('~topic_input_model', TOPIC_MODEL)
    topic_input_real = rospy.get_param('~topic_input_real', TOPIC_REAL)
    sample_window = int(rospy.get_param('~sample_window', SAMPLE_WINDOW))
    # topic_energy_model = rospy.get_param('~topic_energy_model', MODEL_ENERGY_TOPIC)
    # topic_energy_real = rospy.get_param('~topic_energy_real', REAL_ENERGY_TOPIC)
    # topic_diagnostics = rospy.get_param('~topic_diagnostics', DIAGNOSTICS_TOPIC)


    rospy.loginfo('%s: monitor init ... ', name)
    rospy.loginfo('%s: topic input model: %s', rospy.get_name(), topic_input_model)
    rospy.loginfo('%s: topic input real: %s', rospy.get_name(), topic_input_real)
    rospy.loginfo('%s: sample window: %s', rospy.get_name(), sample_window)
    # rospy.loginfo('%s: topic energy model: %s', rospy.get_name(), topic_energy_model)
    # rospy.loginfo('%s: topic energy real: %s', rospy.get_name(), topic_energy_real)
    # rospy.loginfo('%s: topic diagnostics: %s', rospy.get_name(), topic_diagnostics)

    tm = ThrustersMonitor(name, topic_input_model, topic_input_real, sample_window)

    try:
        tm.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('%s shutdown requested ...', name)


if __name__ == '__main__':
    main()

