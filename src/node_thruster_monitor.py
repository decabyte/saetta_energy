#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import time

import rospy
import roslib
roslib.load_manifest('saetta_energy')

from vehicle_core.model import throttle_model as th
from vehicle_core.model import thruster_model as tm
from vehicle_core.config import thrusters_config as tc

from vehicle_interface.msg import ThrusterCommand, ThrusterFeedback, FloatArrayStamped

# constants
TOPIC_REQ = 'thrusters/commands'
TOPIC_MODEL = 'thrusters/model'
TOPIC_REAL = 'thrusters/status'
TOPIC_DIAG = 'thrusters/diagnostics'

TOPIC_ENERGY_REAL = 'thrusters/energy/real'
TOPIC_ENERGY_MODEL = 'thrusters/energy/model'

# configs
SAMPLE_WINDOW = 20          # samples
SAMPLE_TIME = 0.1           # secs

PUBLISH_RATE = 10           # Hz
NOMINAL_VOLTAGE = 28.0      # assumed constant


class ThrustersMonitor(object):
    def __init__(self, name, topic_input_req, topic_input_real, sample_window, **kwargs):
        self.name = name
        self.rate = rospy.Rate(PUBLISH_RATE)

        self.sample_window = sample_window
        self.real_current = np.zeros((6, sample_window))
        self.model_current = np.zeros((6, sample_window))
        self.integrated_error = np.zeros(6)

        self.throttle_request = np.zeros((6, tc.LPF_WINDOW))
        self.last_throttle = np.zeros(6)
        self.predicted_throttle = np.zeros(6)

        #self.limit_rate = bool(kwargs.get('limit_rate', False))
        self.rising_limit = float(kwargs.get('rising_limit', tc.THROTTLE_RISING_LIMIT))
        self.falling_limit = float(kwargs.get('falling_limit', tc.THROTTLE_FALLING_LIMIT))
        self.model_delay = int(kwargs.get('model_delay', 0))

        # subscribers
        self.sub_req = rospy.Subscriber(topic_input_req, ThrusterCommand, self.handle_req, tcp_nodelay=True, queue_size=3)
        self.sub_status = rospy.Subscriber(topic_input_real, ThrusterFeedback, self.handle_status, tcp_nodelay=True, queue_size=3)

        # publishers
        self.pub_model = rospy.Publisher(TOPIC_MODEL, ThrusterFeedback, tcp_nodelay=True, queue_size=1)
        self.pub_error = rospy.Publisher(TOPIC_DIAG, FloatArrayStamped, tcp_nodelay=True, queue_size=1)

        # self.sub_model = rospy.Subscriber(topic_input_model, ThrusterFeedback, self.handle_model, queue_size=1)
        # self.t_energy = rospy.Timer(rospy.Duration(1.0), self.callback_energy)
        # self.pub_real_en = rospy.Publisher(TOPIC_ENERGY_REAL, FloatArrayStamped, queue_size=1)
        # self.pub_model_en = rospy.Publisher(TOPIC_ENERGY_MODEL, FloatArrayStamped, queue_size=1)
        # self.real_energy = np.zeros((6, 1))
        # self.model_energy = np.zeros((6, 1))


    def send_diagnostics(self):
        msg = FloatArrayStamped()
        msg.header.stamp = rospy.Time.now()
        msg.values = self.integrated_error
        self.pub_error.publish(msg)

    def send_model(self):
        msg = ThrusterFeedback()
        msg.header.stamp = rospy.Time.now()
        msg.throttle = self.predicted_throttle
        msg.current = self.model_current[:, -1]
        self.pub_model.publish(msg)


    def run(self):
        while not rospy.is_shutdown():
            # messaging
            self.send_diagnostics()
            self.send_model()

            self.rate.sleep()


    def handle_req(self, data):
        # parse input request
        self.throttle_request[:, -1] = np.array(data.throttle)

        # predict throttle
        self.predicted_throttle = th.predict_throttle(
            self.throttle_request, tc.LPF[0], tc.LPF[1], self.model_delay, tc.MAX_THROTTLE
        )

        # shift input buffer and zero the new entry (in case no further messages are sent from the controller)
        self.throttle_request = np.roll(self.throttle_request, -1, axis=1)
        self.throttle_request[:, -1] = np.zeros(6)

        # rate limiter
        self.predicted_throttle = tm.rate_limiter(
            self.predicted_throttle, self.last_throttle,
            rising_limit=tc.THROTTLE_RISING_LIMIT, falling_limit=tc.THROTTLE_FALLING_LIMIT
        )

        # store previous throttle value
        self.last_throttle = self.predicted_throttle

        # store predicted current
        self.model_current = np.roll(self.model_current, -1, axis=1)
        self.model_current[:, -1] = tm.estimate_current(self.predicted_throttle, tc.THROTTLE_TO_CURRENT)

        # compensate for time delays (warning)
        self.integrated_error = np.trapz(self.model_current[:, 0:-2] - self.real_current[:, 1:-1], dx=SAMPLE_TIME)

        # shift status buffer and zero the new entry (in case no further messages are sent from the controller)
        self.real_current = np.roll(self.real_current, -1, axis=1)
        self.real_current[:, -1] = np.zeros(6)


    def handle_status(self, data):
        # parse sensor data
        self.real_current[:, -1] = np.array(data.current)



    # def handle_model(self, msg):
    #     self.model_current = np.roll(self.model_current, -1, axis=1)
    #     self.model_current[:, -1] = msg.current
    #
    #     # calculate the energy for model (using the last two samples)
    #     #self.model_energy[0:6, 0] += np.trapz(self.model_current[:, -2:], dx=SAMPLE_TIME) * NOMINAL_VOLTAGE
    #
    # def handle_status(self, msg):
    #     self.real_current = np.roll(self.real_current, -1, axis=1)
    #     self.real_current[:, -1] = msg.current
    #
    #     # calculate the energy for real (using the last two samples)
    #     #self.real_energy[0:6, 0] += np.trapz(self.real_current[:, -2:], dx=SAMPLE_TIME) * NOMINAL_VOLTAGE


    # def callback_energy(self, event):
    #     # send messages
    #     msg = FloatArrayStamped()
    #     msg.header.stamp = rospy.Time.now()
    #     msg.values = self.real_energy.flatten().tolist()
    #     self.pub_real_en.publish(msg)
    #
    #     msg = FloatArrayStamped()
    #     msg.header.stamp = rospy.Time.now()
    #     msg.values = self.model_energy.flatten().tolist()
    #     self.pub_model_en.publish(msg)



def main():
    rospy.init_node('thrusters_monitor')
    name = rospy.get_name()
    rospy.loginfo('%s initializing ...', name)

    topic_input_req = rospy.get_param('~topic_input_req', TOPIC_REQ)
    topic_input_real = rospy.get_param('~topic_input_real', TOPIC_REAL)
    sample_window = int(rospy.get_param('~sample_window', SAMPLE_WINDOW))
    # topic_energy_model = rospy.get_param('~topic_energy_model', MODEL_ENERGY_TOPIC)
    # topic_energy_real = rospy.get_param('~topic_energy_real', REAL_ENERGY_TOPIC)
    # topic_diagnostics = rospy.get_param('~topic_diagnostics', DIAGNOSTICS_TOPIC)

    rospy.loginfo('%s: monitor init ... ', name)
    rospy.loginfo('%s: topic input req: %s', rospy.get_name(), topic_input_req)
    rospy.loginfo('%s: topic input real: %s', rospy.get_name(), topic_input_real)
    rospy.loginfo('%s: sample window: %s', rospy.get_name(), sample_window)
    # rospy.loginfo('%s: topic energy model: %s', rospy.get_name(), topic_energy_model)
    # rospy.loginfo('%s: topic energy real: %s', rospy.get_name(), topic_energy_real)
    # rospy.loginfo('%s: topic diagnostics: %s', rospy.get_name(), topic_diagnostics)

    # model config
    config = rospy.get_param('thruster_model', dict())

    tm = ThrustersMonitor(name, topic_input_req, topic_input_real, sample_window, **config)

    try:
        tm.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('%s shutdown requested ...', name)


if __name__ == '__main__':
    main()

