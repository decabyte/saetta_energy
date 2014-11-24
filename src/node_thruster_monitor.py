#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('saetta_energy')

from vehicle_core.model import throttle_model as th
from vehicle_core.model import thruster_model as tm
from vehicle_core.config import thrusters_config as tc
from vehicle_core.util import kalman

from vehicle_interface.msg import ThrusterCommand, ThrusterFeedback, FloatArrayStamped

# constants
TOPIC_REQ = 'thrusters/commands'
TOPIC_MODEL = 'thrusters/model'
TOPIC_REAL = 'thrusters/status'
TOPIC_DIAG = 'thrusters/diagnostics'

# TOPIC_ENERGY_REAL = 'thrusters/energy/real'
# TOPIC_ENERGY_MODEL = 'thrusters/energy/model'

# configs
RATE_MONITOR = 10           # Hz
WIN_CURR = 12               # samples (1 sec + two samples for fixing ROS delay)
WIN_DIAG = 20               # samples
SAMPLE_TIME = 0.1           # secs
NOMINAL_VOLTAGE = 28.0      # assumed constant


class ThrustersMonitor(object):
    def __init__(self, name, topic_input_req, topic_input_real, win_curr, **kwargs):
        self.name = name
        self.rate = rospy.Rate(RATE_MONITOR)

        self.win_curr = win_curr
        self.real_current = np.zeros((6, win_curr))
        self.model_current = np.zeros((6, win_curr))

        self.diagnostic_vector = np.zeros((6, WIN_DIAG))
        self.diagnostic_metric = np.zeros(6)

        self.exp_weights = np.exp(np.linspace(-1.0, 0.0, WIN_DIAG))
        self.exp_weights = self.exp_weights / np.sum(self.exp_weights)

        self.throttle_request = np.zeros((6, tc.LPF_WINDOW))
        self.last_throttle = np.zeros(6)
        self.predicted_throttle = np.zeros(6)

        self.limit_rate = bool(kwargs.get('limit_rate', False))
        self.model_delay = int(kwargs.get('model_delay', 0))
        self.rising_limit = float(kwargs.get('rising_limit', tc.THROTTLE_RISING_LIMIT))
        self.falling_limit = float(kwargs.get('falling_limit', tc.THROTTLE_FALLING_LIMIT))


        # # kalman filter
        # self.n_state = 1
        # self.n_observation = 2
        #
        # self.kf = kalman.KalmanFilter(self.n_state, self.n_observation)
        # self.kf.H[0,0] = 1    # measurement matrix should have ones in order to use all the sensors
        # self.kf.H[1,0] = 1
        # self.kf.Q[0,0] = 1000.0
        # self.kf.R[0,0] = 1.0
        # self.kf.R[1,1] = 1.0
        #
        # self.n = 0
        # self.data = np.zeros((3, 100))

        # subscribers
        self.sub_req = rospy.Subscriber(topic_input_req, ThrusterCommand, self.handle_req, tcp_nodelay=True, queue_size=10)
        self.sub_status = rospy.Subscriber(topic_input_real, ThrusterFeedback, self.handle_status, tcp_nodelay=True, queue_size=10)

        # publishers
        self.pub_model = rospy.Publisher(TOPIC_MODEL, ThrusterFeedback, tcp_nodelay=True, queue_size=10)
        self.pub_diag = rospy.Publisher(TOPIC_DIAG, FloatArrayStamped, tcp_nodelay=True, queue_size=10)


    def send_diagnostics(self):
        msg = FloatArrayStamped()
        msg.header.stamp = rospy.Time.now()
        msg.values = self.diagnostic_metric
        self.pub_diag.publish(msg)

    def send_model(self):
        msg = ThrusterFeedback()
        msg.header.stamp = rospy.Time.now()
        msg.throttle = self.predicted_throttle
        msg.current = self.model_current[:, -1]
        self.pub_model.publish(msg)


    def run(self):
        while not rospy.is_shutdown():
            # predict throttle
            self.predicted_throttle = th.predict_throttle(
                self.throttle_request, b=tc.LPF[0], a=tc.LPF[1], offset=self.model_delay, limit=tc.MAX_THROTTLE
            )

            # shift input buffer and zero the new entry (in case no further messages are sent from the controller)
            self.throttle_request = np.roll(self.throttle_request, -1, axis=1)
            #self.throttle_request[:, -1] = np.zeros(6)

            # rate limiter
            self.predicted_throttle = th.rate_limiter(
                self.predicted_throttle, self.last_throttle,
                rising_limit=tc.THROTTLE_RISING_LIMIT, falling_limit=tc.THROTTLE_FALLING_LIMIT
            )

            # store previous throttle value
            self.last_throttle = self.predicted_throttle

            # store predicted current
            self.model_current = np.roll(self.model_current, -1, axis=1)
            self.model_current[:, -1] = tm.estimate_current(self.predicted_throttle, tc.THROTTLE_TO_CURRENT)

            # calculate the energies over the sample window (fix for ROS delay)
            self.model_energy = np.trapz(self.model_current[:, 2:], dx=SAMPLE_TIME, axis=1) * NOMINAL_VOLTAGE
            self.real_energy = np.trapz(self.real_current[:, :-2], dx=SAMPLE_TIME, axis=1) * NOMINAL_VOLTAGE

            # calculate diagnostic metric (without including delays)
            self.diagnostic_vector[:,-1] = (self.model_energy - self.real_energy)

            # exponential moving average filter
            for n in xrange(self.diagnostic_vector.shape[0]):
                self.diagnostic_metric[n] = np.convolve(self.diagnostic_vector[n,:], self.exp_weights, mode='same')[-1]

            # advance filter
            self.diagnostic_vector = np.roll(self.diagnostic_vector, -1, axis=1)

            # shift status buffer and zero the new entry (in case no further messages are sent from the controller)
            self.real_current = np.roll(self.real_current, -1, axis=1)
            #self.real_current[:, -1] = np.zeros(6)

            self.send_diagnostics()
            self.send_model()

            self.rate.sleep()

            ## data fusion (using kalman filter)
            # measurements = np.array([
            #     # self.real_current[0, -1],
            #     # self.model_current[0, -1],
            #
            #     self.model_current[0, -1],
            #     self.real_current[0, -1]
            # ])
            # mean, cov = self.kf.update(measurements)
            #
            # print('mean: %s' % mean)
            # print('cov: %s' % cov)
            #
            # # store data
            # self.data[0, self.n] = self.model_current[0, -1]
            # self.data[1, self.n] = self.real_current[0, -1]
            # self.data[2, self.n] = mean
            #
            # self.n += 1
            #
            # if self.n >= self.data.shape[1]:
            #     print('kf plot')
            #
            #     import matplotlib.pyplot as plt
            #
            #     t = np.linspace(0, 10, self.data.shape[1])
            #
            #     plt.figure()
            #     plt.plot(t, self.data[0,:], 'b--', label='sensor #1')
            #     plt.plot(t, self.data[1,:], 'g--', label='sensor #2')
            #     plt.plot(t, self.data[2,:], 'r', label='fusion')
            #     plt.legend()
            #     plt.show()
            #
            #     rospy.signal_shutdown('kf end')

    def handle_req(self, data):
        # parse input request
        self.throttle_request[:, -1] = np.array(data.throttle)

    def handle_status(self, data):
        # parse sensor data
        self.real_current[:, -1] = np.array(data.current)


def main():
    rospy.init_node('thrusters_monitor')
    name = rospy.get_name()
    rospy.loginfo('%s initializing ...', name)

    # config
    config = rospy.get_param('thruster_model', dict())
    samples_current = int(rospy.get_param('~samples_current', WIN_CURR))

    topic_input_req = rospy.get_param('~topic_input_req', TOPIC_REQ)
    topic_input_real = rospy.get_param('~topic_input_real', TOPIC_REAL)
    topic_diagnostics = rospy.get_param('~topic_diagnostics', TOPIC_DIAG)

    # show configuration
    rospy.loginfo('%s: monitor init ... ', name)
    rospy.loginfo('%s: topic input req: %s', rospy.get_name(), topic_input_req)
    rospy.loginfo('%s: topic input real: %s', rospy.get_name(), topic_input_real)
    rospy.loginfo('%s: topic diagnostics: %s', rospy.get_name(), topic_diagnostics)
    rospy.loginfo('%s: current samples: %s', rospy.get_name(), samples_current)

    # init monitor
    tm = ThrustersMonitor(name, topic_input_req, topic_input_real, samples_current, **config)

    try:
        tm.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('%s shutdown requested ...', name)


if __name__ == '__main__':
    main()
