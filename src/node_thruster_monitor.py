#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

from pykalman import KalmanFilter

import rospy
import roslib
roslib.load_manifest('saetta_energy')

from vehicle_interface.msg import ThrusterFeedback, FloatArrayStamped
# import thrusters_config as tc
# import thrusters_model as tm

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

OFFSET_MODEL = 0
OFFSET_REAL = 1


class ThrustersMonitor(object):
    def __init__(self, name, topic_input_model, topic_input_real, sample_window):
        self.name = name

        self.sample_window = sample_window
        self.real_current = np.zeros((6, sample_window))
        self.model_current = np.zeros((6, sample_window))
        self.integrated_error = np.zeros((6, 1))

        # data fusion using kalman filter
        self.n_dim_state = 1
        self.n_dim_obs = 2
        self.kf_Q = 1.0
        self.kf_R = np.array([
            [1.0, 0.0],
            [0.0, 0.1]
        ])

        self.kf_mean = np.zeros(self.n_dim_state)
        self.kf_covariances = np.zeros((self.n_dim_state, self.n_dim_state))
        self.measurements = np.zeros(self.n_dim_obs)

        self.kf = KalmanFilter(
            n_dim_state=self.n_dim_state,
            n_dim_obs=self.n_dim_obs,
            transition_covariance=self.kf_Q,
            observation_covariance=self.kf_R
        )

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

        self.x = np.zeros((3, 100))
        self.t = np.linspace(0, 10, 100)
        self.n = 0


        while not rospy.is_shutdown():

            # observer filtering
            #means, covariances = kf.filter(measurements)
            self.measurements[0] = self.model_current[0, -2]
            self.measurements[1] = self.real_current[0, -1]

            self.kf_mean, self.kf_covariances = self.kf.filter_update(
                self.kf_mean, self.kf_covariances, self.measurements
            )

            # compensate for time delays (warning)
            delta = self.model_current[:, 0:-2] - self.real_current[:, 1:-1]
            self.integrated_error = np.trapz(delta, dx=SAMPLE_TIME)

            self.send_error_msg()
            self.l_rate.sleep()

            # debug
            self.kf.observation_covariance[0,0] = np.clip(delta[0,-1] * 10, 2.0, 20.0)

            print(' err: %s\n  mean: %s\n  cov: %s\n  R: %s\n' % (
                self.integrated_error[0],
                self.kf_mean,
                self.kf_covariances,
                self.kf.observation_covariance
            ))

            self.x[0, self.n] = self.measurements[0]
            self.x[1, self.n] = self.measurements[1]
            self.x[2, self.n] = self.kf_mean
            self.n += 1

            if self.n > 99:
                break



        import matplotlib.pyplot as plt

        fig, ax = plt.subplots()
        ax.plot(self.t, self.x[0, :], 'b--')
        ax.plot(self.t, self.x[1, :], 'g--')
        ax.plot(self.t, self.x[2, :], 'r')
        ax.legend(['model', 'measurement', 'fusion'])
        plt.show()


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

