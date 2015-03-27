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
from vehicle_core.control import thrust_allocation as ta
from vehicle_core.config import thrusters_config as tc

from vehicle_interface.msg import ThrusterCommand, ThrusterFeedback, FloatArrayStamped, Vector6Stamped

# constants
TOPIC_REQ = 'thrusters/commands'
TOPIC_MODEL = 'thrusters/model'
TOPIC_REAL = 'thrusters/status'

TOPIC_DIAG = 'thrusters/diagnostics'
TOPIC_EFF = 'thrusters/efficiency'
TOPIC_SPD = 'pilot/speeds'
TOPIC_DOF = 'thrusters/dof'

# configs
RATE_MONITOR = 10           # rate of the diagnostic loop (Hz)
RATE_MODEL = 10             # rate of reporting from simulated thrusters (Hz)
RATE_DIAG = 5               # rate of reporting from diagnostic metric (Hz)
RATE_PILOT = 2              # rate of reporting from pilot node (Hz)

WIN_CURRENTS = 12           # samples (1 sec + two samples for fixing ROS delay)
WIN_FILTER = 20             # samples
SAMPLE_TIME = 0.1           # secs
NOMINAL_VOLTAGE = 28.0      # assumed constant

# diagnostic parameters
THRESH_METRIC = 25.0 * np.ones(6)                                   # threshold for thrusters diagnostic metric
EXC_EFF = 0.1 * np.ones(6)                                          # thruster exclusion threshold (% of reference)
COEFF_ADJ_DEC = 0.01                                                 # thruster weight adaptation rate (decrease)
COEFF_ADJ_INC = COEFF_ADJ_DEC / 1000.0                              # thruster weight adaptation rate (increase)
MAX_SPEED = np.array([2.0, 1.0, 1.0, 0.0, 3.0, 3.0])                # default max speed (m/s and rad/s)

# References:
#   [1]: http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers


class ThrustersMonitor(object):
    def __init__(self, name, topic_input_req, topic_input_real, win_curr, **kwargs):
        self.name = name
        self.rate = rospy.Rate(RATE_MONITOR)

        # config
        config_model = rospy.get_param('thruster_model', dict())

        self.limit_rate = bool(config_model.get('limit_rate', False))
        self.model_delay = int(config_model.get('model_delay', 0))
        self.rising_limit = float(config_model.get('rising_limit', tc.THROTTLE_RISING_LIMIT))
        self.falling_limit = float(config_model.get('falling_limit', tc.THROTTLE_FALLING_LIMIT))

        # thrust allocation
        self.local_TAM = np.copy(tc.TAM)
        self.local_inv_TAM = np.copy(tc.inv_TAM)

        self.max_forces = np.copy(tc.MAX_U)
        self.max_forces[self.max_forces <= 0.0] = 1000.0        # remove zeros (avoid illegal division)

        # load config
        diag_config = rospy.get_param('saetta/diagnostics', dict())
        pilot_config = rospy.get_param('pilot', dict())

        self.threshold_metric = np.array(diag_config.get('threshold_metric', THRESH_METRIC.tolist()))
        self.exclusion_efficiency = np.array(diag_config.get('exclusion_efficiency', EXC_EFF.tolist()))
        self.coeff_adj_dec = float(diag_config.get('coeff_adj_dec', COEFF_ADJ_DEC))
        self.coeff_adj_inc = float(diag_config.get('coeff_adj_inc', COEFF_ADJ_INC))
        self.max_speed = np.array(pilot_config.get('max_speed', MAX_SPEED.tolist()))

        # outputs
        self.diagnostic_metric = np.zeros(6, dtype=np.float64)
        self.thruster_efficiency = np.ones(6, dtype=np.float64)
        self.pilot_speed = np.ones(6, dtype=np.float64)
        self.dof_availability = np.ones(6, dtype=np.float64)

        # internal state
        self.win_curr = win_curr
        self.real_current = np.zeros((6, win_curr), dtype=np.float64)
        self.model_current = np.zeros((6, win_curr), dtype=np.float64)

        self.exp_weights = np.exp(np.linspace(-1.0, 0.0, WIN_FILTER))
        self.exp_weights = self.exp_weights / np.sum(self.exp_weights)

        self.throttle_request = np.zeros((6, tc.LPF_WINDOW), dtype=np.float64)
        self.last_throttle = np.zeros(6, dtype=np.float64)
        self.predicted_throttle = np.zeros(6, dtype=np.float64)
        self.diagnostic_vector = np.zeros((6, WIN_FILTER), dtype=np.float64)

        # subscribers
        self.sub_req = rospy.Subscriber(topic_input_req, ThrusterCommand, self.handle_req, tcp_nodelay=True, queue_size=3)
        self.sub_status = rospy.Subscriber(topic_input_real, ThrusterFeedback, self.handle_status, tcp_nodelay=True, queue_size=3)

        # publishers
        self.pub_model = rospy.Publisher(TOPIC_MODEL, ThrusterFeedback, tcp_nodelay=True, queue_size=10)
        self.pub_diag = rospy.Publisher(TOPIC_DIAG, FloatArrayStamped, tcp_nodelay=True, queue_size=10)
        self.pub_eff = rospy.Publisher(TOPIC_EFF, FloatArrayStamped, tcp_nodelay=True, queue_size=10)
        self.pub_spd = rospy.Publisher(TOPIC_SPD, FloatArrayStamped, tcp_nodelay=True, queue_size=10)
        self.pub_dof = rospy.Publisher(TOPIC_DOF, Vector6Stamped, tcp_nodelay=True, queue_size=10)

        # timers
        self.t_mod = rospy.Timer(rospy.Duration(1.0 / RATE_MODEL), self.send_model)
        self.t_pil = rospy.Timer(rospy.Duration(1.0 / RATE_PILOT), self.send_pilot)
        self.t_dia = rospy.Timer(rospy.Duration(1.0 / RATE_DIAG), self.send_diagnostics)

        # TODO: add a reload configuration service to this node
        #self.t_log = rospy.Timer(rospy.Duration(5.0), self.print_console)


    def send_diagnostics(self, event=None):
        msg = FloatArrayStamped()
        msg.header.stamp = rospy.Time.now()
        msg.values = self.diagnostic_metric
        self.pub_diag.publish(msg)

    def send_pilot(self, event=None):
        # prepare estimates
        self.update_estimates()
        now = rospy.Time.now()

        # estimated efficiency
        msg = FloatArrayStamped()
        msg.header.stamp = now
        msg.values = self.thruster_efficiency
        self.pub_eff.publish(msg)

        # recommended speed limits
        msg = FloatArrayStamped()
        msg.header.stamp = now
        msg.values = self.pilot_speed
        self.pub_spd.publish(msg)

        # estimated dof availability
        msg = Vector6Stamped()
        msg.header.stamp = now
        msg.values = self.dof_availability
        self.pub_dof.publish(msg)

    def send_model(self, event=None):
        msg = ThrusterFeedback()
        msg.header.stamp = rospy.Time.now()
        msg.throttle = self.predicted_throttle
        msg.current = self.model_current[:, -1]
        self.pub_model.publish(msg)


    def handle_req(self, data):
        # parse input request
        self.throttle_request[:, -1] = np.array(data.throttle, dtype=np.float64)
        self.update_model()

    def handle_status(self, data):
        # parse sensor data
        self.real_current[:, -1] = np.array(data.current, dtype=np.float64)


    def update_model(self):
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
            rising_limit=self.rising_limit, falling_limit=self.falling_limit
        )

        # store previous throttle value
        self.last_throttle = self.predicted_throttle

        # store predicted current
        self.model_current = np.roll(self.model_current, -1, axis=1)
        self.model_current[:, -1] = tm.estimate_current(self.predicted_throttle, tc.THROTTLE_TO_CURRENT)


    def update_diagnostics(self):
        # fix ROS delay
        # a, b = 2, self.win_curr
        # c, d = 0, -2

        # ignore ROS delay
        a, b = 0, self.win_curr
        c, d = 0, self.win_curr

        # calculate the energies over the sample window
        self.model_energy = np.trapz(self.model_current[:, a:b], dx=SAMPLE_TIME, axis=1) * NOMINAL_VOLTAGE
        self.real_energy = np.trapz(self.real_current[:, c:d], dx=SAMPLE_TIME, axis=1) * NOMINAL_VOLTAGE

        #rospy.loginfo('%s: delta: %s', self.name, self.model_energy - self.real_energy)

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


        ### update_efficiency(self):
        indexes = np.where(self.diagnostic_metric > self.threshold_metric)[0]

        # update the efficiency and costs
        self.thruster_efficiency[indexes] -= self.coeff_adj_dec                     # reduce thruster efficiency
        self.thruster_efficiency += self.coeff_adj_inc                              # increase thruster efficiency

        self.thruster_efficiency = np.clip(self.thruster_efficiency, 0.0, 1.0)      # prevent negative values

        # check if is better to exclude inefficient thrusters
        # TODO: this is locking a broken thruster (is that the behaviour we want?)
        if np.any(self.thruster_efficiency <= self.exclusion_efficiency):
            idx_disable = np.where(self.thruster_efficiency <= self.exclusion_efficiency)[0]
            self.thruster_efficiency[idx_disable] = 0


    def update_estimates(self):
        ### update_speeds(self):
        self.local_inv_TAM = ta.tam_weighted_inverse(self.local_TAM, self.thruster_efficiency)
        self.available_forces = ta.evaluate_max_force(self.local_inv_TAM)

        # avoid divisions by zero
        for dof in np.argwhere(tc.MAX_U != 0):
            ratio = np.clip(self.available_forces[dof] / tc.MAX_U[dof], 0.0, 1.0)
            self.pilot_speed[dof] = self.max_speed * np.power(ratio, 1.0 / 2.0)

        self.pilot_speed = np.clip(self.pilot_speed, 0, self.max_speed)

        ### update_dof(self):
        ratio_forces = np.clip(self.available_forces / self.max_forces, 0.0, 1.0)
        ratio_tam = np.ones(6)

        for dof in xrange(tc.TAM.shape[0]):
            thrusters_on_dof = tc.TAM[dof, :]
            idx = np.argwhere(thrusters_on_dof != 0.0).flatten()

            if len(idx) != 0:
                ratio_tam[dof] = np.sum(self.thruster_efficiency[idx]) / len(idx)

        ratio_tam = np.clip(ratio_tam, 0.0, 1.0)

        self.dof_availability = np.minimum(ratio_forces, ratio_tam)


    def run(self):
        while not rospy.is_shutdown():
            self.update_diagnostics()
            self.rate.sleep()


    def print_console(self, event=None):
        rospy.loginfo('%s: efficiency: %s', self.name, self.thruster_efficiency)
        rospy.loginfo('%s: speeds: %s', self.name, self.pilot_speed)
        rospy.loginfo('%s: dofs: %s', self.name, self.dof_availability)


def main():
    rospy.init_node('thrusters_monitor')
    name = rospy.get_name()
    rospy.loginfo('%s initializing ...', name)

    # config
    samples_current = int(rospy.get_param('~samples_current', WIN_CURRENTS))
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
    tm = ThrustersMonitor(name, topic_input_req, topic_input_real, samples_current)

    try:
        tm.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('%s shutdown requested ...', name)


if __name__ == '__main__':
    main()
