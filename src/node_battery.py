#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import control

# ros imports
import rospy
import roslib
roslib.load_manifest('saetta_energy')

from std_srvs.srv import Empty
from saetta_energy.msg import BatteryStatus
from vehicle_interface.msg import ThrusterFeedback

# topics
TOPIC_THR = 'thrusters/status'
TOPIC_ENE = 'saetta/energy'
TOPIC_BAT = 'saetta/battery'
SRV_RESET = 'saetta/battery/reset'

# defaults
THR_VNOM = 28.0         # volt
THR_FRAME = 0.1         # secs
TS_UPDATE = 1           # secs


class BatteryNode(object):

    def __init__(self, **kwargs):
        self.name = rospy.get_name()
        self.dt = 0.1

        # thrusters
        #self.n_win = int(kwargs.get('n_win', 100))
        #self.thrs_currents = np.zeros(self.n_win)

        # battery model
        self.n_series = int(kwargs.get('n_series', 7))
        self.n_parallel = float(kwargs.get('n_parallel', 4))

        self.emax = float(kwargs.get('emax', 1174824.0))
        self.vmax = float(kwargs.get('vmax', 29.2))
        self.vmin = float(kwargs.get('vmin', 24.5))

        # pack
        self.energy_max = self.emax * self.n_parallel
        self.energy_residual = np.copy(self.energy_max)
        self.energy_used = 0.0

        self.soc = 1.0
        self.vbat = self.vmax
        self.amps = 0.0

        # parameters
        Cb = float(kwargs.get('Cb', 9309.59))
        Re = float(kwargs.get('Re', 105.0e-3))
        Rt = float(kwargs.get('Rt', 98.0e-3))
        Rc = float(kwargs.get('Rc', 0.4e-3))
        Cc = float(kwargs.get('Cc', 203.69))

        self.cbe = Cb

        # system matrices
        A = np.array([
            [-1 / (Cb * (Re + Rc)), 1 / (Cb * (Re + Rc))],
            [1 / (Cc * (Re + Rc)), -1 / (Cc * (Re + Rc))],
        ])

        B = np.array([
            [-Rc / (Cb * (Re + Rc))],
            [(-1 / Cc) + (Rc / (Cc * (Re + Rc)))]
        ])

        C = np.array([
             [Rc / (Re + Rc), Re / (Re + Rc)]
        ])

        D = np.array([
            [- (Rt + ((Rc * Re) / (Re + Rc))) ]
        ])

        # system (single-cell model)
        self.sys = control.ss(A, B, C, D)
        self.dsys = control.matlab.c2d(self.sys, self.dt)

        self.Ad = self.dsys.A
        self.Bd = self.dsys.B
        self.Cd = self.dsys.C
        self.Dd = self.dsys.D

        self.x = np.zeros((A.shape[0], 1))
        self.y = np.zeros((1, 1))
        self.x[:] = self.vmax

        # ros interface
        self.sub_thrs = rospy.Subscriber(TOPIC_THR, ThrusterFeedback, self.handle_thrusters, queue_size=10)
        self.pub_sts = rospy.Publisher(TOPIC_BAT, BatteryStatus, queue_size=10, latch=True)

        # services
        self.srv_reset = rospy.Service(SRV_RESET, Empty, self.handle_reset)

        # timers
        self.t_upd = rospy.Timer(rospy.Duration(TS_UPDATE), self.publish_state)


    def handle_reset(self, data=None):
        rospy.logwarn('%s: resetting estimations', self.name)

        self.x[:] = self.vmax
        self.energy_residual = np.copy(self.energy_max)
        self.energy_used = 0.0

        self.soc = 1.0
        self.vbat = self.vmax
        self.amps = 0.0

        return []

    def handle_thrusters(self, data):
        # selected thrusters only
        self.amps = np.sum(data.current[0:4])

        # discrete-time state space model
        self.u = self.amps / self.n_parallel
        x = np.copy(self.x)

        self.x = np.dot(self.Ad, x) + np.dot(self.Bd, self.u)
        self.y = np.dot(self.Cd, x) + np.dot(self.Dd, self.u)

        # update capacity
        eused = 0.5 * self.cbe * ((self.vmax ** 2) - (self.x[0, 0] ** 2))

        # assume different packs working ideally
        self.energy_used = eused * self.n_parallel
        self.vbat = self.y

        self.energy_residual = self.energy_max - self.energy_used
        self.energy_residual = np.clip(self.energy_residual, 0.0, self.energy_max)

        self.soc = self.energy_residual / self.energy_max
        self.soc = np.clip(self.soc, 0.0, 1.0)

        # rospy.loginfo('%s: state: %s, output: %s', self.name, self.x, self.y)
        # rospy.loginfo('%s: eint: %.3f J, eused: %.3f', self.name, self.eint, self.energy_used)

    def publish_state(self, event=None):
        # ros messages
        res = BatteryStatus()
        res.header.stamp = rospy.Time.now()
        res.energy_max = self.energy_max
        res.energy_used = self.energy_used
        res.energy_residual = self.energy_residual
        res.soc = self.soc
        res.voltage = self.vbat
        res.current = self.amps

        self.pub_sts.publish(res)

def main():
    rospy.init_node('battery_node')
    rospy.loginfo('%s: init ...', rospy.get_name())

    # config
    config = rospy.get_param('saetta/battery', {})

    pm = BatteryNode(**config)
    rospy.spin()

    # while not rospy.is_shutdown():
    #     rospy.sleep(5.0)

if __name__ == '__main__':
    main()
