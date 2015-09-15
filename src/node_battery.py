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
TOPIC_ENE = 'saetta/report'
TOPIC_BAT = 'saetta/battery'
SRV_RESET = 'saetta/battery/reset'

# defaults
THR_VNOM = 28.0         # volt
THR_FRAME = 0.1         # secs
TS_UPDATE = 5           # secs


class BatteryNode(object):

    def __init__(self, **kwargs):
        self.name = rospy.get_name()
        self.dt = 0.1

        # thrusters
        self.n_win = int(kwargs.get('n_win', 100))
        self.thrs_currents = np.zeros(self.n_win)
        self.thrs_idx = 0

        # battery model
        self.n_packs = float(kwargs.get('n_packs', 4))
        self.cell_series = int(kwargs.get('cell_series', 7))

        self.emax = float(kwargs.get('emax', 172800.0))
        self.vmax = float(kwargs.get('vmax', 4.2))
        self.vmin = float(kwargs.get('vmin', 3.1))
        self.cbe = float(kwargs.get('cbe', 43038.0))

        # pack
        self.energy_max = self.emax * self.n_packs
        self.energy_residual = np.copy(self.energy_max)
        self.soc = 1.0

        # parameters
        Cb = float(kwargs.get('Cb', 86.08e3))
        Re = float(kwargs.get('Re', 7.7e-3))
        Rt = float(kwargs.get('Rt', 15.4e-3))
        Rc = float(kwargs.get('Rc', 0.4e-3))
        Cc = float(kwargs.get('Cc', 4.074e3))

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
        self.x0 = self.vmax

        # ros interface
        self.sub_thrs = rospy.Subscriber(TOPIC_THR, ThrusterFeedback, self.handle_thrusters, queue_size=10)
        self.pub_sts = rospy.Publisher(TOPIC_BAT, BatteryStatus, queue_size=10, latch=True)

        # services
        self.srv_reset = rospy.Service(SRV_RESET, Empty, self.handle_reset)

        # timers
        self.t_upd = rospy.Timer(rospy.Duration(TS_UPDATE), self.publish_state)


    def handle_reset(self, data=None):
        rospy.logwarn('%s: resetting estimations', self.name)

        self.x0 = self.vmax
        self.energy_residual = np.copy(self.energy_max)
        self.soc = 1.0

        return []

    def handle_thrusters(self, data):
        # selected thrusters only
        self.thrs_currents = np.roll(self.thrs_currents, -1)
        self.thrs_currents[-1] = np.sum(data.current[0:4]) / self.n_packs

        self.thrs_idx += 1

        if self.thrs_idx > self.n_win:
            self.update_state()

            self.thrs_idx = 0
            self.thrs_currents = np.zeros(self.n_win)

    def update_state(self):
        # support vectors
        t = np.arange(0.0, self.n_win * self.dt, self.dt)
        u = self.thrs_currents
        x0 = self.x0

        # simulation
        t, yout, xout = control.forced_response(self.sys, t, u, x0)

        # save state
        self.x0 = xout[:, -1]

        # update capacity
        vcb = xout[0, -1]
        eres = 0.5 * self.cbe * ((vcb ** 2) - (self.vmin ** 2))

        self.energy_residual = eres * self.n_packs
        self.soc = self.energy_residual / self.energy_max
        self.vbat = yout[-1] * self.cell_series

        # clipping
        self.energy_residual = np.clip(self.energy_residual, 0.0, self.energy_max)
        self.soc = np.clip(self.soc, 0.0, 1.0)

        #rospy.loginfo('%s: vcb: %.3f, soc: %.3f', self.name, vcb, self.soc * 100)

    def publish_state(self, event=None):
        # ros messages
        res = BatteryStatus()
        res.header.stamp = rospy.Time.now()
        res.energy_max = self.energy_max
        res.energy_residual = self.energy_residual
        res.soc = self.soc
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
