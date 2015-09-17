#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import os
import json
import collections

import numpy as np
np.set_printoptions(precision=3, suppress=True)

# ros imports
import rospy
import roslib
roslib.load_manifest('saetta_energy')

from std_srvs.srv import Empty

from auv_msgs.msg import NavSts
from vehicle_interface.msg import PathRequest, PathStatus, ThrusterFeedback
from vehicle_interface.srv import StringService, BooleanService, BooleanServiceResponse, StringServiceResponse

from saetta_energy.msg import EnergyReport, EnergyStatus, RegressionResult
from actionbus.msg import ActionFeedback, ActionDispatch

# topics
TOPIC_NAV = 'nav/nav_sts'
TOPIC_REQ = 'path/request'
TOPIC_STS = 'path/status'
TOPIC_PIL = 'pilot/status'
TOPIC_THR = 'thrusters/status'
TOPIC_ENE = 'saetta/report'

TOPIC_MAP_EJM = 'saetta/map/energy'
TOPIC_MAP_SPD = 'saetta/map/speed'

TOPIC_DISP = 'action/dispatch'
TOPIC_FEED = 'action/feedback'

SRV_RESET = 'saetta/map/reset'
SRV_DUMP = 'saetta/map/dump'
SRV_SWITCH = 'saetta/map/switch'

# states
S_IDLE = 0
S_RUNNING = 1

# defaults
THR_VNOM = 28.0         # volt
THR_FRAME = 0.1         # secs
TS_UPDATE = 5           # secs
ACTION_GOTO = 'goto'


import scipy as sci
import scipy.optimize

import sklearn as sk
import sklearn.metrics

# curve_fit support function
def poly(x, *args):
    return np.polyval(args, x)


class CustomEncoder(json.JSONEncoder):
    """http://stackoverflow.com/questions/8230315/python-sets-are-not-json-serializable"""
    def default(self, obj):
        if isinstance(obj, collections.deque):
            return list(obj)

        return json.JSONEncoder.default(self, obj)

class PathMonitor(object):
    def __init__(self, **kwargs):
        self.name = rospy.get_name()

        # state
        self.state = S_IDLE
        self.enabled = True

        # vehicle
        self.pos = np.zeros(6)
        self.vel = np.zeros(6)
        self.energy_last = 0.0

        self.thrs_win = int(kwargs.get('thrs_win', 2))
        self.thrs_current = np.zeros((6, self.thrs_win))
        self.thrs_energy = np.zeros(6)

        # params
        self.n_bins = int(kwargs.get('n_bins', 8))
        self.n_win = int(kwargs.get('n_win', 50))
        self.use_actions = bool(kwargs.get('use_actions', True))

        self.sigma_thresh = float(kwargs.get('sigma_phi', np.deg2rad(10.0)))
        self.speed_thresh = float(kwargs.get('speed_thresh', 0.3))

        self.initial_ejm = float(kwargs.get('initial_ejm', 70.0))
        self.initial_spd = float(kwargs.get('initial_spd', 0.5))

        self.est_ejm = [0, self.initial_ejm]
        self.est_spd = [0, self.initial_spd]
        self.mse_ejm = np.inf
        self.mse_spd = np.inf

        # sliding window
        self.win_nav = np.zeros((self.n_win, 6))        # holds nav samples
        self.win_vel = np.zeros((self.n_win, 6))        # holds vel samples
        self.win_energy = np.zeros(self.n_win)          # holds energy readings
        self.win_filled = False                         # check initial window filling
        self.win_cnt = 0                                # counter (samples)

        # features map
        self.map_length = int(kwargs.get('n_length', 25 * 60 * 10))     # time duration of sampling window (samples)
        self.map_features = 7                                           # number of features (time, energy, yaw, yaw_std, v, v_std, dist)
        self.map_idx = 0                                                # counter (samples)

        self._init_map()

        # # direction bins
        # self.phi_edges = np.linspace(0.0, 2 * np.pi, self.n_bins + 1) - np.pi
        # self.phi_bins = self.phi_edges[:-1] + ((2 * np.pi / self.n_bins) / 2.0)

        # ros interface
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_nav, queue_size=10)
        self.sub_sts = rospy.Subscriber(TOPIC_STS, PathStatus, self.handle_status, queue_size=10)
        self.sub_thrs = rospy.Subscriber(TOPIC_THR, ThrusterFeedback, self.handle_thrusters, queue_size=10)

        if self.use_actions:
            self.sub_feed = rospy.Subscriber(TOPIC_FEED, ActionFeedback, self.handle_feedback, queue_size=10)
        else:
            self.sub_req = rospy.Subscriber(TOPIC_REQ, PathRequest, self.handle_request, queue_size=10)

        # maps interface
        self.pub_map_ejm = rospy.Publisher(TOPIC_MAP_EJM, RegressionResult, queue_size=10, latch=True)
        self.pub_map_spd = rospy.Publisher(TOPIC_MAP_SPD, RegressionResult, queue_size=10, latch=True)

        # services
        self.srv_reset = rospy.Service(SRV_RESET, Empty, self.handle_reset)
        self.srv_dump = rospy.Service(SRV_DUMP, StringService, self.handle_dump)
        self.srv_switch = rospy.Service(SRV_SWITCH, BooleanService, self.handle_switch)

        # timers
        self.t_upd = rospy.Timer(rospy.Duration(TS_UPDATE), self.publish_estimations)
        self.t_reg = rospy.Timer(rospy.Duration(10), self.data_regression)


    def _init_map(self):
        # sliding window
        self.win_nav = np.zeros((self.n_win, 6))
        self.win_vel = np.zeros((self.n_win, 6))
        self.win_energy = np.zeros(self.n_win)          # holds energy readings
        self.win_filled = False                         # check initial window filling
        self.win_cnt = 0

        # map
        self.samples = np.zeros((self.map_length, self.map_features))
        self.map_idx = 0

    def handle_dump(self, data):
        user_path = data.request
        dir_path = os.path.dirname(user_path)

        if user_path == '' or not os.path.exists(dir_path):
            return StringServiceResponse(result=False, response=data.request)

        out = {
            'n_bins': self.n_bins,
            'n_win': self.n_win,

            'length': self.map_length,
            'features': self.map_features,

            'index': self.map_idx,
            'samples': self.samples.tolist(),
        }

        with open(user_path, 'wt') as jf:
            json.dump(out, jf, indent=2, cls=CustomEncoder)

        return StringServiceResponse(result=True, response=user_path)

    def handle_switch(self, data):
        self.enabled = data.request

        # discard last measurement if disabled
        if not self.enabled:
            self.state = S_IDLE

        rospy.logwarn('%s: resetting status: %s', self.name, self.enabled)
        return BooleanServiceResponse(response=self.enabled)

    def handle_reset(self, data=None):
        # disable and clean maps
        self.state = S_IDLE
        self.enabled = True

        self._init_map()

        rospy.logwarn('%s: resetting estimations and maps ...', self.name)
        return []


    def handle_thrusters(self, data):
        """parse thruster data and updated energy meter"""
        self.thrs_current = np.roll(self.thrs_current, -1, axis=1)      # roll the vector (shift)
        self.thrs_current[0:4, -1] = data.current[0:4]                  # selected thrusters only

        self.thrs_energy += THR_VNOM * np.trapz(self.thrs_current, dx=THR_FRAME, axis=1)
        self.energy_last = np.sum(self.thrs_energy)

    def handle_nav(self, data):
        """parse navigation data and update virtual odometer"""
        self.pos = np.array([
            data.position.north,
            data.position.east,
            data.position.depth,
            data.orientation.roll,
            data.orientation.pitch,
            data.orientation.yaw
        ])

        self.vel = np.array([
            data.body_velocity.x,
            data.body_velocity.y,
            data.body_velocity.z,
            data.orientation_rate.roll,
            data.orientation_rate.pitch,
            data.orientation_rate.yaw
        ])

        if self.state == S_RUNNING and self.enabled:
            self.process_nav()


    def handle_request(self, data):
        rospy.loginfo('Received new path request!')

        if data.command == PathRequest.CMD_PATH:
            self.state = S_RUNNING

    def handle_status(self, data):
        if self.state == S_IDLE:
            return

        if data.path_status == PathStatus.PATH_IDLE:
            return

        if data.path_status == PathStatus.PATH_RUNNING:
            return
        else:
            self.state = S_IDLE

    def handle_feedback(self, data):
        if data.name != ACTION_GOTO:
            return

        if data.status == ActionFeedback.ACTION_RUNNING:
            self.state = S_RUNNING
        else:
            self.state = S_IDLE


    def process_nav(self):
        # update sliding window
        self.win_nav = np.roll(self.win_nav, -1, axis=0)
        self.win_vel = np.roll(self.win_vel, -1, axis=0)
        self.win_energy = np.roll(self.win_energy, -1)

        self.win_nav[-1, :] = np.copy(self.pos)
        self.win_vel[-1, :] = np.copy(self.vel)
        self.win_energy[-1] = np.copy(self.energy_last)

        # check buffering
        if not self.win_filled:
            self.win_cnt += 1

            if self.win_cnt > self.n_win:
                self.win_filled = True
                rospy.loginfo('%s: initial samples cached, starting estimator ...', self.name)
            else:
                return

        # update features
        dist = np.sum(np.linalg.norm(np.diff(self.win_nav[:, 0:3], axis=0), axis=1))
        cvel = np.linalg.norm(self.win_vel[:, 0:3], axis=1)

        vel_mean = np.mean(cvel)    # vel_mean = dist / (self.n_samples / 10)
        vel_std = np.std(cvel)
        yaw_mean = np.mean(self.win_nav[:, 5])
        yaw_std = np.std(self.win_nav[:, 5])
        energy = self.win_energy[-1] - self.win_energy[0]

        self.samples[self.map_idx, 0] = rospy.Time.now().to_sec()
        self.samples[self.map_idx, 1] = energy
        self.samples[self.map_idx, 2] = yaw_mean
        self.samples[self.map_idx, 3] = yaw_std
        self.samples[self.map_idx, 4] = vel_mean
        self.samples[self.map_idx, 5] = vel_std
        self.samples[self.map_idx, 6] = dist

        self.map_idx = (self.map_idx + 1) % self.map_length


    def data_regression(self, event=None):
        if not self.enabled:
            return

        # index selection
        if self.samples[self.map_idx, 0] == 0:
            idx = self.map_idx

            if self.map_idx < 100:
                return
        else:
            idx = -1

        # samples selection
        #   a) select only measuread values
        #   b) filter values based on vel_std and yaw_std thresholds
        #   c) select valid samples from measures
        measures = self.samples[:idx, :]
        cond = np.logical_and(measures[:, 5] < 0.05, measures[:, 3] < 0.2)
        valid = measures[cond, :]

        # compute metric (energy / distance)
        ejm = valid[:, 1] / valid[:, 6]

        # fit quality estimator
        mse = sklearn.metrics.mean_squared_error
        prev_e = np.inf
        prev_s = np.inf

        # yaw vs ejm
        for n in (3, 5, 7):
            popt_e, pcov_e = sci.optimize.curve_fit(poly, valid[:, 2], ejm, p0=np.ones(n))
            emse_e = mse(ejm, poly(valid[:, 2], *popt_e))

            if emse_e >= prev_e:
                break

            prev_e = emse_e
            self.est_ejm = np.copy(popt_e)
            self.mse_ejm = emse_e

        # yaw vs spd
        for n in (3, 5, 7):
            popt_s, pcov_s = sci.optimize.curve_fit(poly, valid[:, 2], valid[:, 4], p0=np.ones(n))
            emse_s = mse(valid[:, 4], poly(valid[:, 2], *popt_s))

            if emse_s >= prev_s:
                break

            prev_s = emse_s
            self.est_spd = np.copy(popt_s)
            self.mse_spd = emse_s

        #rospy.loginfo('%s: data fitting ejm: degree: %d, mse: %.3f', self.name, len(self.est_ejm), self.mse_ejm)
        #rospy.loginfo('%s: data fitting spd: degree: %d, mse: %.3f', self.name, len(self.est_spd), self.mse_spd)


    def publish_estimations(self, event=None):
        # ros messages
        res = RegressionResult()
        res.header.stamp = rospy.Time.now()
        res.type = 'poly'
        res.coeff = self.est_ejm
        res.mse = self.mse_ejm
        self.pub_map_ejm.publish(res)

        res = RegressionResult()
        res.header.stamp = rospy.Time.now()
        res.type = 'poly'
        res.coeff = self.est_spd
        res.mse = self.mse_spd
        self.pub_map_spd.publish(res)


def main():
    rospy.init_node('path_monitor')
    rospy.loginfo('%s: init ...', rospy.get_name())

    # config
    config = rospy.get_param('saetta/path', {})

    pm = PathMonitor(**config)
    rospy.spin()

    # while not rospy.is_shutdown():
    #     rospy.sleep(5.0)

if __name__ == '__main__':
    main()
