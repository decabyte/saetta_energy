#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import os
import json
import time
import datetime
import collections

import numpy as np
import scipy as sci
import scipy.interpolate

np.set_printoptions(precision=3, suppress=True)

import matplotlib as mpl
import matplotlib.pyplot as plt

mpl.style.use('bmh')
mpl.rcParams['figure.figsize'] = (8, 8)

# ros imports
import rospy
import roslib
roslib.load_manifest('saetta_energy')

from std_srvs.srv import Empty

from auv_msgs.msg import NavSts
from vehicle_interface.msg import PathRequest, PathStatus, FloatArrayStamped
from vehicle_interface.srv import StringService, BooleanService, BooleanServiceResponse, StringServiceResponse

from saetta_energy.msg import EnergyReport, EnergyStatus
from actionbus.msg import ActionFeedback, ActionDispatch

# topics
TOPIC_NAV = 'nav/nav_sts'
TOPIC_REQ = 'path/request'
TOPIC_STS = 'path/status'
TOPIC_PIL = 'pilot/status'
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
DEFAULT_UPDATE = 5          # secs
ACTION_GOTO = 'goto'


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
        self.switch = True

        # vehicle
        self.pos = np.zeros(6)
        self.vel = np.zeros(6)
        self.energy_last = 0.0

        # params
        self.n_bins = int(kwargs.get('n_bins', 8))
        self.n_samples = int(kwargs.get('n_samples', 50))

        self.sigma_thresh = float(kwargs.get('sigma_phi', np.deg2rad(10.0)))
        self.speed_thresh = float(kwargs.get('speed_thresh', 0.3))
        self.dist_thresh = float(kwargs.get('dist_thresh', 1.0))

        self.initial_ejm = float(kwargs.get('initial_ejm', 100.0))
        self.initial_spd = float(kwargs.get('initial_spd', 0.5))

        self.phi_edges = np.linspace(0, 2 * np.pi, self.n_bins + 1) - np.pi
        self.phi_bins = self.phi_edges[:-1] + ((2 * np.pi / self.n_bins) / 2.0)

        # continuous chunking
        self.samples_nav = np.zeros((self.n_samples, 6))
        self.samples_energy = np.zeros(self.n_samples)
        self.samples_cached = False
        self.samples_cnt = 0                # samples
        self.map_length = 10 * 60 * 10      # samples

        # init maps
        self._init_map()

        # ros interface
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_nav, queue_size=10)
        self.sub_ene = rospy.Subscriber(TOPIC_ENE, EnergyReport, self.handle_energy, queue_size=10)

        self.sub_sts = rospy.Subscriber(TOPIC_STS, PathStatus, self.handle_status, queue_size=10)

        # listen to actions system
        self.use_actions = bool(kwargs.get('use_action', True))

        if self.use_actions:
            self.sub_feed = rospy.Subscriber(TOPIC_FEED, ActionFeedback, self.handle_feedback, queue_size=10)
        else:
            self.sub_req = rospy.Subscriber(TOPIC_REQ, PathRequest, self.handle_request, queue_size=10)

        # maps interface
        self.pub_map_ejm = rospy.Publisher(TOPIC_MAP_EJM, FloatArrayStamped, queue_size=10, latch=True)
        self.pub_map_spd = rospy.Publisher(TOPIC_MAP_SPD, FloatArrayStamped, queue_size=10, latch=True)

        # services
        self.srv_reset = rospy.Service(SRV_RESET, Empty, self.handle_reset)
        self.srv_dump = rospy.Service(SRV_DUMP, StringService, self.handle_dump)
        self.srv_switch = rospy.Service(SRV_SWITCH, BooleanService, self.handle_switch)

        # timers
        self.t_upd = rospy.Timer(rospy.Duration(DEFAULT_UPDATE), self.publish_estimations)


    def _init_map(self):
        self.map_ejm = np.zeros((self.n_bins, self.map_length), dtype=np.float32)
        self.map_spd = np.zeros_like(self.map_ejm)
        self.map_flags = np.zeros(self.n_bins, dtype=np.bool)
        self.map_idx = np.zeros(self.n_bins, dtype=np.int)

        self.map_ejm[:, 0] = self.initial_ejm
        self.map_spd[:, 0] = self.initial_spd

    def handle_dump(self, data):
        user_path = data.request
        dir_path = os.path.dirname(user_path)

        if user_path == '' or not os.path.exists(dir_path):
            return StringServiceResponse(result=False, response=data.request)

        out = {
            'n_bins': self.n_bins,
            'map_ejm': self.map_ejm.tolist(),
            'map_spd': self.map_spd.tolist(),
            'map_idx': self.map_idx.tolist(),
            'map_flags': self.map_flags.tolist(),
        }

        with open(user_path, 'wt') as jf:
            json.dump(out, jf, indent=2, cls=CustomEncoder)

        return StringServiceResponse(result=True, response=user_path)

    def handle_switch(self, data):
        self.switch = data.request

        # discard last measurement if disabled
        if not self.switch:
            self.state = S_IDLE

        rospy.logwarn('%s: resetting status: %s', self.name, self.switch)
        return BooleanServiceResponse(response=self.switch)

    def handle_reset(self, data=None):
        # disable and clean maps
        self.state = S_IDLE
        self.switch = True

        self._init_map()

        rospy.logwarn('%s: resetting estimations and maps ...', self.name)
        return []

    def handle_energy(self, data):
        """parse energy meter (Wh) and converts to Joules (J = Wh * 3600)"""
        self.energy_last = data.energy_used * 3600

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

        if self.state == S_RUNNING and self.switch:
            self.process_nav()


    def handle_request(self, data):
        if data.command == PathRequest.CMD_PATH:
            self.state = S_RUNNING

    def handle_status(self, data):
        if self.state == S_IDLE or data.path_status == PathStatus.PATH_IDLE:
            return

        if data.path_status == PathStatus.PATH_RUNNING:
            return
        elif data.path_status in (PathStatus.PATH_COMPLETED):
            self.state = S_IDLE
        else:
            # navigation failed
            self.state = S_IDLE
            rospy.logwarn('%s navigation aborted, skipping measurements ...', self.name)

    def handle_feedback(self, data):
        if data.name != ACTION_GOTO:
            return

        if data.status == ActionFeedback.ACTION_RUNNING:
            self.state = S_RUNNING
        else:
            self.state = S_IDLE


    def process_nav(self):
        self.samples_nav = np.roll(self.samples_nav, -1, axis=0)
        self.samples_energy = np.roll(self.samples_energy, -1)

        self.samples_nav[-1, :] = np.copy(self.pos)
        self.samples_energy[-1] = np.copy(self.energy_last)

        if not self.samples_cached:
            self.samples_cnt += 1

            if self.samples_cnt > self.n_samples:
                self.samples_cached = True
                rospy.loginfo('%s: initial samples cached, starting estimator ...', self.name)
            else:
                return

        # check yaw conditions
        sigma = np.std(self.samples_nav[:, 5])

        if sigma > self.sigma_thresh:
            return

        dist = np.sum(np.linalg.norm(np.diff(self.samples_nav[:, 0:3], axis=0), axis=1))
        cost = self.samples_energy[-1] - self.samples_energy[0]

        if dist <= 0.0:
            return

        metric = cost / dist

        if metric <= 0.0:
            return

        # check spd conditions
        avg_spd = dist / (self.n_samples / 10)

        if avg_spd < self.speed_thresh or avg_spd > 2.0:
            return

        #rospy.loginfo('%s: metric: %.3f', self.name, metric)

        # local binning and mode extraction
        hist, bin_edges = np.histogram(self.samples_nav[:, 5], bins=10)
        mode = bin_edges[np.argmax(hist)] + (bin_edges[1] - bin_edges[0]) / 2.0

        # direction binning (selecting the yaw sector)
        #   bins[i-1] <= x < bins[i]
        curr_bin = np.digitize([mode], self.phi_edges)[0] - 1

        # map update
        curr_idx = self.map_idx[curr_bin]

        self.map_ejm[curr_bin, curr_idx] = metric
        self.map_spd[curr_bin, curr_idx] = avg_spd

        # update index
        next_idx = (self.map_idx[curr_bin] + 1) % self.map_length
        self.map_idx[curr_bin] = next_idx

        # sliding map
        if not self.map_flags[curr_bin] and next_idx < curr_idx:
            self.map_flags[curr_bin] = True

    def publish_estimations(self, event=None):
        self.est_ejm = []
        self.est_spd = []

        for n in xrange(self.n_bins):
            b = self.map_length if self.map_flags[n] else self.map_idx[n] + 1
            ejm = np.mean(self.map_ejm[n, 0:b]).astype(float)
            spd = np.mean(self.map_spd[n, 0:b]).astype(float)

            self.est_ejm.append(ejm)
            self.est_spd.append(spd)

        # ros messages
        fa = FloatArrayStamped()
        fa.header.stamp = rospy.Time.now()
        fa.values = self.est_ejm
        self.pub_map_ejm.publish(fa)

        fa = FloatArrayStamped()
        fa.header.stamp = rospy.Time.now()
        fa.values = self.est_spd
        self.pub_map_spd.publish(fa)

        rospy.loginfo('%s: ejm: %s', self.name, self.est_ejm)

        # # exponential weights
        # weights = np.exp(np.linspace(0, 1, self.n_samples)) / np.exp(1)
        #
        # # weighted average
        # self.est_ejm = np.average(self.map_ejm, weights=weights, axis=1).flatten()
        # self.est_spd = np.average(self.map_spd, weights=weights, axis=1).flatten()

def main():
    rospy.init_node('path_monitor')
    rospy.loginfo('%s: initialization ...', rospy.get_name())

    # config
    config = rospy.get_param('saetta/path', {})

    pm = PathMonitor(**config)
    rospy.spin()

    # while not rospy.is_shutdown():
    #     rospy.sleep(5.0)

if __name__ == '__main__':
    main()
