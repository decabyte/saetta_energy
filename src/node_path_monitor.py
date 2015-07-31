#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import time
import datetime

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

from auv_msgs.msg import NavSts
from vehicle_interface.msg import PathRequest, PathStatus, FloatArrayStamped
from saetta_energy.msg import EnergyReport, EnergyStatus
from std_srvs.srv import Empty

# topics
TOPIC_NAV = 'nav/nav_sts'
TOPIC_REQ = 'path/request'
TOPIC_STS = 'path/status'
TOPIC_PIL = 'pilot/status'
TOPIC_ENE = 'saetta/report'
SRV_RESET = 'saetta/map/reset'
TOPIC_MAP_EJM = 'saetta/map/energy'
TOPIC_MAP_SPD = 'saetta/map/speed'

# states
S_IDLE = 0
S_RUNNING = 1

# defaults
DEFAULT_UPDATE = 2          # secs


class PathMonitor(object):

    def __init__(self, **kwargs):
        self.name = rospy.get_name()

        # state
        self.pos = np.zeros(6)
        self.pos_prev = np.zeros(6)
        self.vel = np.zeros(6)

        self.state = S_IDLE
        self.energy_last = 0.0
        self.energy_start = 0.0
        self.distance_travelled = 0.0

        self.n_bins = int(kwargs.get('n_bins', 8))
        self.t_observe = float(kwargs.get('t_observe', 10.0))
        self.phi_sigma = float(kwargs.get('sigma_phi', np.deg2rad(10.0)))
        self.speed_thresh = float(kwargs.get('speed_thresh', 0.3))

        self.initial_ejm = float(kwargs.get('initial_ejm', 100.0))
        self.initial_spd = float(kwargs.get('initial_spd', 0.5))
        self.n_samples = int(kwargs.get('n_samples', 10))

        self.phi_edges = np.linspace(0, 2 * np.pi, self.n_bins + 1) - np.pi
        self.phi_bins = self.phi_edges[:-1] + ((2 * np.pi / self.n_bins) / 2.0)

        self.chunk_yaw = []
        self.chunk_energy = 0.0
        self.chunk_dist = 0.0
        self.chunk_ejm = 0.0
        self.chunk_start = 0.0

        self.map_ejm = self.initial_ejm * np.ones((self.n_bins, self.n_samples))
        self.map_spd = self.initial_spd * np.ones_like(self.map_ejm)

        self.est_ejm = self.initial_ejm * np.ones(self.n_bins)
        self.est_spd = self.initial_spd * np.ones(self.n_bins)

        # ros interface
        self.sub_req = rospy.Subscriber(TOPIC_REQ, PathRequest, self.handle_request, queue_size=10)
        self.sub_sts = rospy.Subscriber(TOPIC_STS, PathStatus, self.handle_status, queue_size=10)
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_nav, queue_size=10)
        self.sub_ene = rospy.Subscriber(TOPIC_ENE, EnergyReport, self.handle_energy, queue_size=10)

        # maps interface
        self.pub_map_ejm = rospy.Publisher(TOPIC_MAP_EJM, FloatArrayStamped, queue_size=10, latch=True)
        self.pub_map_spd = rospy.Publisher(TOPIC_MAP_SPD, FloatArrayStamped, queue_size=10, latch=True)

        # reset interface
        self.srv_reset = rospy.Service(SRV_RESET, Empty, self.handle_reset)

        # timers
        self.t_upd = rospy.Timer(rospy.Duration(DEFAULT_UPDATE), self.update_costs)

        # (temp) keep tracks of resets
        self.date = datetime.datetime.now()
        self.label = self.date.strftime('%Y%m%d_%H%M%S')


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

        if self.state == S_RUNNING:
            # virtual odometer
            self.distance_travelled += np.linalg.norm(self.pos[0:3] - self.pos_prev[0:3])

            # analyse path
            self.process_nav()

        # save previous nav state
        self.pos_prev = self.pos

    def handle_request(self, data):
        if data.command == PathRequest.CMD_PATH:
            # init chunk
            self.chunk_yaw = []
            self.distance_travelled = 0.0
            self.energy_start = np.copy(self.energy_last)
            self.chunk_start = time.time()

            # update state
            self.state = S_RUNNING

    def handle_status(self, data):
        if self.state == S_IDLE:
            return

        if data.path_status in (PathStatus.PATH_IDLE, PathStatus.PATH_RUNNING):
            return
        elif data.path_status in (PathStatus.PATH_COMPLETED):
            # reset state
            self.state = S_IDLE
        else:
            # navigation failed
            rospy.logwarn('%s navigation aborted, skipping measurements ...', self.name)

            # reset state
            self.state = S_IDLE

    def handle_reset(self, req):
        rospy.logwarn('%s: resetting estimations and maps ...', self.name)

        # disable if running
        self.state = S_IDLE

        # clean initial belief
        self.map_ejm = self.initial_ejm * np.ones((self.n_bins, self.n_samples))
        self.map_spd = self.initial_spd * np.ones_like(self.map_ejm)

        self.est_ejm = self.initial_ejm * np.ones(self.n_bins)
        self.est_spd = self.initial_spd * np.ones(self.n_bins)

        # clean last chunk
        self.reset_chunk()

        # reset label
        self.date = datetime.datetime.now()
        self.label = self.date.strftime('%Y%m%d_%H%M%S')

        return []

    def process_nav(self):
        self.chunk_yaw.append(self.pos[5])
        time_delta = time.time() - self.chunk_start

        if np.std(self.chunk_yaw) > self.phi_sigma or time_delta > self.t_observe:
            # statistical analysis
            #   select the most frequent yaw value from modes (aka the mode)
            mu = np.mean(self.chunk_yaw)
            nu = np.median(self.chunk_yaw)
            sigma = np.std(self.chunk_yaw)

            # local binning and mode extraction
            #   diff extracts the
            hist, bin_edges = np.histogram(self.chunk_yaw, bins=10)
            mode = bin_edges[np.argmax(hist)] + (bin_edges[1] - bin_edges[0]) / 2.0

            rospy.loginfo('%s: chunk: mean: %.3f, sigma: %.3f,  median: %.3f, mode: %.3f', self.name,
                np.rad2deg(mu), np.rad2deg(sigma), np.rad2deg(nu), np.rad2deg(mode)
            )

            # chunk variance rejection
            if sigma > self.phi_sigma * 1.5:
                self.reset_chunk()
                rospy.loginfo('%s: chunk discarded: sigma: %.3f', self.name, np.rad2deg(sigma))
                return

            # update odometer and energy meter
            self.chunk_dist = np.copy(self.distance_travelled)
            self.chunk_energy = self.energy_last - self.energy_start

            # chunk metrics
            self.chunk_duration = time.time() - self.chunk_start
            self.chunk_ejm = self.chunk_energy / self.chunk_dist
            self.chunk_spd = self.chunk_dist / self.chunk_duration

            # chunk speed rejection
            if self.chunk_spd < self.speed_thresh:
                self.reset_chunk()
                rospy.loginfo('%s: chuck discarded: spd: %.3f', self.name, self.chunk_spd)
                return

            # direction binning (selecting the yaw sector)
            #   bins[i-1] <= x < bins[i]
            self.chunk_bin = np.digitize([mode], self.phi_edges)[0] - 1

            if self.chunk_bin < 0:
                self.reset_chunk()
                rospy.loginfo('%s: chuck discarded: bin: %d', self.name, self.chunk_bin)
                return

            # map roll
            self.map_ejm[self.chunk_bin, :] = np.roll(self.map_ejm[self.chunk_bin, :], -1)
            self.map_spd[self.chunk_bin, :] = np.roll(self.map_spd[self.chunk_bin, :], -1)

            # map update
            self.map_ejm[self.chunk_bin, -1] = self.chunk_ejm
            self.map_spd[self.chunk_bin, -1] = self.chunk_spd

            rospy.loginfo('%s: chuck update: bin: %d, ejm: %.3f, spd: %.3f', self.name, self.chunk_bin, self.chunk_ejm, self.chunk_spd)
            self.reset_chunk()


    def reset_chunk(self):
        self.distance_travelled = 0.0
        self.energy_start = np.copy(self.energy_last)
        self.chunk_start = time.time()
        self.chunk_yaw = []

    def update_costs(self, event=None):
        # exponential weights
        weights = np.exp(np.linspace(0, 1, self.n_samples)) / np.exp(1)

        # weighted average
        self.est_ejm = np.average(self.map_ejm, weights=weights, axis=1).flatten()
        self.est_spd = np.average(self.map_spd, weights=weights, axis=1).flatten()

        # broadcast estimations
        fa = FloatArrayStamped()
        fa.header.stamp = rospy.Time.now()
        fa.values = self.est_ejm
        self.pub_map_ejm.publish(fa)

        fa = FloatArrayStamped()
        fa.header.stamp = rospy.Time.now()
        fa.values = self.est_spd
        self.pub_map_spd.publish(fa)


def save_maps(pm, steps=20, **kwargs):
    # http://stackoverflow.com/questions/9071084/polar-contour-plot-in-matplotlib-best-modern-way-to-do-it
    # generate theta axis
    th = np.linspace(pm.phi_edges[0], pm.phi_edges[-1], steps * pm.n_bins)
    label = pm.label

    # estimated ejm
    m = np.copy(pm.est_ejm)
    r = m.reshape((-1, 1)).repeat(steps, axis=1).flatten()

    # ejm plot
    fig, ax = plt.subplots(subplot_kw=dict(projection='polar'))
    ax.set_theta_direction(-1)
    ax.set_theta_zero_location('N')
    ax.set_xlabel('Direction (deg)')
    ax.set_title('Average Energy Usage (J/m) by Direction')

    ax.plot(th, r, 'o')
    ax.set_rmax(np.max(r) * 1.1)

    # # estimated ejm interpolation
    # tck = sci.interpolate.splrep(pm.phi_bins, m)
    # rint = sci.interpolate.splev(th, tck)
    # ax.plot(th, rint)

    try:
        fig.canvas.draw()
        fig.savefig('/tmp/map_ejm_{}.png'.format(label), dpi=90)
    except Exception as e:
        rospy.logwarn('%s: cannot save map:\n%s', rospy.get_name(), e)

    # estimated spd
    m = np.copy(pm.est_spd)
    r = m.reshape((-1, 1)).repeat(steps, axis=1).flatten()

    # ejm plot
    fig, ax = plt.subplots(subplot_kw=dict(projection='polar'))
    ax.set_theta_direction(-1)
    ax.set_theta_zero_location('N')
    ax.set_xlabel('Direction (deg)')
    ax.set_title('Average Cruise Speed (m/s) by Direction')

    ax.plot(th, r, 'o')
    ax.set_rmax(np.max(r) * 1.1)

    # # estimated spd interpolation
    # tck = sci.interpolate.splrep(pm.phi_bins, m)
    # rint = sci.interpolate.splev(th, tck)
    # ax.plot(th, rint)

    try:
        fig.canvas.draw()
        fig.savefig('/tmp/map_spd_{}.png'.format(label), dpi=90)
    except Exception as e:
        rospy.logwarn('%s: cannot save map:\n%s', rospy.get_name(), e)

    # clean plots
    plt.close('all')

def main():
    rospy.init_node('path_monitor')
    rospy.loginfo('%s: initialization ...', rospy.get_name())

    # config
    config = rospy.get_param('saetta/path', {})
    enable_maps = bool(rospy.get_param('~save_maps', False))

    # init
    pm = PathMonitor(**config)

    while not rospy.is_shutdown():
        if enable_maps:
            save_maps(pm)

        rospy.sleep(5.0)

if __name__ == '__main__':
    main()
