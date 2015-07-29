#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('saetta_energy')

from vehicle_core.path import trajectory_tools as tt

from auv_msgs.msg import NavSts
from vehicle_interface.msg import PathRequest, PathStatus
from saetta_energy.msg import EnergyReport, EnergyStatus


# config
TOPIC_NAV = 'nav/nav_sts'
TOPIC_REQ = 'path/request'
TOPIC_STS = 'path/status'
TOPIC_PIL = 'pilot/status'
TOPIC_ENE = 'saetta/report'

S_IDLE = 0
S_RUNNING = 1

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

        self.initial_cost = float(kwargs.get('initial_cost', 100.0))
        self.initial_spd = float(kwargs.get('initial_spd', 0.5))
        self.n_samples = int(kwargs.get('n_samples', 10))

        self.phi_edges = np.linspace(0, 2 * np.pi, self.n_bins + 1) - np.pi
        self.phi_bins = self.phi_edges[:-1] + ((2 * np.pi / self.n_bins) / 2.0)

        self.chunk_yaw = []
        self.chunk_energy = 0.0
        self.chunk_dist = 0.0
        self.chunk_cost = 0.0
        self.chunk_start = 0.0

        self.map_cost = self.initial_cost * np.ones((self.n_bins, self.n_samples))
        self.map_spd = self.initial_spd * np.ones_like(self.map_cost)

        self.est_cost = self.initial_cost * np.ones(self.n_bins)
        self.est_spd = self.initial_spd * np.ones(self.n_bins)

        # ros interface
        self.sub_req = rospy.Subscriber(TOPIC_REQ, PathRequest, self.handle_request, queue_size=10)
        self.sub_sts = rospy.Subscriber(TOPIC_STS, PathStatus, self.handle_status, queue_size=10)
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_nav, queue_size=10)
        self.sub_ene = rospy.Subscriber(TOPIC_ENE, EnergyReport, self.handle_energy, queue_size=10)

        # timers
        self.t_upd = rospy.Timer(rospy.Duration(DEFAULT_UPDATE), self.update_costs)


    def handle_energy(self, data):
        self.energy_last = data.energy_used * 3600      # J = Wh * 3600 = Ws = J

    def handle_nav(self, data):
        # parse navigation data
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
            # pre-analysis
            #self._pre_analyse_path(data)

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
            # post-analysis
            #self._post_analyse_path(data)

            # reset state
            self.state = S_IDLE
        else:
            # navigation failed
            rospy.logwarn('%s navigation aborted, skipping measurements ...', self.name)

            # reset state
            self.state = S_IDLE


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
                rospy.loginfo('%s: chuck discarded: sigma: %.3f', self.name, np.rad2deg(sigma))
                return

            # update odometer and energy meter
            self.chunk_dist = np.copy(self.distance_travelled)
            self.chunk_energy = self.energy_last - self.energy_start

            # chunk metrics
            self.chunk_duration = time.time() - self.chunk_start
            self.chunk_cost = self.chunk_energy / self.chunk_dist
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
            self.map_cost[self.chunk_bin, :] = np.roll(self.map_cost[self.chunk_bin, :], -1)
            self.map_spd[self.chunk_bin, :] = np.roll(self.map_spd[self.chunk_bin, :], -1)

            # map update
            self.map_cost[self.chunk_bin, -1] = self.chunk_cost
            self.map_spd[self.chunk_bin, -1] = self.chunk_spd

            rospy.loginfo('%s: chuck update: bin: %d, cost: %.3f, spd: %.3f', self.name, self.chunk_bin, self.chunk_cost, self.chunk_spd)
            self.reset_chunk()


    def reset_chunk(self):
        self.distance_travelled = 0.0
        self.energy_start = np.copy(self.energy_last)
        self.chunk_start = time.time()
        self.chunk_yaw = []

    def update_costs(self, event=None):
        weights = np.exp(np.linspace(0, 1, self.n_samples)) / np.exp(1)

        self.est_cost = np.average(self.map_cost, weights=weights, axis=1).flatten()
        self.est_spd = np.average(self.map_spd, weights=weights, axis=1).flatten()

    # def _pre_analyse_path(self, data):
    #     wps = np.array([p.values for p in data.points])
    #     wps = np.r_[np.atleast_2d(self.pos), wps]
    #
    #     orientations = []
    #
    #     for n in xrange(1, wps.shape[0]):
    #         cy = tt.calculate_orientation(wps[n-1, :], wps[n, :])
    #         orientations.append(cy)
    #
    #     self.path_yaw_mean = np.mean(orientations)
    #     self.path_yaw_std = np.std(orientations)
    #     self.path_energy_start = np.copy(self.energy_last)
    #     self.path_time_start = time.time()
    #
    #     rospy.loginfo('%s: path: yaw_m: %.3f, yaw_std: %.3f' % (
    #         self.name, np.rad2deg(self.path_yaw_mean), np.rad2deg(self.path_yaw_std)
    #     ))
    #
    # def _post_analyse_path(self, data):
    #     self.path_energy_end = np.copy(self.energy_last)
    #     self.path_time_end = time.time()
    #
    #     self.path_id = data.path_id
    #     self.path_length = data.distance_completed
    #     self.path_speed = data.speed_average
    #     self.path_energy_cost = self.path_energy_end - self.path_energy_start
    #     self.path_duration = data.time_elapsed
    #     self.path_energy_metric = self.path_energy_cost / self.path_length
    #
    #     rospy.loginfo('%s: path %d, time: %.3f, energy: %.3f' % (
    #         self.name, self.path_id, self.path_duration, self.path_energy_cost
    #     ))

def main():
    rospy.init_node('path_monitor')
    pm = PathMonitor()

    import matplotlib as mpl
    import matplotlib.pyplot as plt

    import scipy as sci
    import scipy.interpolate

    mpl.style.use('bmh')
    mpl.rcParams['figure.figsize'] = (8, 8)

    # http://stackoverflow.com/questions/9071084/polar-contour-plot-in-matplotlib-best-modern-way-to-do-it
    fig, ax = plt.subplots(subplot_kw=dict(projection='polar'))

    steps = 20
    th = np.linspace(pm.phi_edges[0], pm.phi_edges[-1], steps * pm.n_bins)

    while not rospy.is_shutdown():
        #m = np.mean(pm.map_cost, axis=1).flatten()

        m = np.copy(pm.est_cost)
        r = m.reshape((-1, 1)).repeat(steps, axis=1).flatten()

        tck = sci.interpolate.splrep(pm.phi_bins, m)
        rint = sci.interpolate.splev(th, tck)

        ax.clear()
        ax.set_theta_direction(-1)
        ax.set_theta_zero_location('N')
        ax.set_xlabel('Heading (deg)')
        #ax.set_ylabel('Energy Cost (J/m)')
        ax.set_title('Energy Cost vs Heading (map)')

        ax.plot(th, r, 'o')
        ax.plot(th, rint)

        ax.set_rmax(np.max(r) * 1.1)

        plt.draw()
        plt.savefig('/tmp/map_cost.png', dpi=90)

        rospy.sleep(2.0)

    # rospy.spin()

if __name__ == '__main__':
    main()
