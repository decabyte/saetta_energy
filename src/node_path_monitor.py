#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import time
import numpy as np
import scipy as sci
import scipy.stats

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

        self.t_observe = float(kwargs.get('t_observe', 10.0))
        self.sigma_phi_thr = kwargs.get('sigma_phi', np.deg2rad(10.0))
        self.n_bins = int(kwargs.get('n_bins', 16))
        self.phi_bins = (np.linspace(0, 2 * np.pi, self.n_bins) - np.pi)

        self.chunk_yaw = []
        self.chunk_energy = 0.0
        self.chunk_dist = 0.0
        self.chunk_cost = 0.0
        self.chunk_start = 0.0

        self.map_cost = self.n_bins * [[]]
        self.map_spd = self.n_bins * [[]]

        # self.path_id = 0
        # self.path_yaw_mean = 0.0
        # self.path_yaw_std = 0.0
        # self.path_energy_start = 0.0
        # self.path_time_start = 0.0

        # ros interface
        self.sub_req = rospy.Subscriber(TOPIC_REQ, PathRequest, self.handle_request, queue_size=10)
        self.sub_sts = rospy.Subscriber(TOPIC_STS, PathStatus, self.handle_status, queue_size=10)
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_nav, queue_size=10)
        self.sub_ene = rospy.Subscriber(TOPIC_ENE, EnergyReport, self.handle_energy, queue_size=10)

        # visualization
        #self.fig = None

        import matplotlib as mpl
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation

        mpl.style.use('bmh')
        mpl.rcParams['figure.figsize'] = (12.0, 6.0)

        self.fig, self.ax = plt.subplots(subplot_kw=dict(polar=True))
        plt.show()

        self.t_vis = rospy.Timer(rospy.Duration(2), self.show_costs)


    def handle_energy(self, data):
        self.energy_last = data.energy_used

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

        if np.std(self.chunk_yaw) > self.sigma_phi_thr or time_delta > self.t_observe:
            # update odometer
            self.chunk_dist = np.copy(self.distance_travelled)
            self.distance_travelled = 0.0

            # update energy meter
            self.chunk_energy = self.energy_last - self.energy_start
            self.energy_start = np.copy(self.energy_last)

            # chunk metrics
            self.chunk_duration = time.time() - self.chunk_start
            self.chunk_cost = self.chunk_energy / self.chunk_dist
            self.chunk_spd = self.chunk_dist / self.chunk_duration

            # statistical analysis
            mu = np.mean(self.chunk_yaw)
            nu = np.median(self.chunk_yaw)
            sigma = np.std(self.chunk_yaw)

            # select the most frequent yaw value
            modes = sci.stats.mode(self.chunk_yaw)
            m = modes[0][np.argmax(modes[1])]

            # binning
            self.chunk_bin = np.digitize([m], self.phi_bins)

            # map update
            self.map_cost[self.chunk_bin].append(self.chunk_cost)
            self.map_spd[self.chunk_bin].append(self.chunk_spd)

            # reset chunk
            self.chunk_yaw = []

            # enable visualization
            self.show_costs()


    def show_costs(self, event=None):
        #if self.fig is None:
            # self.line = self.ax.plot([], [])
            #
            # # initialization function: plot the background of each frame
            # def init():
            #     self.line.set_data([], [])
            #     return self.line
            #
            # # animation function. this is called sequentially
            # def animate(i):
            #     x = np.rad2deg(self.phi_bins)
            #     y = [np.mean(b) for b in self.map_cost]
            #     self.line.set_data(x, y)
            #     return self.line
            #
            # # call the animator.  blit=True means only re-draw the parts that have changed.
            # self.anim = animation.FuncAnimation(self.fig, animate, init_func=init, frames=200, interval=20, blit=True, repeat=True)

        self.ax.clear()
        self.ax.plot(np.rad2deg(self.phi_bins), [np.mean(b) for b in self.map_cost])


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

    rospy.spin()

    # while not rospy.is_shutdown():
    #     print('state: %s ' % pm.state)
    #     rospy.sleep(1.0)



if __name__ == '__main__':
    main()
