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

IDLE = 0
RUNNING = 1


class PathMonitor(object):

    def __init__(self, **kwargs):
        self.name = rospy.get_name()

        # state
        self.pos = np.zeros(6)
        self.vel = np.zeros(6)
        self.state = IDLE

        self.path_id = 0
        self.path_yaw_mean = 0.0
        self.path_yaw_std = 0.0
        self.path_energy_start = 0.0
        self.path_time_start = 0.0

        self.energy_last = 0.0

        # ros interface
        self.sub_req = rospy.Subscriber(TOPIC_REQ, PathRequest, self.handle_request, queue_size=10)
        self.sub_sts = rospy.Subscriber(TOPIC_STS, PathStatus, self.handle_status, queue_size=10)
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_nav, queue_size=10)
        self.sub_ene = rospy.Subscriber(TOPIC_ENE, EnergyReport, self.handle_energy, queue_size=10)


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

    def handle_request(self, data):
        if data.command == PathRequest.CMD_PATH:
            wps = np.array([p.values for p in data.points])
            wps = np.r_[np.atleast_2d(self.pos), wps]

            orientations = []

            for n in xrange(1, wps.shape[0]):
                cy = tt.calculate_orientation(wps[n-1, :], wps[n, :])
                orientations.append(cy)

            self.path_yaw_mean = np.mean(orientations)
            self.path_yaw_std = np.std(orientations)
            self.path_energy_start = np.copy(self.energy_last)
            self.path_time_start = time.time()

            rospy.loginfo('%s: path: yaw_m: %.3f, yaw_std: %.3f' % (
                self.name, np.rad2deg(self.path_yaw_mean), np.rad2deg(self.path_yaw_std)
            ))

            # update state
            self.state = RUNNING

    def handle_status(self, data):
        if self.state == IDLE:
            return

        if data.path_status in (PathStatus.PATH_IDLE, PathStatus.PATH_RUNNING):
            return
        elif data.path_status in (PathStatus.PATH_COMPLETED):
            self.path_energy_end = np.copy(self.energy_last)
            self.path_time_end = time.time()

            self.path_id = data.path_id
            self.path_length = data.distance_completed
            self.path_speed = data.speed_average
            self.path_energy_cost = self.path_energy_end - self.path_energy_start
            self.path_duration = data.time_elapsed
            self.path_energy_metric = self.path_energy_cost / self.path_length

            rospy.loginfo('%s: path %d, time: %.3f, energy: %.3f' % (
                self.name, self.path_id, self.path_duration, self.path_energy_cost
            ))

            # reset state
            self.state = IDLE
        else:
            # navigation failed
            rospy.logwarn('%s navigation aborted, skipping measurements ...', self.name)

            # reset state
            self.state = IDLE


def main():
    rospy.init_node('path_monitor')
    pm = PathMonitor()

    rospy.spin()

    # while not rospy.is_shutdown():
    #     print('state: %s ' % pm.state)
    #     rospy.sleep(1.0)



if __name__ == '__main__':
    main()
