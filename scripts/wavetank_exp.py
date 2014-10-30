#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Assumes vehicle is to start from the left bottom corner of the tank and
# move 15 meters to the left and 10 meters away.

from __future__ import division

import sys
import os
import traceback
import time
import csv
import argparse

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('saetta_energy')


from diagnostic_msgs.msg import KeyValue
from vehicle_interface.msg import PathStatus, Vector6, PilotRequest
from vehicle_interface.srv import PathService
from saetta_energy.msg import EnergyReport


# topics
TOPIC_STATUS = 'path/status'
TOPIC_POS = 'pilot/position_req'
TOPIC_ENERGY = 'saetta/report'
SRV_PATH = 'path/control'
SRV_SWITCH = 'pilot/switch'

# path description  (moving from A to B)
OFFSET_A_NORTH = 1.0
OFFSET_A_EAST = -1.8
OFFSET_B_NORTH = 4.4    # length of the tank's usable space away from our desks
OFFSET_B_EAST = -9.8    # length of the tank's usable space along our desks

DEPTH = 1.0

# set these once the nav is started!
YAW_OFFSET = 0
X_OFFSET = 0
Y_OFFSET = 0

# file
CSV_PREFIX = 'results'


class WavetankExperiment(object):

    def __init__(self, name, offset_yaw, offset_north=0, offset_east=0, suffix='last', **kwargs):
        self.name = name

        # flags
        self.wait = True
        self.path_points = None

        self.cnt = 0
        self.duration = 0
        self.energy_used = 0
        self.energy_initial = 0
        self.energy_last = 0

        # status
        self.mode = kwargs.get('mode', 'lines')
        self.time_started = 0
        self.experiment_running = False

        # export data
        self.csv_file = '{0}_{1}.csv'.format(CSV_PREFIX, suffix)
        self.csv_file = os.path.join(os.getcwd(), self.csv_file)

        self.header = [
            'time', 'theta',
            'Ax', 'Ay', 'Az', 'Ak', 'Am', 'An',
            'Bx', 'By', 'Bz', 'Bk', 'Bm', 'Bn',
            'duration',
            'energy_used'
        ]

        # change sign to switch from marine to maths angle direction convention
        # angle from vehicle's north to wavetank's north
        self.theta = -offset_yaw

        rospy.loginfo('%s: theta: %s', self.name, self.theta)
        rospy.loginfo('%s: x offset: %s', self.name, X_OFFSET)
        rospy.loginfo('%s: y offset: %s', self.name, Y_OFFSET)

        # ros interface
        self.srv_path = rospy.ServiceProxy(SRV_PATH, PathService)
        self.sub_pil = rospy.Subscriber(TOPIC_STATUS, PathStatus, self.handle_status, queue_size=5)
        self.sub_energy = rospy.Subscriber(TOPIC_ENERGY, EnergyReport, self.handle_energy)
        # self.pub_pos = rospy.Publisher(TOPIC_POS, PilotRequest, queue_size=1)



    def handle_energy(self, msg):
        # parse energy data
        self.energy_last = msg.energy_used


    def handle_status(self, msg):
        # values
        self.time_started = msg.time_start

        if msg.path_status == PathStatus.PATH_COMPLETED:
            self.wait = False

            if self.experiment_running:
                self.experiment_completed = True
                self.experiment_running = False

                # update counters
                self.duration = msg.time_elapsed
                self.energy_used = self.energy_last - self.energy_initial

                # dump experiment data
                self.append_data()


    def check_export(self):
        if os.path.exists(self.csv_file):

            with open(self.csv_file, 'r') as exp_file:
                reader = csv.reader(exp_file, delimiter=',')

                # check if an header is already there
                line = reader.next()

                if not line[0] == self.header[0]:
                    add_header = True
                else:
                    add_header = False

        else:
            add_header = True

        if add_header:
            with open(self.csv_file, 'a') as exp_file:
                writer = csv.writer(exp_file, delimiter=',')
                writer.writerow(self.header)

    def append_data(self):
        if self.energy_initial == 0:
            rospy.logwarn('%s: not logging energy ...', self.name)

        with open(self.csv_file, 'a') as exp_file:
            writer = csv.writer(exp_file, delimiter=',')

            # for easier to string conversion
            point_a = list(self.points[0])
            point_b = list(self.points[1])

            row = list()
            row.append(self.time_started)
            row.append(self.theta)
            row.extend(point_a)
            row.extend(point_b)
            row.append(self.duration)
            row.append(self.energy_used)

            rospy.loginfo('%s: saving data to %s', self.name, self.csv_file)
            writer.writerow(row)



    def run(self):
        # check data file
        self.check_export()

        # reset the path controller
        try:
            self.srv_path.call(command='reset')
        except Exception:
            rospy.logerr('%s unable to communicate with path service ...')

        # calculate offsets and points
        rot_matrix = np.array([
            [np.cos(self.theta), -np.sin(self.theta)],
            [np.sin(self.theta), np.cos(self.theta)]
        ])
        xy_A = np.array([OFFSET_A_NORTH, OFFSET_A_EAST])
        xy_A = np.dot(xy_A, rot_matrix)
        xy_B = np.array([OFFSET_B_NORTH, OFFSET_B_EAST])
        xy_B = np.dot(xy_B, rot_matrix)

        self.points = np.array([
            [xy_A[0], xy_A[1], DEPTH, 0, 0, 0],
            [xy_B[0], xy_B[1], DEPTH, 0, 0, 0]
        ])

        self.points[:, 0] += X_OFFSET
        self.points[:, 1] += Y_OFFSET

        # reach initial point
        rospy.loginfo('%s: reaching initial point ...', self.name)

        path_points = [
            # HWU wavetank
            Vector6(self.points[0])
        ]
        options = [
            # HWU wavetank
            KeyValue('mode', 'simple'),
            KeyValue('timeout', '10000')
        ]

        try:
            self.wait = True
            self.srv_path.call(command='path', points=path_points, options=options)
            self.srv_path.call(command='start')
        except Exception:
            self.wait = False
            rospy.logerr('%s unable to communicate with path service ...')


        while self.wait:
            if rospy.is_shutdown():
                sys.exit(-1)
            else:
                rospy.sleep(0.5)

        # wait a little bit
        rospy.sleep(5)

        # experiment
        rospy.loginfo('%s: starting experiment ...', self.name)


        path_points = [
            # HWU wavetank
            Vector6([1.0, -1.8, DEPTH, 0, 0, 0.0]),
            Vector6([4.4, -9.8, DEPTH, 0, 0, 0.0]),
            Vector6([1.0, -9.8, DEPTH, 0, 0, 3.14]),
            Vector6([4.4, -1.8, DEPTH, 0, 0, 0.0]),
            Vector6([1.0, -1.8, DEPTH, 0, 0, 3.14]),
        ]
        options = [
            KeyValue('mode', self.mode),
            KeyValue('timeout', '1000'),
            KeyValue('target_speed', '0.5')
        ]


        try:
            self.experiment_running = True
            self.wait = True
            self.energy_initial = self.energy_last
            self.t_expire = time.time() + 1000
            self.t_last = time.time()

            rospy.loginfo('%s: requesting path (mode: %s) ...', self.name, self.mode)
            self.srv_path.call(command='path', points=path_points, options=options)
            self.srv_path.call(command='start')

        except Exception:
            self.wait = False
            rospy.logerr('%s unable to communicate with path service ...', self.name)


        while self.wait:
            if rospy.is_shutdown():
                rospy.logerr('%s: experiment timeout ...', self.name)
                break
            elif self.t_last >= self.t_expire:
                rospy.logerr('%s: ros is shutdown ...', self.name)
                break
            else:
                rospy.sleep(2.0)

        try:
            self.srv_path.call(command='reset')
        except Exception:
            rospy.logerr('%s unable to communicate with path service ...', self.name)

        # congratulations!
        rospy.loginfo('%s: experiment completed ...', self.name)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Utility for running wavetank experiments using the path controller.',
        epilog='This is part of vehicle_diagnostics module.'
    )

    # navigation group
    # parser.add_argument('n_offset', type=float, help='North offset of initial point in wavetank coordinates.')
    # parser.add_argument('e_offset', type=float, help='East offset of initial point in wavetank coordinates.')
    parser.add_argument('yaw_offset', type=float, help='Yaw offset between magnetic north and wavetank coordinates.')
    parser.add_argument('--mode', default='lines', help='Select the navigation mode.')

    # output group
    parser.add_argument('--output', default='last', help='Output file to save during the experiments.')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print detailed information.')

    args = parser.parse_args()

    # init the ROS node
    rospy.init_node('wavetank_experiment')
    name = rospy.get_name()

    # config
    # off_n = args.n_offset
    # off_e = args.e_offset
    off_yaw = np.deg2rad(args.yaw_offset)
    suffix = args.output

    # if off_n > 10 or off_n < -10:
    #     rospy.logfatal('%s: wrong input commands', name)
    #     sys.exit(-1)
    #
    # if off_e > 10 or off_e < -10:
    #     rospy.logfatal('%s: wrong input commands', name)
    #     sys.exit(-1)

    if off_yaw > np.pi or off_yaw < -np.pi:
        rospy.logfatal('%s: wrong input commands', name)
        sys.exit(-1)

    # run the experiment
    # offset_north=off_n, offset_east=off_e,
    we = WavetankExperiment(name, offset_yaw=off_yaw, suffix=suffix, mode=args.mode)

    try:
        we.run()
    except Exception:
        tb = traceback.format_exc()
        rospy.logfatal('%s: uncaught exception:\n%s', name, tb)
        sys.exit(-1)
