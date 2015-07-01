#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
import os
import traceback
import argparse

import json
import time

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('saetta_energy')


from vehicle_core.path import trajectory_tools as tt

from saetta_energy import tsp
from saetta_energy.utils import *

from vehicle_interface.msg import PathRequest, PathStatus, PilotStatus, Vector6
from vehicle_interface.srv import BooleanService, PathService, PathServiceRequest, PathServiceResponse
from saetta_energy.msg import EnergyReport, EnergyStatus
from diagnostic_msgs.msg import KeyValue


TOPIC_PATH_REQ = 'path/request'
TOPIC_PATH_STS = 'path/status'
TOPIC_PATH_SRV = 'path/control'
TOPIC_PILOT_STS = 'pilot/status'
TOPIC_ENERGY = 'saetta/report'

DEFAULT_RATE = 1.0      # Hz


class InspectionMission(object):

    def __init__(self, **kwargs):
        self.name = rospy.get_name()

        # mission data
        self.ips = []
        self.actions = []

        kips = kwargs.get('ips', None)

        if kips is not None:
            self.ips = np.array(kips)

        # TODO: raise error if ips is not of proper shape

        # path monitoring
        self.current_path_id = -1
        self.last_path_id = 0
        self.path_enabled = False

        # output
        self.output_dir = kwargs.get('output_dir', os.path.expanduser('~'))
        self.output_label = kwargs.get('output_label', 'current')
        self.output_log = None

        self._init_log(self.output_label)

        self.time_start = 0.0
        self.time_end = 0.0
        self.time_elapsed = 0.0
        self.energy_last = 0.0
        self.energy_start = 0.0

        # ros interface
        self.pub_path = rospy.Publisher(TOPIC_PATH_REQ, PathRequest, queue_size=1)
        self.sub_path = rospy.Subscriber(TOPIC_PATH_STS, PathStatus, self.handle_path_status, queue_size=10)
        self.srv_path = rospy.ServiceProxy(TOPIC_PATH_SRV, PathService)
        self.sub_energy = rospy.Subscriber(TOPIC_ENERGY, EnergyReport, self.handle_energy, queue_size=10)

        # rate
        self.r_main = rospy.Rate(DEFAULT_RATE)


    def handle_energy(self, data):
        self.energy_last = data.energy_used

    def _init_log(self, label):
        output_file = '{}_{}.csv'.format(label, id_generator(6))
        self.output_log = os.path.join(self.output_dir, output_file)

        # init output log
        if not os.path.exists(self.output_log):
            rospy.loginfo('%s: saving mission output to file (%s)', self.name, self.output_log)

            with open(self.output_log, 'wt') as mlog:
                mlog.write('name,action,time_start,time_end,time_elapsed,energy_used\n')

    def _write_log(self, label, action):
        # record action stats
        self.time_end = time.time()
        self.time_elapsed = self.time_end - self.time_start
        self.energy_used = self.energy_last - self.energy_start

        # generate mission log
        out = '{},{},{},{},{},{}\n'.format(
            label, action, self.time_start, self.time_end, self.time_elapsed, self.energy_used
        )

        # save mission log
        with open(self.output_log, 'at') as mlog:
            mlog.write(out)


    # TEMP: these are mocking the action system (to be introduced in the future releases)
    def handle_path_status(self, data):
        self.last_path_id = data.path_id

        if not self.path_enabled:
            return

        if data.path_status in (PathStatus.PATH_ABORT, PathStatus.PATH_TIMEOUT):
            self.path_enabled = False

        if data.path_status == PathStatus.PATH_COMPLETED:
            if data.navigation_status == PathStatus.NAV_HOVERING:
                self.path_enabled = False

    def _send_path_request(self, poses, **kwargs):
        self.path_enabled = True

        # params
        mode = kwargs.get('mode', 'fast')
        timeout = kwargs.get('timeout', 1000)
        target_speed = kwargs.get('target_speed', 1.0)
        look_ahead = kwargs.get('look_ahead', 5.0)

        points = [Vector6(x) for x in poses]

        msg = PathRequest()
        msg.header.stamp = rospy.Time.now()
        msg.command = 'path'
        msg.points = points
        msg.options = [
            KeyValue('mode', mode),
            KeyValue('timeout', str(timeout)),
            KeyValue('target_speed', str(target_speed)),
            KeyValue('look_ahead', str(look_ahead)),
        ]

        self.pub_path.publish(msg)

    def _reset_path(self):
        """This functions uses a service request to reset the state of the path controller"""
        try:
            self.srv_path.call(command=PathServiceRequest.CMD_RESET)
        except Exception:
            rospy.logerr('%s: unable to communicate with path service ...', self.name)


    def run(self):
        if len(self.ips) < 1:
            rospy.signal_shutdown('mission abort')
            return

        # initial plan
        k = len(self.ips)
        self.ips_state = {'IP_%d' % n: 0 for n in xrange(k)}
        self.ips_costs = np.zeros((k, k))

        # config
        e_perf = 300.0          # navigation cost (J/m)

        # update costs
        for i in xrange(k):
            for j in xrange(k):

                if i == j:
                    continue

                dist = tt.distance_between(self.ips[i], self.ips[j], spacing_dim=3)
                cost = e_perf * dist

                self.ips_costs[i][j] = cost

        # optimal route
        route, cost, _ = tsp.tsp_problem(self.ips_state.keys(), self.ips_costs)

        rospy.loginfo('%s: found initial route:\n%s', self.name, route)

        # generate action sequence
        for r in route:
            idx = int(r.split('_')[1]) - 1      # extract route index (1-based)

            self.actions.append({
                'name': 'goto',
                'pose': self.ips[idx]
            })

        rospy.loginfo('%s: action sequence:\n%s', self.name, self.actions)

        # mission init
        self._reset_path()
        rospy.sleep(3.0)

        while not rospy.is_shutdown():
            # execute current state
            # ...

            # wait a bit
            self.r_main.sleep()



def parse_arguments(args=None):
    parser = argparse.ArgumentParser(
        description='Utility for running inspection missions.',
        epilog='This is part of saetta_energy module.'
    )

    # mission group
    parser.add_argument('config', type=str, help='External mission config (JSON file).')

    # output group
    parser.add_argument('--output', default='~', help='Output dir to save mission logs.')
    parser.add_argument('--label', default='current', help='Optional comment to add to the result file.')
    parser.add_argument('-v', '--verbose', action='store_true', help='Print detailed information.')

    if args is not None:
        return parser.parse_args(args)

    return parser.parse_args()

def main():
    # parse arguments
    rargv = rospy.myargv()

    if rargv > 0:
        args = parse_arguments(rargv[1:])
    else:
        args = parse_arguments()

    config_file = args.config
    output_dir = args.output
    output_label = args.label

    if output_dir == '~':
        output_dir = os.path.expanduser('~')

    # start ros node
    rospy.init_node('mission_executor')
    name = rospy.get_name()
    rospy.loginfo('%s: init', name)

    # load mission config
    try:
        with open(config_file, 'rt') as mf:
            config = mf.read()

        config = json.loads(config)
    except Exception:
        rospy.logfatal('%s could not load the config file (%s)', name, config_file)
        rospy.logfatal(traceback.format_exc())
        sys.exit(-1)

    # handle config
    current_config = dict()
    current_config.update(config)
    current_config.update({
        'output_dir': output_dir,
        'output_label': output_label
    })

    # init the executor
    me = InspectionMission(**current_config)

    # load inspection
    # ...

    # run the mission
    me.run()

if __name__ == '__main__':
    main()
