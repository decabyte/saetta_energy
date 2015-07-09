#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
import os
import traceback
import argparse
import json
import time
import collections

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('saetta_energy')

from vehicle_core.path import trajectory_tools as tt
from saetta_energy import tsp
from saetta_energy.utils import id_generator, urlify

from auv_msgs.msg import NavSts
from vehicle_interface.msg import PathRequest, PathStatus, PilotStatus, Vector6
from vehicle_interface.srv import BooleanService, PathService, PathServiceRequest, PathServiceResponse
from saetta_energy.msg import EnergyReport, EnergyStatus
from diagnostic_msgs.msg import KeyValue


TOPIC_NAV = 'nav/sts'
TOPIC_PATH_REQ = 'path/request'
TOPIC_PATH_STS = 'path/status'
TOPIC_PATH_SRV = 'path/control'
TOPIC_PILOT_STS = 'pilot/status'
TOPIC_ENERGY = 'saetta/report'

MISSION_IDLE = 'idle'
MISSION_RUN = 'run'
MISSION_COMPLETED = 'completed'
MISSION_ABORT = 'abort'

ACTION_IDLE = 'idle'
ACTION_RUNNING = 'running'
ACTION_ACHIEVED = 'achieved'
ACTION_FAILED = 'failed'

DEFAULT_RATE = 1.0      # Hz

# actions
ACT_GOTO = 'goto'
ACT_HOVER = 'hover'
ACT_CURR = 'estimate_currents'


class MissionExecutor(object):

    def __init__(self, **kwargs):
        self.name = rospy.get_name()

        # mission data
        self.config = None
        self.plan_id = 0
        self.ips_dict = []
        self.actions = []

        # estimation config
        self.e_perf = 300.0          # navigation cost (J/m)

        # mission state machine
        self.state_mission = MISSION_IDLE
        self.map_state_mission = {
            MISSION_IDLE: self.state_idle,
            MISSION_RUN: self.state_run,
            MISSION_COMPLETED: self.state_completed,
            MISSION_ABORT: self.state_abort
        }

        # action state machine
        self.action_id = 0
        self.state_action = ACTION_IDLE
        self.last_action = None

        # vehicle state
        self.pos = np.zeros(6, dtype=np.float64)

        # path monitoring
        self.current_path_id = -1
        self.last_path_id = 0
        self.path_enabled = False

        # actions states to mission states
        self.map_action_mission = {
            ACTION_FAILED: MISSION_ABORT,
            ACTION_ACHIEVED: MISSION_RUN,
            ACTION_RUNNING: MISSION_RUN,
            ACTION_IDLE: MISSION_RUN
        }

        # output
        self.output_dir = kwargs.get('output_dir', os.path.expanduser('~'))
        self.output_label = kwargs.get('output_label', 'current')
        self.output_log = os.path.join(self.output_dir, '{}_{}.csv'.format(self.output_label, id_generator(6)))

        # action stats
        self.time_start = 0.0
        self.time_end = 0.0
        self.time_elapsed = 0.0
        self.energy_last = 0.0
        self.energy_start = 0.0
        self.last_los_orient = 0.0
        self.last_los_dist = 0.0

        # ros interface
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_nav, queue_size=1)
        self.pub_path = rospy.Publisher(TOPIC_PATH_REQ, PathRequest, queue_size=1)
        self.sub_path = rospy.Subscriber(TOPIC_PATH_STS, PathStatus, self.handle_path_status, queue_size=10)
        self.srv_path = rospy.ServiceProxy(TOPIC_PATH_SRV, PathService)
        self.sub_energy = rospy.Subscriber(TOPIC_ENERGY, EnergyReport, self.handle_energy, queue_size=10)

        # rate
        self.r_main = rospy.Rate(DEFAULT_RATE)


    def _init_log(self, label):
        output_file = '{}_{}.csv'.format(label, id_generator(6))
        self.output_log = os.path.join(self.output_dir, output_file)

        # init output log
        if not os.path.exists(self.output_log):
            rospy.loginfo('%s: saving mission output to file (%s)', self.name, self.output_log)

            with open(self.output_log, 'wt') as mlog:
                mlog.write('id,action,label,time_start,time_end,time_elapsed,energy_used,direction,distance\n')

    def _write_log(self, label, action):
        # record action stats
        self.time_end = time.time()
        self.time_elapsed = self.time_end - self.time_start
        self.energy_used = self.energy_last - self.energy_start

        extra = '0.0,0.0'

        if action['name'] == ACT_GOTO:
            extra ='{},{}'.format(self.last_los_orient, self.last_los_dist)

        # generate mission log
        out = '{},{},{},{},{},{},{},{}\n'.format(
            self.action_id, action['name'], label, self.time_start, self.time_end,
            self.time_elapsed, self.energy_used, extra
        )

        # save mission log
        with open(self.output_log, 'at') as mlog:
            mlog.write(out)

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

    def handle_energy(self, data):
        self.energy_last = data.energy_used

    def state_idle(self):
        pass

    def state_run(self):
        if self.state_action == ACTION_IDLE:
            try:
                current_action = self.actions[self.action_id]

                # start a new action
                self.dispatch_action(current_action)
                self.state_action = ACTION_RUNNING
            except IndexError:
                rospy.loginfo('%s: no more action to execute ...', self.name)
                self.state_mission = MISSION_COMPLETED

        elif self.state_action == ACTION_FAILED:
            rospy.loginfo('%s: action %d failed, aborting mission ...', self.name, self.action_id)
            self.state_mission = MISSION_ABORT

        elif self.state_action == ACTION_ACHIEVED:
            rospy.loginfo('%s: action %d completed, continuing mission ...', self.name, self.action_id)

            self.state_action = ACTION_IDLE
            self.action_id += 1

        else:
            pass

    def state_abort(self):
        pass

    def state_completed(self):
        rospy.signal_shutdown('mission completed')

    def dispatch_action(self, action):
        rospy.loginfo('%s: dispatching action[%d]: %s', self.name, self.action_id, action['name'])

        # action stats
        self.last_action = action
        self.time_start = time.time()
        self.energy_start = self.energy_last

        if action['name'] == ACT_GOTO:
            goal = action['params']['pose']
            poses = []
            poses.append(goal)

            self.last_los_orient = tt.calculate_orientation(self.pos, goal)
            self.last_los_dist = tt.distance_between(self.pos, goal, spacing_dim=3)

            self.send_path_request(poses)
        elif action['name'] == ACT_HOVER:
            goal = action['params']['pose']
            poses = []
            poses.append(goal)

            self.send_path_request(poses, mode='simple')
        else:
            rospy.logerr('%s: unknown action: %s', self.name, action)

    def handle_feedback(self, status):
        if self.state_action in (ACTION_RUNNING, ACTION_IDLE):
            return

        # record action stats
        self._write_log(self.output_label, self.last_action)

        # update status
        self.state_action = status

    # TEMP: mock the action system with path requests
    def handle_path_status(self, data):
        self.last_path_id = data.path_id

        if not self.path_enabled:
            return

        if self.state_action != ACTION_RUNNING:
            return

        if data.path_status in (PathStatus.PATH_ABORT, PathStatus.PATH_TIMEOUT):
            self.path_enabled = False
            self.handle_feedback(ACTION_FAILED)

        if data.path_status == PathStatus.PATH_COMPLETED:
            if data.navigation_status == PathStatus.NAV_HOVERING:
                self.path_enabled = False
                self.handle_feedback(ACTION_ACHIEVED)

    # TEMP: mock the action system with path requests
    def send_path_request(self, poses, **kwargs):
        self.path_enabled = True

        # params
        mode = kwargs.get('mode', 'fast')
        timeout = kwargs.get('timeout', 5 * 60)
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

    def reset_path(self):
        """This functions uses a service request to reset the state of the path controller"""
        try:
            self.srv_path.call(command=PathServiceRequest.CMD_RESET)
        except Exception:
            rospy.logerr('%s: unable to communicate with path service ...', self.name)

    def execute_mission(self, config, **kwargs):
        if self.config is not None:
            rospy.logwarn('%s: executing mission, please abort current execution first ...')
            return

        # process mission data
        inspection_points = config.get('ips', None)

        if self.ips_dict is None:
            rospy.logwarn('%s: wrong mission input, skipping ...', self.name)
            return
        else:
            inspection_points = np.array(inspection_points)
            inspection_points = np.atleast_2d(inspection_points)

        # store config
        self.config = config

        if config.get('output_label', None) is not None:
            self.output_label = config['output_label']
            self.output_log = os.path.join(self.output_dir, '{}_{}.csv'.format(self.output_label, id_generator(6)))

        # init mission
        self.state_mission = MISSION_IDLE
        self._init_log(self.output_label)

        # initial plan
        self.ips_dict = {'IP_%d' % n: inspection_points[n, :] for n in xrange(inspection_points.shape[0])}
        self.ips_state = {'IP_%d' % n: 0 for n in xrange(inspection_points.shape[0])}

        self._plan()

        # change state
        self.state_mission = MISSION_RUN


    def _plan_tsp(self):
        # select ips to visit
        ips_labels = [ip for ip, s in self.ips_state.iteritems() if s == 0]

        k = len(ips_labels)
        self.ips_costs = np.zeros((k, k))

        # update costs
        for i in xrange(k):
            for j in xrange(k):
                if i == j:
                    continue

                dist = tt.distance_between(self.ips_dict[ips_labels[i]], self.ips_dict[ips_labels[j]], spacing_dim=3)
                cost = self.e_perf * dist

                self.ips_costs[i][j] = cost

        # optimal route
        route, cost, _ = tsp.tsp_problem(ips_labels, self.ips_costs)

        rospy.loginfo('%s: found inspection route:\n%s', self.name, route)

    def _plan(self):
        k = len(self.ips_dict)
        self.ips_costs = np.zeros((k, k))

        # generate standard route (default ordering)
        route = sorted(self.ips_state.keys(), key=lambda x: int(x.split('IP_')[1]))

        # generate action sequence
        #   first hover on spot (with los for first IP)
        next_pose = np.copy(self.pos)
        next_pose[5] = tt.calculate_orientation(self.pos, self.ips_dict[route[0]])

        self.actions.append({
            'name': 'hover',
            'params': {
                'pose': next_pose.tolist()
            }
        })

        for n in xrange(len(route)):
            self.actions.append({
                'name': 'goto',
                'params': {
                    'pose': self.ips_dict[route[n]].tolist()
                }
            })

            next_pose = np.copy(self.ips_dict[route[n]])

            if n < len(route) - 1:
                next_pose[5] = tt.calculate_orientation(self.ips_dict[route[n]], self.ips_dict[route[n + 1]])

            self.actions.append({
                'name': 'hover',
                'params': {
                    'pose': next_pose.tolist()
                }
            })

        rospy.loginfo('%s: generated action sequence (n=%d)', self.name, len(self.actions))

        # save mission plan
        plan_file = os.path.join(self.output_dir, '{}_plan_{}.json'.format(self.output_label, self.plan_id))

        plan = {
            'ips_count': k,
            #'ips_label': ips_labels,
            'ips_state': self.ips_costs.tolist(),
            'time': time.time(),
            'actions': self.actions,
        }

        with open(plan_file, 'wt') as plog:
            plog.write(json.dumps(plan))

        # increase plan counter (to keep track of future replans)
        self.plan_id += 1


    def run(self):
        # mission init
        self.reset_path()
        rospy.sleep(3.0)

        while not rospy.is_shutdown():
            # execute current state
            self.map_state_mission[self.state_mission]()

            # wait a bit
            self.r_main.sleep()


def parse_arguments(args=None):
    parser = argparse.ArgumentParser(
        description='Utility for running inspection missions with the executor module.',
        epilog='This is part of saetta_energy module.'
    )

    # mission group
    parser.add_argument('config', type=str, help='Mission config file (JSON format).')

    # output group
    parser.add_argument('--output', default=None, help='Output dir to save mission logs.')
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

    if output_dir is None:
        output_dir = os.getcwd()

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
        rospy.logerr(traceback.format_exc())
        rospy.logfatal('%s could not load the config file (%s)', name, config_file)
        sys.exit(-1)

    ex_config = {
        'output_dir': output_dir,
        'output_label': output_label
    }

    # init the executor
    me = MissionExecutor(**ex_config)
    me.execute_mission(config)

    # start the mission
    me.run()

if __name__ == '__main__':
    main()
