#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
import os
import traceback
import argparse
import json
import time
import copy

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('saetta_energy')

from saetta_energy import tsp
from saetta_energy.utils import id_generator, urlify
from vehicle_core.path import trajectory_tools as tt

from auv_msgs.msg import NavSts
from vehicle_interface.msg import PathRequest, PathStatus, PilotStatus, Vector6, FloatArrayStamped
from vehicle_interface.srv import BooleanService, PathService, PathServiceRequest, PathServiceResponse
from saetta_energy.msg import EnergyReport, EnergyStatus
from diagnostic_msgs.msg import KeyValue


TOPIC_NAV = 'nav/nav_sts'
TOPIC_PATH_REQ = 'path/request'
TOPIC_PATH_STS = 'path/status'
TOPIC_PATH_SRV = 'path/control'
TOPIC_PILOT_STS = 'pilot/status'
TOPIC_ENERGY = 'saetta/report'
TOPIC_MAP_EJM = 'saetta/map/energy'

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
        self.n_bins = int(rospy.get_param('saetta/path/n_bins', 8))                 # number of direction bins
        self.initial_ejm = float(rospy.get_param('saetta/path/initial_ejm', 70.0))  # navigation cost (J/m)
        self.map_ejm = self.initial_ejm * np.ones(self.n_bins)                      # map of navigation costs
        self.route_optimization = False

        # update bins and add some slack
        self.phi_edges = np.linspace(-np.pi, np.pi, self.n_bins + 1)                # direction bins (edges)
        self.phi_edges[0] -= 0.001
        self.phi_edges[-1] += 0.001

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
        self.current_action = {}

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
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self.handle_nav, queue_size=10)
        self.pub_path = rospy.Publisher(TOPIC_PATH_REQ, PathRequest, queue_size=10)
        self.sub_path = rospy.Subscriber(TOPIC_PATH_STS, PathStatus, self.handle_path_status, queue_size=10)
        self.srv_path = rospy.ServiceProxy(TOPIC_PATH_SRV, PathService)
        self.sub_energy = rospy.Subscriber(TOPIC_ENERGY, EnergyReport, self.handle_energy, queue_size=10)
        self.sub_map_ejm = rospy.Subscriber(TOPIC_MAP_EJM, FloatArrayStamped, self.handle_map_ejm, queue_size=10)

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
            mlog.flush()

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

    def handle_map_ejm(self, data):
        # support change in the map (assuming linear spacing from -pi to pi)
        self.map_ejm = np.array(data.values)

        if len(data.values) != self.n_bins:
            # update bins and add some slack
            self.n_bins = len(data.values)
            self.phi_edges = np.linspace(-np.pi, np.pi, self.n_bins + 1)
            self.phi_edges[0] -= 0.001
            self.phi_edges[-1] += 0.001


    def state_idle(self):
        pass

    def state_run(self):
        if self.state_action == ACTION_IDLE:
            try:
                # select action
                self.current_action = self.actions[self.action_id]

                # start action
                self.dispatch_action(self.current_action)
                self.state_action = ACTION_RUNNING
            except IndexError:
                rospy.loginfo('%s: no more action to execute ...', self.name)
                self.state_mission = MISSION_COMPLETED

        elif self.state_action == ACTION_FAILED:
            rospy.loginfo('%s: action %d failed, aborting mission ...', self.name, self.action_id)
            self.state_mission = MISSION_ABORT

        elif self.state_action == ACTION_ACHIEVED:
            rospy.loginfo('%s: action %d completed, continuing mission ...', self.name, self.action_id)

            self.evaluate_action_result()
            self.state_action = ACTION_IDLE
        else:
            pass

    def state_abort(self):
        pass

    def state_completed(self):
        rospy.signal_shutdown('mission completed')


    def evaluate_action_result(self):
        if self.current_action['name'] != 'hover':
            self.action_id += 1
            return

        # increment action counter
        self.action_id += 1

        # mark inspection point as visited
        try:
            self.ips_state[self.current_action['ips']] = 1
            self.hop_counter += 1
        except KeyError:
            return

        # evaluate residuals
        hops_costs, _ = self._calculate_cost_route(self.route[self.hop_counter:])
        prev_cost = self.route_cost_hops[self.hop_counter:]

        rospy.loginfo('%s: residual updates:\n  prev: %s\n  new: %s', self.name, sum(prev_cost), sum(hops_costs))

        # (ip achieved) calculate remaining inspection points
        ips_togo = [ip for ip, s in self.ips_state.iteritems() if s == 0]
        ips_togo = sorted(ips_togo, key=lambda x: int(x.split('IP_')[1]))

        # (optimize if there are at least two ips)
        if self.route_optimization and len(ips_togo) > 1:
            rospy.loginfo('%s: route optimization, remaining inspection points %d', self.name, len(ips_togo))

            self._plan(ips_togo)
            self.action_id = 0


    def dispatch_action(self, action):
        rospy.loginfo('%s: dispatching action[%d]: %s', self.name, self.action_id, action['name'])

        # action stats
        self.last_action = action
        self.time_start = time.time()
        self.energy_start = self.energy_last

        if action['name'] == ACT_GOTO:
            params = action['params']
            goal = params.pop('pose')
            poses = [goal]

            # reporting
            self.last_los_orient = tt.calculate_orientation(self.pos, goal)
            self.last_los_dist = tt.distance_between(self.pos, goal, spacing_dim=3)

            self.send_path_request(poses, **params)
        elif action['name'] == ACT_HOVER:
            params = action['params']
            goal = params.pop('pose')
            mode = params.pop('mode', 'simple')
            poses = [goal]

            self.send_path_request(poses, mode=mode, **params)
        else:
            rospy.logerr('%s: unknown action: %s', self.name, action)

    def handle_feedback(self, status):
        # if self.state_action in (ACTION_RUNNING, ACTION_IDLE):
        #     return

        # update action stats
        self.time_end = time.time()
        self.time_elapsed = self.time_end - self.time_start
        self.energy_used = self.energy_last - self.energy_start

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
        self.route_optimization = bool(config.get('route_optimization', False))

        if self.route_optimization:
            rospy.loginfo('%s: route optimization active (mission will be replanned at runtime)')

        if config.get('output_label', None) is not None:
            self.output_label = config['output_label']
            self.output_log = os.path.join(self.output_dir, '{}_{}.csv'.format(self.output_label, id_generator(6)))

        # init mission
        self.state_mission = MISSION_IDLE
        self._init_log(self.output_label)

        # insert inspection points
        self.ips_dict = {'IP_%d' % n: inspection_points[n, :] for n in xrange(inspection_points.shape[0])}
        self.ips_state = {'IP_%d' % n: 0 for n in xrange(inspection_points.shape[0])}

        # add virtual inspection point (auv position)
        self.ips_dict['AUV'] = self.pos
        self.ips_state['AUV'] = -1

        # select ips to visit
        ips_labels = [ip for ip, s in self.ips_state.iteritems() if s == 0]
        ips_labels = sorted(ips_labels, key=lambda x: int(x.split('IP_')[1]))

        # initial plan
        self._plan(ips_labels)

        # change state
        self.state_mission = MISSION_RUN


    def _calculate_cost_hop(self, wp_a, wp_b):
        dist = tt.distance_between(wp_a, wp_b, spacing_dim=3)

        yaw_los = tt.calculate_orientation(wp_a, wp_b)
        yaw_idx = np.digitize([yaw_los], self.phi_edges)[0] - 1

        if yaw_idx >= 0 and yaw_idx < self.n_bins:
            cost = self.map_ejm[yaw_idx] * dist
        else:
            rospy.logwarn('%s: binning failed, using default cost (yaw_los: %.2f, yaw_idx: %d, n_bins: %d)', self.name, yaw_los, yaw_idx, self.n_bins)
            cost = self.initial_ejm * dist

        return cost

    def _calculate_cost_route(self, route):
        cost_hops = []

        for n in xrange(len(route)):
            wp_a = self.ips_dict[route[n - 1]]
            wp_b = self.ips_dict[route[n]]

            cost_hops.append(self._calculate_cost_hop(wp_a, wp_b))

        return cost_hops, sum(cost_hops)


    def _plan_tsp(self, labels):
        if not tsp.HAS_GUROBI:
            rospy.logwarn('%s: tsp solver not available, using naive route ...', self.name)

        # update vehicle position
        self.ips_dict['AUV'] = self.pos

        # create inspection list
        ips_labels = ['AUV']
        ips_labels.extend(labels)

        k = len(ips_labels)
        ips_costs = np.zeros((k, k))

        # update travel costs
        for i in xrange(k):
            for j in xrange(k):
                if i == j:
                    continue

                ips_costs[i][j] = self._calculate_cost_hop(self.ips_dict[ips_labels[i]], self.ips_dict[ips_labels[j]])

        try:
            # ask solver for optimal route
            route, tsp_cost, _ = tsp.solve_problem(ips_labels, ips_costs)
        except:
            rospy.logwarn('%s: tsp solver error, using naive solver ...')
            route, tsp_cost, _ = tsp.naive_solve(ips_labels, ips_costs)

        # intermediate costs
        cost_hops = []

        for n in xrange(len(route)):
            i = ips_labels.index(route[n - 1])
            j = ips_labels.index(route[n])

            cost_hops.append(ips_costs[i][j])

        # remove initial virtual point
        route = route[1:]
        cost_hops = cost_hops[1:]
        cost_total = np.sum(cost_hops)

        return route, cost_hops, cost_total

    def _plan(self, labels):
        # plan route
        self.route, self.route_cost_hops, self.route_cost_tot = self._plan_tsp(labels)
        rospy.loginfo('%s: found inspection route:\n%s', self.name, self.route)

        # route monitor
        self.hop_counter = 0

        # generate action sequence
        self.actions = []

        # first hover on spot (with los for first IP)
        next_pose = np.copy(self.pos)
        next_pose[5] = tt.calculate_orientation(self.pos, self.ips_dict[self.route[0]])

        self.actions.append({
            'name': 'hover',
            'params': {
                'pose': next_pose.tolist()
            }
        })

        # second execute the route (skips the initial piece (going to AUV position)
        for n in xrange(len(self.route)):
            next_pose = np.copy(self.ips_dict[self.route[n]])

            self.actions.append({
                'name': 'goto',
                'params': {
                    'pose': self.ips_dict[self.route[n]].tolist()
                },
                'cost': self.route_cost_hops[n],
            })

            if n < len(self.route) - 1:
                next_pose[5] = tt.calculate_orientation(self.ips_dict[self.route[n]], self.ips_dict[self.route[n + 1]])

            self.actions.append({
                'name': 'hover',
                'params': {
                    'pose': next_pose.tolist()
                },
                'ips': self.route[n]
            })

        rospy.loginfo('%s: generated action sequence (n=%d)', self.name, len(self.actions))

        # save mission plan
        plan_file = os.path.join(self.output_dir, '{}_plan_{}.json'.format(self.output_label, self.plan_id))

        plan = {
            'id': self.plan_id,
            'time': time.time(),
            'ips_count': len(labels),
            'route': self.route,
            'cost_hops': self.route_cost_hops,
            'cost_total': self.route_cost_tot,
            'actions': self.actions,
        }

        with open(plan_file, 'wt') as plog:
            plog.write(json.dumps(plan, indent=2))

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
    parser.add_argument('--label', default='label', help='Optional comment to add to the result file.')
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

    # add initial wait for maps
    try:
        rospy.wait_for_message(TOPIC_MAP_EJM, FloatArrayStamped, timeout=5)
    except rospy.ROSException:
        rospy.logwarn('%s: no map received proceeding with internal estimation ...', rospy.get_name())
    else:
        rospy.loginfo('%s: received initial estimation:\n%s', rospy.get_name(), me.map_ejm)

    # setup the mission
    me.execute_mission(config)

    # start the mission
    me.run()

if __name__ == '__main__':
    main()
