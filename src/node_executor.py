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

from diagnostic_msgs.msg import KeyValue
from actionbus.msg import ActionDispatch, ActionFeedback
from saetta_energy.msg import BatteryStatus, RegressionResult
from auv_msgs.msg import NavSts

TOPIC_NAV = 'nav/nav_sts'
TOPIC_PILOT_STS = 'pilot/status'

TOPIC_BAT = 'saetta/battery'
TOPIC_EJM = 'saetta/map/energy'
TOPIC_SPD = 'saetta/map/speed'

TOPIC_PATH_REQ = 'path/request'
TOPIC_PATH_STS = 'path/status'
TOPIC_PATH_SRV = 'path/control'

TOPIC_DISPATCH = 'action/dispatch'
TOPIC_FEEDBACK = 'action/feedback'

MISSION_IDLE = 'idle'
MISSION_RUN = 'run'
MISSION_COMPLETED = 'completed'
MISSION_ABORT = 'abort'

DEFAULT_RATE = 2    # Hz

# actions
ACT_GOTO = 'goto'
ACT_HOVER = 'hover'


class MissionExecutor(object):

    def __init__(self, **kwargs):
        self.name = rospy.get_name()

        # mission data
        self.config = None
        self.plan_id = 0
        self.ips_dict = []
        self.actions = []
        self.route_optimization = False

        self.initial_ejm = float(rospy.get_param('saetta/path/initial_ejm', 70.0))  # navigation cost (J/m)
        self.initial_spd = float(rospy.get_param('saetta/path/initial_spd', 0.75))  # cruise speed (m/s)
        self.est_acc = float(rospy.get_param('saetta/path/est_acc', 0.125))         # vehicle acceleration (m/s^2)
        self.est_dec = float(rospy.get_param('saetta/path/est_dec', 0.125))         # vehicle deceleration (m/s^2)

        # estimation config
        self.ejm_coeff = [0, self.initial_ejm]          # linear
        self.spd_coeff = [0, self.initial_spd]          # linear
        self.ejm_rmse = self.initial_ejm / 10.0         # initial rmse
        self.spd_rmse = self.initial_spd / 10.0         # initial rmse
        self.cost_offset = 0.5                          # small epsilon for adjusting the tsp problem

        # mission state machine
        self.mission_state = MISSION_IDLE
        self.map_state_mission = {
            MISSION_IDLE: self.state_idle,
            MISSION_RUN: self.state_run,
            MISSION_COMPLETED: self.state_completed,
            MISSION_ABORT: self.state_abort
        }

        # actions states to mission states
        self.map_action_mission = {
            ActionFeedback.ACTION_FAILED: MISSION_ABORT,
            ActionFeedback.ACTION_SUCCESS: MISSION_RUN,
            ActionFeedback.ACTION_RUNNING: MISSION_RUN,
            ActionFeedback.ACTION_IDLE: MISSION_RUN
        }

        # action state machine
        self.action_cnt = 0
        self.action_id = 0
        self.action_last = None
        self.action_state = ActionFeedback.ACTION_IDLE
        self.action_current = {}

        # vehicle state
        self.pos = np.zeros(6, dtype=np.float64)

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
        self.sub_bat = rospy.Subscriber(TOPIC_BAT, BatteryStatus, self.handle_battery, queue_size=10)

        self.sub_ejm = rospy.Subscriber(TOPIC_EJM, RegressionResult, self.handle_ejm, queue_size=10)
        self.sub_spd = rospy.Subscriber(TOPIC_SPD, RegressionResult, self.handle_spd, queue_size=10)

        # action interface
        self.pub_disp = rospy.Publisher(TOPIC_DISPATCH, ActionDispatch, queue_size=1)
        self.sub_feed = rospy.Subscriber(TOPIC_FEEDBACK, ActionFeedback, self.handle_feedback, queue_size=10)

        # rate
        self.r_main = rospy.Rate(DEFAULT_RATE)


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

    def handle_battery(self, data):
        self.energy_last = data.energy_max - data.energy_residual

    def handle_ejm(self, data):
        self.ejm_coeff = np.array(data.coeff)
        self.ejm_rmse = np.sqrt(data.mse)

    def handle_spd(self, data):
        self.spd_coeff = np.array(data.coeff)
        self.spd_rmse = np.sqrt(data.mse)

    def run(self):
        # mission init
        rospy.sleep(2.0)

        while not rospy.is_shutdown():
            # execute current state
            self.map_state_mission[self.mission_state]()
            self.r_main.sleep()

    def state_idle(self):
        if len(self.actions) < 1:
            return

        if self.action_id >= len(self.actions):
            rospy.loginfo('%s: no more action to execute ...', self.name)
            self.mission_state = MISSION_COMPLETED
            return

        # select action
        self.action_current = self.actions[self.action_id]
        self.action_state = ActionFeedback.ACTION_IDLE

        # start action and resume mission
        self.dispatch_action(self.action_current)
        self.mission_state = MISSION_RUN

    def state_run(self):
        if self.action_state in (ActionFeedback.ACTION_FAILED, ActionFeedback.ACTION_REJECT, ActionFeedback.ACTION_TIMEOUT):
            rospy.loginfo('%s: feedback action[%d]: failed', self.name, self.action_id)
            self.mission_state = MISSION_ABORT

        elif self.action_state == ActionFeedback.ACTION_SUCCESS:
            rospy.loginfo('%s: feedback action[%d]: success', self.name, self.action_id)

            self._success_callback()
            self.mission_state = MISSION_IDLE
        else:
            pass

    def state_abort(self):
        rospy.logwarn('%s: mission aborted', self.name)
        rospy.signal_shutdown('mission aborted')

    def state_completed(self):
        rospy.loginfo('%s: mission completed', self.name)
        rospy.signal_shutdown('mission completed')


    def _success_callback(self):
        # increment action counter
        self.action_id += 1

        # if self.action_current['name'] != 'hover':
        #     return

        # mark inspection point as visited
        try:
            self.ips_state[self.action_current['ips']] = 1
            self.leg_cnt += 1
        except KeyError:
            return

        # evaluate residuals
        next_costs, prev_costs = self._analyse_route(self.route[self.leg_cnt:])

        prev_costs = self.cost_legs[self.leg_cnt:]
        prev_times = self.time_legs[self.leg_cnt:]

        rospy.loginfo('%s: residual energy: prev: %.2f J -- new: %.2f J', self.name, sum(prev_costs), sum(next_costs))
        rospy.loginfo('%s: residual time: prev: %.2f s -- new: %.2f s', self.name, sum(prev_times), sum(prev_costs))

        # # (ip achieved) calculate remaining inspection points
        # ips_togo = [ip for ip, s in self.ips_state.iteritems() if s == 0]
        # ips_togo = sorted(ips_togo, key=lambda x: int(x.split('IP_')[1]))
        #
        # # (optimize if there are at least two ips)
        # if self.route_optimization and len(ips_togo) > 1:
        #     rospy.loginfo('%s: route optimization, remaining inspection points %d', self.name, len(ips_togo))
        #
        #     self._plan(ips_togo)
        #     self.action_id = 0


    def dispatch_action(self, action):
        params = dict(**action['params'])
        mode = params.pop('mode', 'fast')
        pose = params.pop('pose')   # array N-by-6

        if action['name'] == ACT_GOTO:
            # reporting
            self.last_los_orient = tt.calculate_orientation(self.pos, pose[0])
            self.last_los_dist = tt.distance_between(self.pos, pose[0], spacing_dim=3)
        elif action['name'] == ACT_HOVER:
            mode = 'simple'
        else:
            rospy.logerr('%s: unknown action: %s', self.name, action)
            return

        # increment action counter
        self.action_cnt += 1
        self.action_last = action

        # send action dispatch
        ad = ActionDispatch()
        ad.header.stamp = rospy.Time.now()
        ad.id = self.action_cnt
        ad.name = action['name']
        ad.command = ActionDispatch.ACTION_START
        ad.feedback = ActionDispatch.FEED_STREAM
        ad.timeout = 0
        ad.params.append(KeyValue('pose', str(pose)))
        ad.params.append(KeyValue('mode', str(mode)))
        ad.params.extend([KeyValue(k, str(v)) for k, v in params])

        rospy.loginfo('%s: dispatch action[%d]: %s', self.name, self.action_id, action['name'])
        self.pub_disp.publish(ad)

        # action stats
        self.time_start = time.time()
        self.energy_start = self.energy_last

    def handle_feedback(self, data):
        if self.action_last is None:
            return

        if data.name != self.action_last['name']:
            return

        if data.id != self.action_cnt:
            return

        if data.status in (ActionFeedback.ACTION_RUNNING, ActionFeedback.ACTION_IDLE):
            return

        if data.status == ActionFeedback.ACTION_SUCCESS and self.action_state != ActionFeedback.ACTION_SUCCESS:
            # update action stats
            self.time_elapsed = data.duration
            self.energy_used = self.energy_last - self.energy_start

            # record action stats
            self._write_log(self.output_label, self.action_last)

        # update action status
        self.action_state = data.status


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
        self.mission_state = MISSION_IDLE
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


    def _calculate_cost_leg(self, wp_a, wp_b):
        dist = tt.distance_between(wp_a, wp_b, spacing_dim=3)
        yaw_los = tt.calculate_orientation(wp_a, wp_b)

        k_ejm = np.polyval(self.ejm_coeff, yaw_los)
        cost = k_ejm * dist + self.cost_offset
        cost = np.maximum(cost, 0.0)

        # 95% prediction interval
        cost_u = (k_ejm + 1.95 * self.ejm_rmse) * dist + self.cost_offset
        cost_l = (k_ejm - 1.95 * self.ejm_rmse) * dist + self.cost_offset

        # avoid out of range values during initial training
        if cost <= 0:
            cost = self.initial_ejm * dist + self.cost_offset
            cost_u = cost
            cost_l = cost

        return cost, (cost_l, cost_u)

    def _calculate_time_leg(self, wp_a, wp_b):
        dist = tt.distance_between(wp_a, wp_b, spacing_dim=3)
        yaw_los = tt.calculate_orientation(wp_a, wp_b)

        vc = np.polyval(self.spd_coeff, yaw_los)
        ltime = (0.5 * (vc / self.est_acc)) + (dist / vc) + (0.5 * (vc / self.est_dec))
        ltime = np.maximum(ltime, 10.0)

        # 95% prediction intervals
        vc_u = vc + 1.95 * self.spd_rmse
        vc_l = vc - 1.95 * self.spd_rmse

        ltime_u = (0.5 * (vc_u / self.est_acc)) + (dist / vc_u) + (0.5 * (vc_u / self.est_dec))
        ltime_u = np.maximum(ltime_u, 10.0)

        ltime_l = (0.5 * (vc_l / self.est_acc)) + (dist / vc_l) + (0.5 * (vc_l / self.est_dec))
        ltime_l = np.maximum(ltime_l, 10.0)

        return ltime, (ltime_l, ltime_u)

    def _analyse_route(self, route, kind=None):
        cost_legs = []
        time_legs = []

        for n in xrange(len(route)):
            wp_a = self.ips_dict[route[n - 1]]
            wp_b = self.ips_dict[route[n]]

            lcost, (cd, cu) = self._calculate_cost_leg(wp_a, wp_b)
            ltime, (td, tu) = self._calculate_time_leg(wp_a, wp_b)

            if kind == 'upper':
                cost_legs.append(cd)
                time_legs.append(td)
            elif kind == 'lower':
                cost_legs.append(cu)
                time_legs.append(tu)
            else:
                cost_legs.append(lcost)
                time_legs.append(ltime)

        return cost_legs, time_legs


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

                ips_costs[i][j], _ = self._calculate_cost_leg(self.ips_dict[ips_labels[i]], self.ips_dict[ips_labels[j]])

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
        self.route, _, _ = self._plan_tsp(labels)
        rospy.loginfo('%s: found inspection route:\n%s', self.name, self.route)

        # route analysis
        self.leg_cnt = 0
        self.cost_legs, self.time_legs = self._analyse_route(self.route)

        # boundary analysis
        cost_upper, time_upper = self._analyse_route(self.route, kind='upper')
        cost_lower, time_lower = self._analyse_route(self.route, kind='lower')

        cost_bound = zip(cost_lower, cost_upper)
        time_bound = zip(time_lower, time_upper)

        # generate action sequence
        self.actions = []

        # first hover on spot (with los for first IP)
        next_pose = np.copy(self.pos)
        next_pose[5] = tt.calculate_orientation(self.pos, self.ips_dict[self.route[0]])

        self.actions.append({
            'name': 'hover',
            'params': {
                'pose': [next_pose.tolist()]
            }
        })

        # second execute the route (skips the initial piece (going to AUV position)
        for n in xrange(len(self.route)):
            #next_pose = np.copy(self.ips_dict[self.route[n]])

            self.actions.append({
                'name': 'goto',
                'params': {
                    'pose': [self.ips_dict[self.route[n]].tolist()]
                },
                'cost': self.cost_legs[n],
                'duration': self.time_legs[n],
                'ips': self.route[n]
            })

            # if n < len(self.route) - 1:
            #     next_pose[5] = tt.calculate_orientation(self.ips_dict[self.route[n]], self.ips_dict[self.route[n + 1]])
            #
            # self.actions.append({
            #     'name': 'hover',
            #     'params': {
            #         'pose': [next_pose.tolist()]
            #     },
            #     'ips': self.route[n]
            # })

        rospy.loginfo('%s: generated action sequence (n=%d)', self.name, len(self.actions))

        # save mission plan
        plan_file = os.path.join(self.output_dir, '{}_plan_{}.json'.format(self.output_label, self.plan_id))

        plan = {
            'id': self.plan_id,
            'time': time.time(),
            'ips_count': len(labels),
            'route': self.route,
            'cost_legs': self.cost_legs,
            'cost_total': sum(self.cost_legs),
            'cost_bound': cost_bound,
            'time_legs': self.time_legs,
            'time_total': sum(self.time_legs),
            'time_bound': time_bound,
            'actions': self.actions,
        }

        with open(plan_file, 'wt') as plog:
            plog.write(json.dumps(plan, indent=2))

        # increase plan counter (to keep track of future replans)
        self.plan_id += 1


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
        rospy.wait_for_message(TOPIC_EJM, RegressionResult, timeout=5)
    except rospy.ROSException:
        rospy.logwarn('%s: no map received proceeding with internal estimation ...', rospy.get_name())
    else:
        rospy.loginfo('%s: received initial estimation:\n%s', rospy.get_name(), me.ejm_coeff)

    # setup the mission
    me.execute_mission(config)

    # start the mission
    me.run()

if __name__ == '__main__':
    main()
