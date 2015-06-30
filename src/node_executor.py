#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import sys
import os
import traceback
import json
import random
import string
import time
import re

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('saetta_energy')

from vehicle_interface.msg import PathRequest, PathStatus, PilotStatus, Vector6
from vehicle_interface.srv import BooleanService, PathService, PathServiceRequest, PathServiceResponse
from saetta_energy.msg import EnergyReport, EnergyStatus
from diagnostic_msgs.msg import KeyValue


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


# Secure ID generation:
#   see http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
def id_generator(size=6, chars=string.ascii_uppercase + string.digits):
    return ''.join(random.SystemRandom().choice(chars) for _ in range(size))

# String cleaning
#   see http://stackoverflow.com/questions/1007481/how-do-i-replace-whitespaces-with-underscore-and-vice-versa
def urlify(s):
    # remove all non-word characters (everything except numbers and letters)
    s = re.sub(r"[^\w\s]", '', s)

    # replace all runs of whitespace with a single dash
    s = re.sub(r"\s+", '-', s)

    return s


class MissionExecutor(object):

    def __init__(self, **kwargs):
        self.name = rospy.get_name()
        self.mission = None

        # mission state machine
        self.state_mission = MISSION_IDLE
        self.map_state_mission = {
            MISSION_IDLE: self.state_idle,
            MISSION_RUN: self.state_run,
            MISSION_COMPLETED: self.state_completed,
            MISSION_ABORT: self.state_abort
        }

        # action state machine
        self.state_action = ACTION_IDLE
        self.action_id = 0

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
        self.mission_dir = os.path.expanduser('~')
        self.mission_log = None

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

    def execute(self, mission):
        if self.mission is not None:
            rospy.logwarn('%s: executing mission, please abort current execution first ...')
            return

        if mission.get('name', None) is None:
            rospy.logwarn('%s: bad mission input, mission name required ...')
            return

        # store mission data
        self.mission = mission
        self.label = urlify(self.mission['name'])

        mfile = '{}_{}.csv'.format(self.label, id_generator(6))
        self.mission_log = os.path.join(self.mission_dir, mfile)

        # init output log
        if not os.path.exists(self.mission_log):
            with open(self.mission_log, 'wt') as mlog:
                mlog.write('name,action,time_start,time_end,time_elapsed,energy_used\n')

        # change state
        if self.state_mission is MISSION_IDLE:
            self.state_mission = MISSION_RUN


    def state_idle(self):
        pass

    def state_run(self):

        if self.state_action == ACTION_IDLE:
            try:
                current_action = self.mission['actions'][self.action_id]

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
        self.time_start = time.time()
        self.energy_start = self.energy_last
        self.last_action = action['name']

        if action['name'] == ACT_GOTO:
            poses = []
            poses.append(action['params']['pose'])
            poses.append(action['params']['pose'])

            # dispatch new action
            # ...

            # TEMP: mock the action system with path requests
            self.path_enabled = True
            self.send_path_request(poses)
        else:
            rospy.logerr('%s: unknown action: %s', self.name, action)

    def handle_feedback(self, status):
        self.state_action = status

        if self.state_action in (ACTION_RUNNING, ACTION_IDLE):
            return

        # record action stats
        self.time_end = time.time()
        self.time_elapsed = self.time_end - self.time_start
        self.energy_used = self.energy_last - self.energy_start

        # generate mission log
        out = '{},{},{},{},{},{}\n'.format(
            self.label, self.last_action, self.time_start, self.time_end, self.time_elapsed, self.energy_used
        )

        # save mission log
        with open(self.mission_log, 'at') as mlog:
            mlog.write(out)


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

    def send_path_request(self, poses, **kwargs):
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

    def reset_path(self):
        """This functions uses a service request to reset the state of the path controller"""
        try:
            self.srv_path.call(command=PathServiceRequest.CMD_RESET)
        except Exception:
            rospy.logerr('%s: unable to communicate with path service ...', self.name)


    def run(self):
        # mission init
        self.reset_path()
        rospy.sleep(3.0)

        while not rospy.is_shutdown():
            # execute current state
            self.map_state_mission[self.state_mission]()

            # wait a bit
            self.r_main.sleep()


def main():
    rospy.init_node('mission_executor')
    name = rospy.get_name()

    rospy.loginfo('%s: init', name)
    args = rospy.myargv()

    mission_file = None

    if len(args) > 1:
        mission_file = args[1]

    # load mission
    try:
        with open(mission_file, 'rt') as mf:
            mission = mf.read()

        mission = json.loads(mission)
    except Exception:
        rospy.logerr(traceback.format_exc())
        rospy.logfatal('%s could not load the mission file (%s)', name, mission_file)
        sys.exit(-1)


    # start the mission
    me = MissionExecutor()
    me.execute(mission)

    me.run()

if __name__ == '__main__':
    main()
