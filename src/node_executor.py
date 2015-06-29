#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('saetta_energy')

from vehicle_interface.msg import PathRequest, PathStatus, PilotStatus, Vector6
from vehicle_interface.srv import BooleanService, PathService, PathServiceRequest, PathServiceResponse

from diagnostic_msgs.msg import KeyValue


TOPIC_PATH_REQ = 'path/request'
TOPIC_PATH_STS = 'path/status'
TOPIC_PATH_SRV = 'path/control'
TOPIC_PILOT_STS = 'pilot/status'

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

        # ros interface
        self.pub_path = rospy.Publisher(TOPIC_PATH_REQ, PathRequest, queue_size=1, tcp_nodelay=True)
        self.sub_path = rospy.Subscriber(TOPIC_PATH_STS, PathStatus, self.handle_path_status, queue_size=10, tcp_nodelay=True)
        self.srv_path = rospy.ServiceProxy(TOPIC_PATH_SRV, PathService)

        # rate
        self.r_main = rospy.Rate(DEFAULT_RATE)



    def execute(self, mission):
        if self.mission is not None:
            rospy.logwarn('%s: executing mission, please abort current execution first ...')
            return

        # store mission data
        self.mission = mission

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
        pass


    def dispatch_action(self, action):
        rospy.loginfo('%s: dispatching action[%d]: %s', self.name, self.action_id, action['name'])

        if action['name'] == ACT_GOTO:
            poses = []
            poses.append(action['params']['pose'])
            poses.append(action['params']['pose'])

            # send new request
            self.path_enabled = True
            self.send_path_request(poses)
        else:
            rospy.logerr('%s: unknown action: %s', self.name, action)


    def handle_path_status(self, data):
        self.last_path_id = data.path_id

        if not self.path_enabled:
            return

        if self.state_action != ACTION_RUNNING:
            return

        if data.path_status in (PathStatus.PATH_ABORT, PathStatus.PATH_TIMEOUT):
            self.state_action = ACTION_FAILED
            self.path_enabled = False

        if data.path_status == PathStatus.PATH_COMPLETED:
            if data.navigation_status == PathStatus.NAV_HOVERING:
                self.state_action = ACTION_ACHIEVED
                self.path_enabled = False


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

    # load mission
    mission = {
        'actions': [
            {'type': 'single', 'name': 'goto', 'params': {'pose': [10.0, 20.0, 3.0, 0.0, 0.0, 0.0]}},
            {'type': 'single', 'name': 'goto', 'params': {'pose': [20.0, 10.0, 3.0, 0.0, 0.0, 0.0]}},
            {'type': 'single', 'name': 'goto', 'params': {'pose': [20.0, 40.0, 3.0, 0.0, 0.0, 0.0]}},
        ]
    }

    # TODO:
    #   add loading mission from file
    #   add trajectory generation
    #   add mission report save to file

    # start the mission
    me = MissionExecutor()
    me.execute(mission)

    me.run()

if __name__ == '__main__':
    main()
