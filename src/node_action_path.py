#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Reference: http://stackoverflow.com/questions/1894269/convert-string-representation-of-list-to-list-in-python
"""
from __future__ import division

import ast
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('saetta_energy')

import actionbus.actions as actions
from vehicle_core.path import trajectory_tools as tt

from diagnostic_msgs.msg import KeyValue
from actionbus.msg import ActionFeedback

from auv_msgs.msg import NavSts
from vehicle_interface.msg import PathRequest, PathStatus, Vector6
from vehicle_interface.srv import PathService, PathServiceRequest

TOPIC_NAV = 'nav/sts'
TOPIC_PATH_REQ = 'path/request'
TOPIC_PATH_STS = 'path/status'
TOPIC_PATH_SRV = 'path/control'
TOPIC_PILOT_STS = 'pilot/status'

# actions
ACTION_GOTO = 'goto'
ACTION_HOVER = 'hover'
ACTION_CURR = 'estcurr'

# defaults
DEFAULT_SLEEP = 1.0     # secs


class PathActionServer(actions.ActionServer):
    def __init__(self, fqn, **kwargs):
        super(PathActionServer, self).__init__(fqn, **kwargs)

        # path monitoring
        self.id_path_last = 0
        self.path_enabled = False

        # vehicle state
        self.pos = np.zeros(6, dtype=np.float64)

        # ros interface
        self.sub_nav = rospy.Subscriber(TOPIC_NAV, NavSts, self._handle_nav, queue_size=1)
        self.pub_path = rospy.Publisher(TOPIC_PATH_REQ, PathRequest, queue_size=1)
        self.sub_path = rospy.Subscriber(TOPIC_PATH_STS, PathStatus, self._handle_path_status, queue_size=10)
        self.srv_path = rospy.ServiceProxy(TOPIC_PATH_SRV, PathService)

    def _handle_nav(self, data):
        # parse navigation data
        self.pos = np.array([
            data.position.north,
            data.position.east,
            data.position.depth,
            data.orientation.roll,
            data.orientation.pitch,
            data.orientation.yaw
        ])

    def _handle_path_status(self, data):
        self.id_path_last = data.path_id

        if not self.path_enabled:
            return

        if self.state != actions.STATE_RUNNING:
            return

        # update action duration
        self.duration = data.time_elapsed

        if data.path_status == PathStatus.PATH_ABORT:
            self.path_enabled = False
            self.feedback = ActionFeedback.ACTION_FAILED
            self.state = actions.STATE_DONE

        elif data.path_status == PathStatus.PATH_TIMEOUT:
            self.path_enabled = False
            self.feedback = ActionFeedback.ACTION_TIMEOUT
            self.state = actions.STATE_DONE

        elif data.path_status == PathStatus.PATH_COMPLETED and data.navigation_status == PathStatus.NAV_HOVERING:
            self.path_enabled = False
            self.feedback = ActionFeedback.ACTION_SUCCESS
            self.state = actions.STATE_DONE

        else:
            self.feedback = ActionFeedback.ACTION_RUNNING

    def _send_path_request(self, poses, **kwargs):
        self.path_enabled = True

        # params
        mode = kwargs.get('mode', 'fast')
        timeout = kwargs.get('timeout', 5 * 60.0)
        target_speed = kwargs.get('target_speed', 1.0)
        look_ahead = kwargs.get('look_ahead', 5.0)

        if timeout <= 0.0:
            timeout = -1.0

        msg = PathRequest()
        msg.header.stamp = rospy.Time.now()
        msg.command = 'path'
        msg.points = [Vector6(x) for x in poses]
        msg.options = [
            KeyValue('mode', mode),
            KeyValue('timeout', str(timeout)),
            KeyValue('target_speed', str(target_speed)),
            KeyValue('look_ahead', str(look_ahead)),
        ]

        self.pub_path.publish(msg)

    def _reset_path(self):
        """This functions uses a service request to reset the state of the path controller"""
        self.srv_path.call(command=PathServiceRequest.CMD_RESET)


class GotoPathServer(PathActionServer):
    def __init__(self, **kwargs):
        super(GotoPathServer, self).__init__(ACTION_GOTO, **kwargs)

    def handle_dispatch(self, timeout, params):
        try:
            pose = ast.literal_eval(params['pose'])
        except (KeyError, SyntaxError):
            rospy.logerr('%s: received path action without pose param', self.__class__.__name__)

            self.feedback = ActionFeedback.ACTION_REJECT
            self.state = actions.STATE_DONE
            return

        mode = params.get('mode', 'fast')
        target_speed = float(params.get('target_speed', 1.0))

        # if timeout <= 0.0:
        #     est_dist = tt.distance_between(self.pos, pose[-1], spacing_dim=3)
        #     est_time = est_dist / (0.5 * target_speed)
        #     path_timeout = max(timeout, est_time)

        rospy.loginfo('%s: requesting goto path with final pose:\n%s', self.__class__.__name__, np.array(pose[-1]))
        self._send_path_request(pose, mode=mode, timeout=timeout, target_speed=target_speed)

        self.feedback = ActionFeedback.ACTION_RUNNING
        self.state = actions.STATE_RUNNING

    def run(self):
        pass


class HoverPathServer(PathActionServer):
    def __init__(self, **kwargs):
        super(HoverPathServer, self).__init__(ACTION_HOVER, **kwargs)

    def handle_dispatch(self, timeout, params):
        try:
            pose = ast.literal_eval(params['pose'])
        except (KeyError, SyntaxError):
            rospy.logerr('%s: received path action without pose param', self.__class__.__name__)

            self.feedback = ActionFeedback.ACTION_REJECT
            self.state = actions.STATE_DONE
            return

        rospy.loginfo('%s: requesting hover path with final pose:\n%s', self.__class__.__name__, np.array(pose[-1]))
        self._send_path_request(pose, mode='simple', timeout=timeout)

        self.feedback = ActionFeedback.ACTION_RUNNING
        self.state = actions.STATE_RUNNING

    def run(self):
        pass


def main():
    # start ros node
    rospy.init_node('node_action_path')
    name = rospy.get_name()
    rospy.loginfo('%s: init', name)

    # init path servers
    sg = GotoPathServer()
    sh = HoverPathServer()

    # init the path service
    sg._reset_path()
    rospy.sleep(DEFAULT_SLEEP)

    while not rospy.is_shutdown():
        sh.loop()
        sg.loop()

        rospy.sleep(DEFAULT_SLEEP)

if __name__ == '__main__':
    main()
