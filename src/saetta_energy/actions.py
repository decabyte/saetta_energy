#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, absolute_import

import rospy
import roslib
roslib.load_manifest('saetta_energy')

from .msg import ActionDispatch, ActionFeedback
from diagnostic_msgs.msg import KeyValue

TOPIC_DISPATCH = 'action/dispatch'
TOPIC_FEEDBACK = 'action/feedback'

STATE_NONE = 0
STATE_RUNNING = 1
STATE_DONE = 2

DEFAULT_SLEEP = 1.0         # seconds


class ActionServer(object):

    def __init__(self, fqn, **kwargs):
        if fqn is None:
            raise ValueError('Action fully-qualified-name can not be None')

        self.id = 1
        self.fqn = fqn
        self.topic_feed = kwargs.get('topic_feed', TOPIC_FEEDBACK)
        self.topic_disp = kwargs.get('topic_disp', TOPIC_DISPATCH)

        # server state
        self.state = STATE_NONE
        self.timeout = 0.0
        self.duration = 0.0

        # ros interface
        self.pub_feed = rospy.Publisher(self.topic_feed, ActionFeedback, queue_size=10)
        self.sub_disp = rospy.Subscriber(self.topic_disp, ActionDispatch, self.handle_dispatch, queue_size=10)


    def handle_dispatch(self, data):
        if data.name != self.fqn:
            return

        if data.id == 0:
            return

        if self.state != STATE_NONE:
            return

        self.id = data.id
        self.timeout = data.timeout

    def _send_feedback(self, **kwargs):
        status = kwargs.get('status', ActionFeedback.ACTION_RUNNING)
        duration = kwargs.get('duration', 0.0)
        info = kwargs.get('info', {})

        msg = ActionFeedback()
        msg.header.stamp = rospy.Time.now()
        msg.id = self.id
        msg.name = self.fqn
        msg.status = status
        msg.duration = duration
        msg.info = [KeyValue(k, v) for k, v in info.iteritems()]

        self.pub_feed.publish(msg)

    def run(self):
        while not rospy.is_shutdown():

            if self.state == STATE_NONE:
                rospy.sleep(DEFAULT_SLEEP)
                continue

            if self.state == STATE_RUNNING:
                self._send_feedback()
                rospy.sleep(DEFAULT_SLEEP)
                continue

            if self.state == STATE_DONE:
                self._send_feedback(status=ActionFeedback.ACTION_SUCCESS)
                self.state = STATE_NONE
                continue

    def __str__(self):
        return '%s[%s]: id[%d] state[%s]' % (ActionServer.__name__, self.fqn, self.id, self.state)


class ActionClient(object):

    def __init__(self, fqn, **kwargs):
        if fqn is None:
            raise ValueError('Action fully-qualified-name can not be None')

        self.id = 1
        self.fqn = fqn
        self.topic_feed = kwargs.get('topic_feed', TOPIC_FEEDBACK)
        self.topic_disp = kwargs.get('topic_disp', TOPIC_DISPATCH)

        # client state
        self.state = STATE_NONE

        # ros interface
        self.pub_disp = rospy.Publisher(self.topic_disp, ActionDispatch, queue_size=10)
        self.sub_feed = rospy.Subscriber(self.topic_feed, ActionFeedback, self.handle_feedback, queue_size=10)


    def handle_feedback(self, data):
        if data.name != self.fqn:
            return

        if data.id == 0:
            return

        if data.id == self.id:
            if data.status == ActionFeedback.ACTION_SUCCESS:
                self.state = STATE_DONE

    def send_action(self, **kwargs):
        if self.state == STATE_RUNNING:
            return

        self.id += 1
        self.state = STATE_RUNNING

        # start action
        self._send_dispatch(command=ActionDispatch.ACTION_START)

    def _send_dispatch(self, **kwargs):
        command = kwargs.get('command', ActionDispatch.ACTION_START)
        params = kwargs.get('params', {})
        timeout = kwargs.get('timeout', 0.0)

        msg = ActionDispatch()
        msg.header.stamp = rospy.Time.now()
        msg.id = self.id
        msg.name = self.fqn
        msg.command = command
        msg.timeout = timeout
        msg.params = [KeyValue(k, v) for k, v in params.iteritems()]

        self.pub_disp.publish(msg)

    def __str__(self):
        return '%s[%s]: id[%d] state[%s]' % (ActionClient.__name__, self.fqn, self.id, self.state)
