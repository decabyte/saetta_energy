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


class ActionServer(object):

    def __init__(self, fqn, **kwargs):
        if fqn is None:
            raise ValueError('Action fully-qualified-name can not be None')

        self.id = 1
        self.fqn = fqn

        self.topic_feed = kwargs.get('topic_feed', TOPIC_FEEDBACK)
        self.topic_disp = kwargs.get('topic_disp', TOPIC_DISPATCH)


        # ros interface
        self.pub_feed = rospy.Publisher(self.topic_feed, ActionFeedback, queue_size=10)
        self.sub_disp = rospy.Subscriber(self.topic_disp, ActionDispatch, self.handle_dispatch, queue_size=10)


    def handle_dispatch(self, data):
        if data.name != self.fqn:
            return


    def __str__(self):
        pass



class ActionClient(object):

    def __init__(self, fqn, **kwargs):
        if fqn is None:
            raise ValueError('Action fully-qualified-name can not be None')

        self.id = 1
        self.fqn = fqn

        self.topic_feed = kwargs.get('topic_feed', TOPIC_FEEDBACK)
        self.topic_disp = kwargs.get('topic_disp', TOPIC_DISPATCH)

        # ros interface
        self.pub_disp = rospy.Publisher(self.topic_disp, ActionDispatch, queue_size=10)
        self.sub_feed = rospy.Subscriber(self.topic_feed, ActionFeedback, self.handle_feedback, queue_size=10)


    def handle_feedback(self, data):
        if data.name != self.fqn:
            return

    def send_action(self, **kwargs):

        params = kwargs.get('params', {})
        duration = kwargs.get('duration', 0.0)

        msg = ActionDispatch()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.fqn
        msg.duration = duration
        msg.params = [KeyValue(k, v) for k, v in params.iteritems()]


    def __str__(self):
        pass

