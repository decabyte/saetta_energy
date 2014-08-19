#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import rospy
import roslib
from samba.dcerpc.nbt.name import name

roslib.load_manifest('saetta_energy')

import collections
import numpy as np

from saetta_energy.msg import EnergyReport
from planning_msgs.msg import ActionDispatch, ActionFeedback

# topics
TOPIC_ENERGY = 'saetta/energy/global'
TOPIC_DISPATCH = 'planning_system/action_dispatch'
TOPIC_FEEDBACK = 'planning_system/action_feedback'

# status
ACTION_REQUESTED = 'requested'
ACTION_STARTED = 'started'
ACTION_TERMINATED = 'ended'

# rates
RATE_UPDATE = 2     # sec
RATE_PUBLISH = 5    # sec


class ActionTracker(object):

    def __init__(self, name):
        # internal state
        self.name = name
        self.energy_report = EnergyReport()

        self.action_counter = 0
        self.actions = []
        self.process_queue = collections.deque()

        # ros interface
        self.sub_dispatch = rospy.Subscriber(TOPIC_DISPATCH, ActionDispatch, self.handle_dispatch)
        self.sub_feedback = rospy.Subscriber(TOPIC_FEEDBACK, ActionFeedback, self.handle_feedback)
        self.sub_energy = rospy.Subscriber(TOPIC_ENERGY, EnergyReport, self.handle_energy, tcp_nodelay=True)

        # timers
        self.t_process = rospy.Timer(rospy.Duration(RATE_UPDATE, 0), self.handle_actions)
        self.t_status = rospy.Timer(rospy.Duration(RATE_PUBLISH, 0), self.handle_status)


    def handle_energy(self, data):
        self.energy_report = data

    def handle_dispatch(self, data):
        rospy.loginfo('[%s]: tracking action #%d (%s)', self.name, data.action_id, data.name)

        self.actions.append({
            'id': data.action_id,
            'name': data.name,
            'timestamp': rospy.Time.now().to_sec(),
            't_begin': -1,
            't_end': -1,
            'e_begin': -1,
            'e_end': -1,
            'status': ACTION_REQUESTED
        })

    def handle_feedback(self, data):
        try:
            action = self.actions[data.action_id]
        except Exception:
            rospy.logerr('[%s]: unknown or untracked action #%d', self.name, data.action_id)
            return

        if data.status == 'action enabled':
            action['t_begin'] = rospy.Time.now().to_sec()
            action['e_begin'] = self.energy_report.energy_used
            action['status'] = ACTION_STARTED

            rospy.loginfo('action %d started', data.action_id)
            rospy.logdebug(action)

        if data.status == 'action achieved':
            action['t_end'] = rospy.Time.now().to_sec()
            action['e_end'] = self.energy_report.energy_used
            action['status'] = ACTION_TERMINATED

            rospy.loginfo('action %d terminated', data.action_id)
            rospy.logdebug(action)

            # add action to processing queue
            self.process_queue.append(action)



    def handle_actions(self, event):
        try:
            action = self.process_queue.pop()
        except Exception:
            return

        # process action
        action['energy'] = action['e_end'] - action['e_begin']
        action['duration'] = action['t_end'] - action['t_begin']

        rospy.loginfo('[%s]: result for action #%d (%s):  dur: %d secs -- cost: %d Wh',
                      self.name, action['id'], action['name'], action['duration'], action['energy'])



    def handle_status(self, event):
        running = sum([1 for a in self.actions if a['status'] == ACTION_STARTED])
        rospy.loginfo('[%s]: tracking actions: running %d / %d', self.name, running, len(self.actions))



if __name__ == '__main__':
    rospy.init_node('saetta_tracker')
    name = rospy.get_name()

    rospy.loginfo('%s is spinning ...', name)

    # load parameters
    #rate = rospy.get_param('saetta/simulator/rate', DEFAULT_RATE)		# simulated sensors rate (sec)

    # run
    try:
        at = ActionTracker(name=name)
        rospy.spin()
    except rospy.ROSInterruptException as ri:
        rospy.logerr('[%s]: caught exception: %s', name, str(ri))
    else:
        rospy.loginfo('[%s]: clean shutdown ...', name)
