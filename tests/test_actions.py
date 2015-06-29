#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division, absolute_import

import traceback
import threading
import numpy as np

import rospy
import roslib
roslib.load_manifest('saetta_energy')

import saetta_energy.actions as actions
from saetta_energy.actions import ActionServer, ActionClient

def main():
    rospy.init_node('test_actions')

    N_SERVERS = 5
    N_CLIENTS = 20

    action_servers = [ActionServer('action-%d' % d) for d in xrange(N_SERVERS)]
    action_clients = [ActionClient('action-%d' % np.random.choice(N_SERVERS)) for _ in xrange(N_CLIENTS)]

    th_servers = [threading.Thread(target=server.run) for server in action_servers]

    # start the servers
    map(lambda x: x.start(), th_servers)

    while not rospy.is_shutdown():
        try:
            cids = np.random.choice(action_clients, 5)

            # send actions
            map(lambda x: x.send_action(params={'x': np.random.random(), 'y': np.random.random()}), cids)

            print('-- running --')
            rospy.sleep(3.0)
        except Exception:
            rospy.logfatal(traceback.format_exc())
            rospy.signal_shutdown('user')

    rospy.loginfo('closing!')

    map(lambda x: x.join(), th_servers)
    rospy.loginfo('done!')

if __name__ == '__main__':
    main()
