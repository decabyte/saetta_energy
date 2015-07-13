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

    cg = ActionClient('hover')

    pose = np.array([0, 0, 3, 0, 0, 0])

    rospy.sleep(1.0)
    rospy.loginfo('sending action')

    cg._send_dispatch(params={'pose': '[0, 0, 3, 0, 0, 0]'})

    while not rospy.is_shutdown():
        print(cg)
        rospy.sleep(1.0)


if __name__ == '__main__':
    main()
