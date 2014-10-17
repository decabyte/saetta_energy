#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

# imports
import numpy as np

import rospy
import roslib
roslib.load_manifest('vehicle_diagnostics')

from vehicle_msgs.msg import Dictionary, FloatArray
from diagnostic_msgs.msg import KeyValue
from vehicle_msgs.srv import DictionaryService, PathService, BooleanService

# config
INIT_LIMIT = 85
STEPS = 5
THRUSTER_ID = 2

LAWNMOWER_ITERATIONS = 2
LAWNMOWER_LEN = 40
LAWNMOWER_WIDTH = 20

# constants
g_wait = True
TOPIC_STATUS = 'path/status'
SRV_PATH = 'path'

SRV_CONTROLLER = 'pilot/switch'
SRV_FAULT = 'thrusters/faults'
FAULT_HARD_LIM = 'hard_limit'


def handle_status(msg):
    global g_wait
    for pair in msg.values:
        if 'path_completed' in pair.key:
            g_wait = False
            break


rospy.init_node('experiment_script')

# subscribers
sub_status = rospy.Subscriber(TOPIC_STATUS, Dictionary, handle_status, queue_size=1)

# services
srv_path = rospy.ServiceProxy(SRV_PATH, PathService)
srv_controller = rospy.ServiceProxy(SRV_CONTROLLER, BooleanService)
srv_fault = rospy.ServiceProxy(SRV_FAULT, DictionaryService)

srv_controller.call(True)
counter = 0
fault_max = np.ones(6) * INIT_LIMIT
fault_min = -np.ones(6) * INIT_LIMIT

while not rospy.is_shutdown() and counter < STEPS:
    points = []
    points.append(FloatArray(values=[0, 0, 2, 0, 0, 0]))
    data = []
    data.append(KeyValue('mode', 'simple'))
    data.append(KeyValue('timeout', '1000'))

    g_wait = True
    rospy.loginfo('Points sent %s', srv_path.call(points=points, data=data))

    while not rospy.is_shutdown() and g_wait:
        rospy.sleep(0.20)

    g_wait = True
    print fault_min

    fault_max[THRUSTER_ID] = np.linspace(INIT_LIMIT, 0, STEPS)[counter]
    fault_min[THRUSTER_ID] = -np.linspace(INIT_LIMIT, 0, STEPS)[counter]
    print fault_min
    print str(fault_min.tolist())

    inj = []
    inj.append(KeyValue('fault_type', FAULT_HARD_LIM))
    inj.append(KeyValue('th_min', str(fault_min.tolist())))
    inj.append(KeyValue('th_max', str(fault_max.tolist())))

    rospy.loginfo('Fault injected %s', srv_fault.call(request=inj))

    # send trajectory
    points = []
    for i in xrange(LAWNMOWER_ITERATIONS):
        points.append(FloatArray(values=[0, 2*i*LAWNMOWER_WIDTH, 2, 0, 0, 0]))
        points.append(FloatArray(values=[LAWNMOWER_LEN, 2*i*LAWNMOWER_WIDTH, 2, 0, 0, 3.14/2]))
        points.append(FloatArray(values=[LAWNMOWER_LEN, 2*i*LAWNMOWER_WIDTH+LAWNMOWER_WIDTH, 2, 0, 0, 3.14]))
        points.append(FloatArray(values=[0, 2*i*LAWNMOWER_WIDTH+LAWNMOWER_WIDTH, 2, 0, 0, 3.14/2]))

    # points.append(FloatArray(values=[20, 0, 2, 0, 0, 0]))
    # points.append(FloatArray(values=[20, 0, 2, 0, 0, 3.14]))
    # points.append(FloatArray(values=[0, 0, 2, 0, 0, 3.14]))
    # points.append(FloatArray(values=[0, 0, 2, 0, 0, 0]))
    data = []
    data.append(KeyValue('mode', 'lines'))
    data.append(KeyValue('timeout', '1000'))

    counter += 1
    g_wait = True
    rospy.loginfo('Sending points %s', srv_path.call(points=points, data=data))

    while not rospy.is_shutdown():
        if not g_wait:
            break
        rospy.sleep(0.20)
