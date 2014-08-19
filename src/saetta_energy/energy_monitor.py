#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np
np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('saetta_energy')

from saetta_energy.msg import EnergyReport, EnergyStatus
from vehicle_msgs.msg import ThrusterFeedback

# config
DEFAULT_BAT = 1800.0    # battery pack capacity for Nessie AUV (Wh)
TS_UPDATE = 2.0         # sec
TS_PUBLISH = 4.0        # sec
AVG_TIME = 60.0         # sec

THR_RATE = 10.0             # Hz
THR_FRAME = 1 / THR_RATE    # sec
THR_VNOM = 28.0             # volts
BAT_VNOM = 28.0             # volts

TOPIC_GLOBAL = 'saetta/report'
TOPIC_THRUSTERS = 'thrusters/status'



class EnergyMonitor(object):

    def __init__(self, name, initial_energy, ts_update, ts_publish):
        # local conf
        self.name = name
        self.ts_update = ts_update
        self.ts_publish = ts_publish

        # thruster subsystems
        self.thruster = {
            'sid': 0,
            'name': 'thrusters',
            'energy_used': 0.0,
            'energy_left': 0.0,
            'energy_full': 0.0,
            'avg_voltage': THR_VNOM,
            'avg_current': 0.0,
            'avg_power': 0.0,
        }

        # battery subsystem
        bat_full = initial_energy / 4.0
        bat_left = initial_energy / 4.0

        self.batteries = [
            {
                'sid': 1,
                'name': 'battery #1',
                'energy_used': 0.0,
                'energy_left': bat_left,
                'energy_full': bat_full,
                'avg_voltage': BAT_VNOM,
                'avg_current': 0.0,
                'avg_power': 0.0,
            },
            {
                'sid': 2,
                'name': 'battery #2',
                'energy_used': 0.0,
                'energy_left': bat_left,
                'energy_full': bat_full,
                'avg_voltage': BAT_VNOM,
                'avg_current': 0.0,
                'avg_power': 0.0,
            },
            {
                'sid': 3,
                'name': 'battery #3',
                'energy_used': 0.0,
                'energy_left': bat_left,
                'energy_full': bat_full,
                'avg_voltage': BAT_VNOM,
                'avg_current': 0.0,
                'avg_power': 0.0,
            },
            {
                'sid': 4,
                'name': 'battery #4',
                'energy_used': 0.0,
                'energy_left': bat_left,
                'energy_full': bat_full,
                'avg_voltage': BAT_VNOM,
                'avg_current': 0.0,
                'avg_power': 0.0,
            }
        ]

        # vehicle status data structure
        self.status = {
            'energy_full': 0.0,
            'energy_left': 0.0,
            'energy_used': 0.0,
            'avg_power': 0.0,
            'subsystems': []
        }

        self.status['subsystems'].append(self.thruster)
        self.status['subsystems'].extend(self.batteries)

        # initial update
        self.status['energy_full'] = np.sum([ sub['energy_full'] for sub in self.status['subsystems'] ])
        self.status['energy_left'] = np.sum([ sub['energy_left'] for sub in self.status['subsystems'] ])

        # ros interface
        self.timer_update = rospy.Timer(rospy.Duration(self.ts_update, 0), self.update_status)
        self.timer_publish = rospy.Timer(rospy.Duration(self.ts_publish, 0), self.publish_status)
        self.pub_report = rospy.Publisher(TOPIC_GLOBAL, EnergyReport, queue_size=3, latch=True)

        # thruster subsystem
        self.thrs_window = AVG_TIME / THR_RATE
        self.thrs_current = np.zeros((6, self.thrs_window))
        self.thrs_energy = np.zeros(6)
        self.sub_thrs = rospy.Subscriber(TOPIC_THRUSTERS, ThrusterFeedback, self.handle_thrusters, queue_size=3)


    def publish_status(self, event):
        report = EnergyReport()
        report.header.stamp = rospy.Time.now()

        report.energy_left = self.status['energy_left']
        report.energy_used = self.status['energy_used']
        report.avg_power = self.status['avg_power']
        report.subsystems = list()

        for sub in self.status['subsystems']:
            stat = EnergyStatus(**sub)
            report.subsystems.append(stat)

        self.pub_report.publish(report)


    def update_status(self, event):
        # old usage
        prev_used = self.status['energy_used']

        # reset counters
        self.status['energy_used'] = 0
        self.status['energy_left'] = 0

        for sub in self.status['subsystems']:
            self.status['energy_used'] += sub['energy_used']

        # placeholder to simulate battery monitoring
        bat_drain = self.status['energy_used'] / len(self.batteries)

        for bat in self.batteries:
            prev_left = bat['energy_left']

            bat['energy_left'] = bat['energy_full'] - bat_drain
            bat['energy_left'] = np.maximum(bat['energy_left'], 0)

            bat['avg_power'] = ((prev_left - bat['energy_left']) / self.ts_update) * 3600
            bat['avg_current'] = bat['avg_power'] / bat['avg_voltage']

        # update counters
        self.status['energy_left'] = np.sum([ sub['energy_left'] for sub in self.status['subsystems'] ])
        self.status['avg_power'] = ((self.status['energy_used'] - prev_used) / self.ts_update) * 3600


    def handle_thrusters(self, data):
        self.thrs_current = np.roll(self.thrs_current, -1, axis=1)
        self.thrs_current[:, -1] = np.array(data.current[0:6])
        self.thrs_energy += THR_VNOM * np.trapz(self.thrs_current[:, -2:], dx=THR_FRAME)

        self.thruster['energy_used'] = np.sum(self.thrs_energy) / 3600.0                       # Wh
        self.thruster['avg_current'] = np.sum(np.mean(self.thrs_current, axis=1))
        self.thruster['avg_power'] = THR_VNOM * self.thruster['avg_current']



if __name__ == '__main__':
    rospy.init_node('energy_monitor')
    name = rospy.get_name()
    rospy.loginfo('%s initializing ...', name)

    # load parameters
    verbose = False
    rate_update = float(rospy.get_param('~rate_update', TS_UPDATE))               # sec
    rate_publish = float(rospy.get_param('~rate_publish', TS_PUBLISH))            # sec
    battery_capacity = float(rospy.get_param('~battery_capacity', DEFAULT_BAT))   # battery capacity (watt-hours)

    # parse args
    args = rospy.myargv()

    if '-v' in args:
        verbose = True

    # print config
    rospy.loginfo('%s: rate of update: %s', name, rate_update)
    rospy.loginfo('%s: rate of report: %s', name, rate_publish)
    rospy.loginfo('%s: initial capacity: %s', name, battery_capacity)


    try:
        es = EnergyMonitor(name, battery_capacity, rate_update, rate_publish)
        rospy.spin()
    except rospy.ROSInterruptException as ri:
        rospy.logerr('[%s]: caught exception: %s', rospy.get_name(), str(ri))
    else:
        rospy.loginfo('[%s]: clean shutdown ...', rospy.get_name())
