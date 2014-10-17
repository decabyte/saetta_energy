#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

import numpy as np

from vehicle_core.control import thrust_allocation as ta
from vehicle_core.config import thruster_config as tc


np.set_printoptions(precision=3, suppress=True)

import rospy
import roslib
roslib.load_manifest('saetta_energy')

from diagnostic_msgs.msg import KeyValue
from vehicle_interface.msg import VehicleStatus, PilotStatus, FloatArrayStamped
from vehicle_interface.srv import StringService, StringServiceResponse, FloatService, BooleanService, BooleanServiceResponse

# config
DEFAULT_RATE = 1    # Hz
TS_SPEED = 2        # sec

SRV_VEHICLE = 'status/vehicle'
TOPIC_VEHICLE = 'status/vehicle'

TOPIC_PILOT = 'pilot/status'
TOPIC_THRS = 'thrusters/diagnostics'

SRV_ADAPT = 'status/adaptive_speed'
SRV_LIMS = 'pilot/speed_limits'


# diagnostics values
U_ADPT_RATE = 0.01      # maximum axial force adaptation rate


class StatusMonitor(object):

    def __init__(self, name, **kwargs):
        self.name = name

        # vehicle status data structure
        self.state = {
            'max_dof_forces': tc.MAX_U,
            'max_dof_speeds': np.array([2, 2, 2, 0, 2, 2]),
            'ctrl_dof_speeds': np.array([2, 2, 2, 0, 2, 2]),
            'thruster_usage': np.ones(6),
            'thruster_metric': np.zeros(6)
        }

        # flags
        self.adaptive_speed = kwargs.get('adaptive_speed', False)

        # ros interface
        self.t_stat = rospy.Timer(rospy.Duration(1 / DEFAULT_RATE), self.send_status)
        self.t_speed = rospy.Timer(rospy.Duration(TS_SPEED), self.send_speeds)

        self.pub_stat = rospy.Publisher(TOPIC_VEHICLE, VehicleStatus, queue_size=1)
        self.srv_stat = rospy.Service(SRV_VEHICLE, StringService, self.handle_service)

        # subscribers
        self.sub_pilot = rospy.Subscriber(TOPIC_PILOT, PilotStatus, self.handle_pilot, queue_size=1)
        self.sub_th_diag = rospy.Subscriber(TOPIC_THRS, FloatArrayStamped, self.handle_th_diag, queue_size=1)

        # services
        self.srv_adaptive = rospy.Service(SRV_ADAPT, BooleanService, self.handle_adapt)
        self.srv_speeds = rospy.ServiceProxy(SRV_LIMS, FloatService)


    # TODO: placeholder for complex status update and estimations
    # def update_status(self, event):
    #     # send messages
    #     self.send_status()


    def send_speeds(self, event=None):
        if self.adaptive_speed:
            try:
                self.srv_speeds.call(request=self.state['ctrl_dof_speeds'])
            except rospy.ServiceException as se:
                rospy.logwarn('%s error contacting pilot service', name)
            except rospy.ROSException as re:
                rospy.logwarn('%s could not contact pilot service', name)


    def send_status(self, event=None):
        vs = VehicleStatus()
        vs.header.stamp = rospy.Time.now()
        vs.info = [ KeyValue(str(k), str(v)) for k,v in self.state.items() ]
        self.pub_stat.publish(vs)


    def handle_adapt(self, data):
        try:
            self.adaptive_speed = data.request
        except:
            self.adaptive_speed = False

        if not self.adaptive_speed:
            self.state['ctrl_dof_speeds'] = self.state['max_dof_speeds']
            rospy.logwarn('%s resetting maximum speeds: %s', self.name, self.state['ctrl_dof_speeds'])

        self.send_speeds()
        return BooleanServiceResponse(response=self.adaptive_speed)


    def handle_service(self, data):
        try:
            res = str(self.state[data.request])
            return StringServiceResponse(result=True, response=res)
        except:
            return StringServiceResponse(result=False)


    def handle_pilot(self, data):
        # unpack pilot info
        info = {}

        for item in data.info:
            if item.value.startswith('['):
                # parse the numpy array
                info[item.key] = np.fromstring(item.value.strip('[]'), sep=',')
            else:
                # use input as string
                info[item.key] = item.value

        # limits
        self.state['thruster_usage'] = np.clip(self.state['thruster_usage'], 0, 1)

        # update maximum forces
        self.local_TAM = np.dot(tc.TAM, np.diag(self.state['thruster_usage']))
        self.local_inv_TAM = np.linalg.pinv(self.local_TAM)
        self.local_MAX_U = ta.evaluate_max_force(self.local_inv_TAM)

        # clip maximum force between 0% and 100% of vehicle reference design
        self.local_MAX_U = np.clip(self.local_MAX_U, 0, tc.MAX_U)

        # scale down the speeds (and avoid division between zeros and by zero)
        ctrl_speeds = self.state['max_dof_speeds'] * np.sqrt(self.local_MAX_U / tc.MAX_U)
        ctrl_speeds[ np.where(np.isnan(ctrl_speeds)) ] = 0
        ctrl_speeds[ np.where(np.isinf(ctrl_speeds)) ] = 0

        self.state['ctrl_dof_speeds'] = ctrl_speeds
        self.state['ctrl_dof_speeds'] = np.clip(self.state['ctrl_dof_speeds'], 0, self.state['max_dof_speeds'])

        try:
            self.state.update(info)
        except KeyError:
            pass


    def handle_th_diag(self, data):
        pass

        # if len(data.values) == 6:
        #     self.state['thruster_metric'] = np.array(data.values[0:6])
        # else:
        #     self.state['thruster_metric'] = np.zeros(6)




if __name__ == '__main__':
    rospy.init_node('status_monitor')
    name = rospy.get_name()
    rospy.loginfo('%s initializing ...', name)

    # global parameters
    adaptive_speed = rospy.get_param('status/adaptive_speed', False)
    rate_loop = rospy.Rate(DEFAULT_RATE)
    verbose = False

    # parse args
    args = rospy.myargv()

    if '-v' in args:
        verbose = True

    # print config
    rospy.loginfo('%s adaptive speed: %s', name, adaptive_speed)

    sm = StatusMonitor(name, adaptive_speed=adaptive_speed)
    rospy.spin()
