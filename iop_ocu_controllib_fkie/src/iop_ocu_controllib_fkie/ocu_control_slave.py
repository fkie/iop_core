#!/usr/bin/python

# ROS/IOP Bridge
# Copyright (c) 2017 Fraunhofer
#
# This program is dual licensed; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# version 2 as published by the Free Software Foundation, or
# enter into a proprietary license agreement with the copyright
# holder.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; or you can read the full license at
# <http://www.gnu.de/documents/gpl-2.0.html>
#
# :author: Alexander Tiderko

from iop_msgs_fkie.msg import OcuControl, OcuControlFeedback, JausAddress
import rospy


class OcuControlSlave():
    '''
    ROS node to manage the robot interface massages.
    The methods are thread-safe.
    '''

    ACCESS_STATE_NOT_AVAILABLE = 0
    ACCESS_STATE_NOT_CONTROLLED = 1
    ACCESS_STATE_CONTROL_RELEASED = 2
    ACCESS_STATE_CONTROL_ACCEPTED = 3
    ACCESS_STATE_TIMEOUT = 4
    ACCESS_STATE_INSUFFICIENT_AUTHORITY = 5

    ACCESS_CONTROL_ON_DEMAND = 10
    ACCESS_CONTROL_REQUEST = 11
    ACCESS_CONTROL_RELEASE = 12

    def __init__(self, subsystem=0, node=0, component=0):
        self._class_control_platform_callback = None
        self._class_control_component_callback = None
        self._class_access_control_callback = None

        # create messages that are used to published control feedback message
        self._msg_control = OcuControl()
        self._msg_control_feedback = OcuControlFeedback()
        self._pub_control_feedback = rospy.Publisher(
            "ocu_control_feedback", OcuControlFeedback, latch=True, queue_size=1)
        control_addr = ""
        authority = 205
        access_control = self.ACCESS_CONTROL_ON_DEMAND
        if rospy.has_param("~control_addr"):
            control_addr = rospy.get_param("~control_addr")
        else:
            control_addr = rospy.get_param("control_addr", control_addr)
        if rospy.has_param("~authority"):
            authority = rospy.get_param("~authority")
        else:
            authority = rospy.get_param("authority", authority)
        if rospy.has_param("~access_control"):
            access_control = rospy.get_param("~access_control")
        else:
            access_control = rospy.get_param("access_control", access_control)
        if control_addr:
            strs = control_addr.split(".:")
            if len(strs) > 3:
                raise Exception(
                    "Invalid control_addr parameter: %s\n" % control_addr)
            else:
                try:
                    self._msg_control.address.subsystem_id = (int)(strs[0])
                    self._msg_control.address.node_id = (int)(strs[1])
                    self._msg_control.address.subsystem_id = (int)(strs[2])
                except Exception:
                    pass
        # try to get the address of the component specified for this slave to
        # avoid discovering
        self._control_component = JausAddress()
        control_component_addr = rospy.get_param("~control_component_addr", "")
        if control_component_addr:
            strs = control_component_addr.split(".:")
            if len(strs) > 3:
                raise Exception(
                    "Invalid control_component_addr parameter: %s\n" % control_component_addr)
            else:
                try:
                    self._control_component.address.subsystem_id = (
                        int)(strs[0])
                    self._control_component.address.node_id = (int)(strs[1])
                    self._control_component.address.subsystem_id = (
                        int)(strs[2])
                except Exception:
                    pass
        self._msg_control.authority = authority
        self._msg_control.access_control = access_control
        rospy.loginfo("[OcuControlSlave] OCU control slave parameter:")
        rospy.loginfo("[OcuControlSlave]   control_addr: %s, decoded to: %d.%d.%d" % (
            control_addr, self._msg_control.address.subsystem_id, self._msg_control.address.node_id, self._msg_control.address.subsystem_id))
        rospy.loginfo("[OcuControlSlave]   control_component_addr: %s, decoded to: %d.%d.%d" % (
            control_component_addr, self._control_component.subsystem_id, self._control_component.node_id, self._control_component.subsystem_id))
        rospy.loginfo("[OcuControlSlave]   authority:  %d" % authority)
        access_control_str = "ACCESS_CONTROL_ON_DEMAND(0)"
        if access_control == 1:
            access_control_str = "ACCESS_CONTROL_REQUEST(1)"
        elif access_control == 2:
            access_control_str = "ACCESS_CONTROL_RELEASE(2)"
        rospy.loginfo("[OcuControlSlave]   access_control: %s" %
                      access_control_str)
        self._msg_control_feedback.addr_reporter.subsystem_id = subsystem
        self._msg_control_feedback.addr_reporter.node_id = node
        self._msg_control_feedback.addr_reporter.component_id = component
        self._msg_control_feedback.authority = authority
        # publish the feedback with settings
        self._pub_control_feedback.publish(self._msg_control_feedback)
        self._sub_control = rospy.Subscriber(
            "ocu_control", OcuControl, self._ros_control, queue_size=10)

    def set_control_platform_handler(self, handler):
        '''
        Sets the handler which listen for platform address to control. The handler is of type (subsystem, node, component, authority)
        :param handler: callback(uint16, uint8, uint8, uint8)
        '''
        self._class_control_platform_callback = handler
        self._class_control_platform_callback(self._msg_control.address.subsystem_id, self._msg_control.address.node_id,
                                              self._msg_control.address.component_id, self._msg_control.authority)

    def set_control_component_handler(self, handler):
        '''
        Sets the handler which listen for component address to control. The handler is of type (subsystem, node, component, authority)
        :param handler: callback(uint16, uint8, uint8, uint8)
        '''
        self._class_control_component_callback = handler
        if self._control_component.component_id != 0:
            self._class_control_component_callback(self._control_component.subsystem_id, self._control_component.node_id,
                                                   self._control_component.component_id, self._msg_control.authority)

    def set_access_control_handler(self, handler):
        '''
        Sets the handler for access control. The handler has an int value of {ACCESS_CONTROL_ON_DEMAND, ACCESS_CONTROL_REQUEST, ACCESS_CONTROL_RELEASE} as parameter.
        :param handler: callback(int)
        '''
        self._class_access_control_callback = handler
        self._class_access_control_callback(self._msg_control.access_control)

    def get_authority(self):
        return self._msg_control.authority

    def get_control_subsystem(self):
        return self._msg_control.address.subsystem_id

    def get_control_platform(self):
        return self._msg_control.address

    def get_control_component(self):
        return self._control_component

    def get_access_control(self):
        return self._msg_control.access_control

    def set_reporter_address(self, subsystem, node, component):
        if (self._msg_feedback.addr_reporter.subsystem_id != subsystem or
                self._msg_feedback.addr_reporter.node_id != node or
                self._msg_feedback.addr_reporter.component_id != component):
            self._msg_feedback.addr_reporter.subsystem_id = subsystem
            self._msg_feedback.addr_reporter.node_id = node
            self._msg_feedback.addr_reporter.component_id = component
            self._pub_control_feedback.publish(self._msg_feedback)

    def set_control_address(self, subsystem, node, component):
        if (self._msg_feedback.addr_control.subsystem_id != subsystem or
                self._msg_feedback.addr_control.node_id != node or
                self._msg_feedback.addr_control.component_id != component):
            self._msg_feedback.addr_control.subsystem_id = subsystem
            self._msg_feedback.addr_control.node_id = node
            self._msg_feedback.addr_control.component_id = component
            self._pub_control_feedback.publish(self._msg_feedback)

    def set_access_state(self, code):
        if self._msg_feedback.access_state != code:
            self._msg_feedback.access_state = code
            self._pub_control_feedback.publish(self._msg_feedback)

    def _ros_control(self, control):
        a2 = control.address
        if not self.eqaulAddress(self._msg_control.address, a2) or self._msg_control.authority != control.authority:
            self._msg_control.address = a2
            self._msg_control.authority = control.authority
            if self._class_control_platform_callback is not None:
                self._class_control_platform_callback(self._msg_control.address.subsystem_id, self._msg_control.address.node_id,
                                                      self._msg_control.address.component_id, self._msg_control.authority)
        if self._msg_control.access_control != control.access_control:
            self._msg_control.access_control = control.access_control
            if self._class_access_control_callback is not None:
                self._class_access_control_callback(
                    self._msg_control.access_control)

    def finish(self):
        if self._pub_control_feedback is not None:
            self._pub_control_feedback.unregister()
        if self._sub_control is not None:
            self._sub_control.unregister()

    def eqaulAddress(self, a1, a2):
        return ((a1.subsystem_id == a2.subsystem_id) and
                (a1.node_id == a2.node_id) and
                (a1.component_id == a2.component_id))
