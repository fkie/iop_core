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

from iop_msgs_fkie.msg import OcuControl, OcuControlFeedback
import rospy


class OcuControlMaster():
    '''
    ROS node to manage the robot interface massages.
    The methods are thread-safe.
    '''

    ACCESS_CONTROL_ON_DEMAND = 10
    ACCESS_CONTROL_REQUEST = 11
    ACCESS_CONTROL_RELEASE = 12

    def __init__(self, handler=None):
        self._class_feedback_callback = None
        # create messages that are used to published control message
        self._msg_control = OcuControl()
        self._pub_control = rospy.Publisher(
            "ocu_control", OcuControl, latch=True, queue_size=1)
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
        self._msg_control.authority = authority
        self._msg_control.access_control = access_control
        rospy.loginfo("[OcuControlMaster] used parameter:")
        rospy.loginfo("[OcuControlMaster]   control_addr: %s, decoded to: %d.%d.%d" % (
            control_addr, self._msg_control.address.subsystem_id, self._msg_control.address.node_id, self._msg_control.address.subsystem_id))
        rospy.loginfo("[OcuControlMaster]   authority:  %d" % authority)
        access_control_str = "ACCESS_CONTROL_ON_DEMAND(0)"
        if access_control == 1:
            access_control_str = "ACCESS_CONTROL_REQUEST(1)"
        elif access_control == 2:
            access_control_str = "ACCESS_CONTROL_RELEASE(2)"
        rospy.loginfo("[OcuControlMaster]   access_control: %s" %
                      access_control_str)
        self._pub_control.publish(self._msg_control)
        self._sub_control_feedback = None

    def set_feedback_handler(self, handler):
        '''
        Sets the handler for feedback messages. The handler has jaus_msgs_fkie.OcuControlFeedback object as parameter.
        :param handler: callback(jaus_msgs_fkie.OcuControlFeedback)
        '''
        self._class_feedback_callback = handler
        if self._sub_control_feedback is not None:
            self._sub_control_feedback.unregister()
        self._sub_control_feedback = rospy.Subscriber(
            "ocu_control_feedback", OcuControlFeedback, self._ros_control_feedback, queue_size=10)

    def set_control(self, subsystem=None, node=None, component=None, authority=None, access_control=None):
        if (self._changed(self._msg_control.address.subsystem_id, subsystem) or
                self._changed(self._msg_control.address.node_id, node) or
                self._changed(self._msg_control.address.subsystem_id, component) or
                self._changed(self._msg_control.authority, authority) or
                self._changed(self._msg_control.access_control, access_control)):
            self._msg_control.address.subsystem_id = subsystem
            self._msg_control.address.node_id = node
            self._msg_control.address.component_id = component
            self._msg_control.authority = authority
            self._msg_control.access_control = access_control
            self._pub_control.publish(self._msg_control)

    def _changed(self, prev_val, val):
        if val is not None:
            if prev_val != val:
                return True
        return False

    def set_access_control(self, value):
        if (self._msg_control.access_control != value):
            self._msg_control.access_control = value
            self._pub_control.publish(self._msg_control)

    def _ros_control_feedback(self, control_feedback):
        if self._class_feedback_callback is not None:
            self._class_feedback_callback(control_feedback)

    def finish(self):
        self.set_access_control(OcuControlMaster.ACCESS_CONTROL_RELEASE)
        if self._sub_control_feedback is not None:
            self._sub_control_feedback.unregister()
        if self._pub_control is not None:
            self._pub_control.unregister()
