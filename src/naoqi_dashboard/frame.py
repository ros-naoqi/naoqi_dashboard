# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# Copyright (c) 2014, Aldebaran Robotics (c)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Ported from pr2_dashboard: Stefan Osswald, University of Freiburg, 2011.
#

import roslib
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from naoqi_bridge_msgs.srv import GetRobotInfo, GetRobotInfoRequest, GetRobotInfoResponse
from naoqi_bridge_msgs.msg import BodyPoseAction, BodyPoseGoal, RobotInfo
import actionlib

import rospy
from rosgraph import rosenv

from .status_control import StatusControl
from .power_state_control import PowerStateControl
from .motors import Motors
from .avahi import AvahiWidget
from .posture import PostureWidget

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.monitor_dash_widget import MonitorDashWidget
from rqt_robot_dashboard.console_dash_widget import ConsoleDashWidget

from PyQt4 import QtGui, QtCore

class NAOqiDashboard(Dashboard):

    def setup(self, context):
        self.name = 'NAOqi Dashboard (%s)'%rosenv.get_master_uri()

        self.robot_prefix = ''

        # get robot info
        rospy.wait_for_service("/naoqi_driver/get_robot_config")
        try:
            get_robot_info = rospy.ServiceProxy("/naoqi_driver/get_robot_config", GetRobotInfo)
            resp = get_robot_info.call( GetRobotInfoRequest() )
            if resp.info.type == 0:
                self.robot_prefix = "nao_robot"
            elif resp.info.type == 2:
                self.robot_prefix = "pepper_robot"
        except rospy.ServiceException, e:
            print "Unable to call naoqi_driver/get_robot_config:%s", e

        #print "gonna work with robot prefix ", self.robot_prefix


        self._robot_combobox = AvahiWidget()

        # Diagnostics
        self._monitor = MonitorDashWidget(self.context)

        # Rosout
        self._console = ConsoleDashWidget(self.context, minimal=False)

        ## Joint temperature
        self._temp_joint_button = StatusControl('Joint temperature', 'temperature_joints')

        ## CPU temperature
        self._temp_head_button = StatusControl('CPU temperature', 'temperature_head')

        ## Motors
        self._motors_button = Motors(self.context, self.robot_prefix)

        ## Postures
        self._postures = PostureWidget(self.robot_prefix)

        ## Battery State
        self._power_state_ctrl = PowerStateControl('Battery')

        self._agg_sub = rospy.Subscriber('/diagnostics_agg', DiagnosticArray, self.new_diagnostic_message)

    def get_widgets(self):
        return [ [self._robot_combobox],
                [self._monitor, self._console, self._temp_joint_button, self._temp_head_button,
                 self._motors_button],
                [self._power_state_ctrl],
                #[self.posture_combobox, self.posture_button]
                [QtGui.QLabel("Posture"), self._postures]
                ]


    def shutdown_dashboard(self):
        self._agg_sub.unregister()

    def new_diagnostic_message(self, msg):
        """
        callback to process dashboard_agg messages

        :param msg: dashboard_agg DashboardState message
        :type msg: pr2_msgs.msg.DashboardState
        """
        self._dashboard_message = msg
        highest_level = DiagnosticStatus.OK
        highest_message = 'All OK'
        for status in msg.status:
            for black in ['/Joystick']:
                if status.name.startswith(black):
                    continue
                if status.level > highest_level:
                    highest_level = status.level
                    highest_message = status.message
            if status.name == '/NAOqi/Joints/Status':
                highestTemp = ""
                lowestStiff = -1.0
                highestStiff = -1.0
                hotJoints = ""
                for kv in status.values:
                     if kv.key == 'Highest Temperature':
                         highestTemp = kv.value
                     elif kv.key == 'Highest Stiffness':
                         highestStiff = float(kv.value)
                     elif kv.key == 'Lowest Stiffness without Hands':
                         lowestStiff = float(kv.value)
                     elif kv.key == 'Hot Joints':
                         hotJoints = str(kv.value)
                self.set_buttonStatus(self._temp_joint_button, status, "Joints ", " (%s deg C  %s)"%(highestTemp, hotJoints))
                # deal with the stiffness button
                if(lowestStiff < 0.0 or highestStiff < 0.0):
                    self._motors_button.set_stale()
                elif(lowestStiff > 0.9):
                    self._motors_button.set_error()
                elif(highestStiff < 0.05):
                    self._motors_button.set_ok()
                else:
                    self._motors_button.set_warn()
                self.set_buttonStatus(self._motors_button, status, "Stiffness ", " (low: %f / high: %f)"%(lowestStiff, highestStiff))
            elif status.name == '/NAOqi/Computer/CPU':
                for kv in status.values:
                    if kv.key == "Temperature":
                        self.set_buttonStatus(self._temp_head_button, status, "CPU temperature ", "(%s C)" % kv.value)
            elif status.name == '/NAOqi/Power System/Status':
                if status.level == 3:
                    self._power_state_ctrl.set_stale()
                else:
                    self._power_state_ctrl.set_power_state(status.values)
        # Override the status of the Monitor as we can blacklist some diagnostic that is not there (and therefore STALE
        if len(msg.status):
            class Status(object):
                pass

            status = Status()
            status.level = highest_level
            self.set_buttonStatus(self._monitor, status, '', ' %s' % highest_message)

    def set_buttonStatus(self, button, status, statusPrefix = "", statusSuffix = ""):
        statusString = "Unknown"
        if status.level == DiagnosticStatus.OK:
            button.update_state(0)
            statusString = "OK"
        elif status.level == DiagnosticStatus.WARN:
            button.update_state(1)
            statusString = "Warn"
        elif status.level == DiagnosticStatus.ERROR:
            button.update_state(2)
            statusString = "Error"
        elif status.level == 3:
            button.update_state(3)
            statusString = "Stale"
        button.setToolTip(statusPrefix + statusString + statusSuffix)

    def save_settings(self, plugin_settings, instance_settings):
        self._console.save_settings(plugin_settings, instance_settings)
        self._monitor.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._console.restore_settings(plugin_settings, instance_settings)
        self._monitor.restore_settings(plugin_settings, instance_settings)
