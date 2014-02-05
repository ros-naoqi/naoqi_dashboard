# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

from nao_msgs.msg import BodyPoseAction, BodyPoseGoal
import actionlib

import std_srvs.srv

import rospy
from rosgraph import rosenv

from .status_control import StatusControl
from .power_state_control import PowerStateControl
from .avahi import AvahiWidget

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.monitor_dash_widget import MonitorDashWidget
from rqt_robot_dashboard.console_dash_widget import ConsoleDashWidget

class NAODashboard(Dashboard):
    
    def setup(self, context):
        self.name = 'NAO Dashboard (%s)'%rosenv.get_master_uri()

        self._robot_combobox = AvahiWidget()

        # Diagnostics
        self._monitor = MonitorDashWidget(self.context)

        # Rosout
        self._console = ConsoleDashWidget(self.context, minimal=False)

        ## Joint temperature
        #self._temp_joint_button = StatusControl(self, wx.ID_ANY, icons_path, "temperature_joints", True)
        #self._temp_joint_button.SetToolTip(wx.ToolTip("Joint temperatures"))

        ## CPU temperature
        #self._temp_head_button = StatusControl(self, wx.ID_ANY, icons_path, "temperature_head", True)
        #self._temp_head_button.SetToolTip(wx.ToolTip("CPU temperature"))

        ## Motors
        #self._motors_button = StatusControl(self, wx.ID_ANY, icons_path, "motor", True)
        #self._motors_button.SetToolTip(wx.ToolTip("Stiffness"))
        #self._motors_button._ok = (wx.Bitmap(path.join(icons_path, "stiffness-off-untoggled.png"), wx.BITMAP_TYPE_PNG), 
                    #wx.Bitmap(path.join(icons_path, "stiffness-off-toggled.png"), wx.BITMAP_TYPE_PNG))
        #self._motors_button._warn = (wx.Bitmap(path.join(icons_path, "stiffness-partially-untoggled.png"), wx.BITMAP_TYPE_PNG), 
                    #wx.Bitmap(path.join(icons_path, "stiffness-partially-toggled.png"), wx.BITMAP_TYPE_PNG))
        #self._motors_button._error = (wx.Bitmap(path.join(icons_path, "stiffness-on-untoggled.png"), wx.BITMAP_TYPE_PNG), 
                     #wx.Bitmap(path.join(icons_path, "stiffness-on-toggled.png"), wx.BITMAP_TYPE_PNG))
        #self._motors_button._stale = (wx.Bitmap(path.join(icons_path, "stiffness-stale-untoggled.png"), wx.BITMAP_TYPE_PNG), 
                    #wx.Bitmap(path.join(icons_path, "stiffness-stale-toggled.png"), wx.BITMAP_TYPE_PNG))
        #self._motors_button.Bind(wx.EVT_LEFT_DOWN, self.on_motors_clicked)

        ## Battery State
        self._power_state_ctrl = PowerStateControl('NAO Battery')
        #PowerStateControl(self.context)

        self._agg_sub = rospy.Subscriber('diagnostics_agg', DiagnosticArray, self.new_diagnostic_message)
        self.bodyPoseClient = actionlib.SimpleActionClient('body_pose', BodyPoseAction)
        self.stiffnessEnableClient = rospy.ServiceProxy("body_stiffness/enable", std_srvs.srv.Empty)
        self.stiffnessDisableClient = rospy.ServiceProxy("body_stiffness/disable", std_srvs.srv.Empty)

    def get_widgets(self):
        return [ [self._robot_combobox], 
                [self._monitor, self._console, #self._temp_joint_button, self._temp_head_button
                 ], #self._motors_button,
                [self._power_state_ctrl]
                ]

    def shutdown_dashboard(self):
        self._agg_sub.unregister()

    def on_motors_clicked(self, evt):      
      menu = wx.Menu()
      menu.Append(ID_INIT_POSE, "Init pose")
      menu.Append(ID_SIT_DOWN, "Sit down && remove stiffness")
      menu.Append(ID_REMOVE_STIFFNESS, "Remove stiffness immediately")
      wx.EVT_MENU(self, ID_INIT_POSE, self.on_init_pose)
      wx.EVT_MENU(self, ID_SIT_DOWN, self.on_sit_down)
      wx.EVT_MENU(self, ID_REMOVE_STIFFNESS, self.on_remove_stiffness)
                        
      #subscriberFound = ... 
      #menu.Enable(ID_INIT_POSE, subscriberFound);
      #menu.Enable(ID_SIT_DOWN, subscriberFound);
      #menu.Enable(ID_REMOVE_STIFFNESS, subscriberFound);      
      self._motors_button.toggle(True)
      self.PopupMenu(menu)
      self._motors_button.toggle(False)
      
    def on_init_pose(self, evt):
        self.stiffnessEnableClient.call()
        self.bodyPoseClient.send_goal_and_wait(BodyPoseGoal(pose_name = 'init'))
  
    def on_sit_down(self, evt):
        self.bodyPoseClient.send_goal_and_wait(BodyPoseGoal(pose_name = 'crouch'))
        state = self.bodyPoseClient.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            self.stiffnessDisableClient.call()
        else:
            wx.MessageBox('crouch pose did not succeed: %s - cannot remove stiffness' % self.bodyPoseClient.get_goal_status_text(), 'Error')
            rospy.logerror("crouch pose did not succeed: %s", self.bodyPoseClient.get_goal_status_text())

  
    def on_remove_stiffness(self, evt):
      msg = wx.MessageDialog(self, 'Caution: Robot may fall. Continue to remove stiffness?', 'Warning', wx.YES_NO | wx.NO_DEFAULT | wx.CENTER | wx.ICON_EXCLAMATION)
      if(msg.ShowModal() == wx.ID_YES):
          self.stiffnessDisableClient.call()
    
    def on_halt_motors(self, evt):
      halt = rospy.ServiceProxy("pr2_etherCAT/halt_motors", std_srvs.srv.Empty)
       
      try:
        halt()
      except rospy.ServiceException, e:
        wx.MessageBox("Failed to halt the motors: service call failed with error: %s"%(e), "Error", wx.OK|wx.ICON_ERROR)
      
    def new_diagnostic_message(self, msg):
        """
        callback to process dashboard_agg messages

        :param msg: dashboard_agg DashboardState message
        :type msg: pr2_msgs.msg.DashboardState
        """
        self._dashboard_message = msg
        for status in msg.status:
            if status.name == '/Nao/Joints':
                highestTemp = ""
                lowestStiff = -1.0
                highestStiff = -1.0
                hotJoints = ""
                for kv in status.values:
                     if kv.key == 'Highest Temperature':
                         highestTemp = " (" + kv.value + "deg C)"
                     elif kv.key == 'Highest Stiffness':
                         highestStiff = float(kv.value)
                     elif kv.key == 'Lowest Stiffness without Hands':
                         lowestStiff = float(kv.value)
                     elif kv.key == 'Hot Joints':
                         hotJoints = str(kv.value)
                #self.set_buttonStatus(self._temp_joint_button, status, "Joints: ", "%s %s"%(highestTemp, hotJoints))
                #if(lowestStiff < 0.0 or highestStiff < 0.0):
                    #self._motors_button.set_stale()
                    #self._motors_button.SetToolTip(wx.ToolTip("Stale"))
                #elif(lowestStiff > 0.9):
                    #self._motors_button.set_error()
                    #self._motors_button.SetToolTip(wx.ToolTip("Stiffness on"))
                #elif(highestStiff < 0.05):
                    #self._motors_button.set_ok()
                    #self._motors_button.SetToolTip(wx.ToolTip("Stiffness off"))
                #else:
                    #self._motors_button.set_warn()
                    #self._motors_button.SetToolTip(wx.ToolTip("Stiffness partially on (between %f and %f)" % (lowestStiff, highestStiff)))
            #elif status.name == '/Nao/CPU':
                #self.set_buttonStatus(self._temp_head_button, status, "CPU temperature: ")
            elif status.name == '/Nao/Battery/Battery':
                if status.level == 3:
                    self._power_state_ctrl.set_stale()
                else:
                    self._power_state_ctrl.set_power_state(status.values)

    def set_buttonStatus(self, button, status, statusPrefix = "", statusSuffix = ""):
        statusString = "Unknown"
        if status.level == DiagnosticStatus.OK:
            button.set_ok()
            statusString = "OK"
        elif status.level == DiagnosticStatus.WARN:
            button.set_warn()
            statusString = "Warn"
        elif status.level == DiagnosticStatus.ERROR:
            button.set_error()
            statusString = "Error"
        elif status.level == 3:
            button.set_stale()
            statusString = "Stale"
        button.SetToolTip(wx.ToolTip(statusPrefix + statusString + statusSuffix))

    def save_settings(self, plugin_settings, instance_settings):
        self._console.save_settings(plugin_settings, instance_settings)
        self._monitor.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._console.restore_settings(plugin_settings, instance_settings)
        self._monitor.restore_settings(plugin_settings, instance_settings)
