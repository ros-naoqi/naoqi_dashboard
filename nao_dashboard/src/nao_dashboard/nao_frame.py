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
import dbus, gobject, dbus.glib
from diagnostic_msgs.msg import *

from nao_msgs.msg import BodyPoseAction, BodyPoseGoal
import actionlib
import std_srvs.srv

#TODO
#ID_INIT_POSE = wx.NewId()
#ID_SIT_DOWN = wx.NewId()
#ID_REMOVE_STIFFNESS = wx.NewId()

#import avahi
class avahi:
    DBUS_NAME = "org.freedesktop.Avahi"
    DBUS_INTERFACE_SERVER = DBUS_NAME + ".Server"
    DBUS_PATH_SERVER = "/"
    DBUS_INTERFACE_ENTRY_GROUP = DBUS_NAME + ".EntryGroup"
    DBUS_INTERFACE_DOMAIN_BROWSER = DBUS_NAME + ".DomainBrowser"
    DBUS_INTERFACE_SERVICE_TYPE_BROWSER = DBUS_NAME + ".ServiceTypeBrowser"
    DBUS_INTERFACE_SERVICE_BROWSER = DBUS_NAME + ".ServiceBrowser"
    DBUS_INTERFACE_ADDRESS_RESOLVER = DBUS_NAME + ".AddressResolver"
    DBUS_INTERFACE_HOST_NAME_RESOLVER = DBUS_NAME + ".HostNameResolver"
    DBUS_INTERFACE_SERVICE_RESOLVER = DBUS_NAME + ".ServiceResolver"
    DBUS_INTERFACE_RECORD_BROWSER = DBUS_NAME + ".RecordBrowser"    
    PROTO_UNSPEC, PROTO_INET, PROTO_INET6  = -1, 0, 1
    IF_UNSPEC = -1
    LOOKUP_RESULT_CACHED = 1
    LOOKUP_RESULT_WIDE_AREA = 2
    LOOKUP_RESULT_MULTICAST = 4
    LOOKUP_RESULT_LOCAL = 8
    LOOKUP_RESULT_OUR_OWN = 16
    LOOKUP_RESULT_STATIC = 32


import std_msgs.msg
import std_srvs.srv

import rospy
from roslib import rosenv

from .status_control import StatusControl
from .power_state_control import PowerStateControl

from rqt_robot_dashboard.dashboard import Dashboard
from rqt_robot_dashboard.monitor_dash_widget import MonitorDashWidget
from rqt_robot_dashboard.console_dash_widget import ConsoleDashWidget

class NAODashboard(Dashboard):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    
    def setup(self, context):
        self.name = 'NAO Dashboard (%s)'%rosenv.get_master_uri()

        #self._robot_combobox = wx.ComboBox(self, wx.ID_ANY, "", choices = [])

        gobject.threads_init()
        dbus.glib.threads_init() 
        self.robots = []
        self.sys_bus = dbus.SystemBus()
        self.avahi_server = dbus.Interface(self.sys_bus.get_object(avahi.DBUS_NAME, '/'), avahi.DBUS_INTERFACE_SERVER)

        self.sbrowser = dbus.Interface(self.sys_bus.get_object(avahi.DBUS_NAME, self.avahi_server.ServiceBrowserNew(avahi.IF_UNSPEC,
            avahi.PROTO_INET, '_naoqi._tcp', 'local', dbus.UInt32(0))), avahi.DBUS_INTERFACE_SERVICE_BROWSER)

        self.sbrowser.connect_to_signal("ItemNew", self.avahiNewItem)
        self.sbrowser.connect_to_signal("ItemRemove", self.avahiItemRemove)

        # Diagnostics
        self._monitor = MonitorDashWidget(self.context)

        # Rosout
        self._console = ConsoleDashWidget(self.context, minimal=False)

        # Joint temperature
        self._temp_joint_button = StatusControl(self, wx.ID_ANY, icons_path, "temperature_joints", True)
        self._temp_joint_button.SetToolTip(wx.ToolTip("Joint temperatures"))

        # CPU temperature
        self._temp_head_button = StatusControl(self, wx.ID_ANY, icons_path, "temperature_head", True)
        self._temp_head_button.SetToolTip(wx.ToolTip("CPU temperature"))

        # Motors
        self._motors_button = StatusControl(self, wx.ID_ANY, icons_path, "motor", True)
        self._motors_button.SetToolTip(wx.ToolTip("Stiffness"))
        self._motors_button._ok = (wx.Bitmap(path.join(icons_path, "stiffness-off-untoggled.png"), wx.BITMAP_TYPE_PNG), 
                    wx.Bitmap(path.join(icons_path, "stiffness-off-toggled.png"), wx.BITMAP_TYPE_PNG))
        self._motors_button._warn = (wx.Bitmap(path.join(icons_path, "stiffness-partially-untoggled.png"), wx.BITMAP_TYPE_PNG), 
                    wx.Bitmap(path.join(icons_path, "stiffness-partially-toggled.png"), wx.BITMAP_TYPE_PNG))
        self._motors_button._error = (wx.Bitmap(path.join(icons_path, "stiffness-on-untoggled.png"), wx.BITMAP_TYPE_PNG), 
                     wx.Bitmap(path.join(icons_path, "stiffness-on-toggled.png"), wx.BITMAP_TYPE_PNG))
        self._motors_button._stale = (wx.Bitmap(path.join(icons_path, "stiffness-stale-untoggled.png"), wx.BITMAP_TYPE_PNG), 
                    wx.Bitmap(path.join(icons_path, "stiffness-stale-toggled.png"), wx.BITMAP_TYPE_PNG))
        self._motors_button.Bind(wx.EVT_LEFT_DOWN, self.on_motors_clicked)

        # Battery State
        self._power_state_ctrl = PowerStateControl(self, wx.ID_ANY, icons_path)
        self._power_state_ctrl.SetToolTip(wx.ToolTip("Battery: Stale"))
        
        self._config = wx.Config("nao_dashboard")
        
        self.Layout()
        self.Fit()
        
        self.load_config()
    
        self._agg_sub = rospy.Subscriber('diagnostics_agg', DiagnosticArray, self.diagnostic_callback)
        self.bodyPoseClient = actionlib.SimpleActionClient('body_pose', BodyPoseAction)
        self.stiffnessEnableClient = rospy.ServiceProxy("body_stiffness/enable", std_srvs.srv.Empty)
        self.stiffnessDisableClient = rospy.ServiceProxy("body_stiffness/disable", std_srvs.srv.Empty)

    def get_widgets(self):
        return [self._robot_combobox, [self._monitor, self._console, self._temp_joint_button, self._temp_head_button], self._motors_button, self._power_state_ctrl]

    def shutdown_dashboard(self):
        self._dashboard_agg_sub.unregister()

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
      
    def diagnostic_callback(self, msg):
      wx.CallAfter(self.new_diagnostic_message, msg)
      
    def new_diagnostic_message(self, msg):
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
                self.set_buttonStatus(self._temp_joint_button, status, "Joints: ", "%s %s"%(highestTemp, hotJoints))
                if(lowestStiff < 0.0 or highestStiff < 0.0):
                    self._motors_button.set_stale()
                    self._motors_button.SetToolTip(wx.ToolTip("Stale"))
                elif(lowestStiff > 0.9):
                    self._motors_button.set_error()
                    self._motors_button.SetToolTip(wx.ToolTip("Stiffness on"))
                elif(highestStiff < 0.05):
                    self._motors_button.set_ok()
                    self._motors_button.SetToolTip(wx.ToolTip("Stiffness off"))
                else:
                    self._motors_button.set_warn()
                    self._motors_button.SetToolTip(wx.ToolTip("Stiffness partially on (between %f and %f)" % (lowestStiff, highestStiff)))
            elif status.name == '/Nao/CPU':
                self.set_buttonStatus(self._temp_head_button, status, "CPU temperature: ")
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

    def avahiNewItem(self, interface, protocol, name, stype, domain, flags):
        self.avahi_server.ResolveService(interface, protocol, name, stype, 
            domain, avahi.PROTO_INET, dbus.UInt32(0), 
            reply_handler=self.service_resolved, error_handler=self.print_error)
        pass
    
    def avahiItemRemove(self, interface, protocol, name, stype, domain, flags):
        print "Remove"
        for robot in self.robots:
            if robot['name'] == str(name) and robot['address'] == str(address) and robot['port'] == int(port):
                self.robots.remove(robot)
        updateRobotCombobox();
      
    def service_resolved(self, interface, protocol, name, type, domain, host, aprotocol, address, port, txt, flags):
        self.robots.append({'name': str(name), 'address': str(address), 'port': int(port)})
        self.updateRobotCombobox()
        
    def updateRobotCombobox(self):
        selected = self._robot_combobox.GetValue()
        self._robot_combobox.Clear()
        id = -1
        for robot in self.robots:
            text = str(robot)
            text = "%s (%s:%d)" % (robot['name'], robot['address'], robot['port'])
            self._robot_combobox.Append(text)
            if(text == selected):
                id = self._robot_combobox.GetCount()-1;
            
        if(self._robot_combobox.GetCount() == 1):
            self._robot_combobox.SetSelection(0)
        elif(id > -1):
            self._robot_combobox.SetSelection(id)

        
    def print_error(self, *args):
        print 'error_handler'
        print args

    def save_settings(self, plugin_settings, instance_settings):
        self._console.save_settings(plugin_settings, instance_settings)
        self._monitor.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._console.restore_settings(plugin_settings, instance_settings)
        self._monitor.restore_settings(plugin_settings, instance_settings)
