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
roslib.load_manifest('nao_dashboard')

import wx
import wx.aui
import wx.py.shell
import rxtools
import rxtools.cppwidgets as rxtools

import dbus, gobject, dbus.glib
from diagnostic_msgs.msg import *

from nao_msgs.msg import BodyPoseAction, BodyPoseGoal
import actionlib
import std_srvs.srv

ID_INIT_POSE = wx.NewId()
ID_SIT_DOWN = wx.NewId()
ID_REMOVE_STIFFNESS = wx.NewId()

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


import robot_monitor
from robot_monitor.robot_monitor_panel import RobotMonitorPanel
import std_msgs.msg
import std_srvs.srv

import rospy
from roslib import rosenv

from os import path
import threading

from status_control import StatusControl
from power_state_control import PowerStateControl
from diagnostics_frame import DiagnosticsFrame
from rosout_frame import RosoutFrame

class NaoFrame(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    
    def __init__(self, parent, id=wx.ID_ANY, title='Nao Dashboard', pos=wx.DefaultPosition, size=(400, 50), style=wx.CAPTION|wx.CLOSE_BOX|wx.STAY_ON_TOP):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
        wx.InitAllImageHandlers()
        
        rospy.init_node('nao_dashboard', anonymous=True)
        try:
            getattr(rxtools, "initRoscpp")
            rxtools.initRoscpp("nao_dashboard_cpp", anonymous=True)
        except AttributeError:
            pass
        
        self.SetTitle('Nao Dashboard (%s)'%rosenv.get_master_uri())
        
        icons_path = path.join(roslib.packages.get_pkg_dir('nao_dashboard'), "icons/")
        
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.SetSizer(sizer)
        
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Robot"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        self._robot_combobox = wx.ComboBox(self, wx.ID_ANY, "", choices = [])
        static_sizer.Add(self._robot_combobox, 0)
        
        gobject.threads_init()
        dbus.glib.threads_init() 
        self.robots = []
        self.sys_bus = dbus.SystemBus()
        self.avahi_server = dbus.Interface(self.sys_bus.get_object(avahi.DBUS_NAME, '/'), avahi.DBUS_INTERFACE_SERVER)

        self.sbrowser = dbus.Interface(self.sys_bus.get_object(avahi.DBUS_NAME, self.avahi_server.ServiceBrowserNew(avahi.IF_UNSPEC,
            avahi.PROTO_INET, '_naoqi._tcp', 'local', dbus.UInt32(0))), avahi.DBUS_INTERFACE_SERVICE_BROWSER)

        self.sbrowser.connect_to_signal("ItemNew", self.avahiNewItem)
        self.sbrowser.connect_to_signal("ItemRemove", self.avahiItemRemove)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Diagnostic"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Diagnostics
        self._diagnostics_button = StatusControl(self, wx.ID_ANY, icons_path, "diag", True)
        self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics"))
        static_sizer.Add(self._diagnostics_button, 0)
        
        # Rosout
        self._rosout_button = StatusControl(self, wx.ID_ANY, icons_path, "rosout", True)
        self._rosout_button.SetToolTip(wx.ToolTip("Rosout"))
        static_sizer.Add(self._rosout_button, 0)
        
        # Joint temperature
        self._temp_joint_button = StatusControl(self, wx.ID_ANY, icons_path, "temperature_joints", True)
        self._temp_joint_button.SetToolTip(wx.ToolTip("Joint temperatures"))
        static_sizer.Add(self._temp_joint_button, 0)

        # CPU temperature
        self._temp_head_button = StatusControl(self, wx.ID_ANY, icons_path, "temperature_head", True)
        self._temp_head_button.SetToolTip(wx.ToolTip("CPU temperature"))
        static_sizer.Add(self._temp_head_button, 0)

        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Stiffness"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)

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
        static_sizer.Add(self._motors_button, 0)
        self._motors_button.Bind(wx.EVT_LEFT_DOWN, self.on_motors_clicked)
                
        static_sizer = wx.StaticBoxSizer(wx.StaticBox(self, wx.ID_ANY, "Battery"), wx.HORIZONTAL)
        sizer.Add(static_sizer, 0)
        
        # Battery State
        self._power_state_ctrl = PowerStateControl(self, wx.ID_ANY, icons_path)
        self._power_state_ctrl.SetToolTip(wx.ToolTip("Battery: Stale"))
        static_sizer.Add(self._power_state_ctrl, 1, wx.EXPAND)
        
        self._config = wx.Config("nao_dashboard")
        
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
        self.Layout()
        self.Fit()
        
        self._diagnostics_frame = DiagnosticsFrame(self, wx.ID_ANY, "Diagnostics")
        self._diagnostics_frame.Hide()
        self._diagnostics_frame.Center()
        self._diagnostics_button.Bind(wx.EVT_BUTTON, self.on_diagnostics_clicked)
        
        self._rosout_frame = RosoutFrame(self, wx.ID_ANY, "Rosout")
        self._rosout_frame.Hide()
        self._rosout_frame.Center()
        self._rosout_button.Bind(wx.EVT_BUTTON, self.on_rosout_clicked)
        
        self.load_config()
        
        self._timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self._timer.Start(500)
    
        self._agg_sub = rospy.Subscriber('diagnostics_agg', DiagnosticArray, self.diagnostic_callback)
        self._last_diagnostics_message_time = 0.0
        self.bodyPoseClient = actionlib.SimpleActionClient('body_pose', BodyPoseAction)
        self.stiffnessEnableClient = rospy.ServiceProxy("body_stiffness/enable", std_srvs.srv.Empty)
        self.stiffnessDisableClient = rospy.ServiceProxy("body_stiffness/disable", std_srvs.srv.Empty)
        
        
    def __del__(self):
        self._dashboard_agg_sub.unregister()
        
    def on_timer(self, evt):
      level = self._diagnostics_frame._diagnostics_panel.get_top_level_state()
      if (level == -1 or level == 3):
        if (self._diagnostics_button.set_stale()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Stale"))
      elif (level >= 2):
        if (self._diagnostics_button.set_error()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Error"))
      elif (level == 1):
        if (self._diagnostics_button.set_warn()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: Warning"))
      else:
        if (self._diagnostics_button.set_ok()):
            self._diagnostics_button.SetToolTip(wx.ToolTip("Diagnostics: OK"))
        
      self.update_rosout()
      
      if (rospy.get_time() - self._last_diagnostics_message_time > 5.0):
          ctrls = [self._motors_button, self._power_state_ctrl, self._temp_head_button, self._temp_joint_button]
          for ctrl in ctrls:
              ctrl.set_stale()
              ctrl.SetToolTip(wx.ToolTip("No message received on diagnostics_agg in the last 5 seconds"))
        
      if (rospy.is_shutdown()):
        self.Close()
        
    def on_diagnostics_clicked(self, evt):
      self._diagnostics_frame.Show()
      self._diagnostics_frame.Raise()
      
    def on_rosout_clicked(self, evt):
      self._rosout_frame.Show()
      self._rosout_frame.Raise()
      
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
        self._last_diagnostics_message_time = rospy.get_time()
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
               
    def update_rosout(self):
      summary_dur = 30.0
      if (rospy.get_time() < 30.0):
          summary_dur = rospy.get_time() - 1.0
          
      if (summary_dur < 0):
          summary_dur = 0.0
    
      summary = self._rosout_frame.get_panel().getMessageSummary(summary_dur)
      
      if (summary.fatal or summary.error):
        self._rosout_button.set_error()
      elif (summary.warn):
        self._rosout_button.set_warn()
      else:
        self._rosout_button.set_ok()
        
        
      tooltip = ""
      if (summary.fatal):
        tooltip += "\nFatal: %s"%(summary.fatal)
      if (summary.error):
        tooltip += "\nError: %s"%(summary.error)
      if (summary.warn):
        tooltip += "\nWarn: %s"%(summary.warn)
      if (summary.info):
        tooltip += "\nInfo: %s"%(summary.info)
      if (summary.debug):
        tooltip += "\nDebug: %s"%(summary.debug)
      
      if (len(tooltip) == 0):
        tooltip = "Rosout: no recent activity"
      else:
        tooltip = "Rosout: recent activity:" + tooltip
    
      if (tooltip != self._rosout_button.GetToolTip().GetTip()):
          self._rosout_button.SetToolTip(wx.ToolTip(tooltip))
        
    def load_config(self):
      # Load our window options
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      if (self._config.HasEntry(self._CONFIG_WINDOW_X)):
          x = self._config.ReadInt(self._CONFIG_WINDOW_X)
      if (self._config.HasEntry(self._CONFIG_WINDOW_Y)):
          y = self._config.ReadInt(self._CONFIG_WINDOW_Y)
      
      self.SetPosition((x, y))
      self.SetSize((width, height))
        
    def save_config(self):
      config = self._config
      
      (x, y) = self.GetPositionTuple()
      (width, height) = self.GetSizeTuple()
      config.WriteInt(self._CONFIG_WINDOW_X, x)
      config.WriteInt(self._CONFIG_WINDOW_Y, y)
      
      config.Flush()
        
    def on_close(self, event):
      self.save_config()
      
      self.Destroy()
      
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
            
                
