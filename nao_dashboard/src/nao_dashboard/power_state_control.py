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

from os import path
import rospy

from python_qt_binding.QtCore import QSize
from rqt_robot_dashboard.widgets import BatteryDashWidget


class PowerStateControl(BatteryDashWidget):
    def __init__(self, icons=None, charge_icons=None, icon_paths=None, suppress_overlays=False):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        """
        super(PowerStateControl, self).__init__('NAO Battery')

        self._power_consumption = 0.0
        self._pct = 0

    def set_power_state(self, msg):
        last_pct = self._pct
        last_discharging = self._discharging
        last_charging = self._charging
        last_full = self._full
        last_plugged_in = self._plugged_in
        
        self.isStale = False
        for kv in msg:
            if kv.key == "Current":
                self._power_consumption = float(kv.value)
            elif kv.key == "Percentage":
                if kv.value == "unknown":
                    isStale = True
                    self.set_stale()
                else:
                    self._pct = float(kv.value)/100.
            elif kv.key == "Discharging flag":
                self._discharging = (kv.value == "True")
            elif kv.key == "Charge Flag":
                self._charging = (kv.value == "True")
            elif kv.key == "Full Charge Flag":
                self._full = (kv.value == "True")
        
        self._plugged_in = ((self._charging or not self._discharging) and not self.isStale)
        if (last_pct != self._pct or last_discharging != self._discharging or last_charging != self._charging or last_full != self._full):
            if(self._full):
                self.SetToolTip(wx.ToolTip("Battery fully charged"))
            else:
                drain_str = "discharging"
                if (self._charging):
                    drain_str = "charging"
                    self.SetToolTip(wx.ToolTip("Battery: %.0f%% (%s)"%(self._pct * 100., drain_str)))
            self.Refresh()
        
    def set_stale(self):
        self._plugged_in = False
        self._discharging = 0
        self._pct = 0
        self._power_consumption = 0
        self.SetToolTip(wx.ToolTip("Battery: Stale"))
        self.isStale = True
        
        self.Refresh()
