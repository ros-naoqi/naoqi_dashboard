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

from rqt_robot_dashboard.widgets import BatteryDashWidget

class PowerStateControl(BatteryDashWidget):
    """
    A Widget that interprets NAO's battery status and displays battery state
    """
    def __init__(self, name):
        """
        :param name: name of the plugin
        :type name: String
        """
        super(PowerStateControl, self).__init__(name)

    def set_power_state(self, msg):
        self.isStale = False
        for kv in msg:
            if kv.key == "Percentage":
                if kv.value == "unknown":
                    isStale = True
                    self.set_stale()
                else:
                    self.update_perc(float(kv.value))
                    # we should display a time here but
                    # we don't want to scare the user with a 0
                    self.update_time(float(kv.value))
            elif kv.key == "Charging":
                if kv.value == "True":
                    self.set_charging(True)
                else:
                    self.set_charging(False)
            elif kv.key == "Message":
                self.setToolTip(kv.value)

    def set_stale(self):
        self.setToolTip("Battery: Stale")
        self.isStale = True
