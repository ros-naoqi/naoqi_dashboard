# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
# Ported from pr2_motors: Vincent Rabaud, Aldebaran Robotics, 2014
#

from python_qt_binding.QtGui import QMessageBox

import actionlib
import rospy
from rqt_robot_dashboard.widgets import MenuDashWidget
import std_srvs.srv

from nao_msgs.msg import BodyPoseAction, BodyPoseGoal


class Motors(MenuDashWidget):
    """
    Dashboard widget to display motor state and allow interaction.
    """
    def __init__(self, context):
        """
        :param context: the plugin context
        :type context: qt_gui.plugin.Plugin
        :param reset_callback: calback for the "reset" action
        :type reset_callback: function
        :param halt_callback: calback for the "reset" action
        :type halt_callback: function
        """
        ok_icon = ['bg-green.svg', 'ic-motors.svg']
        warn_icon = ['bg-yellow.svg', 'ic-motors.svg', 'ol-warn-badge.svg']
        err_icon = ['bg-red.svg', 'ic-motors.svg', 'ol-err-badge.svg']
        stale_icon = ['bg-grey.svg', 'ic-motors.svg', 'ol-stale-badge.svg']

        icons = [ok_icon, warn_icon, err_icon, stale_icon]

        super(Motors, self).__init__('Motors', icons)
        self.update_state(3)

        self.add_action('Init pose', self.on_init_pose)
        self.add_action('Sit down && remove stiffness', self.on_sit_down)
        self.add_action('Remove stiffness immediately', self.on_remove_stiffness)

        # clients for controlling the robot
        self.bodyPoseClient = actionlib.SimpleActionClient('body_pose', BodyPoseAction)
        self.stiffnessEnableClient = rospy.ServiceProxy("body_stiffness/enable", std_srvs.srv.Empty)
        self.stiffnessDisableClient = rospy.ServiceProxy("body_stiffness/disable", std_srvs.srv.Empty)

    def set_ok(self):
        self.update_state(0)

    def set_warn(self):
        self.update_state(1)

    def set_error(self):
        self.update_state(2)

    def set_stale(self):
        self.update_state(3)
      
    def on_init_pose(self):
        self.stiffnessEnableClient.call()
        self.bodyPoseClient.send_goal_and_wait(BodyPoseGoal(pose_name = 'init'))
  
    def on_sit_down(self):
        self.bodyPoseClient.send_goal_and_wait(BodyPoseGoal(pose_name = 'crouch'))
        state = self.bodyPoseClient.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            self.stiffnessDisableClient.call()
        else:
            QMessageBox(self, 'Error', 'crouch pose did not succeed: %s - cannot remove stiffness' % self.bodyPoseClient.get_goal_status_text())
            rospy.logerror("crouch pose did not succeed: %s", self.bodyPoseClient.get_goal_status_text())

    def on_remove_stiffness(self):
      reply = QMessageBox.question(self, 'Caution', 
                     'Robot may fall. Continue to remove stiffness?', QMessageBox.Yes, QMessageBox.No)
      if(reply == QMessageBox.Yes):
          self.stiffnessDisableClient.call()

    def on_halt_motors(self, evt):
        halt = rospy.ServiceProxy("pr2_etherCAT/halt_motors", std_srvs.srv.Empty)
        
        try:
            halt()
        except rospy.ServiceException, e:
            QMessageBox(self, 'Error',
                     'Failed to halt the motors: service call failed with error: %s'%(e))
