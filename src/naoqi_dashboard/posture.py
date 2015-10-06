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

import actionlib
import rospy

from python_qt_binding.QtGui import QComboBox, QMessageBox
from naoqi_bridge_msgs.msg import BodyPoseWithSpeedAction, BodyPoseWithSpeedGoal

class PostureWidget(QComboBox):
    def __init__(self, topic_prefix):
        super(PostureWidget, self).__init__()
        self.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.setInsertPolicy(QComboBox.InsertAlphabetically)
        self.setEditable(True)

        if topic_prefix == "nao_robot":
            posture_list = ["---", "Crouch", "LyingBack", "LyingBelly", "Sit", "SitOnChair", "SitRelax", "Stand", "StandInit", "StandZero"]
        elif topic_prefix == "pepper_robot":
            posture_list = [ "---", "Crouch", "Stand", "StandInit", "StandZero" ]
        else:
            posture_list = []

        self.addItems( posture_list )
        self.currentIndexChanged.connect( self.apply_posture )

        self.bodyPoseClient = actionlib.SimpleActionClient(str(topic_prefix)+'/pose/body_pose_naoqi', BodyPoseWithSpeedAction)

    def apply_posture(self):
        posture = self.currentText()
        rospy.loginfo("go to posture: "+ str(posture))
        self.bodyPoseClient.send_goal_and_wait(BodyPoseWithSpeedGoal(posture_name = posture, speed=0.7))
        state = self.bodyPoseClient.get_state()
        if not state == actionlib.GoalStatus.SUCCEEDED:
            QMessageBox(self, 'Error', str(posture)+' posture did not succeed: %s - cannot remove stiffness' % self.bodyPoseClient.get_goal_status_text())
            rospy.logerror("crouch pose did not succeed: %s", self.bodyPoseClient.get_goal_status_text())
