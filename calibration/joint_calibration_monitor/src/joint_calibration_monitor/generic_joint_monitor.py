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

from robot_msgs.msg import *
from diagnostic_msgs.msg import *

# Should these be constants in the Status msg?
LEVEL_OK = 0
LEVEL_WARN = 1
LEVEL_ERROR = 2


FLAG_OK = 0
FLAG_DEADBAND = 1
FLAG_ERROR = 2

class GenericJointMonitor() :

    def __init__(self, actuator_name, joint_name, predicate, max_hist_len) :
        self._actuator   = actuator_name
        self._joint = joint_name
        self._hist = [ ]
        self._max_hist_len = max_hist_len
        self._flag_lookup = {FLAG_OK:"OK",
                             FLAG_DEADBAND:"Deadband",
                             FLAG_ERROR:"Error"}
        self._predicate = predicate

    def update(self, mech_state) :
        diag = DiagnosticStatus()
        diag.level = LEVEL_OK  # Default the level to 'OK'
        diag.name = "Joint Monitor: %s" % self._joint
        diag.message = "Good"
        diag.values = [ ]
        diag.strings = [ ]

        # Display the time, just so that something is always updating
        #timestamp = mech_state.header.stamp.to_seconds()
        #diag.values.append(DiagnosticValue(value=timestamp, label="Time"))

        # Display the actuator and joint names for this monitor
        diag.strings.append(DiagnosticString(value=self._actuator, label="Actuator"))
        diag.strings.append(DiagnosticString(value=self._joint, label="Joint"))

        # Check if we can find both the joint and actuator
        act_names = [x.name for x in mech_state.actuator_states]
        act_exists = self._actuator in act_names ;

        if act_exists :
            cal_flag = mech_state.actuator_states[act_names.index(self._actuator)].calibration_reading

        diag.values.append(DiagnosticValue(value=(1 if act_exists else 0),
                                           label="Actuator Exists"))

        joint_names = [x.name for x in mech_state.joint_states]
        joint_exists = self._joint in joint_names
        if joint_exists :
            joint_pos = mech_state.joint_states[joint_names.index(self._joint)].position

        diag.values.append(DiagnosticValue(value=(1 if joint_exists else 0),
                                           label="Joint Exists"))

        if not (act_exists and joint_exists) :
            diag.level = max([LEVEL_WARN, diag.level])
            diag.message = "Error finding joint and actuator"
            return diag

        diag.values.append(DiagnosticValue(value=cal_flag,
                                           label="Flag State"))

        diag.values.append(DiagnosticValue(value=joint_pos,
                                           label="Joint Position"))

        # Call the predicate to see what the result is for this joint
        sample_result = self._predicate(cal_flag, joint_pos)

        # Add the current datapoint to our history
        self._hist.append( sample_result )
        while len(self._hist) > self._max_hist_len :
            self._hist.pop(0)

        flag_oks       = len([x for x in self._hist if x == FLAG_OK])
        flag_deadbands = len([x for x in self._hist if x == FLAG_DEADBAND])
        flag_errors    = len([x for x in self._hist if x == FLAG_ERROR])

        diag.strings.append(DiagnosticString(value=self._flag_lookup[sample_result],
                                             label="Flag Check Result"))

        diag.values.append(DiagnosticValue(value=flag_oks,
                                           label="Flag OKs"))
        diag.values.append(DiagnosticValue(value=flag_deadbands,
                                           label="Flag Deadbands"))
        diag.values.append(DiagnosticValue(value=flag_errors,
                                           label="Flag Errors"))

        if flag_errors > 0 :
            diag.level = LEVEL_ERROR
            diag.message = "Incorrect flag reading"

        return diag
