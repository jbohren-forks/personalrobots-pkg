#! /usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Stuart Glaser

import sys, time
import math

import roslib
roslib.load_manifest('plug_in')
import rospy
from robot_msgs.msg import *
from robot_srvs.srv import *
import tf.transformations

CONTROLLER = 'arm_constraint'

class Tracker:
    def __init__(self, topic, Msg):
        self.sub = rospy.Subscriber(topic, Msg, self.callback)
        self.msg = None

    def callback(self, msg):
        self.msg = msg



mechanism_state = Tracker('/mechanism_state', MechanismState)
def last_time():
    global mechanism_state
    if mechanism_state.msg:
        return mechanism_state.msg.header.stamp
    return 0

def xyz(x, y, z):
  p = Point()
  p.x, p.y, p.z = x, y, z
  return p

def rpy(r, p, y):
  a = tf.transformations.quaternion_from_euler(r, p, y, "rzyx")
  q = Quaternion(a[0], a[1], a[2], a[3])
  return q


def main():
    rospy.init_node('plug_in')

    track_outlet_pose = Tracker('/outlet_detector/pose', PoseStamped)
    track_plug_pose = Tracker('/plug_detector/pose', PoseStamped)

    print "Waiting for mechanism control"
    rospy.wait_for_service('kill_and_spawn_controllers')
    kill_and_spawn = rospy.ServiceProxy('kill_and_spawn_controllers', KillAndSpawnControllers)
    if rospy.is_shutdown(): sys.exit(0)

    # Bring down all possible controllers, just in case
    print "Bringing down old controllers"
    kill_and_spawn('<controllers></controllers>', ['arm_pose', CONTROLLER, 'arm_hybrid'])

    rospy.set_param("/arm_pose/p", 15.0)
    rospy.set_param("/arm_pose/i", 0.2)
    rospy.set_param("/arm_pose/d", 0.0)
    rospy.set_param("/arm_pose/i_clamp", 0.5)
    rospy.set_param("/arm_pose/root_name", "torso_lift_link")
    rospy.set_param("/arm_pose/tip_name", "r_gripper_tool_frame")
    rospy.set_param("/arm_pose/twist/fb_trans/p", 15.0)
    rospy.set_param("/arm_pose/twist/fb_trans/i", 0.5)
    rospy.set_param("/arm_pose/twist/fb_trans/d", 0.0)
    rospy.set_param("/arm_pose/twist/fb_trans/i_clamp", 1.0)
    rospy.set_param("/arm_pose/twist/fb_rot/p", 1.0)
    rospy.set_param("/arm_pose/twist/fb_rot/i", 0.1)
    rospy.set_param("/arm_pose/twist/fb_rot/d", 0.0)
    rospy.set_param("/arm_pose/twist/fb_rot/i_clamp", 0.2)

    ######  Finds the outlet

    # Waits for an estimate of the outlet pose
    print "Waiting for outlet pose..."
    outlet_pose = None
    msg = None
    while not msg:
        msg = track_outlet_pose.msg
        if rospy.is_shutdown(): return
        time.sleep(0.1)
    outlet_pose = msg
    print "Found outlet"

    ######  Stages the plug

    if False:

        # Spawns the pose controller
        print "Spawning the pose controller"
        pose_config = '<controller name="arm_pose" type="CartesianPoseControllerNode" />'
        resp = kill_and_spawn(pose_config, [])
        if len(resp.add_name) == 0 or not resp.add_ok[0]:
            raise "Failed to spawn the pose controller"

        # Commands the hand to near the outlet
        print "Staging the plug"
        staging_pose = PoseStamped()
        staging_pose.header.frame_id = 'outlet_pose'
        staging_pose.pose.position = xyz(-0.12, 0.0, 0.0)
        staging_pose.pose.orientation = rpy(0,0,0)
        pub_pose = rospy.Publisher('/arm_pose/command', PoseStamped)
        for i in range(50):
            staging_pose.header.stamp = last_time() - rospy.rostime.Duration(-0.2)
            pub_pose.publish(staging_pose)
            time.sleep(0.1)
        time.sleep(2)

    else:
        # trajectory controller is already spawned

        # TODO: actually check if the traj controller is up.  Spawn if not

        p_up = PoseStamped()
        p_up.header.frame_id = 'base_link'
        p_up.header.stamp = rospy.get_rostime()
        p_up.pose.position = xyz(0.19, 0.04, 0.5)
        p_up.pose.orientation = Quaternion(-0.19, 0.13, 0.68, 0.68)

        p_face = PoseStamped()
        p_face.header.frame_id = 'base_link'
        p_face.header.stamp = rospy.get_rostime()
        p_face.pose.position = xyz(0.33, -0.09, 0.37)
        p_face.pose.orientation = Quaternion(-0.04, 0.26, -0.00, 0.96)

        p_stage1 = PoseStamped()
        p_stage1.header.frame_id = 'outlet_pose'
        p_stage1.header.stamp = rospy.get_rostime()
        p_stage1.pose.position = xyz(-0.12, 0.0, 0.04)
        p_stage1.pose.orientation = rpy(0,0.3,0)

        p_stage2 = PoseStamped()
        p_stage2.header.frame_id = 'outlet_pose'
        p_stage2.header.stamp = rospy.get_rostime()
        p_stage2.pose.position = xyz(-0.07, 0.0, 0.04)
        p_stage2.pose.orientation = rpy(0,0.3,0)

        try:
            print "Waiting for the trajectory controller"
            move_arm = rospy.ServiceProxy('cartesian_trajectory_right/move_to', MoveToPose)
            print "Staging the plug"
            p_up.header.stamp = rospy.get_rostime(); move_arm(p_up)
            p_face.header.stamp = rospy.get_rostime(); move_arm(p_face)
            p_stage1.header.stamp = rospy.get_rostime(); move_arm(p_stage1)
            p_stage2.header.stamp = rospy.get_rostime(); move_arm(p_stage2)
        except rospy.ServiceException, e:
            print "move_to service failed: %s" % e


    ######  Finds an initial estimate of the plug pose

    # TODO: should wait for the hand to settle
    print "Waiting for the first plug pose estimate"
    track_plug_pose.msg = None
    while not track_plug_pose.msg:
        if rospy.is_shutdown(): return
        time.sleep(0.1)
    plug_pose = track_plug_pose.msg
    print "Found plug"

    if False:  # Constraint controller

        ######  Switches to the constraint controller

        print "Spawning the constraint controller"
        rospy.set_param("/arm_constraint/line_pid/p", 750.0)
        rospy.set_param("/arm_constraint/line_pid/i", 1.0)
        rospy.set_param("/arm_constraint/line_pid/d", 5.0)
        rospy.set_param("/arm_constraint/line_pid/i_clamp", 10.0)
        rospy.set_param("/arm_constraint/pose_pid/p", 35.0)
        rospy.set_param("/arm_constraint/pose_pid/i", 8.0)
        rospy.set_param("/arm_constraint/pose_pid/d", 1.0)
        rospy.set_param("/arm_constraint/pose_pid/i_clamp", 10.0)
        rospy.set_param("/arm_constraint/f_r_max", 150.0)
        constraint_config = open('controllers.xml').read()
        resp = kill_and_spawn(constraint_config, ['cartesian_trajectory_right', 'arm_pose'])
        if len(resp.add_name) == 0 or not resp.add_ok[0]:
            raise "Failed to spawn the constraint controller"

        print "Setting tool frame"
        rospy.wait_for_service("/%s/set_tool_frame" % CONTROLLER)
        if rospy.is_shutdown(): return
        set_tool_frame = rospy.ServiceProxy("/%s/set_tool_frame" % CONTROLLER, SetPoseStamped)
        set_tool_frame(plug_pose)
        print "Tool frame set"
        time.sleep(1)


        ######  Visual differencing loop over the plug pose estimate

        pub_command = rospy.Publisher("/%s/outlet_pose" % CONTROLLER, PoseStamped)
        cnt = 0
        while not rospy.is_shutdown():
            cnt += 1
            outlet_pose.header.stamp = last_time()
            pub_command.publish(outlet_pose)
            if cnt % 3 == 0:
                set_tool_frame(track_plug_pose.msg)
            time.sleep(1.0)

        ######  Attempt to plug in

    else:

        ######  Switches to the hybrid controller

        print "Spawning the hybrid controller"
        rospy.set_param("/arm_hybrid/type", "CartesianHybridControllerNode")
        rospy.set_param("/arm_hybrid/initial_mode", 3)
        rospy.set_param("/arm_hybrid/root_link", "torso_lift_link")
        rospy.set_param("/arm_hybrid/tip_link", "r_gripper_tool_frame")
        #rospy.set_param("/arm_hybrid/fb_pose/p", 30.0)#20.0)
        #rospy.set_param("/arm_hybrid/fb_pose/i", 4.0)
        #rospy.set_param("/arm_hybrid/fb_pose/d", 0.0)
        #rospy.set_param("/arm_hybrid/fb_pose/i_clamp", 2.0)
        #rospy.set_param("/arm_hybrid/fb_trans_vel/p", 15.0)
        #rospy.set_param("/arm_hybrid/fb_trans_vel/i", 1.5)
        #rospy.set_param("/arm_hybrid/fb_trans_vel/d", 0.0)
        #rospy.set_param("/arm_hybrid/fb_trans_vel/i_clamp", 6.0)
        #rospy.set_param("/arm_hybrid/fb_rot_vel/p", 1.2)
        #rospy.set_param("/arm_hybrid/fb_rot_vel/i", 0.2)
        #rospy.set_param("/arm_hybrid/fb_rot_vel/d", 0.0)
        #rospy.set_param("/arm_hybrid/fb_rot_vel/i_clamp", 0.4)
        hybrid_config = '<controller name="arm_hybrid" type="CartesianHybridControllerNode" />'
        resp = kill_and_spawn(hybrid_config, ['cartesian_trajectory_right', 'arm_pose'])
        if len(resp.add_name) == 0 or not resp.add_ok[0]:
            raise "Failed to spawn the hybrid controller"

        ######  Sets the tool frame

        print "Setting tool frame"
        rospy.wait_for_service("/arm_hybrid/set_tool_frame")
        if rospy.is_shutdown(): return
        set_tool_frame = rospy.ServiceProxy("/arm_hybrid/set_tool_frame", SetPoseStamped)
        #set_tool_frame(plug_pose)
        set_tool_frame(track_plug_pose.msg)
        print "Tool frame set"
        time.sleep(1)

        ######  Visual differencing

        track_state = Tracker("/arm_hybrid/state", CartesianState)

        pub_hybrid = rospy.Publisher('/arm_hybrid/command', TaskFrameFormalism)
        tff = TaskFrameFormalism()
        tff.header.frame_id = 'outlet_pose'
        tff.mode.vel.x = 3
        tff.mode.vel.y = 3
        tff.mode.vel.z = 3
        tff.mode.rot.x = 3
        tff.mode.rot.y = 3
        tff.mode.rot.z = 3

        while not rospy.is_shutdown():
            time.sleep(0.1)
        return

        while not rospy.is_shutdown():

            # Waits for the controller to settle
            vel = 1
            track_state.msg = None
            while vel > 0.0001:
                while not track_state.msg:
                    time.sleep(0.01)
                if rospy.is_shutdown(): return
                twist_msg = track_state.msg.last_twist_meas
                v = math.sqrt(twist_msg.vel.x**2 + twist_msg.vel.y**2 + twist_msg.vel.z**2) + \
                    0.1 * math.sqrt(twist_msg.rot.x**2 + twist_msg.rot.y**2 + twist_msg.rot.z**2)
                vel = max(v, 0.1*v + 0.9*vel)
                print "Vel:", vel
            print "Settled"

            tool_pose = track_state.msg.last_pose_meas

            # Gets the next plug pose estimate
            track_plug_pose.msg = None
            while not track_plug_pose.msg:
                time.sleep(0.01)
            print "Got a new plug pose"

            #TODO: plug_in_outlet = TF

            # Determines the offset
            #set_tool_frame(track_plug_pose.msg)

            # Commands the controller to move
            pub_hybrid.publish(tff)


if __name__ == '__main__': main()
