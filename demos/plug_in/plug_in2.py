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
import tf

CONTROLLER = 'arm_constraint'

TF = tf.TransformListener()

class Tracker:
    def __init__(self, topic, Msg):
        self.sub = rospy.Subscriber(topic, Msg, self.callback)
        self.msg = None

    def callback(self, msg):
        self.msg = msg


def xyz(x, y, z):
  p = Point()
  p.x, p.y, p.z = x, y, z
  return p

def rpy(r, p, y):
  a = tf.transformations.quaternion_from_euler(r, p, y, "rzyx")
  q = Quaternion(a[0], a[1], a[2], a[3])
  return q


def pose_to_matrix(p):
    return tf.transformations.concatenate_transforms(
        tf.transformations.translation_matrix([p.position.x, p.position.y, p.position.z]),
        tf.transformations.rotation_matrix_from_quaternion([p.orientation.x, p.orientation.y,
                                                            p.orientation.z, p.orientation.w]))

def transform_to_matrix(t):
    if hasattr(t, 'transform'):
        t = t.transform
    print t, type(t)
    return tf.transformations.concatenate_transforms(
        tf.transformations.translation_matrix([t.translation.x, t.translation.y, t.translation.z]),
        tf.transformations.rotation_matrix_from_quaternion([t.rotation.x, t.rotation.y,
                                                            t.rotation.z, t.rotation.w]))

def matrix_to_transform(m):
    quat = tf.transformations.quaternion_from_rotation_matrix(m)
    t = Transform()
    t.translation.x = m[0,3]
    t.translation.y = m[1,3]
    t.translation.z = m[2,3]
    t.rotation.x = quat[0]
    t.rotation.y = quat[1]
    t.rotation.z = quat[2]
    t.rotation.w = quat[3]
    return t


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
    kill_and_spawn('<controllers></controllers>', ['r_arm_pose', 'r_arm_twist', 'r_arm_wrench',
                                                   CONTROLLER, 'arm_hybrid'])

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

    if True:

        # Spawns the pose controller
        print "Spawning the pose controller"
        pose_config = '''
<controllers>
  <controller name="r_arm_wrench" type="CartesianWrenchController" />
  <controller name="r_arm_twist" type="CartesianTwistController" />
  <controller name="r_arm_pose" type="CartesianPoseController" />
</controllers>'''
        #resp = kill_and_spawn(pose_config, [])
        resp = kill_and_spawn('<controller name="r_arm_wrench" type="CartesianWrenchController" />', [])
        resp = kill_and_spawn('<controller name="r_arm_twist" type="CartesianTwistController" />', [])
        resp = kill_and_spawn('<controller name="r_arm_pose" type="CartesianPoseController" />', [])
        if len(resp.add_name) < 1 or not resp.add_ok[0]:
            raise "Failed to spawn the pose controller"

        # Commands the hand to near the outlet
        print "Staging the plug"
        staging_pose = PoseStamped()
        staging_pose.header.frame_id = 'outlet_pose'
        #staging_pose.pose.position = xyz(-0.12, 0.0, 0.0)
        #staging_pose.pose.orientation = rpy(0,0,0)
        staging_pose.pose.position = xyz(-0.08, 0.0, 0.04)
        staging_pose.pose.orientation = rpy(0.0, 0.3, -0.1)
        pub_pose = rospy.Publisher('/r_arm_pose/command', PoseStamped)
        for i in range(20):
            staging_pose.header.stamp = rospy.get_rostime()
            pub_pose.publish(staging_pose)
            time.sleep(0.1)
        time.sleep(1)

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

    ######  Switches to the hybrid controller

    print "Spawning the hybrid controller"
    rospy.set_param("/arm_hybrid/type", "CartesianHybridControllerNode")
    rospy.set_param("/arm_hybrid/initial_mode", 3)
    rospy.set_param("/arm_hybrid/root_link", "torso_lift_link")
    rospy.set_param("/arm_hybrid/tip_link", "r_gripper_tool_frame")

    # Gets the last reported tool frame
    tool_pose = track_plug_pose.msg

    # Transforms the tool pose into the end-effector frame
    t = TF.get_transform(tool_pose.header.frame_id, 'r_gripper_tool_frame', tool_pose.header.stamp)
    tool_pose_ee = t.transform * tf.pose_msg_to_bt(tool_pose.pose)

    rospy.set_param("/arm_hybrid/tool_frame/translation/x", tool_pose_ee.getOrigin().x())
    rospy.set_param("/arm_hybrid/tool_frame/translation/y", tool_pose_ee.getOrigin().y())
    rospy.set_param("/arm_hybrid/tool_frame/translation/z", tool_pose_ee.getOrigin().z())
    rospy.set_param("/arm_hybrid/tool_frame/rotation/x", tool_pose_ee.getRotation().x())
    rospy.set_param("/arm_hybrid/tool_frame/rotation/y", tool_pose_ee.getRotation().y())
    rospy.set_param("/arm_hybrid/tool_frame/rotation/z", tool_pose_ee.getRotation().z())
    rospy.set_param("/arm_hybrid/tool_frame/rotation/w", tool_pose_ee.getRotation().w())

    hybrid_config = '<controller name="arm_hybrid" type="CartesianHybridControllerNode" />'
    resp = kill_and_spawn(hybrid_config, ['cartesian_trajectory_right',
                                          'r_arm_pose', 'r_arm_twist', 'r_arm_wrench'])
    if len(resp.add_name) == 0 or not resp.add_ok[0]:
        raise "Failed to spawn the hybrid controller"



    while not rospy.is_shutdown():
        time.sleep(0.1)


if __name__ == '__main__': main()
