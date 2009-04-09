# Copyright (c) 2009, Willow Garage, Inc.
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

import roslib; roslib.load_manifest("tf")

import tf
import bullet

def transform_msg_to_bt(msg):
    rot = msg.rotation
    tr = msg.translation
    t = bullet.Transform(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w),
                         bullet.Vector3(tr.x, tr.y, tr.z))
    t.setOrigin(bullet.Vector3(tr.x, tr.y, tr.z))
    t.setRotation(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w))
    return t

def transform_stamped_msg_to_bt(msg):
    return tf.TransformStamped(transform_msg_to_bt(msg.transform),
                                    msg.header.stamp.to_seconds(),
                                    msg.header.frame_id,
                                    msg.parent_id)

def transform_bt_to_msg(bt):
    rot = bt.getRotation()
    tr = bt.getOrigin()
    msg = robot_msgs.Transform()
    msg.translation.x = tr.x()
    msg.translation.y = tr.y()
    msg.translation.z = tr.z()
    msg.rotation.x = rot.x()
    msg.rotation.y = rot.y()
    msg.rotation.z = rot.z()
    msg.rotation.w = rot.w()
    return msg

def transform_stamped_bt_to_msg(bt):
    msg = robot_msgs.TransformStamped()
    msg.transform = transform_bt_to_msg(bt.transform)
    msg.header.frame_id = bt.frame_id
    msg.header.stamp = rospy.rostime().from_seconds(bt.stamp)
    msg.parent_id = bt.parent_id
    return msg

def pose_msg_to_bt(msg):
    rot = msg.rotation
    tr = msg.translation
    t = bullet.Pose(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w),
                         bullet.Vector3(tr.x, tr.y, tr.z))
    t.setOrigin(bullet.Vector3(tr.x, tr.y, tr.z))
    t.setRotation(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w))
    return t

def pose_stamped_msg_to_bt(msg):
    return tf.PoseStamped(pose_msg_to_bt(msg.pose),
                                    msg.header.stamp.to_seconds(),
                                    msg.header.frame_id,
                                    msg.parent_id)

def pose_bt_to_msg(bt):
    rot = bt.getRotation()
    tr = bt.getOrigin()
    msg = robot_msgs.Pose()
    msg.translation.x = tr.x()
    msg.translation.y = tr.y()
    msg.translation.z = tr.z()
    msg.rotation.x = rot.x()
    msg.rotation.y = rot.y()
    msg.rotation.z = rot.z()
    msg.rotation.w = rot.w()
    return msg

def pose_stamped_bt_to_msg(bt):
    msg = robot_msgs.PoseStamped()
    msg.pose = pose_bt_to_msg(bt.pose)
    msg.header.frame_id = bt.frame_id
    msg.header.stamp = rospy.rostime().from_seconds(bt.stamp)
    msg.parent_id = bt.parent_id
    return msg

def point_msg_to_bt(msg):
    rot = msg.rotation
    tr = msg.translation
    t = bullet.Point(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w),
                         bullet.Vector3(tr.x, tr.y, tr.z))
    t.setOrigin(bullet.Vector3(tr.x, tr.y, tr.z))
    t.setRotation(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w))
    return t

def point_stamped_msg_to_bt(msg):
    return tf.PointStamped(point_msg_to_bt(msg.point),
                                    msg.header.stamp.to_seconds(),
                                    msg.header.frame_id,
                                    msg.parent_id)

def point_bt_to_msg(bt):
    rot = bt.getRotation()
    tr = bt.getOrigin()
    msg = robot_msgs.Point()
    msg.translation.x = tr.x()
    msg.translation.y = tr.y()
    msg.translation.z = tr.z()
    msg.rotation.x = rot.x()
    msg.rotation.y = rot.y()
    msg.rotation.z = rot.z()
    msg.rotation.w = rot.w()
    return msg

def point_stamped_bt_to_msg(bt):
    msg = robot_msgs.PointStamped()
    msg.point = point_bt_to_msg(bt.point)
    msg.header.frame_id = bt.frame_id
    msg.header.stamp = rospy.rostime().from_seconds(bt.stamp)
    msg.parent_id = bt.parent_id
    return msg

def vector_msg_to_bt(msg):
    rot = msg.rotation
    tr = msg.translation
    t = bullet.Vector(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w),
                         bullet.Vector3(tr.x, tr.y, tr.z))
    t.setOrigin(bullet.Vector3(tr.x, tr.y, tr.z))
    t.setRotation(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w))
    return t

def vector_stamped_msg_to_bt(msg):
    return tf.VectorStamped(vector_msg_to_bt(msg.vector),
                                    msg.header.stamp.to_seconds(),
                                    msg.header.frame_id,
                                    msg.parent_id)

def vector_bt_to_msg(bt):
    rot = bt.getRotation()
    tr = bt.getOrigin()
    msg = robot_msgs.Vector()
    msg.translation.x = tr.x()
    msg.translation.y = tr.y()
    msg.translation.z = tr.z()
    msg.rotation.x = rot.x()
    msg.rotation.y = rot.y()
    msg.rotation.z = rot.z()
    msg.rotation.w = rot.w()
    return msg

def vector_stamped_bt_to_msg(bt):
    msg = robot_msgs.VectorStamped()
    msg.vector = vector_bt_to_msg(bt.vector)
    msg.header.frame_id = bt.frame_id
    msg.header.stamp = rospy.rostime().from_seconds(bt.stamp)
    msg.parent_id = bt.parent_id
    return msg


def quaternion_msg_to_bt(msg):
    rot = msg.rotation
    tr = msg.translation
    t = bullet.Quaternion(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w),
                         bullet.Vector3(tr.x, tr.y, tr.z))
    t.setOrigin(bullet.Vector3(tr.x, tr.y, tr.z))
    t.setRotation(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w))
    return t

def quaternion_stamped_msg_to_bt(msg):
    return tf.QuaternionStamped(quaternion_msg_to_bt(msg.quaternion),
                                    msg.header.stamp.to_seconds(),
                                    msg.header.frame_id,
                                    msg.parent_id)

def quaternion_bt_to_msg(bt):
    rot = bt.getRotation()
    tr = bt.getOrigin()
    msg = robot_msgs.Quaternion()
    msg.translation.x = tr.x()
    msg.translation.y = tr.y()
    msg.translation.z = tr.z()
    msg.rotation.x = rot.x()
    msg.rotation.y = rot.y()
    msg.rotation.z = rot.z()
    msg.rotation.w = rot.w()
    return msg

def quaternion_stamped_bt_to_msg(bt):
    msg = robot_msgs.QuaternionStamped()
    msg.quaternion = quaternion_bt_to_msg(bt.quaternion)
    msg.header.frame_id = bt.frame_id
    msg.header.stamp = rospy.rostime().from_seconds(bt.stamp)
    msg.parent_id = bt.parent_id
    return msg

