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
import tf.tf_swig
import bullet
import robot_msgs.msg
import rospy #for rostime


class PoseStamped(tf.tf_swig.PoseStamped):
    def __getattribute__(self,name):
        if name == "stamp":
            return roslib.rostime.Time(self._sec, self._nsec)
        else:
            return tf.tf_swig.PoseStamped.__getattribute__(self,name)

    def __setattr__(self, name, value):
        if name == "stamp":
            object.__setattr__(self, '_sec',int(value.secs))
            object.__setattr__(self, '_nsec',value.nsecs)
        else:
            object.__setattr__(self,name, value) #calls through to base class
        
class TransformStamped(tf.tf_swig.TransformStamped):
    def __getattribute__(self,name):
        if name == "stamp":
            return roslib.rostime.Time(self._sec, self._nsec)
        else:
            return tf.tf_swig.TransformStamped.__getattribute__(self,name)

    def __setattr__(self, name, value):
        if name == "stamp":
            object.__setattr__(self, '_sec',value.secs)
            object.__setattr__(self, '_nsec',value.nsecs)
        else:
            object.__setattr__(self,name, value) #calls through to base class
        
class PointStamped(tf.tf_swig.PointStamped):
    def __getattribute__(self,name):
        if name == "stamp":
            return roslib.rostime.Time(self._sec, self._nsec)
        else:
            return tf.tf_swig.PointStamped.__getattribute__(self,name)

    def __setattr__(self, name, value):
        if name == "stamp":
            object.__setattr__(self, '_sec',value.secs)
            object.__setattr__(self, '_nsec',value.nsecs)
        else:
            object.__setattr__(self,name, value) #calls through to base class

class VectorStamped(tf.tf_swig.VectorStamped):
    def __getattribute__(self,name):
        if name == "stamp":
            return roslib.rostime.Time(self._sec, self._nsec)
        else:
            return tf.tf_swig.VectorStamped.__getattribute__(self,name)

    def __setattr__(self, name, value):
        if name == "stamp":
            object.__setattr__(self, '_sec',value.secs)
            object.__setattr__(self, '_nsec',value.nsecs)
        else:
            object.__setattr__(self,name, value) #calls through to base class

class QuaternionStamped(tf.tf_swig.QuaternionStamped):
    def __getattribute__(self,name):
        if name == "stamp":
            return roslib.rostime.Time(self._sec, self._nsec)
        else:
            return tf.tf_swig.QuaternionStamped.__getattribute__(self,name)

    def __setattr__(self, name, value):
        if name == "stamp":
            object.__setattr__(self, '_sec',value.secs)
            object.__setattr__(self, '_nsec',value.nsecs)
        else:
            object.__setattr__(self,name, value) #calls through to base class

def transform_msg_to_bt(msg):
    rot = msg.rotation
    tr = msg.translation
    t = bullet.Transform(bullet.Quaternion(.2,.2,.2,.5),#rot.x, rot.y, rot.z, rot.w),
                         bullet.Vector3(tr.x, tr.y, tr.z))
    t.setOrigin(bullet.Vector3(tr.x, tr.y, tr.z))
    t.setRotation(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w))
    return t

def transform_stamped_msg_to_bt(msg):
    return tf.TransformStamped(transform_msg_to_bt(msg.transform),
                                    msg.header.stamp.secs, msg.header.stamp.nsecs,
                                    msg.header.frame_id,
                                    msg.parent_id)

def transform_bt_to_msg(bt):
    rot = bt.getRotation()
    tr = bt.getOrigin()
    msg = robot_msgs.msg.Transform()
    msg.translation.x = tr.x()
    msg.translation.y = tr.y()
    msg.translation.z = tr.z()
    msg.rotation.x = rot.x()
    msg.rotation.y = rot.y()
    msg.rotation.z = rot.z()
    msg.rotation.w = rot.w()
    return msg

def transform_stamped_bt_to_msg(bt):
    msg = robot_msgs.msg.TransformStamped()
    msg.transform = transform_bt_to_msg(bt.transform)
    msg.header.frame_id = bt.frame_id
    msg.header.stamp = roslib.rostime.Time(bt._sec, bt._nsec)
    msg.parent_id = bt.parent_id
    return msg

def pose_msg_to_bt(msg):
    rot = msg.orientation
    tr = msg.position
    t = bullet.Transform(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w),
                         bullet.Vector3(tr.x, tr.y, tr.z))
    t.setOrigin(bullet.Vector3(tr.x, tr.y, tr.z))
    t.setRotation(bullet.Quaternion(rot.x, rot.y, rot.z, rot.w))
    return t

def pose_stamped_msg_to_bt(msg):
    return tf.PoseStamped(pose_msg_to_bt(msg.pose),
                                    msg.header.stamp.secs, msg.header.stamp.nsecs,
                                    msg.header.frame_id)

def pose_bt_to_msg(bt):
    rot = bt.getRotation()
    tr = bt.getOrigin()
    msg = robot_msgs.msg.Pose()
    msg.position.x = tr.x()
    msg.position.y = tr.y()
    msg.position.z = tr.z()
    msg.orientation.x = rot.x()
    msg.orientation.y = rot.y()
    msg.orientation.z = rot.z()
    msg.orientation.w = rot.w()
    return msg

def pose_stamped_bt_to_msg(bt):
    msg = robot_msgs.msg.PoseStamped()
    msg.pose = pose_bt_to_msg(bt.pose)
    msg.header.frame_id = bt.frame_id
    msg.header.stamp = roslib.rostime.Time(bt._sec, bt._nsec)
    return msg

def point_msg_to_bt(msg):
    t = bullet.Vector3(msg.x, msg.y, msg.z)
    return t

def point_stamped_msg_to_bt(msg):
    return tf.PointStamped(point_msg_to_bt(msg.point),
                                    msg.header.stamp.secs, msg.header.stamp.nsecs,
                                    msg.header.frame_id)

def point_bt_to_msg(bt):
    msg = robot_msgs.msg.Point()
    msg.x = bt.x()
    msg.y = bt.y()
    msg.z = bt.z()
    return msg

def point_stamped_bt_to_msg(bt):
    msg = robot_msgs.msg.PointStamped()
    msg.point = point_bt_to_msg(bt.point)
    msg.header.frame_id = bt.frame_id
    msg.header.stamp = bt.stamp
    return msg


def vector_msg_to_bt(msg):
    t = bullet.Vector3(msg.x, msg.y, msg.z)
    return t

def vector_stamped_msg_to_bt(msg):
    return tf.VectorStamped(vector_msg_to_bt(msg.vector),
                                    msg.header.stamp.secs, msg.header.stamp.nsecs,
                                    msg.header.frame_id)

def vector_bt_to_msg(bt):
    msg = robot_msgs.msg.Vector3()
    msg.x = bt.x()
    msg.y = bt.y()
    msg.z = bt.z()
    return msg

def vector_stamped_bt_to_msg(bt):
    msg = robot_msgs.msg.Vector3Stamped()
    msg.vector = vector_bt_to_msg(bt.vector)
    msg.header.frame_id = bt.frame_id
    msg.header.stamp = bt.stamp
    return msg

def quaternion_msg_to_bt(msg):
    t = bullet.Quaternion(msg.x, msg.y, msg.z, msg.w)
    return t

def quaternion_stamped_msg_to_bt(msg):
    return tf.QuaternionStamped(quaternion_msg_to_bt(msg.quaternion),
                                    msg.header.stamp.secs, msg.header.stamp.nsecs,
                                    msg.header.frame_id)

def quaternion_bt_to_msg(bt):
    msg = robot_msgs.msg.Quaternion()
    msg.x = bt.x()
    msg.y = bt.y()
    msg.z = bt.z()
    msg.w = bt.w()
    return msg

def quaternion_stamped_bt_to_msg(bt):
    msg = robot_msgs.msg.QuaternionStamped()
    msg.quaternion = quaternion_bt_to_msg(bt.quaternion)
    msg.header.frame_id = bt.frame_id
    msg.header.stamp = bt.stamp
    return msg

