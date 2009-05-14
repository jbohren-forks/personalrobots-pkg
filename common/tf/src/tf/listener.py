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


## Author Tully Foote tfoote@willowgarage.com

PKG = 'tf'
import roslib
roslib.load_manifest(PKG)

import sys, os

import rospy
import tf
from tf import transformations
import numpy

import bullet

from tf.msg import tfMessage
from robot_msgs.msg import TransformStamped
from tf.srv import FrameGraph,FrameGraphResponse

class NumpyTransformStamped:
    def __init__(self):
        self.mat = numpy.identity(4, dtype=numpy.float64)
        self.stamp = rospy.Time()
        self.frame_id = None
        self.parent_id = None

    def as_transform_stamped(self):
        return transform_stamped_from_numpy_transform_stamped(self)

def transform_stamped_msg_to_numpy(transform):
    quat = numpy.array([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w],dtype=numpy.float64)
    rot_mat = transformations.rotation_matrix_from_quaternion(quat)
    #print "Quat: \n", quat
    #print "Rotation: \n", rot_mat
    trans = numpy.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
    trans_mat = transformations.translation_matrix(trans)
    #print "Trans: \n", trans
    #print "Translation: \n", trans_mat
    net_tr = transformations.concatenate_transforms(trans_mat, rot_mat)
    #print "Net:\n",  net_tr
    return net_tr

def transform_stamped_numpy_to_msg(numpy_transform):
    ts = TransformStamped()
    ts.header.stamp = numpy_transform.stamp
    ts.header.frame_id = numpy_transform.frame_id
    ts.parent_id = numpy_transform.parent_id
    temp_quat = transformations.quaternion_from_rotation_matrix(numpy_transform.mat)
    ts.transform.rot.x = temp_quat[0]
    ts.transform.rot.y = temp_quat[1]
    ts.transform.rot.z = temp_quat[2]
    ts.transform.rot.w = temp_quat[3]
    ###\todo check quaternion order
    ts.transform.translation.x = numpy_transform.mat[0,3]
    ts.transform.translation.y = numpy_transform.mat[1,3]
    ts.transform.translation.z = numpy_transform.mat[2,3]
    return ts

class TransformListener:
    def __init__(self):
        print "Transform Listener initing"
        self.transformer = tf.Transformer()
        #assuming rospy is inited
        rospy.Subscriber("/tf_message", tfMessage, self.callback)
        self.frame_graph_server = rospy.Service('~tf_frames', FrameGraph, self.frame_graph_service)

    def callback(self, data):
        for transform in data.transforms:
            #print "Got data:", transform.header.frame_id
            self.set_transform(tf.transform_stamped_msg_to_bt(transform),data._connection_header["callerid"])

    def frame_graph_service(self, req):
        return FrameGraphResponse(self.all_frames_as_dot())
        
    def set_transform(self, transform_stamped, default_authority="Default Authority"):
        self.transformer.setTransform(transform_stamped, default_authority)

    def get_transform(self, frame_id, parent_id, time):
        return self.transformer.getTransform(frame_id, parent_id, time.to_seconds())

    def get_transform_full(self, target_frame, target_time, source_frame, source_time, fixed_frame):
        return self.transformer.getTransform(target_frame, target_time.to_seconds(), source_frame, source_time.to_seconds(), fixed_frame)

    def can_transform(self, target_frame, source_frame, time):
        return self.transformer.canTransform(target_frame, source_frame, time.to_seconds())

    def can_transform_in_time(self, target_frame, target_time, source_frame, source_time, fixed_frame):
        return self.transformer.canTransform(target_frame, target_time.to_seconds(), source_frame, source_time.to_seconds(), fixed_frame)

    def get_latest_common_time(self, source_frame, target_frame):
        return self.transformer.getLatestCommonTime(source_frame, target_frame)

    def get_chain_as_string(self, target_frame, target_time, source_frame, source_time, fixed_frame):
        return self.transformer.chainAsString(target_frame, target_time, source_frame, source_time, fixed_frame)

    def all_frames_as_string(self):
        return self.transformer.allFramesAsString()

    def all_frames_as_dot(self):
        return self.transformer.allFramesAsDot()

    def set_extrapolation_limit(self, limit):
        self.transformer.setExtrapolationLimit(limit.to_seconds())

    def transform_pose(self, target_frame, pose):
        return self.transformer.transformPose(target_frame, pose)
    
    def transform_pose_in_time(self, target_frame, target_time, fixed_frame, pose):
        return self.transformer.transformPose(target_frame, target_time, pose, fixed_frame)

    def transform_point(self, target_frame, point):
        return self.transformer.transformPoint(target_frame, point)

    def transform_point_in_time(self, target_frame, target_time, fixed_frame, point):
        return self.transformer.transformPoint(target_frame, target_time, point, fixed_frame)

    def transform_vector(self, target_frame, vector):
        return self.transformer.transformVector(target_frame, vector)

    def transform_vector_in_time(self, target_frame, target_time, fixed_frame, vector):
        return self.transformer.transformVector(target_frame, target_time, vector, fixed_frame)

    def transform_quaternion(self, target_frame, quaternion):
        return self.transformer.transformQuaternion(target_frame, quaternion)

    def transform_quaternion_in_time(self, target_frame, target_time, fixed_frame, quaternion):
        return self.transformer.transformQuaternion(target_frame, target_time, quaternion, fixed_frame)

