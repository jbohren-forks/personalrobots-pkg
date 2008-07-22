from pkg import *
from std_msgs.msg import Point3DFloat64
from std_msgs.msg import BaseVel
from std_msgs.msg import RobotBase2DOdom
import sys
import numpy as np
import nodes as n
import camera as cam
import math

class FollowBehavior:
    def __init__(self, velocity_topic):
        self.velocity_topic = velocity_topic
        R = cam.Rx(math.radians(90)) * cam.Ry(math.radians(-90))
        T = np.matrix([-.095, 0,.162]).T
        self.base_T_camera = cam.homo_transform3d(R, T)
        self.robot_pose     = n.ConstNode(n.Pose2D(0, 0, 0))
        self.local_pos      = n.nav.V_KeepLocal_P2d_V(self.robot_pose, n.ConstNode(np.matrix([0,0]).T))
        self.attr	        = n.nav.V_LinearAttract_V(self.local_pos, dead_zone = 0.02, far_dist = 1.0)
        self.cmd            = n.nav.R_Conv_V(self.attr, allow_backwards_driving = False)
        self.should_stop    = self.attr.is_done()
        self.has_stopped   = False
        self.attr.verbosity = -1

    def cursor_moved(self, point):
        #Transform into base's coordinate frame
        point3d   = np.matrix([point.x, point.y, point.z, 1.0]).T
        print 'FollowBehavior.cursor_moved: got point', point3d.T
        new_point = self.base_T_camera * point3d

        #Drop the z, store as homogeneous coordinates
        goal2d = new_point[0:2, 0]
        print 'FollowBehavior.cursor_moved: 2d goal', goal2d.T
        self.local_pos.remember(n.ConstNode(goal2d))
        self.has_stopped = False

    def robot_moved(self, update):
        print 'FollowBehavior.robot_moved:', update.pos.x, update.pos.y, update.pos.th
        self.robot_pose.const = Pose2D(update.pos.x, update.pos.y, update.pos.th)

    def run(self):
        count = 0
        if self.should_stop.val(count) and not self.has_stopped:
            self.velocity_topic.publish(BaseVel(0,0))
            self.has_stopped = True 
        else:
            base_command = self.cmd.val(count)
            msg = BaseVel(base_command.forward_vel, base_command.rot_vel)
            #print 'publishing', base_command.forward_vel, base_command.rot_vel
            #self.velocity_topic.publish(msg)
        count = count + 1

class FakeTopic:
    def publish(self, something):
        pass


if __name__ == '__main__':
    import time
    pub = rospy.TopicPub('cmd_vel', BaseVel)
    follow_behavior = FollowBehavior(pub)
    rospy.TopicSub('odom', RobotBase2DOdom, follow_behavior.robot_moved)
    rospy.TopicSub(CURSOR_TOPIC, Point3DFloat64, follow_behavior.cursor_moved)
    rospy.ready(sys.argv[0])
    while (True):
        follow_behavior.run()
        time.sleep(0.016)

#follow_behavior = FollowBehavior(FakeTopic())
















































#def transform2D(o_p, o_angle):
#    """ 
#        Takes as input o_P_* and angle, both descriptions of the new coordinate
#        frame in the original frame (frame o).  Returns a transfrom *_T_o that 
#        describes points in the new coordinate frame (frame *).  
#        The transformation applies a translation then a rotation.
#    """
#    t = numpy.matrix(numpy.eye(3))
#    t[0:2][:,2] = -1 * o_p
#    return rot_mat(o_angle) * t   
#
#def rot_mat(a):
#    """ 
#        Makes a homogeneous rotation matrix that rotates the coordinate system 
#        counter cw
#        i.e. rotates points clockwise
#    """
#    return numpy.matrix([[math.cos(a),    math.sin(a), 0],
#                         [-1*math.sin(a), math.cos(a), 0],
#                         [0,              0,           1]])
#
#class FollowLight:
#    def __init__(self, velocity_topic):
#        self.base_T_camera  = np.matrix(np.zeros((3,4)))
#        self.dead_zone      = 0.02
#        self.far_dist       = 1.0   #Distance beyond which we don't care
#		self.min_attr       = 0.3   #Set min attraction to be 30% of maximum speed
#
#        self.velocity_topic = velocity_topic
#        self.goal_b         = [0.0, 0.0]
#        self.A_T_O          = transform2d(np.matrix([0.0, 0.0]).T, 0.0)
#
#    def cursor_moved(self, point):
#        #Transform into base's coordinate frame
#        point3d   = np.matrix([point.x, point.y, point.z, 1.0]).T
#        new_point = self.base_T_camera * point3d
#
#        #Drop the z, store as homogeneous coordinates
#        self.goal_b = new_point[0:2, 0]
#
#    def robot_moved(self, update):
#        #Frames
#        # A   previous position of robot in global frame
#        # B   current position of robot in global frame
#        # O   global frame
#        B_T_O = transform2d(np.matrix([update.pos.x, update.pos.y]).T, 
#                update.pos.th)
#        
#        #transform goal with this update
#        goal_a        = self.goal_b
#        B_T_A         = B_T_O * np.linalg.inv(self.A_T_O) 
#        self.goal_b   = B_T_A * goal_a
#        self.A_T_O    = B_T_O
#
#    def run(self):
#        #given goal position, calculate velocity vector
#        distance = np.linalg.norm(goal)
#        if distance > self.dead_zone:
#            mag = ((1 - self.min_attr) / (self.far_dist - self.dead_zone)) * norm_dist + self.min_attr
#			mag = min(max(mag, self.min_attr), 1.0)
#			out = mag * (goal / dist)
#            vx = 
#        else:
#        vw = 
#
#        #send to robot
#        self.velocity_topic.publish(BaseVel(vx,vw))


























