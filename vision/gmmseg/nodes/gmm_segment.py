#!/usr/bin/python
'''
@package gmm_segment
Provides Gaussian mixture segmentation service.
Subscribes to topic 'image' of type std_msgs.msg.Image.
'''
import rostools
rostools.updatePath('gmmseg')
import rospy
import std_msgs.msg.Image as RImage 
import gmmseg.texseg as tg
import gmmseg.srv as srv
from threading import RLock
import pyrob.util as ut
import sys, time, math
import opencv.highgui as hg
import time

CAMERA_FOV    = 60.0 
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480

class SegmentNode:
    def __init__(self):
        self.image = None
        self.lock  = RLock()
        t30     = math.tan(math.radians(CAMERA_FOV/2.0))
        self.fx = CAMERA_WIDTH  / t30
        self.fy = CAMERA_HEIGHT / t30

    def segment(self, request):
        print 'Hey Advait, I want to segment the image'
        while self.image == None:
            time.sleep(0.1)

        self.lock.acquire()
        sego = tg.segment_center_object(self.image, display_on=False)
        self.lock.release()

        z     = request.height
        u     = sego.fg_object_ellipse.center[0] - (CAMERA_WIDTH / 2.0)
        v     = sego.fg_object_ellipse.center[1] - (CAMERA_HEIGHT / 2.0)
        x     = u * z / self.fx
        y     = v * z / self.fy
        theta = sego.fg_object_ellipse.angle

        if theta > 90:
          theta = theta - 180
        if theta < -90:
          theta = theta + 180

        theta = math.radians(theta)

#        if True:
#          image_list = sego.get_images_for_display()
#          image_list.append( cvimg )
#          
#          curtime = time.localtime()
#          date_name = time.strftime('%Y%m%d%I%M', curtime)
#        
#          for i,img in enumerate(image_list):
#            fname = date_name+'_image%d.png'%(i)
#            hg.cvSaveImage( fname, img )


        return srv.hrl_graspResponse(x, y, request.height, theta)

    def set_image(self, image):
        if self.lock.acquire(blocking=False):
            self.image = ut.ros2cv(image)
            self.lock.release()
        hg.cvShowImage('image', self.image)
        hg.cvWaitKey(5)

def run_me_server():
    sn       = SegmentNode()
    hg.cvNamedWindow('image', 1)

    rospy.TopicSub('image', RImage, sn.set_image)
    rospy.ready(sys.argv[0])
    segment_service = rospy.Service('hrl_grasp', srv.hrl_grasp, sn.segment)
    segment_service.register()
    rospy.spin()

if __name__ == '__main__':
    run_me_server()
