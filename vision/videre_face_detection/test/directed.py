import rostools
rostools.update_path('videre_face_detection')
import rospy
import rostest
import videre_face_detection
import visualodometer
import camera
from std_msgs.msg import Image, ImageArray, String, PointStamped
import visual_odometry as VO
import starfeature

from stereo import SparseStereoFrame

import sys
sys.path.append('lib')

import Image
import ImageChops
import ImageDraw

import random
import time
import math
import numpy

import os
import copy

DEBUG = True
SAVE_PICS = True

################# CLASS IMGADAPTED ##################
class imgAdapted:
  def __init__(self, i):
    self.i = i
    self.size = i.size
    self.rawdata = i.tostring()

################## CLASS TRACKER CHILD OF CLASS VISUALODOMETER ################
class Tracker(visualodometer.VisualOdometer) :

  def collect_descriptors(self, frame):
    self.timer['descriptor_collection'].start()
    frame.descriptors = [ VO.grab_16x16(frame.rawdata, frame.size[0], p[0]-7, p[1]-7) for p in frame.kp ]
    self.timer['descriptor_collection'].stop()


################## CLASS PEOPLETRACKER #############
class PeopleTracker:
  def __init__(self):
    self.usebag = False
    self.keyframes = []
    self.current_keyframes = []
    self.p = videre_face_detection.people()
    self.cam = None
    self.camparams = None
    self.vo = None
    self.feature_detector = visualodometer.FeatureDetectorStar()
    self.feature_detector.thresh = 3.0
    self.cascade_file = "cascades/haarcascade_frontalface_alt.xml"
    assert os.access(self.cascade_file, os.R_OK)
    self.faces = None
    self.seq = 0
    self.keyframe_maxlength = 1
    self.sad_thresh = 2000.0
    self.num_feats = 40
    self.line_thresh = 10.0
    self.num_scales = 5
    self.feats_to_centers = []
    self.feats_to_centers_3d = []
    self.face_centers_3d = []
    self.face_sizes_3d = []
    self.min_real_face_size = 50 # in mm
    self.max_real_face_size = 400 # in mm
    self.good_points_thresh = 0.5
    self.recent_good_frames = []
    self.recent_good_rects = []
    self.recent_good_rects_3d = []
    self.same_key_rgfs = []
    self.pub = []


################### RECT_TO_CENTER_DIFF #######################
  def rect_to_center_diff(self, rect):
    diff = ((rect[2]-1)/2.0, (rect[3]-1)/2.0)
    center = (rect[0]+diff[0], rect[1]+diff[1])
    return center, diff

################### GET_FEATURES ##############################
  def get_features(self, frame, target_points, rect):
    
    if DEBUG:
      print "Features from rect ", rect
    
    full = Image.fromstring("L",frame.size,frame.rawdata)
    (x,y,w,h) = (int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3]))
    incr = 16
    subim = full.crop((x-incr,y-incr,x+w+incr,y+h+incr))
    
    sd = starfeature.star_detector(w+2*incr, h+2*incr, self.num_scales, self.feature_detector.thresh, self.line_thresh)
    results = [ (x1,y1) for (x1,y1,s1,r1) in sd.detect(subim.tostring()) ]
    return [(x-incr+x1,y-incr+y1) for (x1,y1) in results if (incr<x1) and (incr<y1) and (x1<w+incr) and (y1<h+incr)]

################### MAKE_FACE_MODEL ###########################
  def make_face_model(self, wincenter, maxdiff,  feat_list) :
    wc = numpy.array(wincenter)
    fl = numpy.array(feat_list)
    feats_to_center = wc-fl
    return feats_to_center.tolist()
    #inbounds = [ftc.tolist() for ftc in feats_to_center if numpy.max(numpy.fabs(ftc)-maxdiff)<0.0] 
    #print feats_to_center
    #print inbounds
    #return inbounds


#################### MEAN SHIFT ###############################
  def mean_shift_sparse_old(self, start_point, window_size, sparse_pred_list, max_iter, eps) :
    dpoint = [0,0]
    total_weight = 0
    half_width = (window_size[0]-1)/2.0
    half_height = (window_size[1]-1)/2.0
    new_point = start_point
    # For each iteration
    for iter in range(0,max_iter) :
      # Add each prediction in the kernel to the displacement
      for (x,y,weight) in sparse_pred_list :
        diffxy = (x-new_point[0], y-new_point[1])
        if (abs(diffxy[0])<=half_width) and (abs(diffxy[1])<=half_height) :
          total_weight += weight
          dpoint[0] += weight*diffxy[0]
          dpoint[1] += weight*diffxy[1]
      # If there weren't any predictions in the kernel, return the old point.
      if total_weight == 0 :
        return new_point
      # Otherwise, move the point
      else :
        dpoint[0] /= total_weight
        dpoint[1] /= total_weight
        new_point = [new_point[0] + dpoint[0], new_point[1] + dpoint[1]]
        
      # If the displacement was small, return the point
      if dpoint[0]*dpoint[0]+dpoint[1]*dpoint[1] <= eps*eps :
        return new_point

    # Reached the maximum number of iterations, return
    return new_point


#################### MEAN SHIFT ###############################
# start_point = Point from which to start tracking.
# sparse_pred_list = List of non-zero probabilities/predictions. 
#                    Each prob has the form: [x,y,...] 
# probs = A weight for each prediction
# bandwidths = The size of the (square) window around each prediction
# max_iter = The maximum number of iterations
# eps = The minimum displacement
  def mean_shift_sparse(self, start_point, sparse_pred_list, probs, bandwidths, max_iter, eps) :
    if len(sparse_pred_list)==0:
      return start_point
    total_weight = 0.0
    new_point = numpy.array(start_point)
    pred_list = numpy.array(sparse_pred_list)
    dpoint = numpy.array(0.0*pred_list[0])
    # For each iteration
    for iter in range(0,max_iter) :
      # Add each prediction in the kernel to the displacement
      for ipred in range(len(pred_list)) :
        diff = pred_list[ipred]-new_point
        dist = numpy.dot(diff,diff)
        if dist < bandwidths[ipred]*bandwidths[ipred]:
          total_weight += probs[ipred]
          dpoint += probs[ipred]*diff
      # If there weren't any predictions in the kernel, return the old point.
      if total_weight == 0 :
        return new_point.tolist()
      # Otherwise, move the point
      else :
        dpoint /= total_weight
        new_point = new_point + dpoint
        
      # If the displacement was small, return the point
      if numpy.dot(dpoint,dpoint) <= eps*eps :
        return new_point.tolist()

    # Reached the maximum number of iterations, return
    return new_point.tolist()


################### PARAMETER CALLBACK ########################
  def params(self, pmsg):

    if not self.vo:
      print "Setting calib params"
      self.cam = camera.VidereCamera(pmsg.data)
      self.vo = Tracker(self.cam)
        

################### STAMPED POINT CALLBACK (SANITY CHECK) #####
  def point_stamped(self,psmsg):
    print "Heard ", psmsg.point.x, psmsg.point.y, psmsg.point.z, psmsg.header.frame_id
  


################### IMAGE CALLBACK ############################
  def frame(self, imarray):

    print "Frame callback"

    # No calibration params yet.
    if not self.vo:
      return

    im = imarray.images[1]
    im_py = Image.fromstring("L", (im.width, im.height), im.data)
    im_r = imarray.images[0]
    im_r_py = Image.fromstring("L", (im_r.width, im_r.height), im_r.data)

    # Detect faces on the first frame
    if not self.current_keyframes :
      self.faces = self.p.detectAllFaces(im.data, im.width, im.height, self.cascade_file, 1.0, None, None, True) 
      if DEBUG:
        print "Faces ", self.faces
      

    sparse_pred_list = []
    sparse_pred_list_2d = []
    matches = []
    sadscores = []
    old_rect = [0,0,0,0]
    ia = SparseStereoFrame(im_py,im_r_py)

    # Track each face
    for iface in range(0,len(self.faces)):
      
      (x,y,w,h) = self.faces[iface]
      if DEBUG:
        print "A face ", (x,y,w,h)
      
      ia.kp2d = self.get_features(ia, self.num_feats, (x, y, w, h))
      if not ia.kp2d:
        continue

      self.vo.find_disparities(ia)
      if not ia.kp:
        continue

      iakp3d = [self.cam.pix2cam(kp[0], kp[1], kp[2]) for kp in ia.kp] 
      iaavgd = sum([tz for (tx,ty,tz) in ia.kp])/len(ia.kp)

      # First frame:
      if len(self.current_keyframes) < iface+1:

        (cen,diff) = self.rect_to_center_diff((x,y,w,h))
        cen3d = self.cam.pix2cam(cen[0],cen[1],iaavgd)
        ltf = self.cam.pix2cam(x,y,iaavgd)
        rbf = self.cam.pix2cam(x+w,y+h,iaavgd)
        fs3d = ( (rbf[0]-ltf[0]) + (rbf[1]-ltf[1]) )/4.0

        # Check that the face is a reasonable size. If not, skip this face.
        if fs3d < self.min_real_face_size or fs3d > self.max_real_face_size:
          continue

        self.vo.collect_descriptors(ia)

        self.current_keyframes.append(0)
        self.keyframes.append(copy.copy(ia))

        self.feats_to_centers.append(self.make_face_model( cen, diff, ia.kp2d ))

        self.face_sizes_3d.append( copy.deepcopy(fs3d) )
        self.feats_to_centers_3d.append( self.make_face_model( cen3d, (fs3d,fs3d,fs3d), iakp3d) )
        self.face_centers_3d.append( copy.deepcopy(cen3d) )

        self.recent_good_frames.append(copy.copy(ia))
        self.recent_good_rects.append(copy.deepcopy([x,y,w,h]))

        self.same_key_rgfs.append(True)

      # Later frames
      else :
        self.vo.collect_descriptors(ia)

        done_matching = False
        bad_frame = False
        while not done_matching:

          # Try matching to the keyframe
          keyframe = self.keyframes[self.current_keyframes[iface]]
          matches = self.vo.temporal_match(keyframe,ia)
          sadscores = [(VO.sad(keyframe.descriptors[a], ia.descriptors[b])) for (a,b) in matches]

          good_matches = [m for m in sadscores if m < self.sad_thresh]
        
          # Not enough matches, get a new keyframe
          if len(keyframe.kp)<2 or len(good_matches) < len(keyframe.kp)/2.0 : #3

            #self.keyframe[self.current_keyframe] = ia
            if DEBUG:
              print "New keyframe"

            # Make a new face model, either from a recent good frame, or from the current image
            if not self.same_key_rgfs[iface] :

              (cen, diff) = self.rect_to_center_diff(self.recent_good_rects[iface])
              self.feats_to_centers[iface] = self.make_face_model( cen, diff, self.recent_good_frames[iface].kp2d )

              avgd = sum([tz for (tx,ty,tz) in self.recent_good_frames[iface].kp])/len(self.recent_good_frames[iface].kp)
              cen3d = self.cam.pix2cam(cen[0],cen[1],avgd)
              self.feats_to_centers_3d[iface] = self.make_face_model( cen3d, 
                                                                      (self.face_sizes_3d[iface], self.face_sizes_3d[iface], self.face_sizes_3d[iface]),
                                                                      [self.cam.pix2cam(kp[0],kp[1],kp[2]) for kp in self.recent_good_frames[iface].kp])
              self.face_centers_3d[iface] = copy.deepcopy(cen3d)
              # Not changing the face size

              self.current_keyframes[iface] = 0
              self.keyframes[self.current_keyframes[iface]] = copy.copy(self.recent_good_frames[iface])

              self.same_key_rgfs[iface] = True

              # Don't need to change the recent good frame yet.

            else :
              # BAD!!!
              # Making a new model off of the current frame but with the old position. 
              # Pretty much guaranteed to make tracking fall off the face as your model is now
              # behind.
              # Try just skipping this frame and wait for another frame.
              # However, if I turn this off, I tend to loose track. 
              #bad_frame = True
              done_matching = True
    #          if DEBUG:
    #            print "Bad frame ", self.seq, " for face ", iface
              self.keyframes[self.current_keyframes[iface]] = copy.copy(ia)
              self.current_keyframes[iface] = 0
              (cen,diff) = self.rect_to_center_diff(self.faces[iface])
              self.feats_to_centers[iface] = self.make_face_model( cen, diff, ia.kp2d )
              cen3d = self.cam.pix2cam(cen[0],cen[1],iaavgd)
              self.feats_to_centers_3d[iface] = self.make_face_model( cen3d, 
                                                                      (self.face_sizes_3d[iface], self.face_sizes_3d[iface], self.face_sizes_3d[iface]),
                                                                      iakp3d)
              self.face_centers_3d[iface] = copy.deepcopy(cen3d)
              matches = []
              self.same_key_rgfs[iface] = True

          # Try matching to the NEW keyframe
          #keyframe = self.keyframes[self.current_keyframes[iface]]
          #matches = self.vo.temporal_match(keyframe,ia)
          #sadscores = [(VO.sad(keyframe.descriptors[a], ia.descriptors[b])) for (a,b) in matches]

#          good_matches = [m for m in sadscores if m < self.sad_thresh]

#          print "Feats to cens ", self.feats_to_centers_3d[iface]
#          print len(self.feats_to_centers_3d[iface])
#          print "matches ", matches


          # Good matches, mark this frame as good
          else:
            done_matching = True
            #self.recent_good_frames[iface] = ia
            #self.same_key_rgfs[iface] = False


        # If we couldn't get enough matches for this frame, skip it.
        if bad_frame:
          continue

        # Track
        sparse_pred_list = []
        sparse_pred_list_2d = []
        probs = []
        bandwidths = []
        size_mult = 1.0
        for imatch in range(0,len(matches)) :
          if sadscores[imatch] < self.sad_thresh :
            (match1,match2) = matches[imatch]
            kp3d = self.cam.pix2cam(ia.kp[match2][0],ia.kp[match2][1],ia.kp[match2][2])
            sparse_pred_list.append( (kp3d[0]+self.feats_to_centers_3d[iface][match1][0], 
                                      kp3d[1]+self.feats_to_centers_3d[iface][match1][1],
                                      kp3d[2]+self.feats_to_centers_3d[iface][match1][2]) )
            bandwidths.append(size_mult*self.face_sizes_3d[iface])
            sparse_pred_list_2d.append( (ia.kp2d[match2][0]+self.feats_to_centers[iface][match1][0], ia.kp2d[match2][1]+self.feats_to_centers[iface][match1][1]) ) 
            probs.append(1.0)
            #bandwidths.append(w)
        
        if DEBUG:
          print "Old center 3d ", self.face_centers_3d[iface]
          print "Self.faces ", self.faces[iface]
          print "face recheck ", (x,y,w,h)
          print "Old center 2d ",(x+(w-1)/2.0, y+(h-1)/2.0) 

        # 3D
        old_rect = self.faces[iface]
        new_center = self.mean_shift_sparse( self.face_centers_3d[iface], sparse_pred_list, probs, bandwidths, 10, 5.0 )
        new_center_2d = self.cam.cam2pix(new_center[0], new_center[1], new_center[2])
        ltf = self.cam.cam2pix( new_center[0]-self.face_sizes_3d[iface], new_center[1]-self.face_sizes_3d[iface], new_center[2])
        rbf = self.cam.cam2pix( new_center[0]+self.face_sizes_3d[iface], new_center[1]+self.face_sizes_3d[iface], new_center[2])
        w = rbf[0]-ltf[0]
        h = rbf[1]-ltf[1]       
                              
        if DEBUG:
          print "new center 3d ", new_center
          print "new_center 2d ", new_center_2d
            
        # 2D
        #new_center_2d = self.mean_shift_sparse( (x+(w-1)/2.0, y+(h-1)/2.0), sparse_pred_list, probs, bandwidths, 10, 1.0)
        
        (nx,ny,nw,nh) = (new_center_2d[0]-(w-1)/2.0, new_center_2d[1]-(h-1)/2.0, w, h)

        # Force the window back into the image.
        dx = max(0,0-nx) + min(0, im.width - nx+nw)
        dy = max(0,0-ny) + min(0, im.height - ny+nh)
        nx += dx
        ny += dy

        self.faces[iface] = [nx, ny, nw, nh]
        self.face_centers_3d[iface] = copy.deepcopy(new_center)
        self.recent_good_rects[iface] = [nx,ny,nw,nh]
        ###
        self.recent_good_frames[iface] = copy.copy(ia)
        self.same_key_rgfs[iface] = False
        ###

        if DEBUG:
          print "face 2d ", self.faces[iface]
          print "face center 3d ", self.face_centers_3d[iface]

    
        # Output the location of this face center in the 3D camera frame (of the left camera), and rotate 
        # the coordinates to match the robot's idea of the 3D camera frame.
        center_uvd = (nx + (nw-1)/2.0, ny + (nh-1)/2.0, (numpy.average(ia.kp,0))[2] )
        center_camXYZ = self.cam.pix2cam(center_uvd[0], center_uvd[1], center_uvd[2])
        center_robXYZ = (center_camXYZ[2], -center_camXYZ[0], -center_camXYZ[1])
        if DEBUG:
          print ("center_uvd", center_uvd)
          print ("center_camXYZ", center_camXYZ)
          print ("center_robXYZ", center_robXYZ)

        if not self.usebag:
          stamped_point = PointStamped()
          stamped_point.point.x = center_robXYZ[0]
          stamped_point.point.y = center_robXYZ[1]
          stamped_point.point.z = center_robXYZ[2]
          stamped_point.header.frame_id = "stereo_block"
          self.pub.publish(stamped_point)
    
      
############ DRAWING ################
      if SAVE_PICS:

        if not self.keyframes or len(self.keyframes) <= iface :
          bigim_py = im_py
          draw = ImageDraw.Draw(bigim_py)
        else :
          key_im = self.keyframes[self.current_keyframes[iface]]
          keyim_py = Image.fromstring("L", key_im.size, key_im.rawdata)
          bigim_py = Image.new("RGB",(im_py.size[0]+key_im.size[0], im_py.size[1]))
          bigim_py.paste(keyim_py.convert("RGB"),(0,0))
          bigim_py.paste(im_py,(key_im.size[0]+1,0))
          draw = ImageDraw.Draw(bigim_py)

          (x,y,w,h) = self.faces[iface]
          draw.rectangle((x,y,x+w,y+h),outline=(255,0,0))
          draw.rectangle((x+key_im.size[0],y,x+w+key_im.size[0],y+h),outline=(255,0,0))
          (x,y,w,h) = old_rect
          draw.rectangle((x,y,x+w,y+h),outline=(255,255,255))
          draw.rectangle((x+key_im.size[0],y,x+w+key_im.size[0],y+h),outline=(255,255,255))

          for (x,y) in ia.kp2d:
            draw.line((x-1+key_im.size[0], y-1, x+1+key_im.size[0], y+1), fill=(255,0,0))
            draw.line((x+1+key_im.size[0], y-1, x-1+key_im.size[0], y+1), fill=(255,0,0))

          for (x,y) in key_im.kp2d :
            draw.line((x-1, y-1, x+1, y+1), fill=(255,0,0))
            draw.line((x+1, y-1, x-1, y+1), fill=(255,0,0))

          if self.seq > 0 :

            for (x,y) in sparse_pred_list_2d :
              draw.line((x-1, y-1, x+1, y+1), fill=(0,0,255))
              draw.line((x+1, y-1, x-1, y+1), fill=(0,0,255))
              draw.line((x-1+key_im.size[0], y-1, x+1+key_im.size[0], y+1), fill=(0,0,255))
              draw.line((x+1+key_im.size[0], y-1, x-1+key_im.size[0], y+1), fill=(0,0,255))

            for ((m1,m2), score) in zip(matches,sadscores) :
              if score > self.sad_thresh :
                color = (255,0,0)
              else :
                color = (0,255,0)
              draw.line((key_im.kp2d[m1][0], key_im.kp2d[m1][1], ia.kp2d[m2][0]+key_im.size[0], ia.kp2d[m2][1]), fill=color)

        bigim_py.save("/tmp/tiff/feats%06d_%03d.tiff" % (self.seq, iface))
  
    self.seq += 1

    


############# MAIN #############
def main(argv) :
    
  people_tracker = PeopleTracker()

  if len(argv)>0 and argv[0]=="-bag":
    people_tracker.usebag = True

  # Use a bag file
  if people_tracker.usebag :
  
    import rosrecord
    filename = "/wg/stor2/prdata/videre-bags/loop1-mono.bag"
    #filename = "/wg/stor2/prdata/videre-bags/face2.bag"

    if SAVE_PICS:
      try:
        os.mkdir("/tmp/tiff/")
      except:
        pass

    num_frames = 0
    start_frame = 4000
    end_frame = 4200
    for topic, msg in rosrecord.logplayer(filename):
      if topic == '/videre/cal_params':
        people_tracker.params(msg)
      elif topic == '/videre/images':
        if num_frames >= start_frame and num_frames < end_frame:
          people_tracker.frame(msg)
        num_frames += 1
      else :
        pass

  # Use new ROS messages, and output the 3D position of the face in the camera frame.
  else :
    print "Non-bag"

    if SAVE_PICS:
      try:
        os.mkdir("/tmp/tiff/")
      except:
        pass

    people_tracker.pub = rospy.Publisher('head_controller/track_point',PointStamped)
    rospy.init_node('videre_face_tracker', anonymous=True)
    #rospy.TopicSub('/head_controller/track_point',PointStamped,people_tracker.point_stamped)
    rospy.TopicSub('/videre/images',ImageArray,people_tracker.frame)
    rospy.TopicSub('/videre/cal_params',String,people_tracker.params)
    rospy.spin()


if __name__ == '__main__' :
  main(sys.argv[1:])
