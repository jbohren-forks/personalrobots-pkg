#!/usr/bin/python

import rostools
rostools.update_path('vslam')
#import rostest
#import rospy

#from pylab import *
from numpy import *
from scipy.optimize import fmin
from scipy.optimize import fmin_bfgs
from scipy.optimize import fmin_powell
from scipy.optimize import fmin_ncg
from scipy.optimize import fmin_cg
from scipy.optimize import leastsq
import transformations
from visualodometer import Pose

# go thru both trajectories in time, pick matching points from curve1 that
# matches curve0 the best in time stamps
def match_trajectory_points(trj0, trj1, time_stamp_offset):
  trj0_len = len(trj0)
  trj1_len = len(trj1)
  trj2 = zeros((trj0_len, 4))
  j = 0
  num_matches=0
  for i in range(0, trj0_len):
    matched = False
    time_stamp0 = trj0[i][0]
    while j<trj1_len-1 and matched==False:
      before = trj1[j  ][0]-time_stamp_offset
      after  = trj1[j+1][0]-time_stamp_offset
      if before > time_stamp0:
        # must be at starting point
        matched = True
        t, x, y, z = trj1[j]
        trj2[i] = array([time_stamp0, x, y, z])
        num_matches+=1
      elif before<=time_stamp0 and after>=time_stamp0:
        matched = True
        interval = after-before
        w0 = (after - time_stamp0)/interval
        w1 = (time_stamp0 - before)/interval
        t, x,y,z = trj1[j]*w0 + trj1[j+1]*w1
        trj2[i] = array([time_stamp0, x, y, z])
        num_matches+=1
      else:
        # move on to next point on trj1
        j+=1
    
    if matched == False:
      # must be a last one
      t, x, y, z = trj1[trj1_len-1]
      trj2[i] = array([time_stamp0, x, y, z])
      num_matches+=1

  print 'matching up ',num_matches,' of ',trj0_len
  return trj2


def transf_curve(curve, pose):
  curvelen = len(curve)
  curve1=zeros((curvelen, 4))
  for i in range(0, curvelen):
    t, x, y, z = curve[i]
    x01, y01, z01 = pose.xform(x, y, z)
    curve1[i] = [t, x01, y01, z01]
  return curve1
  
def scale_curve3(curve, scale):
  curvelen = len(curve)
  curve1=zeros((curvelen, 4))
  for i in range(0, curvelen):
    t, x, y, z = curve[i]
    curve1[i] = [t, x*scale[0], y*scale[1], z*scale[2]]
  return curve1
  
def scale_curve(curve, scale):
  curvelen = len(curve)
  curve1=zeros((curvelen, 4))
  for i in range(0, curvelen):
    t, x, y, z = curve[i]
    curve1[i] = [t, x*scale, y*scale, z*scale]
  return curve1
  
def error_func(transf, curve0, curve1):
  # translation vector
  p = transf[0:3]
  
  # euler angles
  e = transf[3:6]

  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)

  # pose = Pose(identity(3), p)
  
  curve01 = transf_curve(curve0, pose)
  
  return curve01-curve1   

def transf_fit_space(transf, curve0, curve1):
  # translation vector
  p = transf[0:3]
  
  # euler angles
  e = transf[3:6]
  
  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)

  curve01 = transf_curve(curve0, pose)
  
  return ((curve01-curve1)**2).sum()

def transf_fit_space_scale(transf, curve0, curve1):
  # translation vector
  p = transf[0:3]
  
  # euler angles
  e = transf[3:6]
  
  # scale 
  s = transf[6]

  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)
  
  curve01 = transf_curve(curve0, pose)
  curve01 = scale_curve(curve01, s)
  
  return ((curve01-curve1)**2).sum()
  
def transf_fit_angle_scale_time(transf, curve0, curve1):
  # no translation
  p = [0.,0.,0.]
  # euler angles
  e = transf[0:3]
  
  # scale 
  s = transf[3]
  
  # timestamp offset
  t = transf[4]

  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)

  curve01 = transf_curve(curve0, pose)
  curve01 = scale_curve(curve01, s)
  curve01 = curve01 + array([t, 0.0, 0.0, 0.0])
  
  return ((curve01-curve1)**2).sum()  
  
def transf_fit_angle_scale(transf, curve0, curve1):
  # no translation
  p = [0.,0.,0.]
  # euler angles
  e = transf[0:3]
  # scale 
  s = transf[3]
  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)
  curve01 = transf_curve(curve0, pose)
  curve01 = scale_curve(curve01, s)
  return ((curve01-curve1)**2).sum()  
  
def transf_fit_angle_scale3(transf, curve0, curve1):
  # no translation
  p = [0.,0.,0.]
  # euler angles
  e = transf[0:3]
  # scale 
  s = transf[3:6]
  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)
  curve01 = transf_curve(curve0, pose)
  curve01 = scale_curve3(curve01, s)
  return ((curve01-curve1)**2).sum()  
  
def transf_fit_angle(transf, curve0, curve1):
  # no translation
  p = [0.,0.,0.]  
  # euler angles
  e = transf[0:3]
  
  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)

  curve01 = transf_curve(curve0, pose)
  
  return ((curve01-curve1)**2).sum()   

def transf_fit_space_scale_time(transf, curve0, curve1):
  # translation vector
  p = transf[0:3]
  
  # euler angles
  e = transf[3:6]
  
  # scale 
  s = transf[6]
  
  # timestamp offset
  t = transf[7]

  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)

  curve01 = transf_curve(curve0, pose)
  curve01 = scale_curve(curve01, s)
  curve01 = curve01 + array([t, 0.0, 0.0, 0.0])
  
  return ((curve01-curve1)**2).sum()

def test_transf_fit0():
  curve0 = random.random((100, 4))
  p = [3., 4., 5.]
  e = [0.1,0.1,-0.2]
  t = 0.001
  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)
  print pose.M
  # pose = Pose(identity(3), p)
  curve1 = transf_curve(curve0, pose)
  curve1 = scale_curve(curve1, 1.15)
  curve1 = curve1 + array([t, 0., 0., 0.])
  
  print 'diff of time', ((curve1[:,0]-curve0[:,0])**2).sum()

  # fitting  
  transf_fit = transf_fit_space_scale_time
  transf0=[3.1, 4.1, 5.1, 0., 0., 0., 1.0, 0.00]
  transf = fmin(transf_fit, transf0, args=(curve0, curve1),maxiter=10000, maxfun=10000)
  print 'fmin, transf', transf
  
  transf0=[3.1, 4.1, 5.1, 0., 0., 0., 1.0, 0.00]
  transf = fmin_powell(transf_fit, transf0, args=(curve0, curve1))
  print 'fmin_powell, transf', transf
  
  transf0=[3.1, 4.1, 5.1, 0., 0., 0., 1.0, 0.00]
  transf = fmin_cg(transf_fit, transf0, fprime=None, args=(curve0, curve1))
  print 'fmin_cg, transf', transf
  
  transf0=[3.1, 4.1, 5.1, 0., 0., 0., 1.0, 0.00]
  transf = fmin_bfgs(transf_fit, transf0, fprime=None, args=(curve0, curve1))
  print 'fmin_bfgs, transf', transf
  
  # transf, success = leastsq(error_func, transf0, args=(curve0, curve1), maxfev=10000)


if __name__ == "__main__":
  test_transf_fit0()