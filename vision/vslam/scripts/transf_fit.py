from pylab import *
from numpy import *
from scipy.optimize import fmin
from scipy.optimize import fmin_bfgs
from scipy.optimize import fmin_powell
from scipy.optimize import fmin_ncg
from scipy.optimize import fmin_cg
from scipy.optimize import leastsq
import transformations
from visualodometer import Pose

def transf_curve(curve, pose):
  curvelen = len(curve)
  curve1=zeros((curvelen, 4))
  for i in range(0, curvelen):
    t, x, y, z = curve[i]
    x01, y01, z01 = pose.xform(x, y, z)
    curve1[i] = [t, x01, y01, z01]
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

def transf_fit(transf, curve0, curve1):
  # translation vector
  p = transf[0:3]
  
  # euler angles
  e = transf[3:6]
  
  # scale 
  s = transf[6]

  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)

  # pose = Pose(identity(3), p)
  
  curve01 = transf_curve(curve0, pose)
  curve01 = scale_curve(curve01, s)
  
  return ((curve01-curve1)**2).sum()

def test_transf_fit0():
  curve0 = random.random((100, 4))
  p = [3., 4., 5.]
  e = [0.1,0.1,-0.2]
  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)
  print pose.M
  # pose = Pose(identity(3), p)
  curve1 = transf_curve(curve0, pose)
  curve1 = scale_curve(curve1, 1.15)
  
  print 'diff of time', ((curve1[:,0]-curve0[:,0])**2).sum()

  # fitting  
  transf0=[3.1, 4.1, 5.1, 0., 0., 0., 1.0]
  transf = fmin(transf_fit, transf0, args=(curve0, curve1),maxiter=10000, maxfun=10000)
  print 'fmin, transf', transf
  
  transf0=[3.1, 4.1, 5.1, 0., 0., 0., 1.0]
  transf = fmin_powell(transf_fit, transf0, args=(curve0, curve1))
  print 'fmin_powell, transf', transf
  
  transf0=[3.1, 4.1, 5.1, 0., 0., 0., 1.0]
  transf = fmin_cg(transf_fit, transf0, fprime=None, args=(curve0, curve1))
  print 'fmin_cg, transf', transf
  
  transf0=[3.1, 4.1, 5.1, 0., 0., 0., 1.0]
  transf = fmin_bfgs(transf_fit, transf0, fprime=None, args=(curve0, curve1))
  print 'fmin_bfgs, transf', transf
  
  # transf, success = leastsq(error_func, transf0, args=(curve0, curve1), maxfev=10000)


if __name__ == "__main__":
  test_transf_fit0()