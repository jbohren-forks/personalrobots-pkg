from pylab import *
from numpy import *
from scipy.optimize import fmin
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

def transf_fit(transf, curve0, curve1):
  # translation vector
  p = transf[0:3]
  
  # euler angles
  e = transf[3:6]

  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)

  # pose = Pose(identity(3), p)
  
  curve01 = transf_curve(curve0, pose)
  
  return ((curve01-curve1)**2).sum()

def test_transf_fit0():
  curve0 = random.random((100, 4))
  p = [3., 4., 5.]
  e = [0.1,0.1,-0.2]
  pose = Pose(transformations.rotation_matrix_from_euler(e[0], e[1], e[2], 'sxyz')[:3,:3], p)
  print pose.M
  # pose = Pose(identity(3), p)
  curve1 = transf_curve(curve0, pose)
  
  print 'diff of time', ((curve1[:,0]-curve0[:,0])**2).sum()
  
  # print curve1
  transf0=[3.1, 4.1, 5.1, 0., 0., 0.]
  transf = fmin(transf_fit, transf0, args=(curve0, curve1),maxiter=10000, maxfun=10000)
  print 'transf', transf

if __name__ == "__main__":
  test_transf_fit0()